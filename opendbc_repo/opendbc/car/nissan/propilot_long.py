"""Read-only decode of 2019 Nissan Rogue ProPILOT longitudinal CAN.

Nothing in this module sends, edits, or blocks a frame. Bit positions that the
drive-log summary did not pin down are marked provisional on the DBC signals
and again on ProPilotLong.
"""

from __future__ import annotations

import re
from dataclasses import dataclass


NO_DRIVE_TORQUE = -500
TORQUE_BITS = 12
BRAKE_DECEL_PER_PRESSURE = 0.004
# Estimate only. Final route fit: aEgo ≈ -(0.004*pressure + 0.11), lag 0.3 s.
BRAKE_DECEL_OFFSET = 0.11
# While engaged, the 0x15A 10-bit value tracked about 3.14 times 0x1B6 byte 7.
PEDAL_PER_SMOOTH_BYTE = 3.14

# 0x1B6 byte 4 bit 6. Observed as 0x13 (off) vs 0x53 (on). Flag meaning stays provisional.
ENGAGED_BYTE4_MASK = 0x40
# 0x2D4 byte 0 is 0x80 while ICC is engaged (bit 7).
STATUS_ENGAGED_MASK = 0x80
# Provisional: 0x2D4 byte 4 bit 7, armed / set speed stored. Not a hold flag.
STATUS_ARMED_MASK = 0x80

_BO_RE = re.compile(r"^BO_ (\d+) (\w+) *: (\d+) ")
_SG_RE = re.compile(
  r"^SG_ (\w+) : (\d+)\|(\d+)@(\d)([+-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\)"
)


@dataclass(frozen=True)
class Signal:
  name: str
  start: int
  size: int
  little: bool
  signed: bool
  factor: float
  offset: float


@dataclass
class ProPilotLong:
  """One sample. Provisional fields are named in the comments above the masks."""

  valid: bool = False
  drive_torque: int = 0
  drive_torque_inverse_ok: bool = False
  drive_torque_smooth: int = 0
  byte4: int = 0
  engaged: bool = False
  brake_pressure: int = 0
  brake_active: bool = False
  brake_decel: float = 0.0
  status_byte0: int = 0
  status_byte4: int = 0
  status_engaged: bool = False
  armed: bool = False
  res_pressed: bool = False
  pedal_request: int = 0
  pedal_state: int = 0
  pedal_engaged: bool = False
  lead: tuple[int, int, int, int] = (0, 0, 0, 0)
  seen_torque: bool = False
  seen_brake: bool = False
  seen_status: bool = False
  seen_pedal: bool = False
  seen_lead: bool = False


def sign_extend(raw: int, bits: int) -> int:
  raw &= (1 << bits) - 1
  if raw & (1 << (bits - 1)):
    raw -= 1 << bits
  return raw


def inverse_bits(raw12: int) -> int:
  return (~raw12) & 0xFFF


def inverse_ok(torque: int, inverse_raw: int) -> bool:
  """True when the second 12 bits are the bitwise inverse of the first 12."""
  return inverse_bits(torque & 0xFFF) == (inverse_raw & 0xFFF)


def brake_decel(pressure: int) -> float:
  """Estimated deceleration magnitude, m/s^2. Not a measured accel.

  decel ≈ 0.004 * BRAKE_PRESSURE + 0.11, about 0.3 s behind the body.
  The route fit was aEgo ≈ -(0.004*pressure + 0.11).
  """
  return BRAKE_DECEL_PER_PRESSURE * int(pressure) + BRAKE_DECEL_OFFSET


def _be_bits() -> list[int]:
  return [j + i * 8 for i in range(64) for j in range(7, -1, -1)]


def value_bit_positions(start: int, size: int, little: bool) -> list[int]:
  """DBC bit numbers, value bit 0 (LSB) first."""
  if little:
    return list(range(start, start + size))
  bits = _be_bits()
  idx = bits.index(start)
  return list(reversed(bits[idx:idx + size]))


def extract_raw(data: bytes, start: int, size: int, little: bool) -> int:
  raw = 0
  for i, pos in enumerate(value_bit_positions(start, size, little)):
    raw |= ((data[pos // 8] >> (pos % 8)) & 1) << i
  return raw


def insert_raw(data: bytearray, start: int, size: int, little: bool, value: int) -> None:
  for i, pos in enumerate(value_bit_positions(start, size, little)):
    bit = (int(value) >> i) & 1
    mask = 1 << (pos % 8)
    if bit:
      data[pos // 8] |= mask
    else:
      data[pos // 8] &= ~mask & 0xFF


def physical(raw: int, sig: Signal) -> float:
  if sig.signed:
    raw = sign_extend(raw, sig.size)
  return raw * sig.factor + sig.offset


def load_dbc(text: str) -> dict[int, dict[str, Signal]]:
  messages: dict[int, dict[str, Signal]] = {}
  current: int | None = None
  for line in text.splitlines():
    line = line.strip()
    bo = _BO_RE.match(line)
    if bo:
      current = int(bo.group(1))
      messages[current] = {}
      continue
    sg = _SG_RE.match(line)
    if sg and current is not None:
      messages[current][sg.group(1)] = Signal(
        name=sg.group(1),
        start=int(sg.group(2)),
        size=int(sg.group(3)),
        little=sg.group(4) == "1",
        signed=sg.group(5) == "-",
        factor=float(sg.group(6)),
        offset=float(sg.group(7)),
      )
  return messages


def decode_frame(data: bytes, signals: dict[str, Signal]) -> dict[str, float]:
  out: dict[str, float] = {}
  for name, sig in signals.items():
    out[name] = physical(extract_raw(data, sig.start, sig.size, sig.little), sig)
  return out


def encode_frame(values: dict[str, int], signals: dict[str, Signal], size: int = 8) -> bytes:
  data = bytearray(size)
  for name, value in values.items():
    sig = signals[name]
    raw = int(value)
    if sig.signed:
      raw &= (1 << sig.size) - 1
    insert_raw(data, sig.start, sig.size, sig.little, raw)
  return bytes(data)


def observe(
  torque: int,
  torque_inverse: int,
  smooth: int,
  byte4: int,
  brake_pressure: int,
  brake_active: bool,
  status_byte0: int,
  status_byte4: int,
  res_pressed: bool,
  pedal_request: int,
  pedal_state: int,
  lead: tuple[int, int, int, int],
  seen_torque: bool,
  seen_brake: bool,
  seen_status: bool,
  seen_pedal: bool,
  seen_lead: bool,
) -> ProPilotLong:
  pressure = int(brake_pressure)
  lead_bytes = (int(lead[0]) & 0xFF, int(lead[1]) & 0xFF, int(lead[2]) & 0xFF, int(lead[3]) & 0xFF)
  return ProPilotLong(
    valid=bool(seen_torque),
    drive_torque=int(torque),
    drive_torque_inverse_ok=bool(seen_torque) and inverse_ok(torque, torque_inverse),
    drive_torque_smooth=int(smooth) & 0xFF,
    byte4=int(byte4) & 0xFF,
    engaged=bool(int(byte4) & ENGAGED_BYTE4_MASK),
    brake_pressure=pressure,
    brake_active=bool(brake_active),
    brake_decel=brake_decel(pressure) if seen_brake and brake_active else 0.0,
    status_byte0=int(status_byte0) & 0xFF,
    status_byte4=int(status_byte4) & 0xFF,
    status_engaged=bool(int(status_byte0) & STATUS_ENGAGED_MASK),
    armed=bool(int(status_byte4) & STATUS_ARMED_MASK),
    res_pressed=bool(res_pressed),
    pedal_request=int(pedal_request),
    pedal_state=int(pedal_state) & 0xFF,
    pedal_engaged=int(pedal_state) == 2,
    lead=lead_bytes,
    seen_torque=bool(seen_torque),
    seen_brake=bool(seen_brake),
    seen_status=bool(seen_status),
    seen_pedal=bool(seen_pedal),
    seen_lead=bool(seen_lead),
  )
