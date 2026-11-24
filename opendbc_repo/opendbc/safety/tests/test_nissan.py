#!/usr/bin/env python3
import unittest

from opendbc.car.nissan.values import NissanSafetyFlags
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common


class TestNissanSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):

  DBC = "nissan_x_trail_2017_generated"
  SAFETY_MODEL = CarParams.SafetyModel.nissan

  TX_MSGS = [[0x169, 0], [0x2b1, 0], [0x4cc, 0], [0x20b, 2], [0x280, 2]]
  GAS_PRESSED_THRESHOLD = 3
  RELAY_MALFUNCTION_ADDRS = {0: (0x169, 0x2b1, 0x4cc), 2: (0x280,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x280], 2: [0x169, 0x2b1, 0x4cc]}

  EPS_BUS = 0
  CRUISE_BUS = 2

  # Angle control limits
  STEER_ANGLE_MAX = 600  # deg, reasonable limit
  DEG_TO_CAN = 100

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [5., .8, .15]  # windup limit
  ANGLE_RATE_DOWN = [5., 3.5, .4]  # unwind limit

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"DESIRED_ANGLE": angle, "LKA_ACTIVE": 1 if enabled else 0}
    return self.packer.make_can_msg_safety("LKAS", 0, values)

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE": angle}
    return self.packer.make_can_msg_safety("STEER_TORQUE_SENSOR", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_ENABLED": enable}
    return self.packer.make_can_msg_safety("CRUISE_STATE", self.CRUISE_BUS, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_%s" % s: speed * 3.6 for s in ["RR", "RL"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS_REAR", self.EPS_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"USER_BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("DOORS_LIGHTS", self.EPS_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return self.packer.make_can_msg_safety("GAS_PEDAL", self.EPS_BUS, values)

  def _acc_button_cmd(self, cancel=0, propilot=0, flw_dist=0, _set=0, res=0):
    no_button = not any([cancel, propilot, flw_dist, _set, res])
    values = {"CANCEL_BUTTON": cancel, "PROPILOT_BUTTON": propilot,
              "FOLLOW_DISTANCE_BUTTON": flw_dist, "SET_BUTTON": _set,
              "RES_BUTTON": res, "NO_BUTTON_PRESSED": no_button}
    return self.packer.make_can_msg_safety("CRUISE_THROTTLE", 2, values)

  def test_acc_buttons(self):
    btns = [
      ("cancel", True),
      ("propilot", False),
      ("flw_dist", False),
      ("_set", False),
      ("res", False),
      (None, False),
    ]
    for controls_allowed in (True, False):
      for btn, should_tx in btns:
        self.safety.set_controls_allowed(controls_allowed)
        args = {} if btn is None else {btn: 1}
        tx = self._tx(self._acc_button_cmd(**args))
        self.assertEqual(tx, should_tx)


# Real idle CRUISE_THROTTLE frame from a 2019 Rogue (2026-11-23 drive logs): counter in byte 2 bits 0-1,
# byte 2 bit 6 = brake NOT pressed, bytes 3-5 gas pedal + inverse.
ROGUE_IDLE_0x20B = bytes.fromhex("632041003ff0")
SET, RES, NONE, CANCEL = 0x08, 0x10, 0x20, 0x02


def _with_buttons(frame: bytes, buttons: int) -> bytes:
  data = bytearray(frame)
  data[1] = (data[1] & 0xC0) | buttons
  return bytes(data)


class TestNissanIcbmSafety(TestNissanSafety):
  """Rogue cruise-button management, opt-in safety flag."""
  SAFETY_PARAM = NissanSafetyFlags.ICBM

  def setUp(self):
    super().setUp()
    self.t = 1_000_000
    self.safety.set_timer(self.t)

  def _advance(self, ms: float):
    self.t += int(ms * 1000)
    self.safety.set_timer(self.t)

  def _car(self, frame: bytes = ROGUE_IDLE_0x20B):
    self._rx(common.make_msg(0, 0x20B, dat=frame))

  def _op(self, buttons: int, frame: bytes = ROGUE_IDLE_0x20B, bus: int = 2):
    return self._tx(common.make_msg(bus, 0x20B, dat=_with_buttons(frame, buttons)))

  def _engage(self, speed: float = 20.0):
    for _ in range(6):
      self._rx(self._speed_msg(speed))
    self._rx(self._user_brake_msg(False))
    self._rx(self._user_gas_msg(0))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def _tap(self, buttons: int, frames: int = 5) -> list[bool]:
    out = []
    for _ in range(frames):
      self._car()
      self._advance(2)
      out.append(self._op(buttons))
      self._advance(18)
    return out

  def _release(self, frames: int = 8):
    for _ in range(frames):
      self._car()
      self._advance(2)
      self.assertTrue(self._op(NONE))
      self._advance(18)

  def test_acc_buttons(self):
    # Without a fresh car frame and engagement only the stock cancel rule applies.
    btns = [("cancel", True), ("propilot", False), ("flw_dist", False), ("_set", False), ("res", False), (None, False)]
    for controls_allowed in (True, False):
      for btn, should_tx in btns:
        self.safety.set_controls_allowed(controls_allowed)
        args = {} if btn is None else {btn: 1}
        self.assertEqual(self._tx(self._acc_button_cmd(**args)), should_tx)

  def test_tap_allowed_when_engaged(self):
    self._engage()
    for btn in (SET, RES):
      self.assertTrue(all(self._tap(btn)))
      self._release()

  def test_cancel_always_allowed(self):
    self.assertTrue(self._op(CANCEL))
    self._engage()
    self._car()
    self.assertTrue(self._op(CANCEL))

  def test_only_one_button(self):
    self._engage()
    self._car()
    for buttons in (SET | RES, SET | CANCEL, RES | NONE, 0x01, 0x04, 0x01 | NONE, 0x04 | NONE, SET | 0x04):
      self.assertFalse(self._op(buttons), hex(buttons))

  def test_never_propilot_or_follow(self):
    self._engage()
    self._car()
    self.assertFalse(self._op(0x01))
    self.assertFalse(self._op(0x04))

  def test_bus_2_only(self):
    self._engage()
    self._car()
    for bus in (0, 1):
      self.assertFalse(self._op(SET, bus=bus))

  def test_not_engaged_blocked(self):
    for _ in range(6):
      self._rx(self._speed_msg(20.0))
    self._car()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._op(SET))
    self.assertFalse(self._op(RES))
    # NO_BUTTON copies carry no command and stay allowed
    self.assertTrue(self._op(NONE))

  def test_cruise_disengaged_blocked(self):
    self._engage()
    self._rx(self._pcm_status_msg(False))
    self.safety.set_controls_allowed(True)
    self._car()
    self.assertFalse(self._op(SET))
    self.assertFalse(self._op(RES))

  def test_low_speed_and_standstill_blocked(self):
    for speed in (0.0, 1.0, 2.9):
      self._engage(speed=20.0)
      for _ in range(6):
        self._rx(self._speed_msg(speed))
      self._car()
      self.assertFalse(self._op(RES), speed)
      self.assertFalse(self._op(SET), speed)

  def test_pedals_block_taps(self):
    self._engage()
    self._rx(self._user_brake_msg(True))
    self._car()
    self.assertFalse(self._op(SET))
    self._rx(self._user_brake_msg(False))
    self._engage()
    self._rx(self._user_gas_msg(100))
    self._car()
    self.assertFalse(self._op(SET))

  def test_car_frame_driver_input_blocks(self):
    self._engage()
    brake = bytearray(ROGUE_IDLE_0x20B)
    brake[2] &= ~0x40
    gas = bytearray(ROGUE_IDLE_0x20B)
    gas[3] = 0x40
    for frame in (_with_buttons(ROGUE_IDLE_0x20B, 0x01), _with_buttons(ROGUE_IDLE_0x20B, 0x04),
                  _with_buttons(ROGUE_IDLE_0x20B, SET), _with_buttons(ROGUE_IDLE_0x20B, RES), bytes(brake), bytes(gas)):
      self._car(frame)
      self.assertFalse(self._op(SET, frame=frame), frame.hex())
      self.assertFalse(self._op(NONE, frame=frame), frame.hex())

  def test_must_copy_car_frame(self):
    self._engage()
    self._car()
    for i in (0, 2, 3, 4, 5):
      for bit in range(8):
        frame = bytearray(ROGUE_IDLE_0x20B)
        frame[i] ^= 1 << bit
        self.assertFalse(self._op(SET, frame=bytes(frame)), (i, bit))
        self.assertFalse(self._op(NONE, frame=bytes(frame)), (i, bit))
    for bit in (6, 7):
      frame = bytearray(ROGUE_IDLE_0x20B)
      frame[1] ^= 1 << bit
      self.assertFalse(self._op(SET, frame=bytes(frame)))
    # wrong counter (byte 2 bits 0-1)
    frame = bytearray(ROGUE_IDLE_0x20B)
    frame[2] = (frame[2] & 0xFC) | ((frame[2] + 1) & 0x3)
    self.assertFalse(self._op(SET, frame=bytes(frame)))
    self.assertTrue(self._op(SET))

  def test_stale_car_frame(self):
    self._engage()
    self.assertFalse(self._op(SET))  # never seen
    self._car()
    self._advance(25)
    self.assertTrue(self._op(SET))
    self._release(0)
    self._engage()
    self._car()
    self._advance(26)
    self.assertFalse(self._op(NONE))
    self.assertFalse(self._op(SET))

  def test_press_length_limit(self):
    self._engage()
    out = self._tap(SET, frames=14)  # 20 ms apart, press starts at frame 0
    self.assertTrue(all(out[:11]))   # up to 200 ms
    self.assertFalse(any(out[11:]))

  def test_release_and_rate_limit(self):
    self._engage()
    self.assertTrue(all(self._tap(RES)))   # 5 frames ~ 80 ms
    self._release(3)                       # only ~60 ms of release
    self.assertFalse(self._tap(RES, frames=1)[0])
    self._release(2)                       # >= 100 ms since last press frame, >= 200 ms since press start
    self.assertTrue(self._tap(RES, frames=1)[0])

  def test_five_presses_per_second(self):
    self._engage()
    presses = 0
    for i in range(50):  # 1 s at 50 Hz: try a new one-frame tap every third frame, NO_BUTTON otherwise
      self._car()
      self._advance(2)
      if i % 3 == 0:
        presses += int(self._op(SET))
      else:
        self.assertTrue(self._op(NONE))
      self._advance(18)
    self.assertLessEqual(presses, 5)
    self.assertGreaterEqual(presses, 4)

  def test_switch_direction_needs_release(self):
    self._engage()
    self.assertTrue(all(self._tap(SET, frames=3)))
    self.assertFalse(self._tap(RES, frames=1)[0])

  def test_fwd_hold_back_only_after_press(self):
    car = common.make_msg(0, 0x20B, dat=ROGUE_IDLE_0x20B)
    self.assertEqual(self.safety.safety_fwd_packet_hook(car), 2)
    self._engage()
    self._car()
    # a NO_BUTTON copy is identical to the car frame and never holds the car stream back
    self.assertTrue(self._op(NONE))
    self._advance(10)
    self.assertEqual(self.safety.safety_fwd_packet_hook(car), 2)
    self.assertTrue(self._op(SET))
    self._advance(10)
    self.assertEqual(self.safety.safety_fwd_packet_hook(car), -1)
    # driver input is never held back
    for buttons in (0x01, 0x02, 0x04, SET, RES):
      self.assertEqual(self.safety.safety_fwd_packet_hook(common.make_msg(0, 0x20B, dat=_with_buttons(ROGUE_IDLE_0x20B, buttons))), 2)
    brake = bytearray(ROGUE_IDLE_0x20B)
    brake[2] &= ~0x40
    self.assertEqual(self.safety.safety_fwd_packet_hook(common.make_msg(0, 0x20B, dat=bytes(brake))), 2)
    # window lapses
    self._advance(60)
    self.assertEqual(self.safety.safety_fwd_packet_hook(car), 2)
    # other messages untouched
    for addr in (0x1B6, 0x1C3, 0x2D4, 0x30F, 0x2B1, 0x15A):
      for bus in (0, 1, 2):
        self.assertEqual(self.safety.safety_fwd_packet_hook(common.make_msg(bus, addr)), self.safety.safety_fwd_hook(bus, addr))

  def test_rejected_frame_does_not_open_window(self):
    car = common.make_msg(0, 0x20B, dat=ROGUE_IDLE_0x20B)
    self._car()
    self.assertFalse(self._op(SET))
    self.assertEqual(self.safety.safety_fwd_packet_hook(car), 2)

  def test_no_new_messages(self):
    self._engage()
    for addr in (0x1B6, 0x1C3, 0x2D4, 0x15A, 0x30F, 0x7DF, 0x71D, 0x71F, 0x18DA00F1):
      for bus in (0, 1, 2):
        self.assertFalse(self._tx(common.make_msg(bus, addr)), (hex(addr), bus))


class TestNissanIcbmFlagOff(TestNissanSafety):
  """Flag clear: ICBM-shaped frames must be blocked exactly as before."""

  def test_icbm_frames_blocked(self):
    for _ in range(6):
      self._rx(self._speed_msg(20.0))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self._rx(common.make_msg(0, 0x20B, dat=ROGUE_IDLE_0x20B))
    for buttons in (SET, RES, NONE):
      self.assertFalse(self._tx(common.make_msg(2, 0x20B, dat=_with_buttons(ROGUE_IDLE_0x20B, buttons))))
    self.assertTrue(self._tx(common.make_msg(2, 0x20B, dat=_with_buttons(ROGUE_IDLE_0x20B, CANCEL))))
    self.assertTrue(self._tx(common.make_msg(2, 0x20B, dat=_with_buttons(ROGUE_IDLE_0x20B, NONE))) is False)
    car = common.make_msg(0, 0x20B, dat=ROGUE_IDLE_0x20B)
    self.assertEqual(self.safety.safety_fwd_packet_hook(car), 2)


class TestNissanIcbmAltimaIgnored(unittest.TestCase):
  def test_altima_never_enables_icbm(self):
    safety = common.libsafety_py.libsafety
    try:
      safety.set_alternative_experience(0)
      self.assertEqual(safety.set_safety_hooks(CarParams.SafetyModel.nissan, NissanSafetyFlags.ICBM | NissanSafetyFlags.ALT_EPS_BUS), 0)
      safety.init_tests()
      safety.set_controls_allowed(True)
      safety.safety_rx_hook(common.make_msg(0, 0x20B, dat=ROGUE_IDLE_0x20B))
      for bus in (1, 2):
        self.assertFalse(safety.safety_tx_hook(common.make_msg(bus, 0x20B, dat=_with_buttons(ROGUE_IDLE_0x20B, SET))))
    finally:
      safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)


class TestNissanSafetyAltEpsBus(TestNissanSafety):
  """Altima uses different buses"""
  SAFETY_PARAM = NissanSafetyFlags.ALT_EPS_BUS

  EPS_BUS = 1
  CRUISE_BUS = 1

class TestNissanLeafSafety(TestNissanSafety):

  DBC = "nissan_leaf_2018_generated"

  def _user_brake_msg(self, brake):
    values = {"USER_BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_safety("CRUISE_THROTTLE", 0, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return self.packer.make_can_msg_safety("CRUISE_THROTTLE", 0, values)

  # TODO: leaf should use its own safety param
  def test_acc_buttons(self):
    pass


class TestNissanUnsupportedAolProfile(unittest.TestCase):
  def test_unsupported_words_deny_every_nissan_transmit(self):
    safety = common.libsafety_py.libsafety
    packer = common.CANPackerSafety("nissan_x_trail_2017_generated")
    cancel = packer.make_can_msg_safety("CRUISE_THROTTLE", 2, {"CANCEL_BUTTON": 1})
    hud = common.make_msg(0, 0x2B1)
    try:
      safety.set_alternative_experience(0)
      self.assertEqual(safety.set_safety_hooks(CarParams.SafetyModel.nissan, 0), 0)
      self.assertTrue(safety.safety_tx_hook(cancel))
      self.assertTrue(safety.safety_tx_hook(hud))
      for param in (2, 3, 5, 0xFFFF):
        with self.subTest(param=param):
          safety.set_alternative_experience(32)
          self.assertEqual(safety.set_safety_hooks(CarParams.SafetyModel.nissan, param), 0)
          safety.init_tests()
          safety.set_controls_allowed(True)
          safety.set_aol_test_heartbeat(True)
          safety.aol_set_host_request(3)
          self.assertEqual(safety.aol_get_permission_mask(), 0)
          self.assertFalse(safety.safety_tx_hook(cancel))
          self.assertFalse(safety.safety_tx_hook(hud))
          for addr, bus, length in ((0x169, 0, 8), (0x2B1, 0, 8), (0x4CC, 0, 8),
                                    (0x20B, 2, 6), (0x20B, 1, 6), (0x280, 2, 8)):
            self.assertFalse(safety.safety_tx_hook(common.make_msg(bus, addr, length)))
      # Leaving the unsupported profile restores the ordinary TX catalog.
      safety.set_alternative_experience(0)
      self.assertEqual(safety.set_safety_hooks(CarParams.SafetyModel.nissan, 0), 0)
      self.assertTrue(safety.safety_tx_hook(cancel))
      self.assertTrue(safety.safety_tx_hook(hud))
    finally:
      safety.set_alternative_experience(0)
      safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)


if __name__ == "__main__":
  unittest.main()
