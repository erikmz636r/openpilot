"""Rogue cruise-button control runtime (card side).

Reads the NissanRogueIcbmMode setting once at car start, sets the panda opt-in flag only for Active,
and every controls frame hands the opendbc controller (opendbc/car/nissan/icbm.py) a curve speed
target worked out from the driving model. The opendbc controller decides the taps; panda safety
checks every frame again. Off -> nothing here runs and nothing changes.
"""
import math

import numpy as np

from openpilot.starpilot.car.nissan.icbm_settings import ICBM_ACTIVE, ICBM_OFF, ICBM_WATCH, read_mode

MS_TO_MPH = 2.2369362920544

# Curve speed (idea after sunnypilot's vision curve controller, MIT): lateral accel the model
# predicts along its path, the speed that keeps it at A_LAT_COMFORT, and how fast we must start
# slowing so stock ProPILOT (about -0.85 m/s^2 for a lowered set speed) gets there in time.
A_LAT_COMFORT = 1.9        # m/s^2
PLAN_DECEL = 0.7           # m/s^2, a bit under what ProPILOT does per lowered mph
REACTION_S = 1.5           # taps + ProPILOT response
MIN_CURVATURE = 1e-4
MAX_LOOKAHEAD_S = 8.0
MIN_ACTIVE_SPEED = 8.0     # m/s; below this no curve target (city turns are the driver's)

TARGET_DOWN_MPH_S = 2.0
TARGET_DOWN_FAST_MPH_S = 4.0  # curve binding within FAST_WINDOW_S
FAST_WINDOW_S = 3.0
TARGET_ABOVE_SPEED_MPH = 5.0
MODEL_MAX_AGE_NS = 250_000_000


def curve_target_ms(v_ego: float, t: list[float], pos_x: list[float], vel_x: list[float], yaw_rate: list[float],
                    a_lat: float = A_LAT_COMFORT, decel: float = PLAN_DECEL, reaction_s: float = REACTION_S) -> tuple[float, float]:
  """Highest speed now that still lets ProPILOT slow to a comfortable curve speed in time.
  Returns (speed m/s, time s to the binding point). inf if no curve limits us."""
  n = min(len(t), len(pos_x), len(vel_x), len(yaw_rate))
  if n == 0:
    return math.inf, math.inf
  t_a = np.asarray(t[:n], dtype=float)
  x = np.asarray(pos_x[:n], dtype=float)
  v = np.maximum(np.asarray(vel_x[:n], dtype=float), 1.0)
  curv = np.abs(np.asarray(yaw_rate[:n], dtype=float)) / v
  keep = (t_a <= MAX_LOOKAHEAD_S) & np.isfinite(curv) & np.isfinite(x)
  if not keep.any():
    return math.inf, math.inf
  v_curve = np.sqrt(a_lat / np.maximum(curv[keep], MIN_CURVATURE))
  dist = np.maximum(x[keep] - max(v_ego, 0.0) * reaction_s, 0.0)
  v_now = np.sqrt(v_curve ** 2 + 2.0 * decel * dist)
  i = int(np.argmin(v_now))
  return float(v_now[i]), float(t_a[keep][i])


class CurvePlanner:
  """Slew-limited curve target in display mph (None while there is no usable model)."""

  def __init__(self):
    self.target_mph: float | None = None
    self.last_ns = 0

  def update(self, v_ego: float, model, now_ns: int, model_ok: bool) -> float | None:
    if not model_ok:
      self.target_mph = None
      self.last_ns = now_ns
      return None
    raw_ms, t_bind = curve_target_ms(v_ego, list(model.t), list(model.position.x), list(model.velocity.x),
                                     list(model.orientationRate.z))
    if v_ego < MIN_ACTIVE_SPEED:
      raw_ms = math.inf
    # Never let a straight-road value run far above actual speed, or the slew would need seconds
    # to come back down when a curve shows up.
    raw = min(raw_ms * MS_TO_MPH, v_ego * MS_TO_MPH + TARGET_ABOVE_SPEED_MPH)
    dt = max(0.0, min((now_ns - self.last_ns) * 1e-9, 0.1)) if self.last_ns else 0.0
    self.last_ns = now_ns
    if self.target_mph is None:
      # Start from actual speed so a fresh start never jumps the target down at once.
      self.target_mph = max(raw, v_ego * MS_TO_MPH)
    down = TARGET_DOWN_FAST_MPH_S if t_bind <= FAST_WINDOW_S else TARGET_DOWN_MPH_S
    if raw < self.target_mph:
      self.target_mph = max(raw, self.target_mph - down * dt)
    else:
      # Up is immediate here: the controller already returns gently (1.5 s wait, <= actual speed + 3 mph,
      # one mph per tap). A lagging target would look like a curve while ProPILOT accelerates.
      self.target_mph = raw
    # At the cap there is no curve limiting us: report "no limit" so a slower car ahead (ProPILOT
    # following a lead) never pulls the set speed down.
    if self.target_mph >= v_ego * MS_TO_MPH + TARGET_ABOVE_SPEED_MPH - 0.01:
      return math.inf
    return self.target_mph


class IcbmRuntime:
  def __init__(self, mode: int):
    from openpilot.cereal import messaging
    self.mode = mode
    self.sm = messaging.SubMaster(['modelV2'])
    self.planner = CurvePlanner()

  def update(self, card, cs, cc, now_ns: int) -> None:
    controller = getattr(card.CI.CC, "icbm", None)
    if controller is None:
      return
    self.sm.update(0)
    model_ok = (self.sm.seen['modelV2'] and self.sm.valid['modelV2'] and
                0 <= now_ns - int(self.sm.logMonoTime['modelV2']) <= MODEL_MAX_AGE_NS)
    target = self.planner.update(float(cs.vEgo), self.sm['modelV2'], now_ns, model_ok)
    controller.set_target(target, now_ns)


def configure(card, services=None):
  """Called once from card at startup, before CarParams is published."""
  cp = card.CP
  if cp.carFingerprint != "NISSAN_ROGUE" or card.CI.CC is None or getattr(card.CI.CC, "icbm", None) is None:
    return None
  mode = read_mode(card.params)
  if mode == ICBM_OFF or cp.passive or cp.dashcamOnly or cp.openpilotLongitudinalControl or not cp.pcmCruise:
    return None
  if mode == ICBM_ACTIVE:
    from opendbc.car.nissan.values import NissanSafetyFlags
    from opendbc.car.structs import CarParams
    if len(cp.safetyConfigs) != 1 or cp.safetyConfigs[0].safetyModel != CarParams.SafetyModel.nissan:
      mode = ICBM_WATCH
    else:
      cp.safetyConfigs[0].safetyParam |= NissanSafetyFlags.ICBM.value
  card.CI.CC.icbm.mode = mode
  card.CI.CC.icbm.status.mode = mode
  return IcbmRuntime(mode)


def fill_rogue_icbm(state, status) -> None:
  msg = state.rogueIcbm
  msg.mode = int(status.mode) & 0xFF
  msg.state = int(status.state) & 0xFF
  msg.setMph = max(0, min(int(status.set_mph), 255))
  msg.carSetMph = max(0, min(int(status.car_set_mph), 255))
  msg.ceilingMph = max(0, min(int(status.ceiling_mph), 255))
  msg.desiredMph = float(status.desired_mph)
  msg.targetMph = float(status.target_mph)
  msg.hasTarget = bool(status.has_target)
  msg.direction = int(status.direction)
  msg.tapsRequested = int(status.taps_requested) & 0xFFFFFFFF
  msg.tapsConfirmed = int(status.taps_confirmed) & 0xFFFFFFFF
  msg.misses = int(status.misses) & 0xFF
  msg.faultReason = str(status.fault_reason)
  msg.alert = bool(status.alert)
  msg.sending = bool(status.sending)
