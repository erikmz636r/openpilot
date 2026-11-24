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
# predicts along its path, the speed that keeps it at A_LAT_COMFORT, and where we must start
# slowing so stock ProPILOT gets there in time. A lowered set speed makes ProPILOT brake at about
# SET_BRAKE_DECEL at most, so we plan with PLAN_DECEL (a margin under it), and add the time the
# taps need before ProPILOT starts braking (BASE_REACTION_S plus one tap per mph the set speed is
# above actual speed: those taps do not slow the car yet).
A_LAT_COMFORT = 1.9        # m/s^2
SET_BRAKE_DECEL = 0.85     # m/s^2, most ProPILOT does for a lowered set speed (measured)
PLAN_DECEL = 0.6           # m/s^2, planned with margin under SET_BRAKE_DECEL
BASE_REACTION_S = 0.8      # first tap + ProPILOT starting to brake
TAPS_PER_S = 3.8           # opendbc icbm tap shape (5 + 8 car frames at 50 Hz)
REACTION_MAX_S = 3.0
REACTION_S = 1.5           # default when the set speed is not known
MIN_CURVATURE = 1e-4
MAX_LOOKAHEAD_S = 10.0     # the model's whole plan (modelV2 position.t runs to 10 s)
MIN_ACTIVE_SPEED = 8.0     # m/s; below this no curve target (city turns are the driver's)

# Smoothed target: fast down (react to a curve the moment the model shows it), slow up (anti-bounce).
TARGET_DOWN_MPH_S = 10.0
TARGET_DOWN_FAST_MPH_S = 15.0  # curve binding within FAST_WINDOW_S
FAST_WINDOW_S = 3.0
TARGET_UP_MPH_S = 2.0
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


def reaction_time(v_ego: float, set_mph: float) -> float:
  """Seconds before ProPILOT starts braking: first tap + response, plus a tap for every mph the set
  speed is above actual speed (those taps do not slow the car)."""
  if not set_mph or not math.isfinite(set_mph):
    return REACTION_S
  above = max(0.0, float(set_mph) - v_ego * MS_TO_MPH)
  return min(BASE_REACTION_S + above / TAPS_PER_S, REACTION_MAX_S)


class CurvePlanner:
  """Slew-limited curve target in display mph (None while there is no usable model).

  The curve math runs only when a new modelV2 arrives (20 Hz); every other frame only the cheap slew runs."""

  def __init__(self):
    self.target_mph: float | None = None
    self.last_ns = 0
    self.raw_ms = math.inf
    self.t_bind = math.inf

  def observe_model(self, v_ego: float, model, reaction_s: float = REACTION_S) -> None:
    # modelV2 has no top-level t: the time axis lives in each XYZTData (position.t).
    pos = model.position
    self.raw_ms, self.t_bind = curve_target_ms(v_ego, list(pos.t), list(pos.x), list(model.velocity.x),
                                               list(model.orientationRate.z), reaction_s=reaction_s)

  def update(self, v_ego: float, model, now_ns: int, model_ok: bool, model_new: bool = True,
             set_mph: float = 0.0) -> float | None:
    if not model_ok:
      self.target_mph = None
      self.last_ns = now_ns
      self.raw_ms, self.t_bind = math.inf, math.inf
      return None
    if model_new or self.target_mph is None:
      self.observe_model(v_ego, model, reaction_time(v_ego, set_mph))
    raw_ms = self.raw_ms if v_ego >= MIN_ACTIVE_SPEED else math.inf
    # Never let a straight-road value run far above actual speed, or the slew would need seconds
    # to come back down when a curve shows up.
    raw = min(raw_ms * MS_TO_MPH, v_ego * MS_TO_MPH + TARGET_ABOVE_SPEED_MPH)
    dt = max(0.0, min((now_ns - self.last_ns) * 1e-9, 0.1)) if self.last_ns else 0.0
    self.last_ns = now_ns
    if self.target_mph is None:
      # Start from actual speed so a fresh start never jumps the target down at once.
      self.target_mph = max(raw, v_ego * MS_TO_MPH)
    down = TARGET_DOWN_FAST_MPH_S if self.t_bind <= FAST_WINDOW_S else TARGET_DOWN_MPH_S
    cap = v_ego * MS_TO_MPH + TARGET_ABOVE_SPEED_MPH
    if raw >= cap - 0.01:
      # No curve limits us at all: "no limit" right away (a lagging target would look like a curve
      # while ProPILOT accelerates), and a slower car ahead never pulls the set speed down.
      self.target_mph = raw
      return math.inf
    if raw < self.target_mph:
      self.target_mph = max(raw, self.target_mph - down * dt)
    else:
      # Still in/near a curve: come back up slowly so model wobble does not bounce the set speed.
      self.target_mph = min(raw, self.target_mph + TARGET_UP_MPH_S * dt)
    if self.target_mph >= cap - 0.01:
      return math.inf
    return self.target_mph


def disable(card, where: str) -> None:
  """Any error: turn the feature off for this drive. Steering and sendcan must never stall for it."""
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.exception(f"rogue icbm disabled after error in {where}")
  except Exception:
    pass
  try:
    controller = getattr(card.CI.CC, "icbm", None)
    if controller is not None:
      controller.disable()
  except Exception:
    pass
  card.rogue_icbm = None


class IcbmRuntime:
  def __init__(self, mode: int, sm=None):
    if sm is None:
      from openpilot.cereal import messaging
      sm = messaging.SubMaster(['modelV2'])
    self.mode = mode
    self.sm = sm
    self.planner = CurvePlanner()

  def update(self, card, cs, cc, now_ns: int) -> None:
    try:
      controller = getattr(card.CI.CC, "icbm", None)
      if controller is None or controller.mode == ICBM_OFF:
        return
      self.sm.update(0)
      model_ok = bool(self.sm.seen['modelV2'] and self.sm.valid['modelV2'] and
                      0 <= int(now_ns) - int(self.sm.logMonoTime['modelV2']) <= MODEL_MAX_AGE_NS)
      model_new = bool(self.sm.updated['modelV2'])
      set_mph = float(getattr(controller.status, "set_mph", 0) or 0)
      target = self.planner.update(float(cs.vEgo), self.sm['modelV2'] if model_ok else None, int(now_ns), model_ok,
                                   model_new, set_mph)
      controller.set_target(target, int(now_ns))
    except Exception:
      disable(card, "runtime update")


def configure(card, services=None):
  """Called once from card at startup, before CarParams is published. Never raises."""
  try:
    return _configure(card)
  except Exception:
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception("rogue icbm configure failed; feature off")
    except Exception:
      pass
    try:
      card.CI.CC.icbm.disable()
    except Exception:
      pass
    try:
      card.CP.safetyConfigs[0].safetyParam &= ~0x100  # NissanSafetyFlags.ICBM
    except Exception:
      pass
    return None


def _configure(card):
  cp = card.CP
  if cp.carFingerprint != "NISSAN_ROGUE" or card.CI.CC is None or getattr(card.CI.CC, "icbm", None) is None:
    return None
  mode = read_mode(card.params)
  if mode == ICBM_OFF or cp.passive or cp.dashcamOnly or cp.openpilotLongitudinalControl or not cp.pcmCruise:
    return None
  runtime = IcbmRuntime(mode)  # build first: if this fails nothing below has changed
  if mode == ICBM_ACTIVE:
    from opendbc.car.nissan.values import NissanSafetyFlags
    from opendbc.car.structs import CarParams
    if len(cp.safetyConfigs) != 1 or cp.safetyConfigs[0].safetyModel != CarParams.SafetyModel.nissan:
      mode = ICBM_WATCH
    else:
      cp.safetyConfigs[0].safetyParam |= NissanSafetyFlags.ICBM.value
  card.CI.CC.icbm.mode = mode
  card.CI.CC.icbm.status.mode = mode
  runtime.mode = mode
  return runtime


def fill_rogue_icbm(state, status) -> None:
  def finite(x):
    x = float(x)
    return x if math.isfinite(x) else 0.0
  msg = state.rogueIcbm
  msg.mode = int(status.mode) & 0xFF
  msg.state = int(status.state) & 0xFF
  msg.setMph = max(0, min(int(status.set_mph), 255))
  msg.carSetMph = max(0, min(int(status.car_set_mph), 255))
  msg.ceilingMph = max(0, min(int(status.ceiling_mph), 255))
  msg.desiredMph = finite(status.desired_mph)
  msg.targetMph = finite(status.target_mph)
  msg.hasTarget = bool(status.has_target)
  msg.direction = max(-1, min(int(status.direction), 1))
  msg.tapsRequested = int(status.taps_requested) & 0xFFFFFFFF
  msg.tapsConfirmed = int(status.taps_confirmed) & 0xFFFFFFFF
  msg.misses = int(status.misses) & 0xFF
  msg.faultReason = str(status.fault_reason)[:120]
  msg.alert = bool(status.alert)
  msg.sending = bool(status.sending)
