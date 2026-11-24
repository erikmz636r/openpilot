"""Extra set-speed targets for the Rogue cruise-button control (card side), all opt-in, Active only.

Each planner returns a target in the car's own speed scale (display mph, same as the set speed) or
inf for "no limit from me". The runtime takes min() with the curve target; the opendbc controller
still owns every tap and every rule (never above the driver's speed, never below 20 mph, yields to
any pedal/button, verifies each tap). None of these can stop the car or brake harder than stock
ProPILOT does for a lowered set speed; stock radar following and AEB stay in charge.

Speeds: v_true = carState.vEgo * speed_scale (SpeedScale below). The model and the set speed are on
that scale; vEgo on this car reads ~12% low.
"""
import math

MS_TO_MPH = 2.2369362920544
PLAN_DECEL = 0.6
BASE_REACTION_S = 0.8
UP_MPH_S = 2.0
DOWN_MPH_S = 10.0


# ---------------------------------------------------------------- speed scale
SCALE_MIN, SCALE_MAX = 1.0, 1.2
SCALE_MIN_SPEED = 8.0          # m/s vEgo
SCALE_TAU_S = 20.0
SCALE_WARMUP_S = 10.0          # samples needed before the learned scale is used


class SpeedScale:
  """Learns car-scale speed / vEgo from the model's own speed estimate (modelV2.velocity.x[0]).
  Logs: 1.13-1.15 on every drive, matching ProPILOT holding vEgo at ~0.877 x its set speed."""

  def __init__(self):
    self.ratio = 1.0
    self.samples_s = 0.0

  @property
  def scale(self) -> float:
    return min(max(self.ratio, SCALE_MIN), SCALE_MAX) if self.samples_s >= SCALE_WARMUP_S else 1.0

  def update(self, v_ego: float, model_v0: float, dt: float) -> float:
    if v_ego >= SCALE_MIN_SPEED and math.isfinite(model_v0) and 0.85 <= model_v0 / v_ego <= 1.35 and 0 < dt <= 0.2:
      self.ratio += (model_v0 / v_ego - self.ratio) * min(dt / SCALE_TAU_S, 1.0)
      self.samples_s += dt
    return self.scale


def _slew(cur: float, raw: float, dt: float, cap: float) -> float:
  """Fast down / slow up; inf (no target) once at or above cap."""
  if not math.isfinite(raw) and not math.isfinite(cur):
    return math.inf
  if not math.isfinite(cur):
    cur = cap
  if raw < cur:
    cur = max(raw, cur - DOWN_MPH_S * dt)
  else:
    cur = min(raw, cur + UP_MPH_S * dt)
  return math.inf if cur >= cap - 0.01 else cur


# ---------------------------------------------------------------- posted speed limit
LIMIT_FRESH_NS = 1_000_000_000
LIMIT_HOLD_S = 4.0             # vision readings flicker valid/stale about once a second
LIMIT_CONFIRM_S = 1.0          # a new value must be read for this long (no other value in between)
LIMIT_MIN_MPH, LIMIT_MAX_MPH = 15, 85
LIMIT_SOURCES = ("vision", "map", "dashboard", "online")


class LimitPlanner:
  """Posted speed limit from StarPilot's slcState (only 'valid' observations, fresh, a plausible
  US limit, confirmed for a second). Unknown / stale for LIMIT_HOLD_S -> no target."""

  def __init__(self, offset_mph: int = 0):
    self.offset = int(offset_mph)
    self.limit = 0
    self.last_valid_ns = 0
    self.cand = 0
    self.cand_since = 0
    self.source = ""
    self.target_mph = math.inf
    self.last_ns = 0
    self.overridden = 0           # limit value the driver overrode (RES/SET up); ignored until it changes

  def override(self) -> None:
    if self.limit:
      self.overridden = self.limit
      self.target_mph = math.inf

  def update(self, slc, slc_ok: bool, now: int, v: float) -> float:
    dt = max(0.0, min((now - self.last_ns) * 1e-9, 0.1)) if self.last_ns else 0.0
    self.last_ns = now
    if slc_ok and slc is not None and str(slc.observationKind) == "valid" and str(slc.source) in LIMIT_SOURCES:
      lim_f = float(slc.speedLimit) * MS_TO_MPH
      lim = int(round(lim_f)) if math.isfinite(lim_f) else 0
      if LIMIT_MIN_MPH <= lim <= LIMIT_MAX_MPH and lim % 5 == 0 and abs(lim_f - lim) < 0.3:
        if lim != self.cand:
          self.cand, self.cand_since = lim, now
        if lim == self.limit:
          self.last_valid_ns = now
        elif now - self.cand_since >= LIMIT_CONFIRM_S * 1e9:
          self.limit, self.last_valid_ns, self.source = lim, now, str(slc.source)
    if self.limit and now - self.last_valid_ns > LIMIT_HOLD_S * 1e9:
      self.limit = 0
    if self.overridden and self.limit and self.limit != self.overridden:
      self.overridden = 0         # a new posted limit: act again
    if self.overridden and self.limit == self.overridden:
      self.target_mph = math.inf
      return math.inf
    raw = float(self.limit + self.offset) if self.limit else math.inf
    cap = v * MS_TO_MPH + 5.0
    if not math.isfinite(raw) and math.isfinite(self.target_mph):
      raw = cap  # limit gone: climb back slowly
    if math.isfinite(raw) and raw >= cap - 0.01 and not math.isfinite(self.target_mph):
      return math.inf
    self.target_mph = _slew(self.target_mph, raw, dt, cap)
    return self.target_mph


# ---------------------------------------------------------------- red light / stop ahead
STOP_MIN_SPEED = 9.0           # m/s (20 mph): below that the 20 mph floor makes this moot
STOP_TIME_S = 7.7              # StarPilot CEM model-stop time: horizon shorter than v * this
STOP_END_RATIO = 0.35          # model plans to end below 35% of its current speed ...
STOP_END_MS = 4.5              # ... or below this
STOP_MAX_YAW = 0.3             # rad over the plan: more is a turn/curve (curve target owns it)
STOP_HOLD_S = 1.0
STOP_RELEASE_S = 1.5
STOP_LEAD_MARGIN_M = 15.0


class StopPlanner:
  """Lead-less model stop (red light / stop sign ahead): the driving model plans to come to (near) a
  stop within its horizon with no car in front. Lowers the set speed early so ProPILOT eases off;
  it cannot stop the car (20 mph floor) - the driver brakes."""

  def __init__(self):
    self.active = False
    self.cond_since = 0
    self.clear_since = 0
    self.target_mph = math.inf
    self.suppressed = False       # driver override: wait until this stop has cleared

  def override(self, now: int) -> None:
    if self.active or self.cond_since:
      self.suppressed = True
    self.active, self.target_mph, self.cond_since = False, math.inf, 0

  def update(self, v: float, model, lead, now: int, ok: bool) -> float:
    cond = False
    d_stop = math.inf
    if ok and model is not None and v >= STOP_MIN_SPEED:
      vx = list(model.velocity.x)
      px = list(model.position.x)
      yaw = list(model.orientation.z)
      if len(vx) >= 2 and len(px) == len(vx) and len(yaw) == len(vx):
        v0, vend, xend = float(vx[0]), float(vx[-1]), float(px[-1])
        i = next((k for k, s in enumerate(vx) if s < 1.5), len(vx) - 1)
        d_stop = float(px[i])
        lead_near = (lead is not None and bool(lead.present) and float(lead.modelProb) > 0.5 and
                     float(lead.dRel) < xend + STOP_LEAD_MARGIN_M)
        cond = (math.isfinite(v0) and v0 > 1.0 and xend < STOP_TIME_S * v0 and vend <= max(STOP_END_MS, STOP_END_RATIO * v0)
                and abs(float(yaw[-1])) < STOP_MAX_YAW and not lead_near)
    if self.suppressed:
      if cond:
        self.clear_since = 0
        return math.inf
      self.clear_since = self.clear_since or now
      if now - self.clear_since < STOP_RELEASE_S * 1e9:
        return math.inf
      self.suppressed, self.clear_since = False, 0
    if cond:
      self.clear_since = 0
      self.cond_since = self.cond_since or now
      if now - self.cond_since >= STOP_HOLD_S * 1e9:
        self.active = True
    else:
      self.cond_since = 0
      if self.active:
        self.clear_since = self.clear_since or now
        if not ok or now - self.clear_since >= STOP_RELEASE_S * 1e9:
          self.active, self.target_mph = False, math.inf
    if self.active and cond:
      raw = math.sqrt(2.0 * PLAN_DECEL * max(d_stop - v * BASE_REACTION_S - 5.0, 0.0)) * MS_TO_MPH
      self.target_mph = min(self.target_mph, max(raw, self.target_mph - DOWN_MPH_S * 0.05)) if math.isfinite(self.target_mph) \
        else max(raw, v * MS_TO_MPH - 2.0)
    return self.target_mph if self.active else math.inf
