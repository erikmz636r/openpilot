"""Rogue ProPILOT cruise-button management (ICBM).

openpilot never commands gas or brake on this car. In Active mode it nudges the stock ProPILOT set
speed with SET- / RES+ taps (one mph per tap) so ProPILOT starts easing off before curves, then
taps back up to the speed the driver chose. ProPILOT keeps doing all gas, braking, lead following
and stops.

Modes (param NissanRogueIcbm): 0 Off, 1 Watch only (decides and logs taps, sends nothing new),
2 Active (sends taps; needs the NISSAN_PARAM_ICBM panda flag, set at startup).

Rules that are enforced here (and again, independently, by panda safety in Active mode):
* never above the driver's own set speed (the "ceiling"), never below 20 mph
* only while openpilot and ProPILOT are engaged, moving above 3.5 m/s, no gas, no brake
* any driver cruise button / gas / engage edge -> stop and yield, then re-read the car's set speed
* every tap must move the car's set speed by exactly one in the right direction; 3 misses in a row
  or any wrong/bigger change -> latched fault + alert until ProPILOT disengages
* the car's set speed is re-read every frame (0x2B1 on the ADAS bus, display = raw - 1)

Pure python; no CAN I/O here. carcontroller packs the frames this returns.
"""
import math
from dataclasses import dataclass, field
from enum import IntEnum

MODE_OFF = 0
MODE_WATCH = 1
MODE_ACTIVE = 2

MS_TO_MPH = 2.2369362920544

# Tap shape in car CRUISE_THROTTLE frames (50 Hz), one frame sent per new car frame (same counter).
# Panda: press <= 200 ms, >= 100 ms release, <= 5 presses/s. 5 + 8 frames -> at most ~3.8 taps/s.
# The release frames are byte-identical copies of the car's own NO_BUTTON frames; they bridge the
# panda's 60 ms hold-back window after the last SET/RES frame so the ADAS module sees no gap.
PRESS_FRAMES = 5
RELEASE_FRAMES = 8

MIN_SET_MPH = 20               # ProPILOT minimum (CAN 21)
MIN_SPEED_MS = 3.5             # panda allows > 3.0
CEILING_MAX_MPH = 90
VERIFY_TIMEOUT_NS = 400_000_000  # car answered within 48-80 ms in every logged tap
MAX_MISSES = 3
YIELD_BUTTON_NS = 2_000_000_000
YIELD_ENGAGE_NS = 1_000_000_000
YIELD_GAS_NS = 1_000_000_000
CAR_FRAME_MAX_AGE_NS = 100_000_000
SET_FRAME_MAX_AGE_NS = 300_000_000
TARGET_MAX_AGE_NS = 500_000_000
UNEXPECTED_CHANGE_NS = 200_000_000
ALERT_NS = 6_000_000_000

# Deadbands (display mph) so the set speed does not bounce around the target.
DOWN_ERR_MPH = 1.5
DOWN_HOLD_NS = 250_000_000     # was 500 ms: start slowing for a curve sooner
UP_ERR_MPH = 2.0
UP_HOLD_NS = 1_500_000_000
# While a curve is still ahead/under us, going back up needs a clearly higher target for longer
# (anti-bounce: the model's curve estimate wobbles a couple of mph inside a curve).
IN_CURVE_UP_ERR_MPH = 5.0        # was 3.0: a 4.6 mph wobble still bounced in a highway curve (seg 23 sim)
IN_CURVE_UP_HOLD_NS = 4_000_000_000  # was 2.5 s
IN_CURVE_GRACE_NS = 2_500_000_000    # target gone this briefly between bends: still in the curve
# Set speed above actual speed: taps there do not slow the car yet, so do not wait on the deadband.
ABOVE_SPEED_MPH = 1.0
UP_OVER_SPEED_MPH = 3.0        # going back up, stay within this of actual speed
UP_FOLLOW_SPEED_NS = 6_000_000_000
OVERRIDE_TIGHTER_MPH = 3.0     # after a driver change mid-curve, only a curve this much tighter acts

BUTTON_SIGNALS = ("PROPILOT_BUTTON", "CANCEL_BUTTON", "FOLLOW_DISTANCE_BUTTON", "SET_BUTTON", "RES_BUTTON")
ALL_SIGNALS = ("COUNTER", "PROPILOT_BUTTON", "CANCEL_BUTTON", "GAS_PEDAL_INVERTED", "SET_BUTTON", "RES_BUTTON",
               "FOLLOW_DISTANCE_BUTTON", "NO_BUTTON_PRESSED", "GAS_PEDAL", "USER_BRAKE_PRESSED",
               "USER_BRAKE_PRESSED_INVERTED", "NEW_SIGNAL_2", "GAS_PRESSED_INVERTED", "unsure1", "unsure2", "unsure3")


class Phase(IntEnum):
  IDLE = 0
  PRESS = 2
  RELEASE = 3


class State(IntEnum):
  OFF = 0
  STANDBY = 1      # not engaged / not ready
  YIELD = 2        # driver just acted; following the car's set speed
  HOLDING = 3
  DECREASING = 4
  INCREASING = 5
  FAULT = 6


@dataclass
class CarInputs:
  """One controls frame of inputs, all read from the car this frame."""
  now_ns: int
  op_enabled: bool              # openpilot engaged (CC.enabled)
  cruise_enabled: bool          # 0x30F CRUISE_ENABLED
  propilot_engaged: bool        # 0x1B6 byte 4 bit 6
  v_ego: float
  standstill: bool
  gas_pressed: bool
  brake_pressed: bool
  throttle: dict                # latest car CRUISE_THROTTLE (bus 0) signals
  throttle_ns: int              # receive time of that frame (0 = never)
  set_raw: int                  # PROPILOT_HUD SET_SPEED raw (display + 1); 0 off, 255 unset
  set_ns: int
  speed_mph_units: bool
  pcm_cancel: bool = False


@dataclass
class Status:
  mode: int = MODE_OFF
  state: State = State.OFF
  set_mph: int = 0              # car display set speed (Watch: simulated)
  car_set_mph: int = 0
  ceiling_mph: int = 0
  desired_mph: float = 0.0
  target_mph: float = 0.0
  has_target: bool = False
  direction: int = 0            # last tap direction (+1 RES, -1 SET)
  taps_requested: int = 0
  taps_confirmed: int = 0
  misses: int = 0
  fault_reason: str = ""
  alert: bool = False
  sending: bool = False
  speed_scale: float = 1.0      # car/dash speed per openpilot vEgo (set by the runtime)
  curve_target_mph: float = 0.0  # per-source targets, logging only (0 = none)
  limit_target_mph: float = 0.0
  stop_target_mph: float = 0.0
  limit_mph: int = 0
  features: int = 0             # 2 speed limit, 4 stops (bit 1 reserved)


def throttle_driver_input(t: dict) -> bool:
  return (any(bool(t.get(s, 0)) for s in BUTTON_SIGNALS) or t.get("GAS_PEDAL", 0) > 3 or
          not bool(t.get("USER_BRAKE_PRESSED_INVERTED", 1)))


def throttle_buttons(t: dict) -> bool:
  return any(bool(t.get(s, 0)) for s in BUTTON_SIGNALS)


def make_throttle_values(t: dict, button: str) -> dict:
  """Exact copy of the car frame with only the button bits replaced."""
  values = {s: t[s] for s in ALL_SIGNALS}
  for s in BUTTON_SIGNALS:
    values[s] = 0
  values["NO_BUTTON_PRESSED"] = 0
  values[button] = 1
  return values


@dataclass
class _Tap:
  direction: int
  set_before: int
  start_ns: int = 0
  started: bool = False


@dataclass
class IcbmController:
  mode: int = MODE_OFF
  status: Status = field(default_factory=Status)

  def __post_init__(self):
    self.status.mode = self.mode
    self.phase = Phase.IDLE
    self.phase_frames = 0
    self.press_dir = 0
    self.last_throttle_ns = 0
    self.tap: _Tap | None = None
    self.pending_tap = 0
    self.engaged_prev = False
    self.gas_prev = False
    self.yield_until_ns = 0
    self.ceiling = 0
    self.expected_set = 0
    self.virtual_set = 0
    self.last_car_set = 0
    self.unexpected_since_ns = 0
    self.err_since_ns = 0
    self.err_dir = 0
    self.misses = 0
    self.fault = ""
    self.alert_until_ns = 0
    self.target_mph: float | None = None
    self.target_ns = 0
    self.after_yield = False
    self.button_yield = False
    self.override_target: float | None = None
    self.last_curve_ns = 0
    # vEgo (wheel speeds) reads ~12% under the car's own speed scale on the Rogue (set speed, cluster,
    # model). Every comparison of actual speed with a set speed uses v_ego * speed_scale.
    self.speed_scale = 1.0

  def set_speed_scale(self, scale: float) -> None:
    s = float(scale)
    self.speed_scale = s if math.isfinite(s) and 1.0 <= s <= 1.25 else 1.0
    self.status.speed_scale = self.speed_scale

  def disable(self) -> None:
    """Off for the rest of this drive (used after any unexpected error). Sends nothing afterward."""
    self.mode = MODE_OFF
    self.phase = Phase.IDLE
    self.phase_frames = 0
    self.tap = None
    self.pending_tap = 0
    self.status.mode = MODE_OFF
    self.status.state = State.OFF
    self.status.sending = False

  # ---- inputs from openpilot (planner side) ----
  def set_target(self, target_mph: float | None, now_ns: int) -> None:
    self.target_mph = target_mph
    self.target_ns = now_ns

  # ---- helpers ----
  def _engaged(self, c: CarInputs) -> bool:
    return c.op_enabled and c.cruise_enabled and c.propilot_engaged

  def _set_valid(self, c: CarInputs) -> bool:
    return (MIN_SET_MPH + 1 <= c.set_raw <= CEILING_MAX_MPH + 1 and c.speed_mph_units and
            0 < c.set_ns and c.now_ns - c.set_ns <= SET_FRAME_MAX_AGE_NS)

  def _reset_cycle(self) -> None:
    self.tap = None
    self.pending_tap = 0
    self.err_since_ns = 0
    self.err_dir = 0
    self.unexpected_since_ns = 0

  def _raise_fault(self, reason: str, now_ns: int) -> None:
    if not self.fault:
      self.fault = reason
      self.alert_until_ns = now_ns + ALERT_NS
    self._reset_cycle()

  # ---- main step, called every controls frame (100 Hz) ----
  def update(self, c: CarInputs) -> str | None:
    """Returns the CRUISE_THROTTLE button to send this frame ("SET_BUTTON", "RES_BUTTON",
    "NO_BUTTON_PRESSED") or None for nothing. Never returns anything in Off / Watch mode."""
    st = self.status
    st.mode = self.mode
    st.alert = c.now_ns < self.alert_until_ns
    if self.mode == MODE_OFF:
      st.state = State.OFF
      return None

    engaged = self._engaged(c)
    car_set = c.set_raw - 1 if self._set_valid(c) else 0
    st.car_set_mph = car_set
    t = c.throttle
    new_frame = c.throttle_ns != self.last_throttle_ns and c.throttle_ns > 0
    if new_frame:
      self.last_throttle_ns = c.throttle_ns
    throttle_fresh = c.throttle_ns > 0 and c.now_ns - c.throttle_ns <= CAR_FRAME_MAX_AGE_NS

    # A fault holds until ProPILOT disengages.
    if not engaged:
      self.fault = ""
    if engaged and not self.engaged_prev:
      self.yield_until_ns = max(self.yield_until_ns, c.now_ns + YIELD_ENGAGE_NS)
    if throttle_fresh and throttle_buttons(t):
      self.yield_until_ns = max(self.yield_until_ns, c.now_ns + YIELD_BUTTON_NS)
      self.button_yield = True
    if self.gas_prev and not c.gas_pressed:
      self.yield_until_ns = max(self.yield_until_ns, c.now_ns + YIELD_GAS_NS)
    self.engaged_prev = engaged
    self.gas_prev = c.gas_pressed

    ready = (engaged and car_set > 0 and throttle_fresh and c.v_ego > MIN_SPEED_MS and not c.standstill and
             not c.gas_pressed and not c.brake_pressed and not c.pcm_cancel and not throttle_driver_input(t))
    yielding = c.now_ns < self.yield_until_ns or c.gas_pressed

    if not engaged or car_set == 0:
      st.state = State.STANDBY
      self._reset_cycle()
      self.ceiling = 0
      self.override_target = None
      self.button_yield = False
      self.after_yield = False
      self.virtual_set = car_set
      self.last_car_set = car_set
    elif self.fault:
      st.state = State.FAULT
    elif yielding or self.ceiling == 0:
      # Driver owns the set speed: follow it, never fight it.
      st.state = State.YIELD
      self.after_yield = self.button_yield
      self._reset_cycle()
      self.ceiling = car_set
      self.expected_set = car_set
      self.virtual_set = car_set
    else:
      self._step_engaged(c, car_set, ready)

    self.last_car_set = car_set
    st.fault_reason = self.fault
    st.ceiling_mph = self.ceiling
    st.set_mph = self.virtual_set if self.mode == MODE_WATCH else car_set
    st.misses = self.misses

    return self._frame(c, new_frame, ready and not self.fault and st.state in (State.DECREASING, State.INCREASING, State.HOLDING))

  def _current_set(self, car_set: int) -> int:
    return self.virtual_set if self.mode == MODE_WATCH else car_set

  def _step_engaged(self, c: CarInputs, car_set: int, ready: bool) -> None:
    st = self.status
    now = c.now_ns

    # Set-speed feedback (Active: real car, Watch: simulated).
    if self.mode == MODE_ACTIVE:
      if self.tap is not None and self.tap.started:
        delta = car_set - self.tap.set_before
        if delta == self.tap.direction:
          st.taps_confirmed += 1
          self.misses = 0
          self.expected_set = car_set
          self.tap = None
        elif delta != 0:
          self._raise_fault(f"set speed jumped {delta:+d} on a {self.tap.direction:+d} tap", now)
          st.state = State.FAULT
          return
        elif now - self.tap.start_ns > VERIFY_TIMEOUT_NS:
          self.misses += 1
          self.tap = None
          if self.misses >= MAX_MISSES:
            self._raise_fault("set speed not responding to taps", now)
            st.state = State.FAULT
            return
      elif car_set != self.expected_set:
        # Change we did not cause and no driver button was seen: stop.
        if self.unexpected_since_ns == 0:
          self.unexpected_since_ns = now
        elif now - self.unexpected_since_ns >= UNEXPECTED_CHANGE_NS:
          self._raise_fault(f"unexpected set speed change {self.expected_set}->{car_set}", now)
          st.state = State.FAULT
          return
      else:
        self.unexpected_since_ns = 0
    else:
      # Watch: the car's set speed never moves for us; follow the driver if it changes.
      if car_set != self.last_car_set:
        self.virtual_set = car_set
        self.ceiling = car_set
        self.tap = None
      if self.tap is not None and self.tap.started and now - self.tap.start_ns > 60_000_000:
        self.virtual_set += self.tap.direction
        st.taps_confirmed += 1
        self.tap = None

    current = self._current_set(car_set)

    # Desired set speed.
    target_fresh = self.target_mph is not None and 0 <= now - self.target_ns <= TARGET_MAX_AGE_NS
    st.has_target = target_fresh and math.isfinite(self.target_mph)  # inf = no curve ahead
    st.target_mph = float(self.target_mph) if st.has_target else 0.0
    if self.after_yield:
      # The driver just chose this speed. If we are already in a curve, respect it: do not take it
      # back down for this curve. Only a clearly tighter curve (3 mph lower target) or the end of the
      # curve clears this.
      self.after_yield = False
      self.button_yield = False
      self.override_target = float(self.target_mph) if target_fresh and self.target_mph < self.ceiling - 0.5 else None
    if self.override_target is not None and target_fresh and (
        self.target_mph >= self.ceiling - 1.0 or self.target_mph < self.override_target - OVERRIDE_TIGHTER_MPH):
      self.override_target = None
    if not target_fresh:
      desired = float(current)  # no fresh plan: hold where we are
    elif self.override_target is not None:
      desired = float(min(current, self.ceiling))
    else:
      desired = min(float(self.ceiling), max(float(MIN_SET_MPH), float(self.target_mph)))
      if desired < self.ceiling:
        self.last_curve_ns = now
      # Right after a curve, climb back no faster than the car actually speeds up. Once the road has
      # been clear for a while (e.g. ProPILOT is held back by a car ahead) go back to the driver's speed.
      if desired > current and now - self.last_curve_ns < UP_FOLLOW_SPEED_NS:
        desired = min(desired, max(float(current), c.v_ego * self.speed_scale * MS_TO_MPH + UP_OVER_SPEED_MPH))
    st.desired_mph = desired
    goal = int(round(desired))
    err = desired - current

    # Direction with deadbands; keep going until the rounded goal is reached.
    direction = 0
    if st.state == State.DECREASING and current > goal and current > MIN_SET_MPH:
      direction = -1
    elif st.state == State.INCREASING and current < goal and current < self.ceiling:
      direction = 1
    else:
      # Going back up to the driver's own speed finishes the last mph too.
      # "In a curve" = a curve target is what holds us below the driver's speed right now.
      # A curve target that drops out for a moment (winding road) still counts for IN_CURVE_GRACE_NS.
      curve_now = st.has_target and float(self.target_mph) < self.ceiling
      curve_recent = self.last_curve_ns > 0 and 0 < now - self.last_curve_ns < IN_CURVE_GRACE_NS
      in_curve = self.override_target is None and (curve_now or curve_recent)
      up_err = 1.0 if goal >= self.ceiling else UP_ERR_MPH
      up_hold = UP_HOLD_NS
      if in_curve and goal < self.ceiling:
        up_err, up_hold = max(up_err, IN_CURVE_UP_ERR_MPH), IN_CURVE_UP_HOLD_NS
      # A curve target below a set speed that is above actual speed: those first taps do not slow
      # the car at all yet, so start right away (still one mph per tap, still the deadband to stop).
      down_now = in_curve and err <= -DOWN_ERR_MPH and current >= c.v_ego * self.speed_scale * MS_TO_MPH + ABOVE_SPEED_MPH
      want = -1 if err <= -DOWN_ERR_MPH else (1 if err >= up_err else 0)
      if want != 0 and want == self.err_dir:
        hold = DOWN_HOLD_NS if want < 0 else up_hold
        if now - self.err_since_ns >= hold or (want < 0 and down_now):
          direction = want
      else:
        self.err_dir = want
        self.err_since_ns = now
    if direction < 0 and current <= MIN_SET_MPH:
      direction = 0
    if self.unexpected_since_ns != 0:
      direction = 0  # the car's set speed changed without us: never tap over it while that is checked
    if direction > 0 and current >= self.ceiling:
      direction = 0

    st.state = State.DECREASING if direction < 0 else (State.INCREASING if direction > 0 else State.HOLDING)
    self.pending_tap = direction if ready and self.tap is None else 0

  def _frame(self, c: CarInputs, new_frame: bool, allowed: bool) -> str | None:
    st = self.status
    t = c.throttle
    if self.phase != Phase.IDLE and (throttle_driver_input(t) or c.pcm_cancel):
      # Driver input goes straight through (panda forwards it); never cover it.
      self.phase = Phase.IDLE
      if self.tap is not None and not self.tap.started:
        self.tap = None
    if not new_frame:
      st.sending = False
      return None

    button = None
    if self.phase == Phase.IDLE:
      if allowed and self.pending_tap != 0:
        self._start_tap(c)
        self.phase, self.phase_frames = Phase.PRESS, 0
    if self.phase == Phase.PRESS:
      if not allowed:
        self.phase, self.phase_frames = Phase.RELEASE, 0
        button = "NO_BUTTON_PRESSED"
      else:
        if self.phase_frames == 0 and self.tap is not None:
          self.tap.started = True
          self.tap.start_ns = c.now_ns
          st.taps_requested += 1
          st.direction = self.tap.direction
        # The press runs its full length even once the car has answered (the tap is then already
        # confirmed and cleared); press_dir keeps the direction.
        button = "RES_BUTTON" if self.press_dir > 0 else "SET_BUTTON"
        self.phase_frames += 1
        if self.phase_frames >= PRESS_FRAMES:
          self.phase, self.phase_frames = Phase.RELEASE, 0
    elif self.phase == Phase.RELEASE:
      button = "NO_BUTTON_PRESSED"
      self.phase_frames += 1
      if self.phase_frames >= RELEASE_FRAMES:
        if allowed and self.pending_tap != 0 and self.tap is None:
          self._start_tap(c)
          self.phase, self.phase_frames = Phase.PRESS, 0
        else:
          self.phase, self.phase_frames = Phase.IDLE, 0

    st.sending = button is not None and self.mode == MODE_ACTIVE
    return button if self.mode == MODE_ACTIVE else None

  def _start_tap(self, c: CarInputs) -> None:
    current = self.virtual_set if self.mode == MODE_WATCH else c.set_raw - 1
    self.tap = _Tap(direction=self.pending_tap, set_before=current)
    self.press_dir = self.pending_tap
    self.pending_tap = 0
