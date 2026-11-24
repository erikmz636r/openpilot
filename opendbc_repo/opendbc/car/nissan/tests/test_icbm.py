from opendbc.car.nissan.icbm import (MODE_ACTIVE, MODE_OFF, MODE_WATCH, PRESS_FRAMES, RELEASE_FRAMES, CarInputs, IcbmController,
                                     State, make_throttle_values)

IDLE = {"COUNTER": 0, "PROPILOT_BUTTON": 0, "CANCEL_BUTTON": 0, "GAS_PEDAL_INVERTED": 1023, "SET_BUTTON": 0, "RES_BUTTON": 0,
        "FOLLOW_DISTANCE_BUTTON": 0, "NO_BUTTON_PRESSED": 1, "GAS_PEDAL": 0, "USER_BRAKE_PRESSED": 0,
        "USER_BRAKE_PRESSED_INVERTED": 1, "NEW_SIGNAL_2": 0, "GAS_PRESSED_INVERTED": 1, "unsure1": 1, "unsure2": 0, "unsure3": 1}
MPH = 0.44704


class FakeRogue:
  """ProPILOT as logged: each SET/RES press of 60-330 ms moves the set speed by one, ~60 ms after it starts."""

  def __init__(self, mode, set_mph=60, v_mph=60.0, respond=True, step=1):
    self.icbm = IcbmController(mode=mode)
    self.t = 1_000_000_000
    self.set_mph = set_mph
    self.v = v_mph * MPH
    self.engaged = True
    self.gas = False
    self.brake = False
    self.standstill = False
    self.driver = {}
    self.counter = 0
    self.frame_ns = 0
    self.frame = dict(IDLE)
    self.respond = respond
    self.step = step
    self.pressed = None
    self.press_start = 0
    self.press_applied = False
    self.sent = []
    self.target = None
    self.follow = False

  def run(self, seconds, target=None):
    for _ in range(int(seconds * 100)):
      self.t += 10_000_000
      if self.t // 20_000_000 != self.frame_ns // 20_000_000:  # new 50 Hz car frame
        self.counter = (self.counter + 1) % 4
        self.frame = dict(IDLE, COUNTER=self.counter, **self.driver)
        if self.gas:
          self.frame["GAS_PEDAL"] = 200
        if self.brake:
          self.frame["USER_BRAKE_PRESSED_INVERTED"] = 0
        self.frame_ns = self.t
      t_arg = target if target is not None else self.target
      if t_arg is not None:
        self.icbm.set_target(t_arg, self.t)
      btn = self.icbm.update(CarInputs(
        now_ns=self.t, op_enabled=self.engaged, cruise_enabled=self.engaged, propilot_engaged=self.engaged,
        v_ego=self.v, standstill=self.standstill, gas_pressed=self.gas, brake_pressed=self.brake,
        throttle=self.frame, throttle_ns=self.frame_ns, set_raw=self.set_mph + 1, set_ns=self.t,
        speed_mph_units=True))
      if btn is not None:
        self.sent.append((self.t, btn, self.frame["COUNTER"]))
      self._car_response(btn)
      if self.follow:  # ProPILOT chasing its set speed at ~0.8 m/s^2
        goal = self.set_mph * MPH
        self.v += max(-0.008, min(0.008, goal - self.v))

  def _car_response(self, btn):
    if btn in ("SET_BUTTON", "RES_BUTTON"):
      if self.pressed != btn:
        self.pressed, self.press_start, self.press_applied = btn, self.t, False
    elif btn == "NO_BUTTON_PRESSED":
      self.pressed = None
    if self.pressed and not self.press_applied and self.t - self.press_start >= 60_000_000 and self.respond:
      self.set_mph += (-self.step if self.pressed == "SET_BUTTON" else self.step)
      self.press_applied = True

  def presses(self, btn):
    n, prev = 0, None
    for _, b, _ in self.sent:
      if b == btn and prev != btn:
        n += 1
      prev = b
    return n


def settle(car, seconds=3.0):
  car.run(seconds)


def test_off_never_sends():
  car = FakeRogue(MODE_OFF)
  car.run(10, target=30)
  assert car.sent == [] and car.set_mph == 60 and car.icbm.status.state == State.OFF


def test_watch_never_sends_but_counts():
  car = FakeRogue(MODE_WATCH)
  settle(car)
  car.run(10, target=50)
  assert car.sent == []
  assert car.set_mph == 60
  assert car.icbm.status.taps_requested >= 10
  assert car.icbm.status.set_mph == 50


def test_active_slows_for_curve_and_returns():
  car = FakeRogue(MODE_ACTIVE)
  settle(car)
  car.run(8, target=50)
  assert car.set_mph == 50
  assert car.presses("RES_BUTTON") == 0
  car.follow = True
  car.run(25, target=80)
  assert car.set_mph == 60  # back to the driver's speed, never above it
  assert car.icbm.status.state == State.HOLDING
  assert car.icbm.status.fault_reason == ""


def test_tap_shape_and_counter_sync():
  car = FakeRogue(MODE_ACTIVE)
  settle(car)
  car.run(3, target=58)
  sets = [s for s in car.sent if s[1] == "SET_BUTTON"]
  assert sets
  # one frame per car frame, exactly PRESS_FRAMES per press, then RELEASE_FRAMES of no-button
  run, runs = 0, []
  for _, b, _ in car.sent:
    if b == "SET_BUTTON":
      run += 1
    elif run:
      runs.append(run)
      run = 0
  assert all(r == PRESS_FRAMES for r in runs)
  counters = [c for _, _, c in car.sent]
  assert all((b - a) % 4 == 1 for a, b in zip(counters, counters[1:], strict=False)) or len(set(counters)) > 1
  gaps = [b[0] - a[0] for a, b in zip(car.sent, car.sent[1:], strict=False)]
  assert min(gaps) >= 20_000_000 - 1
  assert car.presses("SET_BUTTON") == 2
  _ = RELEASE_FRAMES


def test_never_above_ceiling_or_below_floor():
  car = FakeRogue(MODE_ACTIVE, set_mph=24)
  settle(car)
  car.run(10, target=5)
  assert car.set_mph == 20
  car.run(15, target=200)
  assert car.set_mph == 24
  assert max(s for s in [car.set_mph]) <= 24


def test_up_limited_near_actual_speed():
  car = FakeRogue(MODE_ACTIVE, set_mph=60)
  settle(car)
  car.v = 40 * MPH
  car.run(8, target=40)
  assert car.set_mph == 40
  car.v = 41 * MPH
  car.run(5, target=60)
  assert car.set_mph <= 44  # right after the curve: no more than actual speed + 3
  car.run(15, target=60)
  assert car.set_mph == 60  # road clear for a while (car ahead holding us back): back to the driver's speed


def test_deadband_no_bounce():
  car = FakeRogue(MODE_ACTIVE)
  settle(car)
  for target in (59.2, 60.4, 58.7, 59.9, 61.0):
    car.run(2, target=target)
  assert car.sent == []


def test_driver_button_yields_and_recaptures_ceiling():
  car = FakeRogue(MODE_ACTIVE)
  settle(car)
  car.driver = {"RES_BUTTON": 1, "NO_BUTTON_PRESSED": 0}
  car.run(0.1)
  car.set_mph = 65
  car.driver = {}
  car.run(0.5, target=80)
  assert car.icbm.status.state == State.YIELD
  assert car.sent == []
  car.run(10, target=80)
  assert car.set_mph == 65
  assert car.icbm.status.ceiling_mph == 65


def test_no_taps_with_pedals_or_low_speed():
  for attr, value in (("gas", True), ("brake", True), ("standstill", True), ("v", 2.0)):
    car = FakeRogue(MODE_ACTIVE)
    settle(car)
    setattr(car, attr, value)
    car.run(5, target=40)
    assert not any(b in ("SET_BUTTON", "RES_BUTTON") for _, b, _ in car.sent), attr


def test_not_engaged_no_taps():
  car = FakeRogue(MODE_ACTIVE)
  car.engaged = False
  car.run(5, target=40)
  assert car.sent == []
  assert car.icbm.status.state == State.STANDBY


def test_three_misses_fault_and_alert():
  car = FakeRogue(MODE_ACTIVE, respond=False)
  settle(car)
  car.run(5, target=40)
  assert car.presses("SET_BUTTON") == 3
  assert car.icbm.status.state == State.FAULT
  assert car.icbm.status.alert
  assert "not responding" in car.icbm.status.fault_reason
  # stays stopped until ProPILOT disengages, then works again
  n = len(car.sent)
  car.run(5, target=40)
  assert len([s for s in car.sent[n:] if s[1] == "SET_BUTTON"]) == 0
  car.engaged = False
  car.run(0.5)
  car.engaged = True
  car.respond = True
  car.run(10, target=55)
  assert car.set_mph == 55


def test_wrong_size_step_faults():
  car = FakeRogue(MODE_ACTIVE, step=5)
  settle(car)
  car.run(3, target=40)
  assert car.icbm.status.state == State.FAULT
  assert car.set_mph == 55
  assert car.presses("SET_BUTTON") == 1


def test_unexpected_change_faults():
  car = FakeRogue(MODE_ACTIVE)
  settle(car)
  car.set_mph = 63  # changed without any button we saw
  car.run(1, target=80)
  assert car.icbm.status.state == State.FAULT
  assert not any(b == "RES_BUTTON" for _, b, _ in car.sent)


def test_stale_target_holds():
  car = FakeRogue(MODE_ACTIVE)
  settle(car)
  car.icbm.set_target(40, car.t)
  car.run(3)  # target never refreshed -> stale after 0.5 s
  assert car.set_mph >= 58


def test_driver_input_mid_press_stops_sending():
  car = FakeRogue(MODE_ACTIVE)
  settle(car)
  car.run(0.6, target=40)
  assert car.sent
  car.brake = True
  n = len(car.sent)
  car.run(1.0, target=40)
  assert len(car.sent) == n


def test_make_values_copies_everything_but_buttons():
  t = dict(IDLE, COUNTER=3, GAS_PEDAL=2, unsure2=5)
  v = make_throttle_values(t, "SET_BUTTON")
  assert v["SET_BUTTON"] == 1 and v["NO_BUTTON_PRESSED"] == 0 and v["RES_BUTTON"] == 0
  for k in ("COUNTER", "GAS_PEDAL", "GAS_PEDAL_INVERTED", "USER_BRAKE_PRESSED_INVERTED", "unsure1", "unsure2", "unsure3"):
    assert v[k] == t[k]


def test_driver_raise_mid_curve_is_respected():
  car = FakeRogue(MODE_ACTIVE, set_mph=50)
  settle(car)
  car.run(3, target=47)
  assert car.set_mph == 47
  # Erik taps RES up to 55 in the curve; we must not take it back down for this curve
  car.driver = {"RES_BUTTON": 1, "NO_BUTTON_PRESSED": 0}
  car.run(0.1, target=47)
  car.set_mph = 55
  car.driver = {}
  car.run(10, target=47)
  assert car.set_mph == 55
  # a clearly tighter curve still acts
  car.run(10, target=40)
  assert car.set_mph == 40


def test_carcontroller_error_turns_icbm_off_and_keeps_steering():
  from opendbc.car import structs
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.nissan.values import CAR
  CI = interfaces[CAR.NISSAN_ROGUE]
  CP = CI.get_params(CAR.NISSAN_ROGUE, {0: {}, 1: {}, 2: {}}, [], alpha_long=False, is_release=False, docs=False)
  ci = CI(CP)
  ci.update([(900_000_000, [])])
  ci.CC.icbm.mode = MODE_WATCH
  del ci.CS.cruise_throttle_nanos  # simulate an unexpected input problem
  CC = structs.CarControl(enabled=True, latActive=True).as_reader()
  _, sends = ci.apply(CC, 1_000_000_000)
  assert ci.CC.icbm.mode == MODE_OFF
  assert any(addr == 0x169 for addr, _, _ in sends)  # LKAS steering frame still goes out
  _, sends = ci.apply(CC, 1_010_000_000)
  assert any(addr == 0x169 for addr, _, _ in sends)
  assert not any(addr == 0x20B for addr, _, _ in sends)
