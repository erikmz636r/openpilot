"""Rogue cruise-button extras: speed scale, posted limit, stops (opt-in, Active only)."""
import math
from types import SimpleNamespace as NS

from opendbc.car.nissan.icbm import IcbmController, MODE_ACTIVE, MODE_WATCH
import openpilot.starpilot.car.nissan.icbm as rt
from openpilot.starpilot.car.nissan import icbm_settings as S
from openpilot.starpilot.car.nissan.icbm_targets import HoldPlanner, LimitPlanner, SpeedScale, StopPlanner, MS_TO_MPH

SEC = 1_000_000_000
DT = 50_000_000  # 20 Hz


def slc(mph, kind='valid', source='vision'):
  return NS(observationKind=kind, source=source, speedLimit=mph / MS_TO_MPH)


def lead(d, v_lead, a=0.0, prob=0.99, y=0.0, present=True):
  return NS(present=present, modelProb=prob, dRel=d, yRel=y, vLeadK=v_lead, aLeadK=a)


def stop_model(v0, x_end, v_end, yaw_end=0.0, n=33):
  t = [i * 10.0 / (n - 1) for i in range(n)]
  vx = [v0 + (v_end - v0) * (ti / 10.0) for ti in t]
  px = [0.0]
  for i in range(1, n):
    px.append(px[-1] + 0.5 * (vx[i] + vx[i - 1]) * (t[i] - t[i - 1]))
  k = x_end / px[-1] if px[-1] > 0 else 1.0
  px = [p * k for p in px]
  return NS(velocity=NS(x=vx), position=NS(x=px), orientation=NS(z=[yaw_end * ti / 10.0 for ti in t]))


# ------------------------------------------------------------------ speed scale
def test_speed_scale_learns_model_over_vego_and_clamps():
  s = SpeedScale()
  assert s.scale == 1.0
  for _ in range(100):  # 5 s: still warming up
    s.update(25.0, 28.5, 0.05)
  assert s.scale == 1.0
  for _ in range(2000):
    s.update(25.0, 28.5, 0.05)
  assert abs(s.scale - 1.14) < 0.01
  for _ in range(4000):  # nonsense ratios are ignored, low speed ignored
    s.update(25.0, 60.0, 0.05)
    s.update(3.0, 4.0, 0.05)
  assert abs(s.scale - 1.14) < 0.01
  s2 = SpeedScale()
  s2.ratio, s2.samples_s = 1.3, 99
  assert s2.scale == 1.2


def test_controller_compares_on_car_scale():
  ctl = IcbmController(mode=MODE_WATCH)
  ctl.set_speed_scale(1.14)
  assert ctl.speed_scale == 1.14
  ctl.set_speed_scale(float('nan'))
  assert ctl.speed_scale == 1.0
  ctl.set_speed_scale(3.0)
  assert ctl.speed_scale == 1.0  # out of range: back to raw vEgo


# ------------------------------------------------------------------ posted limit
def run_limit(p, readings, v=29.0, t0=SEC):
  out, now = [], t0
  for r in readings:
    now += DT
    out.append(p.update(r, r is not None, now, v))
  return out, now


def test_limit_needs_confirmation_then_lowers_to_limit_plus_offset():
  p = LimitPlanner(offset_mph=3)
  out, _ = run_limit(p, [slc(55)] * 10)  # 0.5 s: not confirmed yet
  assert all(math.isinf(x) for x in out)
  out, _ = run_limit(p, [slc(55)] * 200, t0=SEC + 10 * DT)
  assert p.limit == 55 and abs(out[-1] - 58.0) < 0.01 and p.source == 'vision'


def test_limit_unknown_stale_or_odd_values_do_nothing():
  for bad in ([None] * 200, [slc(55, kind='stale')] * 200, [slc(55, kind='unknown')] * 200, [slc(55, source='none')] * 200,
              [slc(57)] * 200, [slc(5)] * 200, [slc(120)] * 200, [slc(float('nan'))] * 200):
    p = LimitPlanner()
    out, _ = run_limit(p, bad)
    assert all(math.isinf(x) for x in out) and p.limit == 0


def test_limit_flicker_is_held_but_long_gap_releases():
  p = LimitPlanner()
  seq = ([slc(45)] * 30 + [None] * 12) * 10  # ~1.5 s valid / 0.6 s gaps, like vision
  out, now = run_limit(p, seq, v=27.0)
  assert p.limit == 45 and math.isfinite(out[-1])
  out, _ = run_limit(p, [None] * 400, v=27.0, t0=now)  # 20 s nothing
  assert p.limit == 0 and math.isinf(out[-1])


def test_limit_never_above_speed_and_other_value_needs_confirm():
  p = LimitPlanner()
  out, now = run_limit(p, [slc(70)] * 100, v=25.0)  # 70 mph limit at 56 mph: nothing to lower
  assert all(math.isinf(x) for x in out)
  out, now = run_limit(p, [slc(70)] * 100 + [slc(35)] * 5 + [slc(70)] * 50, v=25.0, t0=now)  # brief misread
  assert p.limit == 70 and all(math.isinf(x) for x in out)


def test_limit_driver_override_until_limit_changes():
  p = LimitPlanner()
  out, now = run_limit(p, [slc(45)] * 200)
  assert out[-1] < 50
  p.override()
  out, now = run_limit(p, [slc(45)] * 200, t0=now)
  assert all(math.isinf(x) for x in out)
  out, now = run_limit(p, [slc(35)] * 200, t0=now)  # new limit: acts again
  assert p.limit == 35 and out[-1] < 40


# ------------------------------------------------------------------ stops
def run_stop(p, model, n, v=20.0, ld=None, t0=SEC, ok=True):
  out, now = [], t0
  for _ in range(n):
    now += DT
    out.append(p.update(v, model, ld, now, ok))
  return out, now


def test_stop_needs_sustained_detection_and_only_goes_down():
  p = StopPlanner()
  m = stop_model(20.0, 110.0, 0.5)
  out, now = run_stop(p, m, 15)  # 0.75 s
  assert all(math.isinf(x) for x in out)
  out, now = run_stop(p, m, 60, t0=now)
  fin = [x for x in out if math.isfinite(x)]
  assert fin and all(b <= a + 1e-9 for a, b in zip(fin, fin[1:], strict=False))
  assert fin[-1] < 20.0 * MS_TO_MPH


def test_stop_ignores_curves_leads_low_speed_and_far_stops():
  for m, ld, v in ((stop_model(20.0, 110.0, 0.5, yaw_end=0.6), None, 20.0),  # a turn
                   (stop_model(20.0, 110.0, 0.5), lead(60.0, 0.0), 20.0),     # a car stopped ahead: radar's job
                   (stop_model(8.0, 40.0, 0.5), None, 8.0),                   # below 20 mph
                   (stop_model(20.0, 200.0, 18.0), None, 20.0)):              # just a plan, no stop
    p = StopPlanner()
    out, _ = run_stop(p, m, 100, v=v, ld=ld)
    assert all(math.isinf(x) for x in out)


def test_stop_releases_with_hysteresis_and_on_override():
  p = StopPlanner()
  out, now = run_stop(p, stop_model(20.0, 110.0, 0.5), 60)
  assert math.isfinite(out[-1])
  go = stop_model(20.0, 200.0, 20.0)
  out, now = run_stop(p, go, 20, t0=now)  # 1 s clear: still held
  assert math.isfinite(out[-1])
  out, now = run_stop(p, go, 20, t0=now)
  assert math.isinf(out[-1])
  out, now = run_stop(p, stop_model(20.0, 110.0, 0.5), 60, t0=now)
  assert math.isfinite(out[-1])
  p.override(now)
  out, now = run_stop(p, stop_model(20.0, 110.0, 0.5), 100, t0=now)  # same stop: driver said go
  assert all(math.isinf(x) for x in out)


# ------------------------------------------------------------------ settings / runtime
class Params:
  def __init__(self, d): self.d = d
  def get(self, k): return self.d.get(k)
  def get_bool(self, k): return bool(self.d.get(k, False))


def test_features_default_off_and_offset_validated():
  assert S.read_features(Params({})) == 0
  assert S.read_features(Params({S.ICBM_LIMIT_KEY: True, S.ICBM_STOPS_KEY: True})) == S.FEATURE_LIMIT | S.FEATURE_STOPS
  assert S.read_features(Params({"NissanRogueIcbmLead": True})) == 0  # not shipped
  assert S.read_features(Params({S.ICBM_HOLD_KEY: True})) == S.FEATURE_HOLD == 8
  assert S.read_limit_offset(Params({S.ICBM_LIMIT_OFFSET_KEY: 5})) == 5
  assert S.read_limit_offset(Params({S.ICBM_LIMIT_OFFSET_KEY: 4})) == 0
  assert S.read_limit_offset(Params({S.ICBM_LIMIT_OFFSET_KEY: "x"})) == 0

  class Broken:
    def get(self, k): raise RuntimeError
    def get_bool(self, k): raise RuntimeError
  assert S.read_features(Broken()) == 0 and S.read_limit_offset(Broken()) == 0


def test_features_only_in_active(monkeypatch):
  import openpilot.cereal.messaging as messaging
  monkeypatch.setattr(messaging, 'SubMaster', lambda services: NS(services=services))
  from opendbc.car.structs import CarParams
  allp = {S.ICBM_LIMIT_KEY: True, S.ICBM_STOPS_KEY: True, S.ICBM_HOLD_KEY: True}

  def card(mode):
    cp = CarParams(carFingerprint="NISSAN_ROGUE", pcmCruise=True)
    cp.safetyConfigs = [CarParams.SafetyConfig(safetyModel=CarParams.SafetyModel.nissan)]
    return NS(CP=cp, CI=NS(CC=NS(icbm=IcbmController())), params=Params({S.ICBM_KEY: mode, **allp}))
  r = rt.configure(card(S.ICBM_WATCH), [])
  assert r is not None and r.features == 0 and r.sm.services == ['modelV2']
  r = rt.configure(card(S.ICBM_ACTIVE), [])
  assert r is not None and r.features == 14 and set(r.sm.services) == {'modelV2', 'radarState', 'slcState'}


class SM:
  def __init__(self, msgs, mono):
    self.msgs = msgs
    names = list(msgs)
    self.seen = dict.fromkeys(names, True)
    self.valid = dict.fromkeys(names, True)
    self.updated = dict.fromkeys(names, True)
    self.logMonoTime = dict.fromkeys(names, mono)
  def update(self, timeout=0): pass
  def __getitem__(self, k): return self.msgs[k]


def straight_model(v):
  from openpilot.starpilot.car.nissan.tests.test_icbm_runtime import model_msg
  return model_msg(v=v)


def test_runtime_limit_combines_with_min_and_driver_raise_overrides():
  cc = NS(icbm=IcbmController(mode=MODE_ACTIVE))
  card = NS(CI=NS(CC=cc), rogue_icbm=None)
  runtime = rt.IcbmRuntime(MODE_ACTIVE, sm=SM({'modelV2': straight_model(29.0), 'slcState': slc(45)}, SEC),
                           features=S.FEATURE_LIMIT)
  card.rogue_icbm = runtime
  now = SEC
  for _ in range(100):
    now += DT
    runtime.sm.logMonoTime = dict.fromkeys(runtime.sm.logMonoTime, now)
    runtime.update(card, NS(vEgo=29.0), None, now)
  assert card.rogue_icbm is runtime and cc.icbm.target_mph < 50 and cc.icbm.status.limit_mph == 45
  # driver pushes RES while we yield to him -> limit ignored until it changes
  cc.icbm.status.state, cc.icbm.status.car_set_mph = 2, 55
  runtime.update(card, NS(vEgo=29.0), None, now + DT)
  cc.icbm.status.car_set_mph = 56
  runtime.update(card, NS(vEgo=29.0), None, now + 2 * DT)
  assert runtime.limit.overridden == 45 and math.isinf(cc.icbm.target_mph)


def test_runtime_extras_error_turns_everything_off():
  cc = NS(icbm=IcbmController(mode=MODE_ACTIVE))
  card = NS(CI=NS(CC=cc), rogue_icbm=None)
  runtime = rt.IcbmRuntime(MODE_ACTIVE, sm=SM({'modelV2': straight_model(29.0), 'slcState': object()}, SEC),
                           features=S.FEATURE_LIMIT)
  card.rogue_icbm = runtime
  runtime.update(card, NS(vEgo=29.0), None, SEC + DT)
  assert card.rogue_icbm is None and cc.icbm.mode == 0


# ------------------------------------------------------------------ hold RES+ while slowing ahead
def plan(v0, v_end):
  return NS(velocity=NS(x=[v0, (v0 + v_end) / 2, v_end]))


def rlead(d, v_rel, prob=0.99, present=True):
  return NS(present=present, modelProb=prob, dRel=d, vRel=v_rel)


def run_hold(h, n, model=None, a=0.0, ld=None, t0=SEC, ok=True):
  t = t0
  out = False
  for _ in range(n):
    t += DT
    if model is not None and ok:
      h.observe_model(model)
    out = h.update(t, a, ok, ld)
  return out, t


def test_hold_on_model_slowdown_with_release():
  h = HoldPlanner()
  held, t = run_hold(h, 40, plan(17.0, 16.0))
  assert not held  # cruising
  held, t = run_hold(h, 20, plan(17.0, 4.0), t0=t)
  assert held and h.reason == "plan"
  held, t = run_hold(h, 25, plan(17.0, 17.0), t0=t)  # 1 s mean recovers after ~0.6 s, then 1.5 s release
  assert held
  held, t = run_hold(h, 40, plan(17.0, 17.0), t0=t)
  assert not held


def test_hold_single_frame_flicker_ignored():
  h = HoldPlanner()
  held, t = run_hold(h, 40, plan(17.0, 17.0))
  for _ in range(5):  # one bad frame a second (drive 5 plans alternate like this)
    h.observe_model(plan(17.0, 3.0))
    held, t = run_hold(h, 19, plan(17.0, 17.0), t0=t)
    assert not held


def test_hold_on_braking_and_closing_lead_only():
  h = HoldPlanner()
  assert run_hold(h, 2, plan(17.0, 17.0), a=-0.8)[0] and h.reason == "braking"
  h = HoldPlanner()
  assert run_hold(h, 2, plan(17.0, 17.0), ld=rlead(25.0, -5.0))[0] and h.reason == "lead"
  for ld in (rlead(25.0, 0.2), rlead(55.0, -1.1), rlead(40.0, -3.0), rlead(90.0, -20.0), rlead(25.0, -5.0, prob=0.3),
             rlead(25.0, -5.0, present=False), None):
    h = HoldPlanner()
    assert not run_hold(h, 40, plan(17.0, 17.0), a=-0.2, ld=ld)[0]


def test_hold_no_model_clears_window():
  h = HoldPlanner()
  run_hold(h, 20, plan(17.0, 4.0))
  held, t = run_hold(h, 60, ok=False)
  assert not held and h.ends == []


def test_runtime_hold_only_blocks_res_and_never_sets_target():
  cc = NS(icbm=IcbmController(mode=MODE_ACTIVE))
  card = NS(CI=NS(CC=cc), rogue_icbm=None)
  radar = NS(leadOne=rlead(20.0, -4.0))
  runtime = rt.IcbmRuntime(MODE_ACTIVE, sm=SM({'modelV2': straight_model(29.0), 'radarState': radar}, SEC),
                           features=S.FEATURE_HOLD)
  card.rogue_icbm = runtime
  now = SEC
  for _ in range(20):
    now += DT
    runtime.sm.logMonoTime = dict.fromkeys(runtime.sm.logMonoTime, now)
    runtime.update(card, NS(vEgo=29.0, aEgo=0.0), None, now)
  assert card.rogue_icbm is runtime and cc.icbm.hold_up and cc.icbm.status.hold_up
  assert math.isinf(cc.icbm.target_mph)  # no slow-down asked for
  radar.leadOne = rlead(30.0, 0.5)
  for _ in range(40):
    now += DT
    runtime.sm.logMonoTime = dict.fromkeys(runtime.sm.logMonoTime, now)
    runtime.update(card, NS(vEgo=29.0, aEgo=0.0), None, now)
  assert not cc.icbm.hold_up


def test_runtime_without_hold_feature_never_holds():
  cc = NS(icbm=IcbmController(mode=MODE_ACTIVE))
  card = NS(CI=NS(CC=cc), rogue_icbm=None)
  runtime = rt.IcbmRuntime(MODE_ACTIVE, sm=SM({'modelV2': straight_model(29.0), 'radarState': NS(leadOne=rlead(30.0, -4.0))}, SEC),
                           features=S.FEATURE_STOPS)
  card.rogue_icbm = runtime
  runtime.update(card, NS(vEgo=29.0, aEgo=-2.0), None, SEC + DT)
  assert card.rogue_icbm is runtime and not cc.icbm.hold_up
