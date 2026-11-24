"""Rogue cruise-button control, card side, with real capnp messages (no fake attribute objects)."""
import math
from types import SimpleNamespace as NS

from openpilot.cereal import log, custom
from opendbc.car.nissan.icbm import IcbmController, MODE_OFF, MODE_WATCH
import openpilot.starpilot.car.nissan.icbm as rt


def model_msg(v=25.0, radius=None, curve_start_m=75.0):
  t = [i * 0.25 for i in range(33)]
  x = [v * ti for ti in t]
  yaw = [(v / radius if radius and xi >= curve_start_m else 0.0) for xi in x]
  msg = log.Event.new_message()
  md = msg.init('modelV2')
  for name, xs in (('position', x), ('velocity', [v] * 33), ('orientationRate', yaw)):
    xyz = getattr(md, 'init')(name)
    xyz.x = xs if name != 'orientationRate' else [0.0] * 33
    xyz.y = [0.0] * 33
    xyz.z = yaw if name == 'orientationRate' else [0.0] * 33
    xyz.t = t
  return msg.as_reader().modelV2


class FakeSM:
  def __init__(self, model, mono):
    self.model, self.mono = model, mono
    self.seen = {'modelV2': True}; self.valid = {'modelV2': True}; self.updated = {'modelV2': True}
    self.logMonoTime = {'modelV2': mono}
  def update(self, timeout=0): pass
  def __getitem__(self, k): return self.model


def test_planner_reads_real_modelv2():
  p = rt.CurvePlanner()
  assert p.update(25.0, model_msg(), 1_000_000_000, True, True) == math.inf
  p = rt.CurvePlanner()
  now = 1_000_000_000
  for _ in range(800):
    now += 10_000_000
    target = p.update(25.0, model_msg(radius=100.0), now, True, True)
  assert 30 < target < 40


def test_runtime_update_with_real_model_and_errors_turn_it_off():
  cc = NS(icbm=IcbmController(mode=MODE_WATCH))
  card = NS(CI=NS(CC=cc), rogue_icbm=None)
  runtime = rt.IcbmRuntime(MODE_WATCH, sm=FakeSM(model_msg(radius=100.0), 1_000_000_000))
  card.rogue_icbm = runtime
  runtime.update(card, NS(vEgo=25.0), None, 1_010_000_000)
  assert card.CI.CC.icbm.target_mph is not None and card.rogue_icbm is runtime
  # broken model input -> feature switches itself off, never raises
  runtime.sm = FakeSM(object(), 1_020_000_000)
  runtime.planner = rt.CurvePlanner()
  runtime.update(card, NS(vEgo=25.0), None, 1_030_000_000)
  assert card.rogue_icbm is None and card.CI.CC.icbm.mode == MODE_OFF


def test_fill_rogue_icbm_handles_inf_and_round_trips():
  ctl = IcbmController(mode=MODE_WATCH)
  ctl.status.target_mph = math.inf
  ctl.status.desired_mph = float('nan')
  ctl.status.direction = -1
  ctl.status.fault_reason = 'x' * 500
  sp = custom.StarPilotCarState.new_message()
  rt.fill_rogue_icbm(sp, ctl.status)
  with custom.StarPilotCarState.from_bytes(sp.to_bytes()) as out:
    out = out.as_builder()
  assert out.rogueIcbm.mode == MODE_WATCH and out.rogueIcbm.targetMph == 0.0 and out.rogueIcbm.direction == -1
  assert len(out.rogueIcbm.faultReason) <= 120


def test_configure_never_raises():
  bad = NS(CP=NS(carFingerprint="NISSAN_ROGUE"), CI=NS(CC=NS(icbm=IcbmController())), params=None)
  assert rt.configure(bad, []) is None
