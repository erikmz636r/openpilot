"""Names shared by the settings UIs and the Rogue cruise-button control runtime (no heavy imports)."""

ICBM_KEY = "NissanRogueIcbmMode"
ICBM_OFF, ICBM_WATCH, ICBM_ACTIVE = 0, 1, 2

ICBM_TITLE = "Rogue Cruise Button Control"
ICBM_DESCRIPTION = (
  "2019 Rogue with ProPILOT only. Watch Only: openpilot works out where it would tap SET- or RES+ to ease off for "
  "curves and records it, but sends nothing new. Active: openpilot taps SET- to slow ProPILOT before curves and RES+ "
  "to return to your set speed. Never above your set speed, never below 20 mph, never from a stop. Brake, gas, or "
  "any cruise button and it backs off. Change only while parked; takes effect on the next drive."
)
ICBM_MICI_TITLE = "rogue cruise buttons"
ICBM_MICI_OPTIONS = ["off", "watch only", "active"]
ICBM_MICI_DESCRIPTION = (
  "Rogue ProPILOT only. Watch only records where it would tap SET-/RES+ for curves and sends nothing. Active taps "
  "SET- before curves and RES+ back to your set speed, never above it, never under 20 mph. Change while parked."
)

FEATURE_LIMIT, FEATURE_STOPS = 2, 4  # bit 1 reserved (slower-car-ahead, not shipped)
ICBM_LIMIT_KEY = "NissanRogueIcbmSpeedLimit"
ICBM_LIMIT_OFFSET_KEY = "NissanRogueIcbmSpeedLimitOffset"
ICBM_STOPS_KEY = "NissanRogueIcbmStops"
ICBM_LIMIT_OFFSETS = (0, 3, 5, 7, 10)
ICBM_NOTE = " Needs Rogue Cruise Button Control set to Active. Never above your set speed, never below 20 mph. Change only while parked."
ICBM_LIMIT_TITLE = "Rogue Cruise Buttons: Speed Limit"
ICBM_LIMIT_DESCRIPTION = (
  "Lowers your set speed to the posted speed limit (plus the offset below) when openpilot knows it, from StarPilot's "
  "speed-limit sources (on this car: sign vision). Unknown or stale limit: nothing happens. Press RES/SET yourself "
  "and it keeps your speed until the limit changes." + ICBM_NOTE)
ICBM_LIMIT_OFFSET_TITLE = "Rogue Speed Limit Offset (mph)"
ICBM_LIMIT_OFFSET_DESCRIPTION = "Added to the posted limit before lowering your set speed. Change only while parked."
ICBM_STOPS_TITLE = "Rogue Cruise Buttons: Red Lights / Stops"
ICBM_STOPS_DESCRIPTION = (
  "When the driving model plans to stop ahead with no car in front (red light, stop sign), it taps SET- early so "
  "ProPILOT eases off toward 20 mph. It cannot stop the car: you brake." + ICBM_NOTE)
ICBM_LIMIT_MICI_TITLE = "rogue: speed limit"
ICBM_STOPS_MICI_TITLE = "rogue: lights / stops"
ICBM_MICI_NOTE = " Active mode only. Change while parked."


def _get_bool(params, key) -> bool:
  try:
    return params.get_bool(key) is True
  except Exception:
    return False


def read_features(params) -> int:
  return ((FEATURE_LIMIT if _get_bool(params, ICBM_LIMIT_KEY) else 0) |
          (FEATURE_STOPS if _get_bool(params, ICBM_STOPS_KEY) else 0))


def read_limit_offset(params) -> int:
  try:
    value = params.get(ICBM_LIMIT_OFFSET_KEY)
  except Exception:
    return 0
  return value if type(value) is int and value in ICBM_LIMIT_OFFSETS else 0


def read_mode(params) -> int:
  try:
    value = params.get(ICBM_KEY)
  except Exception:
    return ICBM_OFF
  return value if type(value) is int and ICBM_OFF <= value <= ICBM_ACTIVE else ICBM_OFF
