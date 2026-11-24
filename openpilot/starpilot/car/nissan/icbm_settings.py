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


def read_mode(params) -> int:
  try:
    value = params.get(ICBM_KEY)
  except Exception:
    return ICBM_OFF
  return value if type(value) is int and ICBM_OFF <= value <= ICBM_ACTIVE else ICBM_OFF
