#pragma once
#include "openpilot/selfdrive/pandad/aol_protocol.h"
inline bool nissan_aol_param(uint16_t param) {
  // 0x100 = Rogue cruise-button control opt-in (NISSAN_PARAM_ICBM); independent of AOL.
  const uint16_t base = param & static_cast<uint16_t>(~0x100U);
  return (base == 0U) || (base == 1U) || (base == 4U);
}
inline constexpr AolSafetyProfile NISSAN_AOL_PROFILE{14U, nissan_aol_param, true};
