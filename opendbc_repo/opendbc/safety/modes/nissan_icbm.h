#pragma once

// Rogue / X-Trail cruise-button management (ICBM).
//
// Opt-in only (NISSAN_PARAM_ICBM). With the flag clear every function here is a no-op and the
// Nissan mode behaves exactly as before.
//
// With the flag set, openpilot may additionally send CRUISE_THROTTLE (0x20B) on bus 2 (toward the
// ProPILOT ADAS module) carrying SET (decrease) or RES (increase) taps, or NO_BUTTON frames around
// a tap. Every such frame must be an exact copy of the car's latest 0x20B
// frame from bus 0 (pedal bits, counter and everything else) except the button bits. The car's own
// 0x20B frames are held back from bus 2 only within 60 ms of an accepted SET/RES frame and only if
// the car frame carries no driver button, brake or gas, so driver input always reaches the ADAS module.
//
// Nothing here touches 0x1B6 / 0x1C3 / 0x2D4 or any other ADAS-module output, and no
// diagnostic traffic is allowed. CANCEL keeps the stock rule.

#define NISSAN_PARAM_ICBM 0x100U

#define NISSAN_ICBM_BTN_SET 0x08U
#define NISSAN_ICBM_BTN_RES 0x10U
#define NISSAN_ICBM_BTN_NONE 0x20U
#define NISSAN_ICBM_DRIVER_BTNS 0x1FU  // PROPILOT, CANCEL, FOLLOW, SET, RES

#define NISSAN_ICBM_CAR_MAX_AGE_US 25000U       // car frame used as the copy source
#define NISSAN_ICBM_PRESS_MAX_US 200000U        // longest single press
#define NISSAN_ICBM_PRESS_GAP_US 60000U         // frames closer than this belong to the same press
#define NISSAN_ICBM_RELEASE_MIN_US 100000U      // release time between presses
#define NISSAN_ICBM_PRESS_PERIOD_MIN_US 200000U // at most 5 presses per second
#define NISSAN_ICBM_HOLD_WINDOW_US 60000U       // car frames held back this long after a SET/RES frame
#define NISSAN_ICBM_MIN_SPEED 3000              // 3.0 m/s in vehicle_speed units (x1000)

static bool nissan_icbm = false;
static bool nissan_icbm_car_valid = false;
static uint8_t nissan_icbm_car[6] = {0U, 0U, 0U, 0U, 0U, 0U};
static uint32_t nissan_icbm_car_ts = 0U;
static bool nissan_icbm_tx_seen = false;
static uint32_t nissan_icbm_tx_ts = 0U;
static bool nissan_icbm_pressed = false;
static uint8_t nissan_icbm_press_btn = 0U;
static uint32_t nissan_icbm_press_start_ts = 0U;
static uint32_t nissan_icbm_press_last_ts = 0U;

static void nissan_icbm_reset(void) {
  nissan_icbm_car_valid = false;
  nissan_icbm_car_ts = 0U;
  nissan_icbm_tx_seen = false;
  nissan_icbm_tx_ts = 0U;
  nissan_icbm_pressed = false;
  nissan_icbm_press_btn = 0U;
  nissan_icbm_press_start_ts = 0U;
  nissan_icbm_press_last_ts = 0U;
  for (unsigned int i = 0U; i < 6U; i++) {
    nissan_icbm_car[i] = 0U;
  }
}

static bool nissan_icbm_driver_input(const uint8_t *data) {
  // byte 1 bits 0-4: PROPILOT, CANCEL, FOLLOW_DISTANCE, SET, RES
  // byte 2 bit 6: USER_BRAKE_PRESSED_INVERTED (0 = brake pressed)
  // bytes 3-4: GAS_PEDAL (10 bit), pressed above 3 like the 0x15C check
  const uint32_t gas = ((uint32_t)data[3] << 2) | ((uint32_t)data[4] >> 6);
  return ((data[1] & NISSAN_ICBM_DRIVER_BTNS) != 0U) || ((data[2] & 0x40U) == 0U) || (gas > 3U);
}

// Car-side CRUISE_THROTTLE, bus 0. Called for every received frame (optional RX).
static void nissan_icbm_observe(const CANPacket_t *msg) {
  if (nissan_icbm && (msg->addr == 0x20BU) && (msg->bus == 0U) && (GET_LEN(msg) == 6U)) {
    for (unsigned int i = 0U; i < 6U; i++) {
      nissan_icbm_car[i] = msg->data[i];
    }
    nissan_icbm_car_ts = microsecond_timer_get();
    nissan_icbm_car_valid = true;
  }
}

// Hold back the car's own frame toward the ADAS module only while openpilot is mid-sequence and
// the frame carries no driver input.
static bool nissan_icbm_fwd_block(const CANPacket_t *msg) {
  bool block = false;
  if (nissan_icbm && nissan_icbm_tx_seen && (msg->addr == 0x20BU) && (msg->bus == 0U) && (GET_LEN(msg) == 6U)) {
    const bool recent = safety_get_ts_elapsed(microsecond_timer_get(), nissan_icbm_tx_ts) < NISSAN_ICBM_HOLD_WINDOW_US;
    block = recent && !nissan_icbm_driver_input(msg->data);
  }
  return block;
}

// Returns true only for an ICBM frame that passes every rule. Anything else falls through to the
// stock cancel-only check in the caller.
static bool nissan_icbm_tx_allowed(const CANPacket_t *msg) {
  bool allowed = false;
  if (nissan_icbm && (msg->bus == 2U) && (GET_LEN(msg) == 6U) && nissan_icbm_car_valid) {
    const uint32_t now = microsecond_timer_get();
    const uint8_t buttons = msg->data[1] & 0x3FU;
    const bool is_set = buttons == NISSAN_ICBM_BTN_SET;
    const bool is_res = buttons == NISSAN_ICBM_BTN_RES;
    const bool is_none = buttons == NISSAN_ICBM_BTN_NONE;

    const bool fresh = safety_get_ts_elapsed(now, nissan_icbm_car_ts) <= NISSAN_ICBM_CAR_MAX_AGE_US;
    // Exact copy of the car frame except byte 1 bits 0-5 (counter, pedals and brake included).
    const bool copy = (msg->data[0] == nissan_icbm_car[0]) &&
                      ((msg->data[1] & 0xC0U) == (nissan_icbm_car[1] & 0xC0U)) &&
                      (msg->data[2] == nissan_icbm_car[2]) && (msg->data[3] == nissan_icbm_car[3]) &&
                      (msg->data[4] == nissan_icbm_car[4]) && (msg->data[5] == nissan_icbm_car[5]);
    // Never mask a driver button, brake or gas.
    const bool car_quiet = !nissan_icbm_driver_input(nissan_icbm_car);

    if ((is_set || is_res || is_none) && fresh && copy && car_quiet) {
      if (is_none) {
        allowed = true;
      } else {
        const bool engaged = controls_allowed && cruise_engaged_prev && !gas_pressed && !brake_pressed &&
                             (vehicle_speed.min > NISSAN_ICBM_MIN_SPEED);
        if (engaged) {
          const bool same_press = nissan_icbm_pressed && (nissan_icbm_press_btn == buttons) &&
                                  (safety_get_ts_elapsed(now, nissan_icbm_press_last_ts) < NISSAN_ICBM_PRESS_GAP_US);
          if (same_press) {
            allowed = safety_get_ts_elapsed(now, nissan_icbm_press_start_ts) <= NISSAN_ICBM_PRESS_MAX_US;
          } else {
            const bool spaced = !nissan_icbm_pressed ||
                                ((safety_get_ts_elapsed(now, nissan_icbm_press_last_ts) >= NISSAN_ICBM_RELEASE_MIN_US) &&
                                 (safety_get_ts_elapsed(now, nissan_icbm_press_start_ts) >= NISSAN_ICBM_PRESS_PERIOD_MIN_US));
            if (spaced) {
              allowed = true;
              nissan_icbm_pressed = true;
              nissan_icbm_press_btn = buttons;
              nissan_icbm_press_start_ts = now;
            }
          }
          if (allowed) {
            nissan_icbm_press_last_ts = now;
          }
        }
      }
    }

    // Only a SET/RES frame holds the car's frames back, so the ADAS module never sees the car's
    // NO_BUTTON frame interleaved with a press (which could read as several presses). A NO_BUTTON
    // copy is byte-identical to the car frame it copies, so it never needs to replace anything.
    if (allowed && !is_none) {
      nissan_icbm_tx_seen = true;
      nissan_icbm_tx_ts = now;
    }
  }
  return allowed;
}
