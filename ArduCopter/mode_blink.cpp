#include "Copter.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// ====== USER SETTINGS (change these) ======
static constexpr uint8_t  BLINK_OUT_CH   = 8;     // 0-based output channel. 8 = OUTPUT9 (often AUX1)
static constexpr uint16_t BLINK_PWM_ON   = 1900;  // microseconds
static constexpr uint16_t BLINK_PWM_OFF  = 1100;  // microseconds
static constexpr uint32_t BLINK_PERIOD_MS = 500;  // toggle every 500ms
// =========================================

bool ModeBlink::init(bool ignore_checks)
{
    // We are NOT flying. Make sure motors stay shut down.
    if (motors != nullptr) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }

    // Optional: immediately write "OFF" so it starts from a known state
    hal.rcout->write(BLINK_OUT_CH, BLINK_PWM_OFF);

    // ignore_checks is intentionally ignored — this mode is for bench testing
    return true;
}

void ModeBlink::run()
{
    // Keep motors shut down forever in this mode
    if (motors != nullptr) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }

    const uint32_t now = AP_HAL::millis();

    static uint32_t last_toggle_ms = 0;
    static bool state_on = false;

    if (now - last_toggle_ms >= BLINK_PERIOD_MS) {
        last_toggle_ms = now;
        state_on = !state_on;

        const uint16_t pwm = state_on ? BLINK_PWM_ON : BLINK_PWM_OFF;
        hal.rcout->write(BLINK_OUT_CH, pwm);
    }
}
