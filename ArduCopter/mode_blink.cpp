#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>   // for gcs().send_text()

extern const AP_HAL::HAL& hal;

// ====== USER SETTINGS (change these) ======
static constexpr uint8_t  BLINK_OUT_CH   = 8;     // 0-based output channel. 8 = OUTPUT9 (often AUX1)
static constexpr uint16_t BLINK_PWM_ON   = 1900;  // microseconds
static constexpr uint16_t BLINK_PWM_OFF  = 1100;  // microseconds
static constexpr uint32_t BLINK_PERIOD_MS = 500;  // toggle every 500ms

static constexpr uint32_t MSG_PERIOD_MS   = 2000; // send status text every 2s
// =========================================

bool ModeBlink::init(bool ignore_checks)
{
    // keep your motor shutdown
    if (motors) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }
    // Start from known output state
    hal.rcout->write(BLINK_OUT_CH, BLINK_PWM_OFF);

    return true;
}

void ModeBlink::run()
{
    // Keep motors shut down forever in this mode
    if (motors != nullptr) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }

    const uint32_t now = AP_HAL::millis();

    // PWM blinking
    static uint32_t last_toggle_ms = 0;
    static bool state_on = false;

    if (now - last_toggle_ms >= BLINK_PERIOD_MS) {
        last_toggle_ms = now;
        state_on = !state_on;

        const uint16_t pwm = state_on ? BLINK_PWM_ON : BLINK_PWM_OFF;
        hal.rcout->write(BLINK_OUT_CH, pwm);
    }

    // Periodic MAVLink "still in blink" message (throttled)
    static uint32_t last_msg_ms = 0;
    if (now - last_msg_ms >= MSG_PERIOD_MS) {
        last_msg_ms = now;

        gcs().send_text(MAV_SEVERITY_DEBUG, "BLINK running");
    }
}
