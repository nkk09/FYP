#include "Copter.h"
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

static constexpr uint16_t BLINK_PWM_ON    = 1900;
static constexpr uint16_t BLINK_PWM_OFF   = 1100;
static constexpr uint32_t BLINK_PERIOD_MS = 500;
static constexpr uint32_t MSG_PERIOD_MS   = 2000;

bool ModeBlink::init(bool ignore_checks)
{
    if (motors) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }

    // reset mode state each time you enter the mode
    _last_toggle_ms = AP_HAL::millis();
    _state_on = false;
    _last_msg_ms = 0;

    // Drive SERVO9 via its function mapping: SERVO9_FUNCTION must be Scripting1 (94)
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting1, BLINK_PWM_OFF);

    return true;
}

void ModeBlink::run()
{
    if (motors) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    }

    const uint32_t now = AP_HAL::millis();

    if (now - _last_toggle_ms >= BLINK_PERIOD_MS) {
        _last_toggle_ms = now;
        _state_on = !_state_on;

        SRV_Channels::set_output_pwm(
            SRV_Channel::k_scripting1,
            _state_on ? BLINK_PWM_ON : BLINK_PWM_OFF
        );
    }

    if (now - _last_msg_ms >= MSG_PERIOD_MS) {
        _last_msg_ms = now;
        gcs().send_text(MAV_SEVERITY_DEBUG, "BLINK: scripting1 pwm toggling");
    }
}
