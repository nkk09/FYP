#include "Copter.h"
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

static inline float deg2rad(float d) { return d * (M_PI / 180.0f); }

bool ModeTiltServo::init(bool ignore_checks)
{
    // On entry: set safe defaults
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting1, SERVO_PWM_TRIM);

    // Second tilt output (map this to SERVO1 via params, e.g. SERVO1_FUNCTION = Scripting6)
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting6, SERVO_PWM_TRIM);

    // Motors OFF initially (we are forcing them via scripting outputs)
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting2, MOTOR_PWM_OFF);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting3, MOTOR_PWM_OFF);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting4, MOTOR_PWM_OFF);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting5, MOTOR_PWM_OFF);

    _last_msg_ms = 0;
    return true;
}

void ModeTiltServo::run()
{
    const uint32_t now = AP_HAL::millis();

    // Pitch in radians, nose-up positive
    const float pitch_rad = ahrs.get_pitch();
    const float deadband_rad = deg2rad(PITCH_DEADBAND_DEG);

    // ==============================
    // 1) FORCE MOTORS ON/OFF by pitch
    // ==============================
    // Pitch up => ON, else => OFF
    const uint16_t motor_pwm = (pitch_rad > deadband_rad) ? MOTOR_PWM_ON : MOTOR_PWM_OFF;

    // Example: QUAD motors mapped to Scripting2..5
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting2, motor_pwm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting3, motor_pwm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting4, motor_pwm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting5, motor_pwm);

    // ==================================
    // 2) SERVO proportional to pitch angle
    // ==================================
    const float pitch_deg = degrees(pitch_rad);
    const float pitch_clamped = constrain_float(pitch_deg, -SERVO_PITCH_RANGE_DEG, SERVO_PITCH_RANGE_DEG);

    // Normalize to 0..1
    const float t = (pitch_clamped + SERVO_PITCH_RANGE_DEG) / (2.0f * SERVO_PITCH_RANGE_DEG);

    // Reversed direction: MAX -> MIN as pitch increases
    const float servo_pwm_f = SERVO_PWM_MAX - t * (SERVO_PWM_MAX - SERVO_PWM_MIN);
    const uint16_t servo_pwm =
        (uint16_t)constrain_int16((int16_t)servo_pwm_f, SERVO_PWM_MIN, SERVO_PWM_MAX);

    // Drive SERVO9 via Scripting1
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting1, servo_pwm);

    // Also drive SERVO1 (map SERVO1_FUNCTION to Scripting6 in params)
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting6, servo_pwm);

    // ===========
    // 3) Debug msg
    // ===========
    if (now - _last_msg_ms >= MSG_PERIOD_MS) {
        _last_msg_ms = now;
        gcs().send_text(MAV_SEVERITY_DEBUG,
                        "TILTSERVO: pitch=%.1f deg, servo=%u, motors=%u",
                        (double)pitch_deg, (unsigned)servo_pwm, (unsigned)motor_pwm);
    }
}