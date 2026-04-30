#include "Copter.h"
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <math.h>

static inline float deg2rad(float d)
{
    return d * (M_PI / 180.0f);
}

static inline float rad2deg(float r)
{
    return r * (180.0f / M_PI);
}

bool ModeTiltServo::init(bool ignore_checks)
{
    // Tilt servos centered initially
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting1, SERVO_PWM_TRIM);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting6, SERVO_PWM_TRIM);

    // Motors OFF initially
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting2, MOTOR_PWM_OFF);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting3, MOTOR_PWM_OFF);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting4, MOTOR_PWM_OFF);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting5, MOTOR_PWM_OFF);

    _last_msg_ms = 0;

    // Reset controller states
    _rate_error_integral = 0.0f;
    _last_rate_error = 0.0f;
    _d_term_filtered = 0.0f;
    _last_run_ms = AP_HAL::millis();

    return true;
}

void ModeTiltServo::run()
{
    const uint32_t now = AP_HAL::millis();

    float dt = (now - _last_run_ms) * 0.001f;
    _last_run_ms = now;

    // Protect against first-loop weirdness
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.02f;   // fallback: 50 Hz
    }

    // =========================
    // Physical constants
    // =========================
    const float m = 1.123f;
    const float GRAVITY = 9.81f;
    const float L = 0.167f;
    const float I = 0.0217f;
    const float c = 0.09f;

    // =========================
    // Controller gains
    // =========================
  const float KP_ANGLE = 0.6f;
const float KP_RATE  = 1.2f;
const float KI_RATE  = 0.1f;
const float KD_RATE  = -0.002f;
const float N_FILTER = 0.0f;

    // =========================
    // 1. Get IMU data
    // =========================
    const float current_pitch_rad = -ahrs.get_pitch();
    const float current_pitch_deg = rad2deg(current_pitch_rad);

    const Vector3f gyro = ahrs.get_gyro();
    const float current_pitch_rate_rads = -gyro.y;

    // =========================
    // 2. Outer loop: angle -> desired rate
    // =========================
    const float desired_pitch_rad = deg2rad(90.0f);

    const float angle_error = desired_pitch_rad - current_pitch_rad;
    const float desired_rate = KP_ANGLE * angle_error;

    // =========================
    // 3. Inner loop: rate PID -> desired angular acceleration
    // =========================
    const float rate_error = desired_rate - current_pitch_rate_rads;

    _rate_error_integral += rate_error * dt;

    // Optional anti-windup because otherwise this can explode
    _rate_error_integral = constrain_float(_rate_error_integral, -10.0f, 10.0f);

    const float d_error = (rate_error - _last_rate_error) / dt;

    _d_term_filtered =
        (_d_term_filtered + KD_RATE * N_FILTER * d_error * dt) /
        (1.0f + N_FILTER * dt);

    const float desired_accel =
        (KP_RATE * rate_error) +
        (KI_RATE * _rate_error_integral) +
        _d_term_filtered;

    _last_rate_error = rate_error;

    // =========================
    // 4. Physical calculations
    // =========================
    const float Mreq = I * desired_accel;

    const float Rx = 0.0f;
    const float Ry = m * GRAVITY;

    const float sT = sinf(current_pitch_rad);
    const float cT = cosf(current_pitch_rad);

    const float A =
        Mreq
        - L * (1.0f - c) * (Ry * cT)
        - L * (1.0f - c) * (Rx * sT);

    const float B =
        -L * (1.0f - c) * Ry * sT
        + L * (1.0f - c) * Rx * cT
        + c * L * (Rx * cT - Ry * sT);

const float alpha_rad = atan2f(-B, A);
const float alpha_raw_deg = rad2deg(alpha_rad);

// atan2 gives [-180, 180].
// Your servo only represents a physical tilt angle [0, 90].
// So fold angles greater than 90 back into the 0–90 range.
float alpha_deg = fabsf(alpha_raw_deg);

if (alpha_deg > 90.0f) {
    alpha_deg = 180.0f - alpha_deg;
}

const float alpha_clamped = constrain_float(alpha_deg, 0.0f, 90.0f);

    // Left:  0 deg = 500,  90 deg = 2200
    // Right: 0 deg = 2200, 90 deg = 500
    const float pwm_L = 500.0f + (alpha_clamped / 90.0f) * 1700.0f;
    const float pwm_R = 2200.0f - (alpha_clamped / 90.0f) * 1700.0f;

    // =========================
    // 5. Write outputs
    // =========================
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting1, (uint16_t)pwm_L);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting6, (uint16_t)pwm_R);

    // Motors remain off for now
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting2, MOTOR_PWM_OFF);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting3, MOTOR_PWM_OFF);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting4, MOTOR_PWM_OFF);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting5, MOTOR_PWM_OFF);

    // =========================
    // 6. Debug message
    // =========================
    if (now - _last_msg_ms >= MSG_PERIOD_MS) {
        _last_msg_ms = now;

        gcs().send_text(
            MAV_SEVERITY_DEBUG,
            "TILTSERVO: pitch=%.1f deg alpha=%.1f pwmL=%u pwmR=%u desired accel=%u angle error=%.1f desired pitch=%.1f current pitch=%.1f",
            (double)current_pitch_deg,
            (double)alpha_clamped,
            (unsigned)pwm_L,
            (unsigned)pwm_R,
            (unsigned)(desired_accel),
            (double) angle_error,
            (double) desired_pitch_rad,
            (double) current_pitch_rad
        );
    }
}