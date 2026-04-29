/*
   Implementation of the custom tricopter mixer.

   This intentionally short-circuits AP_MotorsMatrix's thrust distribution:
   instead of mixing roll/pitch/yaw/throttle into motor channels, we take
   physical targets (Tf, Tr, alpha) that AC_CustomControl_FYP already
   allocated, and write them directly to SRV_Channels.

   All mixing math here mirrors apply_final_mixer() in the FYP doc, with
   the following ArduPilot-idiomatic changes:
     - hal.rcout->write(ch, pwm) is replaced by SRV_Channels::set_output_pwm
     - normalized throttle 0..1 is converted to PWM via servo_min/servo_max
*/
#include "AP_MotorsTricopterFYP.h"

#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// ---------------------------------------------------------------------
// Channel mapping (aligned with SDF's ArduPilotPlugin <control channel>
// entries and the tricopter.parm SERVOn_FUNCTION settings).
// ---------------------------------------------------------------------
//   SRV_Channel::k_motor1   (ch 0) -> front LEFT  TOP
//   SRV_Channel::k_motor2   (ch 1) -> front LEFT  BOT
//   SRV_Channel::k_motor3   (ch 2) -> front RIGHT TOP
//   SRV_Channel::k_motor4   (ch 3) -> front RIGHT BOT
//   SRV_Channel::k_motor5   (ch 4) -> tail UP-pusher
//   SRV_Channel::k_motor6   (ch 5) -> tail DOWN-pusher
//   SRV_Channel::k_motor_tilt           (ch 6) -> LEFT tilt servo
//   SRV_Channel::k_tiltMotorRight       (ch 7) -> RIGHT tilt servo

void AP_MotorsTricopterFYP::init(motor_frame_class frame_class,
                                 motor_frame_type  frame_type)
{
    // Register our motor channels so SRV_Channels::set_output_pwm_chan
    // and the standard arming machinery know they exist.
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_3);
    add_motor_num(AP_MOTORS_MOT_4);
    add_motor_num(AP_MOTORS_MOT_5);
    add_motor_num(AP_MOTORS_MOT_6);

    // Park servos at neutral on init.
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor_tilt,     1500);
    SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, 1500);

    set_initialised_ok(true);
    set_update_rate(_speed_hz);
}

void AP_MotorsTricopterFYP::set_frame_class_and_type(motor_frame_class frame_class,
                                                     motor_frame_type  frame_type)
{
    // Frame class/type parameters don't change the mix. Just record and
    // call init so SRV_Channels is populated.
    _active_frame_class = frame_class;
    _active_frame_type  = frame_type;
    init(frame_class, frame_type);
}

uint32_t AP_MotorsTricopterFYP::get_motor_mask()
{
    // Outputs in use: channels 1..6 for motors.
    const uint32_t motor_mask =
        (1U << (AP_MOTORS_MOT_1)) |
        (1U << (AP_MOTORS_MOT_2)) |
        (1U << (AP_MOTORS_MOT_3)) |
        (1U << (AP_MOTORS_MOT_4)) |
        (1U << (AP_MOTORS_MOT_5)) |
        (1U << (AP_MOTORS_MOT_6));
    return motor_mask_to_srv_channel_mask(motor_mask);
}

void AP_MotorsTricopterFYP::set_fyp_targets(float Tf_newtons,
                                            float Tr_newtons,
                                            float alpha_rad)
{
    _fyp_Tf_N      = isfinite(Tf_newtons) ? Tf_newtons : 0.0f;
    _fyp_Tr_N      = isfinite(Tr_newtons) ? Tr_newtons : 0.0f;
    _fyp_alpha_rad = isfinite(alpha_rad)  ? alpha_rad  : 0.0f;
    // roll_cmd is no longer accepted here. The roll command from the main
    // ArduPilot attitude controller is already stored in _roll_in by the
    // time output_armed_stabilizing() runs (CC_AXIS_MASK leaves roll to
    // the default controller, which calls motors.set_roll() before output).
}

void AP_MotorsTricopterFYP::output_armed_stabilizing()
{
    const float roll_trim = constrain_float(_roll_in * 0.3f, -0.3f, 0.3f);

    // --- FALLBACK: standard throttle path when FYP controller inactive ---
    if (_fyp_Tf_N < 0.01f && fabsf(_fyp_Tr_N) < 0.01f) {
        const float thr = constrain_float(_throttle_in, 0.0f, 1.0f);
        _thrust_rpyt_out[AP_MOTORS_MOT_1] = constrain_float(thr + roll_trim, 0.0f, 1.0f);
        _thrust_rpyt_out[AP_MOTORS_MOT_2] = constrain_float(thr + roll_trim, 0.0f, 1.0f);
        _thrust_rpyt_out[AP_MOTORS_MOT_3] = constrain_float(thr - roll_trim, 0.0f, 1.0f);
        _thrust_rpyt_out[AP_MOTORS_MOT_4] = constrain_float(thr - roll_trim, 0.0f, 1.0f);
        _thrust_rpyt_out[AP_MOTORS_MOT_5] = thr;
        _thrust_rpyt_out[AP_MOTORS_MOT_6] = 0.0f;

        _log_counter++;
        if (_log_counter >= 200) {
            _log_counter = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                "MOT dflt thr=%.2f m1=%.2f m3=%.2f roll=%.2f",
                (double)thr,
                (double)_thrust_rpyt_out[AP_MOTORS_MOT_1],
                (double)_thrust_rpyt_out[AP_MOTORS_MOT_3],
                (double)roll_trim);
        }
        return;
    }

    // --- FYP path: physics-based targets from AC_CustomControl_FYP ---
    const float base_front_N = (_fyp_Tf_N * 0.5f) / COAX_FACTOR;
    const float base_front_throttle =
        constrain_float(base_front_N / MAX_MOT_NEWTONS, 0.0f, 1.0f);

    const float throttle_left  = constrain_float(base_front_throttle + roll_trim, 0.0f, 1.0f);
    const float throttle_right = constrain_float(base_front_throttle - roll_trim, 0.0f, 1.0f);

    _thrust_rpyt_out[AP_MOTORS_MOT_1] = throttle_left;
    _thrust_rpyt_out[AP_MOTORS_MOT_2] = throttle_left;
    _thrust_rpyt_out[AP_MOTORS_MOT_3] = throttle_right;
    _thrust_rpyt_out[AP_MOTORS_MOT_4] = throttle_right;

    const float throttle_tail =
        constrain_float(fabsf(_fyp_Tr_N) / MAX_MOT_NEWTONS, 0.0f, 1.0f);

    if (_fyp_Tr_N >= 0.0f) {
        _thrust_rpyt_out[AP_MOTORS_MOT_5] = throttle_tail;
        _thrust_rpyt_out[AP_MOTORS_MOT_6] = 0.0f;
    } else {
        _thrust_rpyt_out[AP_MOTORS_MOT_5] = 0.0f;
        _thrust_rpyt_out[AP_MOTORS_MOT_6] = throttle_tail;
    }

    _log_counter++;
    if (_log_counter >= 200) {
        _log_counter = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
            "MOT fyp Tf=%.1fN Tr=%.1fN a=%.1fdeg mF=%.2f mT=%.2f",
            (double)_fyp_Tf_N,
            (double)_fyp_Tr_N,
            (double)(_fyp_alpha_rad * RAD_TO_DEG),
            (double)throttle_left,
            (double)throttle_tail);
    }
}

void AP_MotorsTricopterFYP::output_to_motors()
{
    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        // All motors off, servos at neutral.
        rc_write(AP_MOTORS_MOT_1, get_pwm_output_min());
        rc_write(AP_MOTORS_MOT_2, get_pwm_output_min());
        rc_write(AP_MOTORS_MOT_3, get_pwm_output_min());
        rc_write(AP_MOTORS_MOT_4, get_pwm_output_min());
        rc_write(AP_MOTORS_MOT_5, get_pwm_output_min());
        rc_write(AP_MOTORS_MOT_6, get_pwm_output_min());
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor_tilt,     1500);
        SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, 1500);
        break;

    case SpoolState::GROUND_IDLE:
    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
    {
        // --- Motors ---
        // rc_write() is defined in AP_Motors_Class.cpp. It calls
        // SRV_Channels::set_output_pwm via the motor function mapping,
        // and respects SERVOn_MIN/MAX/TRIM. We convert our normalized
        // [0,1] thrust to PWM by linearly scaling between MOT_SPIN_MIN
        // and MOT_SPIN_MAX using thrust_to_actuator(), then to PWM range.
        // This is the same path AP_MotorsMatrix uses for each motor.
        const uint16_t pwm_min = get_pwm_output_min();
        const uint16_t pwm_max = get_pwm_output_max();

        auto to_pwm = [&](float thrust_normalized) -> uint16_t {
            // thrust_to_actuator() applies spin-min/max linearization
            // and returns a value in [0, 1].
            const float actuator = this->thr_lin.thrust_to_actuator(thrust_normalized);
            return (uint16_t)(pwm_min + actuator * (pwm_max - pwm_min));
        };

        rc_write(AP_MOTORS_MOT_1, to_pwm(_thrust_rpyt_out[AP_MOTORS_MOT_1]));
        rc_write(AP_MOTORS_MOT_2, to_pwm(_thrust_rpyt_out[AP_MOTORS_MOT_2]));
        rc_write(AP_MOTORS_MOT_3, to_pwm(_thrust_rpyt_out[AP_MOTORS_MOT_3]));
        rc_write(AP_MOTORS_MOT_4, to_pwm(_thrust_rpyt_out[AP_MOTORS_MOT_4]));
        rc_write(AP_MOTORS_MOT_5, to_pwm(_thrust_rpyt_out[AP_MOTORS_MOT_5]));
        rc_write(AP_MOTORS_MOT_6, to_pwm(_thrust_rpyt_out[AP_MOTORS_MOT_6]));

        // --- Tilt servos ---
        // Direct PWM formula from FYP doc (apply_final_mixer):
        //   servo_offset = (alpha_deg / 90) * 500
        //   pwm_L = 1500 + servo_offset   (left  arm, SERVO7)
        //   pwm_R = 1500 - servo_offset   (right arm, SERVO8 — mirrored geometry)
        // The Gazebo SDF right-servo already has a negative multiplier so the
        // mirrored PWM produces the same physical tilt angle on both arms.
        const float alpha_deg    = _fyp_alpha_rad * RAD_TO_DEG;
        const float servo_offset = (abs(alpha_deg) / 90.0f) * 1700.0f;
        const uint16_t pwm_L = (uint16_t)constrain_float(500.0f + servo_offset, 500.0f, 2200.0f);
        const uint16_t pwm_R = (uint16_t)constrain_float(2200.0f - servo_offset, 5000.0f, 2200.0f);
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor_tilt,     pwm_L);
        SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, pwm_R);
        break;
    }
    }
}
