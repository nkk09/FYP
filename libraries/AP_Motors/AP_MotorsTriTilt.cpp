#include "AP_Motors_config.h"

#if AP_MOTORS_TRI_TILT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include "AP_MotorsTriTilt.h"

extern const AP_HAL::HAL& hal;

int16_t AP_MotorsTriTilt::tilt_to_scaled(float tilt_rad)
{
    // map [-TILT_MAX_RAD .. +TILT_MAX_RAD] -> [-1000 .. +1000]
    const float s = (tilt_rad / TILT_MAX_RAD) * 1000.0f;
    return (int16_t)constrain_float(s, -1000.0f, 1000.0f);
}

void AP_MotorsTriTilt::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // Motors: 1,2,4
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_4);

    set_update_rate(_speed_hz);

    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;

    _pitch_reversed = (frame_type == MOTOR_FRAME_TYPE_PLUSREV);

    _mav_type = MAV_TYPE_TRICOPTER;

    // record successful initialisation if desired frame_class is TRI
    set_initialised_ok(frame_class == MOTOR_FRAME_TRI);
}

void AP_MotorsTriTilt::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    _pitch_reversed = (frame_type == MOTOR_FRAME_TYPE_PLUSREV);
    set_initialised_ok(frame_class == MOTOR_FRAME_TRI);
}

void AP_MotorsTriTilt::set_update_rate(uint16_t speed_hz)
{
    _speed_hz = speed_hz;

    // update rate for motors 1,2,4
    const uint32_t mask =
        (1U << AP_MOTORS_MOT_1) |
        (1U << AP_MOTORS_MOT_2) |
        (1U << AP_MOTORS_MOT_4);

    rc_set_freq(mask, _speed_hz);
}

uint32_t AP_MotorsTriTilt::get_motor_mask()
{
    // outputs used by motors 1,2,4
    uint32_t motor_mask =
        (1U << AP_MOTORS_MOT_1) |
        (1U << AP_MOTORS_MOT_2) |
        (1U << AP_MOTORS_MOT_4);

    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask (includes other needed outputs)
    mask |= AP_MotorsMulticopter::get_motor_mask();

    // NOTE: tilt servos are on SCRIPTING outputs; their output channels depend on SERVOx mapping.
    // We cannot reliably add them here without digging output channel mappings.
    return mask;
}

void AP_MotorsTriTilt::output_to_motors()
{
    // choose tilt outputs based on spool state
    int16_t tilt_r_scaled = 0;
    int16_t tilt_l_scaled = 0;

    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        // minimum outputs
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled_mask(i)) {
                _actuator[AP_MOTORS_MOT_1 + i] = 0;
            }
        }
        tilt_r_scaled = 0;
        tilt_l_scaled = 0;
        break;

    case SpoolState::GROUND_IDLE:
        set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], actuator_spin_up_to_ground_idle());
        set_actuator_with_slew(_actuator[AP_MOTORS_MOT_2], actuator_spin_up_to_ground_idle());
        set_actuator_with_slew(_actuator[AP_MOTORS_MOT_4], actuator_spin_up_to_ground_idle());
        tilt_r_scaled = 0;
        tilt_l_scaled = 0;
        break;

    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        // thrust requests -> actuator
        set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], thr_lin.thrust_to_actuator(_thrust_right));
        set_actuator_with_slew(_actuator[AP_MOTORS_MOT_2], thr_lin.thrust_to_actuator(_thrust_left));
        set_actuator_with_slew(_actuator[AP_MOTORS_MOT_4], thr_lin.thrust_to_actuator(_thrust_rear));

        tilt_r_scaled = tilt_to_scaled(_tilt_right);
        tilt_l_scaled = tilt_to_scaled(_tilt_left);
        break;
    }

    // write motor PWMs
    rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
    rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[AP_MOTORS_MOT_2]));
    rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[AP_MOTORS_MOT_4]));

    // write tilt servos (scaled output is clean and respects SERVO min/max/trim/rev)
    SRV_Channels::set_output_scaled(TILT_RIGHT_FN, tilt_r_scaled);
    SRV_Channels::set_output_scaled(TILT_LEFT_FN,  tilt_l_scaled);
}

void AP_MotorsTriTilt::output_armed_stabilizing()
{
    // Inputs are in "motor units", we convert to thrust (0..1) downstream
    float roll_thrust;
    float pitch_thrust;
    float yaw_thrust;
    float throttle_thrust;
    float throttle_avg_max;

    float throttle_thrust_best_rpy;
    float rpy_scale = 1.0f;
    float rpy_low = 0.0f;
    float rpy_high = 0.0f;
    float thr_adj;

    // voltage + air pressure compensation
    const float compensation_gain = thr_lin.get_compensation_gain();
    roll_thrust     = (_roll_in  + _roll_in_ff)  * compensation_gain;
    pitch_thrust    = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust      = (_yaw_in   + _yaw_in_ff)   * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;
    throttle_avg_max = _throttle_avg_max * compensation_gain;

    if (_pitch_reversed) {
        pitch_thrust *= -1.0f;
    }

    // ---- Simple tilt law (GET IT WORKING FIRST) ----
    // tilt commanded from pitch demand (you will replace this later with your real controller)
    // same sign on both by default; if left servo mechanically mirrored set _tilt_left = -tilt_cmd
    const float tilt_cmd = constrain_float(pitch_thrust, -1.0f, 1.0f) * TILT_MAX_RAD;
    _tilt_right = tilt_cmd;
    _tilt_left  = tilt_cmd; // change to -tilt_cmd if mirrored

    // ---- Motor mixing (based on tricopter-ish roll/pitch) ----
    // start from your existing mixing (kept close to what you pasted)
    _thrust_right = roll_thrust * -0.5f + pitch_thrust * 0.5f;
    _thrust_left  = roll_thrust *  0.5f + pitch_thrust * 0.5f;
    _thrust_rear  = pitch_thrust * -0.5f;

    // Add yaw via differential thrust (since no yaw servo)
    // Positive yaw increases right, decreases left
    _thrust_right += yaw_thrust * 0.5f;
    _thrust_left  -= yaw_thrust * 0.5f;

    // sanity check throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    // rpy bounds
    rpy_low = MIN(_thrust_right, _thrust_left);
    rpy_high = MAX(_thrust_right, _thrust_left);
    rpy_low = MIN(rpy_low, _thrust_rear);
    rpy_high = MAX(rpy_high, _thrust_rear);

    // how much room for rpy at current throttle
    throttle_thrust_best_rpy = MIN(0.5f - (rpy_low + rpy_high) * 0.5f, throttle_avg_max);

    if (is_zero(rpy_low)) {
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(-throttle_thrust_best_rpy / rpy_low, 0.0f, 1.0f);
    }

    thr_adj = throttle_thrust - throttle_thrust_best_rpy;

    if (rpy_scale < 1.0f) {
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else {
        if (thr_adj < -(throttle_thrust_best_rpy + rpy_low)) {
            thr_adj = -(throttle_thrust_best_rpy + rpy_low);
        } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy + rpy_high)) {
            thr_adj = 1.0f - (throttle_thrust_best_rpy + rpy_high);
            limit.throttle_upper = true;
        }
    }

    const float throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

    // apply scaled rpy
    _thrust_right = throttle_thrust_best_plus_adj + rpy_scale * _thrust_right;
    _thrust_left  = throttle_thrust_best_plus_adj + rpy_scale * _thrust_left;
    _thrust_rear  = throttle_thrust_best_plus_adj + rpy_scale * _thrust_rear;

    // constrain to 0..1
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left  = constrain_float(_thrust_left,  0.0f, 1.0f);
    _thrust_rear  = constrain_float(_thrust_rear,  0.0f, 1.0f);
}

void AP_MotorsTriTilt::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // Motor test order:
    // 1: front right motor (M1)
    // 2: front left motor  (M2)
    // 3: tail motor        (M4)
    // 4: right tilt servo  (SCRIPTING1)
    // 5: left tilt servo   (SCRIPTING2)

    switch (motor_seq) {
    case 1:
        rc_write(AP_MOTORS_MOT_1, pwm);
        break;
    case 2:
        rc_write(AP_MOTORS_MOT_2, pwm);
        break;
    case 3:
        rc_write(AP_MOTORS_MOT_4, pwm);
        break;
    case 4:
        SRV_Channels::set_output_pwm(TILT_RIGHT_FN, pwm);
        break;
    case 5:
        SRV_Channels::set_output_pwm(TILT_LEFT_FN, pwm);
        break;
    default:
        break;
    }
}

void AP_MotorsTriTilt::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        float thrust[4] { _thrust_right, _thrust_left, 0.0f, _thrust_rear };
        _thrust_compensation_callback(thrust, 4);
        _thrust_right = thrust[0];
        _thrust_left  = thrust[1];
        _thrust_rear  = thrust[3];
    }
}

void AP_MotorsTriTilt::output_motor_mask(float thrust, uint32_t mask, float rudder_dt)
{
    // normal multicopter output
    AP_MotorsMulticopter::output_motor_mask(thrust, mask, rudder_dt);

    // keep tilt servos neutral during mask output
    SRV_Channels::set_output_scaled(TILT_RIGHT_FN, 0);
    SRV_Channels::set_output_scaled(TILT_LEFT_FN,  0);
}

float AP_MotorsTriTilt::get_roll_factor(uint8_t i)
{
    switch (i) {
    case AP_MOTORS_MOT_1: return -1.0f; // right
    case AP_MOTORS_MOT_2: return  1.0f; // left
    default: return 0.0f;
    }
}

bool AP_MotorsTriTilt::arming_checks(size_t buflen, char *buffer) const
{
    // Require tilt servos mapped
    if (!SRV_Channels::function_assigned(TILT_RIGHT_FN) ||
        !SRV_Channels::function_assigned(TILT_LEFT_FN)) {
        hal.util->snprintf(buffer, buflen,
            "tilt servos not assigned: set SERVOx_FUNCTION to SCRIPTING1 and SCRIPTING2");
        return false;
    }

    return AP_MotorsMulticopter::arming_checks(buflen, buffer);
}

#endif // AP_MOTORS_TRI_TILT_ENABLED