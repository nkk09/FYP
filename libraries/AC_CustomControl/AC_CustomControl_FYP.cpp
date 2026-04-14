#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_FYP_ENABLED

#include "AC_CustomControl_FYP.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>

// Scale factor: converts tilt angle (radians) to SRV_Channels scaled output units.
// SRV_Channels uses -4500..+4500 centidegrees, so multiply radians by rad->centideg.
static constexpr float TILT_SCALE    = degrees(1.0f) * 100.0f;  // rad -> centidegrees
static constexpr float K_PITCH       = 0.1f;    // pitch differential thrust gain
static constexpr float MAX_PITCH_ACC = 10.0f;   // rad/s^2, normalisation for mixer

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_FYP::var_info[] = {

    AP_GROUPINFO("ANG_P_PIT", 1, AC_CustomControl_FYP, _kp_angle_pitch,      4.0f),
    AP_GROUPINFO("PIT_RMAX",  2, AC_CustomControl_FYP, _pitch_rate_max_cdss, 20000.0f),
    AP_GROUPINFO("PIT_OMAX",  3, AC_CustomControl_FYP, _pitch_out_max,       0.5f),
    AP_GROUPINFO("RATE_P",    4, AC_CustomControl_FYP, _rate_p,              0.10f),
    AP_GROUPINFO("RATE_I",    5, AC_CustomControl_FYP, _rate_i,              0.01f),
    AP_GROUPINFO("RATE_D",    6, AC_CustomControl_FYP, _rate_d,              0.00f),
    AP_GROUPINFO("RATE_IMAX", 7, AC_CustomControl_FYP, _rate_imax,           0.30f),

    AP_GROUPEND
};

AC_CustomControl_FYP::AC_CustomControl_FYP(AC_CustomControl& frontend,
                                           AP_AHRS_View*& ahrs,
                                           AC_AttitudeControl*& att_control,
                                           AP_MotorsMulticopter*& motors,
                                           float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _p_angle_pitch(4.0f),
    _pid_pitch_rate(0.10f, 0.01f, 0.0f, 0.0f, 0.3f,  10.0f, 0.0f, 10.0f, dt),
    _pid_position_x(1.0f,  0.0f,  0.0f, 0.0f, 1.0f,  2.0f,  0.0f, 2.0f,  dt),
    _pid_velocity_x(0.5f,  0.1f,  0.05f,0.0f, 0.5f,  5.0f,  2.0f, 5.0f,  dt),
    _front_thrust(0.0f),
    _rear_thrust(0.0f),
    _tilt_angle(0.0f),
    _dt(dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

Vector3f AC_CustomControl_FYP::update(void)
{
    // ---------------------------------------------------------------
    // PITCH LOOP (top path of block diagram)
    // ---------------------------------------------------------------

    float current_pitch      = _ahrs->pitch;
    float current_pitch_rate = _ahrs->get_gyro_latest().y;

    float desired_pitch = _att_control->get_attitude_target_quat().get_euler_pitch();

    // Sync AC_P gain from tunable AP_Float each cycle
    _p_angle_pitch.set_kP(_kp_angle_pitch);
    float desired_pitch_rate = _p_angle_pitch.get_p(desired_pitch - current_pitch);

    float desired_pitch_acc = _pid_pitch_rate.update_all(
        desired_pitch_rate, current_pitch_rate, _dt);

    // ---------------------------------------------------------------
    // POSITION / VELOCITY LOOP (bottom path of block diagram)
    // ---------------------------------------------------------------

    Vector3f position;
    Vector3f velocity;

    if (!_ahrs->get_relative_position_NED_home(position)) {
        position.zero();
    }

    if (!_ahrs->get_velocity_NED(velocity)) {
        velocity.zero();
    }

    // Hold home (0) by default; replace with waypoint/GCS target when available
    float desired_x_pos = 0.0f;

    float desired_vel = _pid_position_x.update_all(
        desired_x_pos, position.x, _dt);

    float desired_acc = _pid_velocity_x.update_all(
        desired_vel, velocity.x, _dt);

    // ---------------------------------------------------------------
    // CONTROL ALLOCATION
    // ---------------------------------------------------------------

    float total_thrust = _motors->get_throttle();

    _front_thrust = total_thrust / 2.0f + desired_pitch_acc * K_PITCH;
    _rear_thrust  = total_thrust / 2.0f - desired_pitch_acc * K_PITCH;

    if (is_positive(total_thrust)) {
        _tilt_angle = asinf(constrain_float(
            desired_acc / (total_thrust * 9.81f), -1.0f, 1.0f));
    } else {
        _tilt_angle = 0.0f;
    }

    // ---------------------------------------------------------------
    // MOTOR MIXER OUTPUT
    // ---------------------------------------------------------------

    float pitch_out = constrain_float(desired_pitch_acc / MAX_PITCH_ACC,
                                      -_pitch_out_max, _pitch_out_max);

    _motors->set_pitch(pitch_out);
    _motors->set_throttle(total_thrust);

    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt,
                                    _tilt_angle * TILT_SCALE);

    return Vector3f(0.0f, pitch_out, 0.0f);
}

void AC_CustomControl_FYP::reset(void)
{
    _pid_pitch_rate.reset_filter();
    _pid_pitch_rate.reset_I();
    _pid_position_x.reset_filter();
    _pid_position_x.reset_I();
    _pid_velocity_x.reset_filter();
    _pid_velocity_x.reset_I();
    _front_thrust = 0.0f;
    _rear_thrust  = 0.0f;
    _tilt_angle   = 0.0f;
}

#endif  // AP_CUSTOMCONTROL_FYP_ENABLED