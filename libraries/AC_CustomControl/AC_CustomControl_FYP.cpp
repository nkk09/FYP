#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_FYP_ENABLED

#include "AC_CustomControl_FYP.h"
#include "../AP_Motors/AP_MotorsTricopterFYP.h"   // adjust if you place the
                                                  // mixer elsewhere
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <SRV_Channel/SRV_Channel.h>

/*
   AC_CustomControl_FYP
   --------------------
   Full tilt-tricopter longitudinal controller matching the block
   diagram and allocation equations from the FYP Pitch_Controller
   document. Four cascaded control loops plus a physics-based
   allocation:

     x_pos -> [PID] -> vel_sim -> [PID] -> ax_sim       (position path)
     pitch -> [ P ] -> pitch_rate -> [PID] -> pitch_acc (attitude path)

     Control allocation (Simulink math):
       Mreq  = I * pitch_acc
       Rx    = m * ax_sim
       Ry    = m * g
       A     = Mreq - L(1-c)(Ry cos(theta) + Rx sin(theta))
       B     = -L(1-c) Ry sin(theta) + L(1-c) Rx cos(theta)
               + cL(Rx cos(theta) - Ry sin(theta))
       alpha = atan2(-B, A)
       Tf    = (Rx cos(theta) - Ry sin(theta)) / det
       Tr    = (Ry sin(alpha+theta) - Rx cos(alpha+theta)) / det
       where det = sin(alpha+theta)cos(theta) - sin(theta)cos(alpha+theta).

   COORDINATE FRAMES:
     ArduPilot "real" frame: +X forward (NED projection to body)
     Simulink model "sim" frame: +X LEFT (the allocation was derived
       using this orientation).
     The conversion happens at the input of the position loop:
       current_x_sim  = -current_x_real
       current_vx_sim = -current_vx_real
       target_x_sim   = -target_x_real
     Pitch angle keeps ArduPilot convention (nose up = positive).

   OUTPUTS:
     - pitch axis override via Vector3f return (handed back to the
       standard mixer when CC_AXIS_MASK bit 1 is set). Kept so that
       ArduPilot's rate controller stays consistent with the GCS PFD.
     - Tf, Tr, alpha, roll_cmd passed to AP_MotorsTricopterFYP, which
       actually writes the motor and tilt-servo PWMs.
     - CFYP log message for post-flight analysis.
*/

const AP_Param::GroupInfo AC_CustomControl_FYP::var_info[] = {

    // Outer pitch angle -> rate P gain (P-only, from LongitudinalController.h)
    AP_GROUPINFO("ANG_P_PIT", 1, AC_CustomControl_FYP, _kp_angle_pitch, 3.0f),

    // Inner pitch rate PID — top block from pidconstants.pdf
    AP_GROUPINFO("RATE_P",    2, AC_CustomControl_FYP, _rate_p,    22.0334966821808f),
    AP_GROUPINFO("RATE_I",    3, AC_CustomControl_FYP, _rate_i,    45.3902568226565f),
    AP_GROUPINFO("RATE_D",    4, AC_CustomControl_FYP, _rate_d,    -0.163263956378754f),
    AP_GROUPINFO("RATE_IMAX", 5, AC_CustomControl_FYP, _rate_imax, 0.5f),

    // Position -> velocity PID — bottom block from pidconstants.pdf
    AP_GROUPINFO("POS_P",     6, AC_CustomControl_FYP, _pos_p,    1.19580143744922f),
    AP_GROUPINFO("POS_I",     7, AC_CustomControl_FYP, _pos_i,    0.274728927037584f),
    AP_GROUPINFO("POS_D",     8, AC_CustomControl_FYP, _pos_d,    -0.00589313506125274f),
    AP_GROUPINFO("POS_IMAX",  9, AC_CustomControl_FYP, _pos_imax, 2.0f),

    // Velocity -> acceleration PID — same tuning run as position
    AP_GROUPINFO("VEL_P",     10, AC_CustomControl_FYP, _vel_p,    1.19580143744922f),
    AP_GROUPINFO("VEL_I",     11, AC_CustomControl_FYP, _vel_i,    0.274728927037584f),
    AP_GROUPINFO("VEL_D",     12, AC_CustomControl_FYP, _vel_d,    -0.00589313506125274f),
    AP_GROUPINFO("VEL_IMAX",  13, AC_CustomControl_FYP, _vel_imax, 5.0f),

    // Physical parameters (keep in sync with AP_MotorsTricopterFYP
    // and the real airframe)
    AP_GROUPINFO("MASS",      14, AC_CustomControl_FYP, _mass,          4.9f),
    AP_GROUPINFO("INERTIA",   15, AC_CustomControl_FYP, _inertia_pitch, 0.158f),
    AP_GROUPINFO("ARM_L",     16, AC_CustomControl_FYP, _arm_L,         0.459f),
    AP_GROUPINFO("C_TAIL",    17, AC_CustomControl_FYP, _c_tail,        0.1f),

    AP_GROUPINFO("POS_EN",    18, AC_CustomControl_FYP, _pos_loop_enable, 0),

    AP_GROUPEND
};

AC_CustomControl_FYP::AC_CustomControl_FYP(AC_CustomControl& frontend,
                                           AP_AHRS_View*& ahrs,
                                           AC_AttitudeControl*& att_control,
                                           AP_MotorsMulticopter*& motors,
                                           float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _p_angle_pitch(3.0f),
    _pid_pitch_rate(22.0334966821808f, 45.3902568226565f, -0.163263956378754f,
                    0.0f, 0.5f, 10.0f, 0.0f, 5.83f, dt),
    _pid_position_x(1.19580143744922f, 0.274728927037584f, -0.00589313506125274f,
                    0.0f, 2.0f, 2.0f, 0.0f, 2.0f, dt),
    _pid_velocity_x(1.19580143744922f, 0.274728927037584f, -0.00589313506125274f,
                    0.0f, 5.0f, 5.0f, 2.0f, 2.0f, dt),
    _last_Tf(0), _last_Tr(0), _last_alpha(0),
    _last_pitch_acc(0), _last_ax_sim(0),
    _dt(dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

Vector3f AC_CustomControl_FYP::update(void)
{
    // =================================================================
    // 1. SENSOR READS
    // =================================================================
    const float current_pitch      = -_ahrs->pitch;
    const float current_pitch_rate = -_ahrs->get_gyro_latest().y;

    Vector3f position_NED, velocity_NED;
    if (!_ahrs->get_relative_position_NED_home(position_NED)) {
        position_NED.zero();
    }
    if (!_ahrs->get_velocity_NED(velocity_NED)) {
        velocity_NED.zero();
    }
    // In NED, +x is North -> for body-forward position we really want the
    // projection onto the nose direction. For the simulation/FYP test
    // assume the drone starts facing North so NED.x == body forward.
    const float current_x_real  = position_NED.x;
    const float current_vx_real = velocity_NED.x;

    // =================================================================
    // 2. COORDINATE FLIP (ArduPilot real frame -> FYP sim frame)
    // =================================================================
    const float current_x_sim  = -current_x_real;
    const float current_vx_sim = -current_vx_real;

    // Attitude target from the AC_AttitudeControl outer loops.
    const float target_pitch_real = 60;
    //    -(_att_control->get_attitude_target_quat().get_euler_pitch());

    // Hold hover (x=0) unless a higher-level mode writes to a setpoint.
    // A future extension can pull a waypoint here.
    const float target_x_sim = 0.0f;

    // =================================================================
    // 3. POSITION / VELOCITY LOOPS
    // =================================================================
    float ax_sim = 0.0f;
    if (_pos_loop_enable != 0) {
        // Keep live gain sync
        _pid_position_x.kP()    = _pos_p;
        _pid_position_x.kI()    = _pos_i;
        _pid_position_x.kD()    = _pos_d;
        _pid_position_x.kIMAX() = _pos_imax;
        _pid_velocity_x.kP()    = _vel_p;
        _pid_velocity_x.kI()    = _vel_i;
        _pid_velocity_x.kD()    = _vel_d;
        _pid_velocity_x.kIMAX() = _vel_imax;

        const float target_vel_sim  = _pid_position_x.update_all(target_x_sim, current_x_sim, _dt);
        ax_sim = _pid_velocity_x.update_all(target_vel_sim, current_vx_sim, _dt);
    } else {
        _pid_position_x.reset_I();
        _pid_velocity_x.reset_I();
    }

    // =================================================================
    // 4. ATTITUDE LOOPS (pitch only)
    // =================================================================
    _p_angle_pitch.set_kP(_kp_angle_pitch);
    _pid_pitch_rate.kP()    = _rate_p;
    _pid_pitch_rate.kI()    = _rate_i;
    _pid_pitch_rate.kD()    = _rate_d;
    _pid_pitch_rate.kIMAX() = _rate_imax;

    const float angle_error      = target_pitch_real - current_pitch;
    const float target_pitch_rate = _p_angle_pitch.get_p(angle_error);
    const float pitch_acc = _pid_pitch_rate.update_all(
        target_pitch_rate, current_pitch_rate, _dt);

    // =================================================================
    // 5. CONTROL ALLOCATION
    //    (Simulink A/B/alpha/Tf/Tr math from the doc)
    // =================================================================
    const float m     = _mass;
    const float I     = _inertia_pitch;
    const float L     = _arm_L;
    const float c     = _c_tail;
    const float theta = current_pitch;   // use actual pitch for allocation
    const float Mreq  = I * pitch_acc;
    const float Rx    = m * ax_sim;
    const float Ry    = m * GRAVITY_MSS;

    const float sT = sinf(theta);
    const float cT = cosf(theta);

    const float A = Mreq
                  - L * (1.0f - c) * (Ry * cT)
                  - L * (1.0f - c) * (Rx * sT);

    const float B = -L * (1.0f - c) * Ry * sT
                  +  L * (1.0f - c) * Rx * cT
                  +  c * L * (Rx * cT - Ry * sT);

    float alpha = atan2f(-B, A);

    // Matrix solve for Tf, Tr
    const float sAT = sinf(alpha + theta);
    const float cAT = cosf(alpha + theta);
    const float det = sAT * cT - sT * cAT;

    float Tf, Tr;
    if (fabsf(det) < 0.001f) {
        // Singular (happens at hover: alpha≈π gives det≈0).
        // Fall back to pure vertical split and reset alpha to 0 so servos
        // stay vertical instead of commanding 180-degree deflection.
        Tf    = Ry * (1.0f - c);
        Tr    = Ry * c;
        alpha = 0.0f;
    } else {
        Tf = (Rx * cT - Ry * sT) / det;
        Tr = (Ry * sAT - Rx * cAT) / det;
    }

    // =================================================================
    // 6. HAND OFF TO CUSTOM MOTOR MIXER
    // =================================================================
    // Roll passthrough from ArduPilot's attitude controller (it runs
    // ahead of us and already computed a roll command). Cast the
    // AP_MotorsMulticopter pointer to our subclass; if the cast fails
    // this just means the user hasn't switched the frame — we silently
    // skip the direct-write path and fall back to pitch-only mode.
    AP_MotorsTricopterFYP* fyp_motors =
        dynamic_cast<AP_MotorsTricopterFYP*>(_motors);

    if (fyp_motors != nullptr) {
        fyp_motors->set_fyp_targets(Tf, Tr, alpha);
    }

    _last_Tf        = Tf;
    _last_Tr        = Tr;
    _last_alpha     = alpha;
    _last_pitch_acc = pitch_acc;
    _last_ax_sim    = ax_sim;

    // =================================================================
    // 7. LOG + RETURN
    // =================================================================
    AP::logger().WriteStreaming(
        "CFYP",
        "TimeUS,PTgt,PAct,PAcc,AxSim,Tf,Tr,Alpha,PosX,VelX",
        "Qfffffffff",
        AP_HAL::micros64(),
        target_pitch_real,
        current_pitch,
        pitch_acc,
        ax_sim,
        Tf, Tr, alpha,
        current_x_real,
        current_vx_real);

    // Rate-limited GCS text for live console visibility (~2 Hz)
    static uint16_t _gcs_ctr = 0;
    if (++_gcs_ctr >= 200) {
        _gcs_ctr = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
            "CC pos=%.2fm vel=%.2fm/s pit=%.1fdeg Tf=%.1f Tr=%.1f a=%.1fdeg",
            (double)current_x_real,
            (double)current_vx_real,
            (double)(current_pitch * RAD_TO_DEG),
            (double)Tf, (double)Tr,
            (double)(alpha * RAD_TO_DEG));
    }

    // Normalized pitch axis output for the standard mixer fallback.
    const float pitch_out = constrain_float(pitch_acc / 10.0f, -1.0f, 1.0f);
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

    _last_Tf = _last_Tr = _last_alpha = 0.0f;
    _last_pitch_acc = _last_ax_sim = 0.0f;

    AP_MotorsTricopterFYP* fyp_motors =
        dynamic_cast<AP_MotorsTricopterFYP*>(_motors);
    if (fyp_motors != nullptr) {
        fyp_motors->set_fyp_targets(0.0f, 0.0f, 0.0f);
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 0);
}

#endif  // AP_CUSTOMCONTROL_FYP_ENABLED
