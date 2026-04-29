#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_FYP_ENABLED

#include "AC_CustomControl_Backend.h"
#include <AC_PID/AC_P.h>
#include <AC_PID/AC_PID.h>

// Forward declaration of the custom motor mixer.
class AP_MotorsTricopterFYP;

class AC_CustomControl_FYP : public AC_CustomControl_Backend {
public:
    AC_CustomControl_FYP(AC_CustomControl& frontend,
                         AP_AHRS_View*& ahrs,
                         AC_AttitudeControl*& att_control,
                         AP_MotorsMulticopter*& motors,
                         float dt);

    Vector3f update(void) override;
    void reset(void) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // ---------------------------------------------------------------
    // Outer-loop tunables  (appear as CC3_* in GCS when CC_TYPE = 3)
    // ---------------------------------------------------------------
    AP_Float _kp_angle_pitch;
    AP_Float _rate_p, _rate_i, _rate_d, _rate_imax;
    AP_Float _pos_p,  _pos_i,  _pos_d,  _pos_imax;
    AP_Float _vel_p,  _vel_i,  _vel_d,  _vel_imax;

    // Physical parameters (match allocation equations).
    AP_Float _mass;          // kg
    AP_Float _inertia_pitch; // kg*m^2
    AP_Float _arm_L;         // m
    AP_Float _c_tail;        // dimensionless

    AP_Int8  _pos_loop_enable;

    // ---------------------------------------------------------------
    // Controllers
    // ---------------------------------------------------------------
    AC_P   _p_angle_pitch;
    AC_PID _pid_pitch_rate;
    AC_PID _pid_position_x;
    AC_PID _pid_velocity_x;

    // Most recent allocation outputs (kept for logging + mixer handoff).
    float _last_Tf;
    float _last_Tr;
    float _last_alpha;
    float _last_pitch_acc;
    float _last_ax_sim;

    float _dt;
};

#endif  // AP_CUSTOMCONTROL_FYP_ENABLED
