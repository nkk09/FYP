#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_FYP_ENABLED

#include "AC_CustomControl_Backend.h"
#include <AC_PID/AC_P.h>
#include <AC_PID/AC_PID.h>

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
    // Tunable parameters (visible in GCS as CC3_ANG_P_PIT etc.)
    // ---------------------------------------------------------------
    AP_Float _kp_angle_pitch;       // outer P gain: pitch angle → rate
    AP_Float _pitch_rate_max_cdss;  // max desired pitch rate (rad/s * 100)
    AP_Float _pitch_out_max;        // clamp on normalised pitch output [-1,1]
    AP_Float _rate_p;               // inner pitch-rate PID gains
    AP_Float _rate_i;
    AP_Float _rate_d;
    AP_Float _rate_imax;

    // Position loop tuning
    AP_Float _pos_p;
    AP_Float _pos_i;
    AP_Float _pos_d;
    AP_Float _pos_imax;
    AP_Float _vel_p;
    AP_Float _vel_i;
    AP_Float _vel_d;
    AP_Float _vel_imax;

    // Control allocation constants
    AP_Float _k_pitch;              // maps pitch_acc to differential thrust
    AP_Float _tilt_scale;           // scales tilt angle to servo output units

    // ---------------------------------------------------------------
    // Runtime controller objects  (initialised in constructor)
    // ---------------------------------------------------------------
    // Pitch angle → rate (outer P)
    AC_P   _p_angle_pitch;

    // Pitch rate → motor output (inner PID)
    AC_PID _pid_pitch_rate;

    // X position → velocity (outer PID)
    AC_PID _pid_position_x;

    // X velocity → acceleration (inner PID)
    AC_PID _pid_velocity_x;

    // ---------------------------------------------------------------
    // Control allocation state
    // ---------------------------------------------------------------
    float _front_thrust;
    float _rear_thrust;
    float _tilt_angle;
    float _dt;              // timestep stored from constructor argument
};

#endif  // AP_CUSTOMCONTROL_FYP_ENABLED