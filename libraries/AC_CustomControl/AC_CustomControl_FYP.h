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

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
// Pitch loop (top path in your diagram)
AC_P   _p_angle_pitch;     // P-controller: Desired Pitch → Desired Pitch Rate
AC_PID _pid_pitch_rate;    // PID: Desired Pitch Rate → Desired Pitch Acc

// Position/X loop (bottom path)
AC_PID _pid_position_x;    // PID: Desired X-pos → Desired Velocity
AC_PID _pid_velocity_x;    // PID: Desired Velocity → Desired Acc

// Control allocation outputs
float _front_thrust;
float _rear_thrust;
float _tilt_angle;
};

#endif  // AP_CUSTOMCONTROL_FYP_ENABLED