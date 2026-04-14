#include "Copter.h"

#if MODE_FYP_ENABLED

bool ModeFYP::init(bool ignore_checks)
{
    return true;
}

void ModeFYP::run()
{
    const float pitch_stick = channel_pitch->norm_input_dz();
    const float roll_stick  = channel_roll->norm_input_dz();
    const float yaw_stick   = channel_yaw->norm_input_dz();

    const float MAX_PITCH_CD = 18000.0f;   // if you truly want ±180°
    const float MAX_ROLL_CD  = 18000.0f;   

    const float pitch_target_cd = pitch_stick * MAX_PITCH_CD;
    const float roll_target_cd  = roll_stick  * MAX_ROLL_CD;
    const float yaw_rate_cds    = yaw_stick * 1500.0f;

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(
        roll_target_cd,
        pitch_target_cd,
        yaw_rate_cds
    );

    // Read targets directly from attitude controller state.
    const float pitch_target_from_ac_cd = attitude_control->get_att_target_euler_cd().y;
    const float pitch_rate_target_rads  = attitude_control->get_attitude_target_ang_vel().y;
    const float pitch_rate_current_rads = ahrs.get_gyro().y;

    static uint32_t last_dbg_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_dbg_ms >= 1000U) {
        last_dbg_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "FYP tgt_pitch_cd=%.1f tgt_q_r=%.3f gyro_p=%.3f",
                        pitch_target_from_ac_cd,
                        pitch_rate_target_rads,
                        pitch_rate_current_rads);
    }
}
#endif