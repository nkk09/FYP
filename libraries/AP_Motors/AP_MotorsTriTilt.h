/// @file    AP_MotorsTriTilt.h
/// @brief   Motor control class for 2-tilt front + fixed tail tricopter

#pragma once

#include "AP_Motors_config.h"

#if AP_MOTORS_TRI_TILT_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMulticopter.h"

/// @class AP_MotorsTriTilt
/// Layout (ArduPilot motor numbers):
///  - AP_MOTORS_MOT_1 : Front Right motor (tilt servo = SCRIPTING1)
///  - AP_MOTORS_MOT_2 : Front Left  motor (tilt servo = SCRIPTING2)
///  - AP_MOTORS_MOT_4 : Tail motor (fixed, no servo)
class AP_MotorsTriTilt : public AP_MotorsMulticopter {
public:
    AP_MotorsTriTilt(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {}

    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;
    void set_update_rate(uint16_t speed_hz) override;

    void output_to_motors() override;
    uint32_t get_motor_mask() override;

    void output_motor_mask(float thrust, uint32_t mask, float rudder_dt) override;
    float get_roll_factor(uint8_t i) override;

    bool arming_checks(size_t buflen, char *buffer) const override;

protected:
    void output_armed_stabilizing() override;
    void thrust_compensation(void) override;

    const char* _get_frame_string() const override { return "TRI_TILT_2F"; }
    const char* get_type_string() const override { return _pitch_reversed ? "pitch-reversed" : ""; }

    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

private:
    // tilt servos are mapped via SERVOx_FUNCTION = SCRIPTING1 / SCRIPTING2
    static constexpr SRV_Channel::Function TILT_RIGHT_FN = SRV_Channel::k_scripting1;
    static constexpr SRV_Channel::Function TILT_LEFT_FN  = SRV_Channel::k_scripting2;

    static constexpr float TILT_MAX_RAD = 0.70f; // ~40deg max mechanical tilt (adjust)

    // motor thrusts (0..1)
    float _thrust_right = 0.0f;
    float _thrust_left  = 0.0f;
    float _thrust_rear  = 0.0f;

    // tilt angles (rad)
    float _tilt_right = 0.0f;
    float _tilt_left  = 0.0f;

    bool _pitch_reversed = false;

    // helper: radians -> scaled [-1000..1000]
    static int16_t tilt_to_scaled(float tilt_rad);
};

#endif // AP_MOTORS_TRI_TILT_ENABLED