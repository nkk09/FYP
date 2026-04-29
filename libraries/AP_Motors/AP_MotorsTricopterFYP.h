/*
   Custom motor mixer for the FYP tilt-tricopter airframe.

   Layout (matches AC_CustomControl_FYP allocation):
     Motor 1 (SRV_Channel k_motor1) = Front LEFT  TOP
     Motor 2 (SRV_Channel k_motor2) = Front LEFT  BOT
     Motor 3 (SRV_Channel k_motor3) = Front RIGHT TOP
     Motor 4 (SRV_Channel k_motor4) = Front RIGHT BOT
     Motor 5 (SRV_Channel k_motor5) = Tail UP-pusher   (throttle when Tr >= 0)
     Motor 6 (SRV_Channel k_motor6) = Tail DOWN-pusher (throttle when Tr <  0)
     Servo 7 (SRV_Channel k_motor_tilt)      = Left  tilt  (pwm_L = 1500 + offset)
     Servo 8 (SRV_Channel k_tiltMotorRight)  = Right tilt  (pwm_R = 1500 - offset)

   Unlike AP_MotorsMatrix this class does NOT perform classical
   roll/pitch/yaw/throttle mixing. The allocation is done upstream in
   AC_CustomControl_FYP (which computes Tf, Tr, alpha). This class
   is just the thin glue between those physical targets and the
   SRV_Channel outputs.
*/
#pragma once

#include <AP_Motors/AP_MotorsMulticopter.h>

class AP_MotorsTricopterFYP : public AP_MotorsMulticopter {
public:
    // inherit constructor of AP_MotorsMulticopter
    using AP_MotorsMulticopter::AP_MotorsMulticopter;

    // init - performs any required initialisation for this frame.
    // Called by AP_MotorsMulticopter::set_frame_class_and_type.
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // output - pushes the commanded values to the SRV_Channels layer.
    void output_to_motors() override;

    // External inputs from AC_CustomControl_FYP. Called every loop.
    // Tf_newtons: total front-pair thrust (Newtons)
    // Tr_newtons: total tail thrust (Newtons, signed)
    // alpha_rad:  tilt angle (radians)
    // Roll command is NOT passed here — it is read from _roll_in which the
    // main ArduPilot attitude controller populates via set_roll() before
    // output_armed_stabilizing() runs (CC_AXIS_MASK leaves roll to default).
    void set_fyp_targets(float Tf_newtons,
                         float Tr_newtons,
                         float alpha_rad);

    // These are required pure-virtuals that we must provide.
    void set_frame_class_and_type(motor_frame_class frame_class,
                                  motor_frame_type  frame_type) override;

    // tell motors to stop
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override {}

    const char* _get_frame_string() const override { return "TRICOPTER_FYP"; }

    // mask of which outputs are being used for motors
    uint32_t get_motor_mask() override;

protected:
    // standard overrides — many are no-ops because we don't use
    // the conventional mixer.
    void output_armed_stabilizing() override;

    // Cached FYP targets, updated by set_fyp_targets().
    float _fyp_Tf_N       = 0.0f;
    float _fyp_Tr_N       = 0.0f;
    float _fyp_alpha_rad  = 0.0f;
    // _roll_in from base class AP_Motors is used directly for roll trim.
    float _thrust_rpyt_out[AP_MOTORS_MAX_NUM_MOTORS] = {};

  // frame class and type tracking
  motor_frame_class   _active_frame_class = MOTOR_FRAME_UNDEFINED;
  motor_frame_type    _active_frame_type  = MOTOR_FRAME_TYPE_PLUS;

    // Physical constants
    static constexpr float COAX_FACTOR     = 1.6f;
    static constexpr float MAX_MOT_NEWTONS = 30.0f;

    uint16_t _log_counter = 0;
};
