#include "Copter.h"

#include "Copter.h"

static constexpr uint8_t  TEST_SERVO_CH   = 10;
static constexpr uint32_t SERVO_PERIOD_MS = 1000;


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    SRV_Channels::set_output_pwm_chan(LED_SERVO_CH, LED_PWM_ON);
}
#endif
// FMU Aux PWM Out 1 -> servo 9 (1-based) -> index 8 (0-based)
static constexpr uint8_t  LED_SERVO_CH   = 8;
static constexpr uint16_t LED_PWM_OFF    = 1100; // pulse for "off" (LED will be dim/off)
static constexpr uint16_t LED_PWM_ON     = 1900; // pulse for "on"  (LED will be bright)
static constexpr uint32_t LED_PERIOD_MS  = 500;  // toggle every 500ms

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    static uint32_t last_toggle = 0;
    static bool led_on = false;

    const uint32_t now = AP_HAL::millis();
    if (now - last_toggle >= LED_PERIOD_MS) {
        last_toggle = now;
        led_on = !led_on;

        // apply PWM override and push to hardware immediately
        auto &srv = AP::srv();
        srv.cork();
        SRV_Channels::set_output_pwm_chan(LED_SERVO_CH, led_on ? LED_PWM_ON : LED_PWM_OFF);
        srv.push();
    }
}
#endif
//     // put your 100Hz code here
// {
//     static uint32_t last_toggle = 0;
//     static bool led_on = false;

//     const uint32_t now = AP_HAL::millis();
//     if (now - last_toggle >= LED_PERIOD_MS) {
//         led_on = !led_on;
//         last_toggle = now;
//     }

//     SRV_Channels::set_output_pwm_chan(LED_SERVO_CH,
//                                       led_on ? LED_PWM_ON : LED_PWM_OFF);
// }
//     static uint32_t last_toggle = 0;
//     static bool high = false;

//     const uint32_t now = AP_HAL::millis();
//     if (now - last_toggle >= SERVO_PERIOD_MS) {
//         last_toggle = now;
//         high = !high;

//         const uint16_t pwm = high ? 1900 : 1100;   // two extreme positions
//         SRV_Channels::set_output_pwm_chan(TEST_SERVO_CH, pwm);
//     }
// }
// #endif


#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    static uint32_t last_ms = 0;
    const uint32_t now = AP_HAL::millis();
    if (now - last_ms >= 5000 && gcs().any_connected()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Hello FYP TEAM");
        last_ms = now;
    }
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
