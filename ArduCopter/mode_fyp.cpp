// Physical parameters (from Simulink)
const float m = 4.9f;
const float g = 9.81f;
const float W = m * g;

const float L = 0.459f;
const float c = 0.1f;
const float I = 0.158f;

// Control gains (tune these)
float Kp_angle = 5.0f;      // outer loop P
float Kp_rate  = 0.8f;      // inner PID
float Ki_rate  = 0.1f;
float Kd_rate  = 0.02f;
float rate_integrator = 0.0f;
float prev_rate_error = 0.0f;
void update_control(float theta, float theta_dot, float theta_des, float dt)
{
    // =====================================================
    // 1 OUTER LOOP — Angle P controller
    // =====================================================
    float theta_error = theta_des - theta;
    float theta_dot_des = Kp_angle * theta_error;

    // =====================================================
    // 2 INNER LOOP — Rate PID
    // Outputs Tr directly
    // =====================================================
    float rate_error = theta_dot_des - theta_dot;

    rate_integrator += rate_error * dt;
    float rate_derivative = (rate_error - prev_rate_error) / dt;

    float Tr = Kp_rate * rate_error
             + Ki_rate * rate_integrator
             + Kd_rate * rate_derivative;

    prev_rate_error = rate_error;

    // Saturate rear thrust
    Tr = constrain(Tr, 0.0f, 40.0f);   // adjust max thrust as needed

    // =====================================================
    // 3 Compute alpha using your exact physics
    // (ax=0, ay=0, no disturbances)
    // =====================================================

    float num = -Tr * sinf(theta);
    float den =  W - Tr * cosf(theta);

    float alpha = atan2f(num, den) - theta;

    // =====================================================
    // 4 Solve Tf from vertical force equation
    // =====================================================

    float Tf = (W - Tr * cosf(theta)) / cosf(alpha + theta);

    Tf = constrain(Tf, 0.0f, 40.0f);

    // =====================================================
    // 5 Convert thrusts to PWM
    // =====================================================
    float pwm_rear  = thrust_to_pwm(Tr);
    float pwm_front = thrust_to_pwm(Tf);

    float servo_pwm = angle_to_servo_pwm(alpha);

    send_pwm(REAR_MOTOR, pwm_rear);
    send_pwm(FRONT_MOTOR, pwm_front);
    send_pwm(SERVO, servo_pwm);
}
// EXAMPLE CONVERSION FUNCTIONS
float thrust_to_pwm(float T)
{
    float T_min = 0.0f;
    float T_max = 40.0f;

    float pwm_min = 1000.0f;
    float pwm_max = 2000.0f;

    return pwm_min + (T - T_min) * (pwm_max - pwm_min) / (T_max - T_min);
}
float angle_to_servo_pwm(float alpha)
{
    float alpha_max = 0.5f;   // radians (~28 deg)
    float pwm_center = 1500.0f;
    float pwm_range  = 400.0f;

    return pwm_center + (alpha / alpha_max) * pwm_range;
}
