#include "LQR.h"

LQR::LQR() {
    // 1. PLACE YOUR MATLAB K-MATRIX GAINS HERE!
    // K[1] = -56.9780f;  // Cube Angle (Theta)
    // K[2] = -5.5464f;  // Cube Velocity (Theta_dot)
    // K[3] = -0.0826f;  // Wheel Velocity    
    K[1] = -75.6944f;  // Cube Angle (Theta)
    K[2] = -6.0373f;  // Cube Velocity (Theta_dot)
    K[3] = -0.0817f;  // Wheel Velocity

    //K[4] = 0.0005f;      
    wheel_integral = 0.0f;
    
    // FIXED: Safe default. Update this from main.cpp using setCurrentLimit()
    current_limit = MotorTuning.current_limit; 

    friction_comp = 0.0f;
}

void LQR::setFrictionComp(float comp) {
    friction_comp = comp;
}

void LQR::setGains(float k1, float k2, float k3) {
    K[1] = k1;
    K[2] = k2;
    K[3] = k3;
}

void LQR::setCurrentLimit(float max_current) { // FIXED: Variable names match header
    current_limit = max_current;
}

void LQR::resetFilter() {
    float xf = 0;
}

void LQR::resetIntegral() {
    wheel_integral = 0.0f;
}

float LQR::compute(float theta, float theta_dot, float wheel_vel, float dt) {

    if (std::abs(theta) < 0.10f) {
        xf = (1.0f - xf_alpha) * xf + (xf_alpha * theta);
    }

    const float MAX_OFFSET = 0.052f;
    if (xf > MAX_OFFSET) xf = MAX_OFFSET;
    if (xf < -MAX_OFFSET) xf = -MAX_OFFSET;

    float unbiased_theta = theta - xf;

    wheel_integral += (wheel_vel * dt);
    
    // Anti-windup for safety
    if (wheel_integral > 500.0f) wheel_integral = 500.0f;
    if (wheel_integral < -500.0f) wheel_integral = -500.0f;

    float u = -(K[1] * unbiased_theta + K[2] * theta_dot + K[3] * wheel_vel ); // + K[4] * wheel_integral;

    if (u > 0.001f) {
        u += friction_comp;
    } else if (u < -0.001f) {
        u -= friction_comp;
    } else {
        u = 0.0f; // Absolute dead center, command zero.
    }

    // 4. Saturation
    if (u > current_limit) u = current_limit;
    if (u < -current_limit) u = -current_limit;

    return u;
}