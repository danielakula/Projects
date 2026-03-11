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

    friction_comp = 0.1f;
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

    float u = -(K[1] * theta + K[2] * theta_dot + K[3] * wheel_vel ); // + K[4] * wheel_integral;

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