#pragma once
#include <Arduino.h>
#include "Config.h"

class LQR {
public:
    // Constructor
    LQR();

    // The core calculation function
    float compute(float theta, float theta_dot, float wheel_vel, float dt);

    // Allows you to dynamically update gains
    void setGains(float k1, float k2, float k3);
    
    // Safety limit for the output torque/current
    void setCurrentLimit(float max_current); // FIXED: Renamed to match variable

    void setFrictionComp(float comp);

    void resetFilter(); 

    void resetIntegral();

private:
    // The LQR Gain Matrix
    float K[4]; 
    //float K[5];

    float wheel_integral;

    float xf_alpha = 0.0f;
    float xf = 0.0f;

    // Safety limit
    float current_limit;

    float friction_comp;
};