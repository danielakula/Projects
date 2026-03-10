#pragma once
#include <Arduino.h>

struct InterchipPacket {
    float target2;
    float target3;
    uint32_t faultCode;  
    
    // New Tuning & State Variables
    float Kp_outer;
    float Ki_outer;
    float k1;
    float k2;
    float k3;
    uint8_t robotState;  // 0 = disabled, 1 = active
    uint8_t targetEdge;  // e.g., 1, 2, or 3

    float motor2Velocity;
    float motor3Velocity;
    float motor2Current;
    float motor3Current;
    
    // Orientation
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float BASE_ALPHA;
    float ACCEL_TOLERANCE;
} __attribute__((packed));