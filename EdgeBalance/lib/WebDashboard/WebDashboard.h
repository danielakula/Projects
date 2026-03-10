#ifndef WEBDASHBOARD_H
#define WEBDASHBOARD_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// Struct to hold tuning parameters (Written by Core 0, Read by Core 1)
struct ControlParams {
    float lqr_weights[4];
    int state;       // 0 = disabled, 1 = balancing
    int target_edge;
};

// Struct to hold robot state (Written by Core 1, Read by Core 0)
struct TelemetryData {
    float pitch;
    int motor_rpm[2];
};

class WebDashboard {
public:
    WebDashboard(const char* ssid, const char* password);
    
    // Call this in setup()
    void begin(); 

    // Used by your control loop (Core 1) to update the dashboard
    void updateTelemetry(float pitch, int rpmA, int rpmB);

    // Used by your control loop (Core 1) to get the latest tuning weights
    ControlParams getLatestParams();

private:
    const char* _ssid;
    const char* _password;
    
    AsyncWebServer _server;
    AsyncWebSocket _ws;

    // FreeRTOS task handles
    TaskHandle_t _telemetryTaskHandle;

    // Shared data and Mutexes for Thread Safety
    ControlParams _currentParams;
    TelemetryData _currentTelemetry;
    SemaphoreHandle_t _dataMutex;

    // Internal methods
    void initWiFi();
    void initLittleFS();
    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
    
    // Static wrapper for the FreeRTOS task
    static void telemetryTaskWrapper(void* parameter);
    void telemetryTask();
};

#endif