#include "WebDashboard.h"

// Initialize the server on port 80 and the WebSocket on "/ws"
WebDashboard::WebDashboard(const char* ssid, const char* password) 
    : _ssid(ssid), _password(password), _server(80), _ws("/ws") {
    
    // Initialize default parameters
    _currentParams = {{1.0, 0.5, 0.0, 0.0}, 0, 0};
    _currentTelemetry = {0.0, {0, 0}};
    
    // Create the mutex for thread-safe data sharing
    _dataMutex = xSemaphoreCreateMutex();
}

void WebDashboard::begin() {
    initLittleFS();
    initWiFi();

    // Serve the HTML file when the phone connects to the IP
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html");
    });

    // Handle WebSocket Events (using a lambda to capture 'this')
    _ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
        if (type == WS_EVT_DATA) {
            this->handleWebSocketMessage(arg, data, len);
        }
    });

    _server.addHandler(&_ws);
    _server.begin();
    Serial.println("Web Server Started.");

    // Start the Telemetry Broadcast Task pinned to Core 0
    xTaskCreatePinnedToCore(
        WebDashboard::telemetryTaskWrapper, 
        "Telemetry Task", 
        4096, 
        this, 
        1, 
        &_telemetryTaskHandle, 
        0 // Pinned to Core 0 (WiFi Core)
    );
}

void WebDashboard::handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    // A command arrived from the phone! 
    // Just forward the raw text directly to MCU 1 over Serial1
    Serial1.write(data, len); 
    Serial1.println(); // Add a newline so MCU 1 knows the message is done
}

// ---- Safe Data Getters & Setters for Core 1 to use ----

void WebDashboard::updateTelemetry(float pitch, int rpmA, int rpmB) {
    if (xSemaphoreTake(_dataMutex, 0)) { // 0 ticks to wait (don't block control loop)
        _currentTelemetry.pitch = pitch;
        _currentTelemetry.motor_rpm[0] = rpmA;
        _currentTelemetry.motor_rpm[1] = rpmB;
        xSemaphoreGive(_dataMutex);
    }
}

ControlParams WebDashboard::getLatestParams() {
    ControlParams paramsCopy;
    if (xSemaphoreTake(_dataMutex, 0)) {
        paramsCopy = _currentParams;
        xSemaphoreGive(_dataMutex);
    }
    return paramsCopy;
}

// ---- The Core 0 Telemetry Broadcast Task ----

void WebDashboard::telemetryTaskWrapper(void* parameter) {
    WebDashboard* dashboard = static_cast<WebDashboard*>(parameter);
    dashboard->telemetryTask();
}

void WebDashboard::telemetryTask() {
    StaticJsonDocument<128> doc;
    char buffer[128];

    for (;;) {
        // Only do work if a phone is actually connected
        if (_ws.count() > 0) {
            
            // Safely grab the latest telemetry
            if (xSemaphoreTake(_dataMutex, portMAX_DELAY)) {
                doc["angle_x"] = _currentTelemetry.pitch;
                doc["motor_speeds"][0] = _currentTelemetry.motor_rpm[0];
                doc["motor_speeds"][1] = _currentTelemetry.motor_rpm[1];
                xSemaphoreGive(_dataMutex);
            }

            // Pack into JSON and send to all connected phones
            size_t len = serializeJson(doc, buffer);
            _ws.textAll(buffer, len);
        }
        
        // Wait 50ms (Broadcast at 20Hz)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Boilerplate initialization
void WebDashboard::initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void WebDashboard::initLittleFS() {
    if (!LittleFS.begin(true)) {
        Serial.println("An Error has occurred while mounting LittleFS");
        return;
    }
}