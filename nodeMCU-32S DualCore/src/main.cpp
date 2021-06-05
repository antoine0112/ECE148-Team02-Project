// Dependancies
#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Macros
#define RED_PIN 0
#define BLUE_PIN 15
#define HTTP_PORT 80

// WiFi Credentials
const char *WIFI_SSID = "ESP32";
const char *WIFI_PASS = "jetsonucsd";

// Button Component
struct Button {
    uint8_t pin;
    bool    on;

    // Update Button On Status
    void update() {digitalWrite(pin, on ? HIGH : LOW);}
};

// Button Instances
Button red_Button = {RED_PIN, false };
Button blue_Button = {BLUE_PIN, false };

// Server Instances
AsyncWebServer server(HTTP_PORT);
AsyncWebSocket ws("/ws");

// Task Handles
TaskHandle_t server_Handle;
TaskHandle_t pwm_Handle;

// Queue Handles
QueueHandle_t server2PWM_Handle;
int queueSize = 10;

// FiLL In HTML Buttons For New Connections Using Current Button Values
String processor(const String &var) {
    String buttons = "";
    if(var == "BUTTONS"){
        String red_state = red_Button.on?"on":"off",
               blue_state = blue_Button.on?"on":"off";
        buttons += "<button id='red_Button' class="+red_state+"></button>";
        buttons += "<button id='blue_Button' class="+blue_state+"></button>";
    }
    return buttons;
}

// Locate HTML Page
void onRootRequest(AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html", false, processor);
}

// WebSocket Events
void onEvent(AsyncWebSocket       *server,  //
             AsyncWebSocketClient *client,  //
             AwsEventType          type,    // the signature of this function is defined
             void                 *arg,     // by the `AwsEventHandler` interface
             uint8_t              *data,    //
             size_t                len) {   //

    // Incoming Message
    if(type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            // Deseralize Message
            const uint8_t size = JSON_OBJECT_SIZE(2);
            StaticJsonDocument<size> json;
            DeserializationError err = deserializeJson(json, data);
            if(err){return;}
            
            // Update Red Button
            if (strcmp(json["Button"], "red_Button") == 0) {
                red_Button.on = !red_Button.on;
                json["status"] = red_Button.on? "on": "off";
            } 
            // Update blue Button
            else if (strcmp(json["Button"], "blue_Button") == 0) {
                blue_Button.on = !blue_Button.on;
                json["status"] = blue_Button.on? "on": "off";
            }
               
            // Sync Devices
            char syncData[40];
            ws.textAll(syncData, serializeJson(json, syncData));
        }
    }
}

// Server Initialization
void serverSetup() {
    // Define Pin Behavior
    pinMode(red_Button.pin, OUTPUT);
    pinMode(blue_Button.pin, OUTPUT);
    
    // Activate Serial Monitor 
    Serial.begin(115200); delay(500);
  
    // Start SPIFFS
    SPIFFS.begin();
    
    // Create Soft Access Point
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    
    // Start Web Socket
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    
    // Launch Web Server
    server.on("/", onRootRequest);
    server.serveStatic("/", SPIFFS, "/");
    server.begin();
}

// Server Loop
void serverTasks(void* param) {
    Serial.print("serverTasks running on core ");
    Serial.println(xPortGetCoreID());

    while (true)
    {   // Update Buttons
        red_Button.update();
        blue_Button.update();

        // Close Lingering WebSockets
        if(millis()%1000 == 0){ ws.cleanupClients();}
    }
}

// PWM Task
void pwmTasks(void* param) {
    Serial.print("pwmTasks running on core ");
    Serial.println(xPortGetCoreID());

    while (true)
    {   blue_Button.on = !blue_Button.on;
        blue_Button.update();
        delay(1000);
    }
}

// Set Up
void setup()
{   // Start Server
    serverSetup();

    // Create Server to PWM Queue
    server2PWM_Handle = xQueueCreate(queueSize, sizeof(int));
    
    // Server Task
    xTaskCreatePinnedToCore(serverTasks, "ServerTasks", 10000, NULL, 1, &server_Handle, 1);
    delay(500); 

    // PWM Task
    xTaskCreatePinnedToCore(pwmTasks, "PwmTasks", 10000, NULL, 1, &pwm_Handle, 0);
    delay(500);
}

// Main Loop
void loop() {
    //serverTasks();
}