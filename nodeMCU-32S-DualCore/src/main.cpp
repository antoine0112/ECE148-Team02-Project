// Dependancies
#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <math.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <ESP32Servo.h>

// Macros
#define SERVO_PIN 0
#define ESC_PIN 4
#define RED_PIN 15
#define BLUE_PIN 16
#define HTTP_PORT 80
#define TIMEOUT 500
#define WDT_TIMEOUT 3
#define STEERING_CHN 3
#define IDLE_THROT 1500
#define BRAKE_THROT 1000
#define MAX_THROT 2000
#define LEFT_STEER 84
#define MID_STEER 116
#define RIGHT_STEER 148
#define QUEUE_SIZE 10

// WiFi Credentials
const char *WIFI_SSID = "ESP32";
const char *WIFI_PASS = "jetsonucsd";

// Button Component
struct Button {
    uint8_t pin;
    bool    on;
    void update() {digitalWrite(pin, on ? HIGH : LOW);}
};

// Instances
Button red_Button = {RED_PIN, false };
Button blue_Button = {BLUE_PIN, false };
AsyncWebServer server(HTTP_PORT);
AsyncWebSocket ws("/ws");
Servo pwmThrottle;
TaskHandle_t server_Handle;
TaskHandle_t pwm_Handle;
QueueHandle_t server2PWM_Handle;

// FiLL In Buttons For New Users Using Current Values
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
void onEvent(AsyncWebSocket       *server,
             AsyncWebSocketClient *client,
             AwsEventType          type,  
             void                 *arg,   
             uint8_t              *data,  
             size_t                len) { 

    // Incoming Message
    if(type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            // Deseralize Message
            const uint8_t size = JSON_OBJECT_SIZE(2);
            StaticJsonDocument<size> json;
            DeserializationError err = deserializeJson(json, data);
            if(err){ return;}
            
            // Update Red Button
            if(strcmp(json["Button"], "red_Button") == 0) {
                red_Button.on = !red_Button.on;
                json["status"] = red_Button.on? "on": "off";
            } 
            // Update blue Button
            else if(strcmp(json["Button"], "blue_Button") == 0) {
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
    Serial.println("\nServer Ready to go");
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

// PWM Initialization
void pwmSetup() {
    // Initialize throttle
    pinMode(ESC_PIN, OUTPUT);
    pwmThrottle.attach(ESC_PIN);
    pwmThrottle.writeMicroseconds(IDLE_THROT);
    
    // Initialize steering
    pinMode(SERVO_PIN, OUTPUT);
    ledcSetup(STEERING_CHN, 300, 8);
    ledcAttachPin(SERVO_PIN, STEERING_CHN);
    ledcWrite(STEERING_CHN, MID_STEER);
    
    //enable panic so ESP32 restarts
    esp_task_wdt_init(WDT_TIMEOUT, false); 
    
    // Delay to calibrate ESC
    delay(7000);
    Serial.println("PWM Ready to go");
}

// PWM Loop
void pwmTasks(void* param) {
    Serial.print("pwmTasks running on core ");
    Serial.println(xPortGetCoreID());
    while(true)
    {   esp_task_wdt_add(NULL);
        
        // Serial Check
        unsigned long begin = millis();
        unsigned long end = millis();
        while(!Serial.available()) {end = millis();}
        if(end-begin >= 200) {
            Serial.print("Enterning backup routine");
            while(1) {
                pwmThrottle.writeMicroseconds(IDLE_THROT);
                ledcWrite(STEERING_CHN, MID_STEER);
            }
        }
        
        // Get Message
        String payload;
        if(Serial.available()) {
            begin = millis();
            esp_task_wdt_reset();
            payload = Serial.readStringUntil( '\n' );
        }
        
        // Deserialize Message
        const uint8_t size = JSON_OBJECT_SIZE(1000);
        StaticJsonDocument<size> doc;
        DeserializationError error = deserializeJson(doc, payload);
        if(error) {return;};
        
        // Calculate Steering and Throttle
        float normalized_throttle = doc["throttle"];
        float normalized_steering = doc["steering"];
        int steering_pwm = MID_STEER+int(normalized_steering*int((RIGHT_STEER-LEFT_STEER)/2));
        int throttle_pwm = IDLE_THROT+int(normalized_throttle*500);
        ledcWrite(STEERING_CHN, steering_pwm);
        pwmThrottle.writeMicroseconds(throttle_pwm);
        Serial.print("steering_pwm="+steering_pwm);
        Serial.print(" throttle_pwm="+throttle_pwm);
    }
}

// Set Up
void setup()
{   // Start Server
    serverSetup();
    xTaskCreatePinnedToCore(serverTasks, "ServerTasks", 10000, NULL, 1, &server_Handle, 0);
    delay(500); 
    
    // Start PWM
    pwmSetup();
    xTaskCreatePinnedToCore(pwmTasks, "pwmTasks", 10000, NULL, 1, &pwm_Handle, 1);
    delay(500);
    
    // Create Server to PWM Queue
    server2PWM_Handle = xQueueCreate(QUEUE_SIZE, sizeof(int));
}

// Main Loop
void loop() {}
