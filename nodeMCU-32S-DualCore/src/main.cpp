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
#define RED_PIN 15
#define BLUE_PIN 16
#define HTTP_PORT 80
#define WDT_TIMEOUT 3

// WiFi Credentials
const char *WIFI_SSID = "ESP32";
const char *WIFI_PASS = "jetsonucsd";

// WIP Variables
int mostRecentHrtbt = 0;
int timeout = 500; // millis
Servo pwmThrottle;
int servoPin = 0;
int escPin = 4;
int steering_chn = 3;
int idle_throt = 1500;
int brake_throt = 1000;
int max_throt = 2000;
int left = 84;
int mid = 116;
int right = 148;

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
TaskHandle_t wip_Handle;

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

void initIO() {
  delay(1000);
  pinMode(escPin, OUTPUT);
  pwmThrottle.attach(escPin);
  pwmThrottle.writeMicroseconds(idle_throt);
  delay(1000);
}

void wipSetup() {
    initIO(); // Initialize throttle
    Serial.begin(115200); 
    esp_task_wdt_init(WDT_TIMEOUT, false); //enable panic so ESP32 restarts

    // wait for serial to start up
    while(!Serial) {
    }
    Serial.print("step1\n");

    pinMode(servoPin, OUTPUT);
    ledcSetup(steering_chn, 300, 8);
    ledcAttachPin(servoPin, steering_chn);
    ledcWrite(steering_chn, mid);
    Serial.print("step2\n");

    // Delay to calibrate ESC
    Serial.println("Turn on ESC");
    Serial.print("step3\n");
    delay(7000);
    Serial.print("step4\n");
    Serial.println("Ready to go");
    Serial.print("step5\n");
}

void pwm(float normalized_throttle,float normalized_steering){
    // pwm = normalized throttle(-1 to 1) * (pwmMax - pwm min) 
    int steering_pwm = mid + int(normalized_steering * int((right - left) / 2));
    int throttle_pwm = idle_throt + int(normalized_throttle * 500);

    Serial.print("steering_pwm=");
    Serial.print(steering_pwm);
    Serial.print(" throttle_pwm=");
    Serial.println(throttle_pwm);

    ledcWrite(steering_chn, steering_pwm);

    pwmThrottle.writeMicroseconds(throttle_pwm);
}

void breakSubRoutine() {
    Serial.print("Enterning backup routine");
    while (1) {
        pwmThrottle.writeMicroseconds(idle_throt);
        ledcWrite(steering_chn, mid);
    }
}

// WIP Loop
void wipTasks(void* param) {
    Serial.print("wipTasks running on core ");
    Serial.println(xPortGetCoreID());

    while (true)
    {   String  payload;
        esp_task_wdt_add(NULL);
        unsigned long begin = millis();
        unsigned long end = millis();

        while(!Serial.available()) {
            end = millis();
        }
        if(end-begin >= 200) {
            breakSubRoutine();
        }
        if(Serial.available()) {
            begin = millis();
            esp_task_wdt_reset();
            payload = Serial.readStringUntil( '\n' );
        }
        const uint8_t size = JSON_OBJECT_SIZE(1000);
        StaticJsonDocument<size> doc;
        // Serial.print(payload);

        DeserializationError error = deserializeJson(doc, payload);
        if(error) {
            Serial.println(error.c_str()); 
            return;
        };
        float normalized_throttle = doc["throttle"];
        float normalized_steering = doc["steering"];
        pwm(normalized_throttle,normalized_steering);
        delay(10);
    }
}

// Set Up
void setup()
{   // Start Server
    serverSetup();
    xTaskCreatePinnedToCore(serverTasks, "ServerTasks", 10000, NULL, 1, &server_Handle, 0);
    delay(500); 
    
    // Start WIP
    wipSetup();
    xTaskCreatePinnedToCore(wipTasks, "WIPTasks", 10000, NULL, 1, &wip_Handle, 1);
    delay(500);
    
    // Create Server to PWM Queue
    server2PWM_Handle = xQueueCreate(queueSize, sizeof(int));
}

// Main Loop
void loop() {}