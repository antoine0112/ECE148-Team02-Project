// Dependancies
#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Macros
#define MESSAGE_PIN 2
#define RED_PIN 0
#define BLUE_PIN 15
#define WIFI_ERROR_BLINKS 1
#define SPIFFS_ERROR_BLINKS 2
#define SERIAL_ERROR_BLINKS 3
#define BLINK_CYCLE 1000
#define HTTP_PORT 80

// WiFi Credentials
const char *WIFI_SSID = "ESP32";
const char *WIFI_PASS = "jetsonucsd";

// Led Component
struct LED {
    uint8_t pin;
    bool    on;

    void update() {digitalWrite(pin, on ? HIGH : LOW);}
};


// LED Instances
LED message_LED = {MESSAGE_PIN, false };
LED red_LED = {RED_PIN, false };
LED blue_LED = {BLUE_PIN, false };

// Server Instances
AsyncWebServer server(HTTP_PORT);
AsyncWebSocket ws("/ws");

// LED Blinks
void blink(LED led, int blinks) {
    // Active
    for(int i=0; i<blinks*2-1; i++) {   
        led.on = !led.on;
        led.update();   
        delay(BLINK_CYCLE/(blinks*2-1));
    }
    // Rest
    led.on = !led.on;
    led.update();       
    delay(BLINK_CYCLE);
}

// Serial Peripheral Interface Flash File System
void initSPIFFS() {
    if (!SPIFFS.begin()) {
        // Blink Error
        while(true) {blink(message_LED, SPIFFS_ERROR_BLINKS);}
    }
}

// WiFi Connection
void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Trying to connect [%s] ", WiFi.macAddress().c_str());
    // Blink While Connecting
    while(WiFi.status() != WL_CONNECTED) {blink(message_LED, WIFI_ERROR_BLINKS);}
    Serial.printf(" %s\n", WiFi.localIP().toString().c_str());
}

// Access Point
void initAccessPoint() {
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.println("Creating Access Point... IP: "+WiFi.softAPIP().toString());
}

// FiLL In HTML LEDs
String processor(const String &var) {
    String buttons = "";
    if(var == "LED_BUTTONS"){
        String red_state = red_LED.on?"on":"off",
               blue_state = blue_LED.on?"on":"off";
        buttons += "<button id='red_LED' class="+red_state+"></button>";
        buttons += "<button id='blue_LED' class="+blue_state+"></button>";
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

    // Connect Event
    if(type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    }
    // Disconnect Event    
    else if(type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    }    
    // Incoming Message
    else if(type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            // Deseralize Message
            const uint8_t size = JSON_OBJECT_SIZE(2);
            StaticJsonDocument<size> json;
            DeserializationError err = deserializeJson(json, data);
            // Catch Error
            if (err) {
                Serial.print(F("deserializeJson() failed with code "));
                Serial.println(err.c_str());
                blink(message_LED, SERIAL_ERROR_BLINKS);
                return;
            }
            // Update LEDs
            if (strcmp(json["led"], "red_LED") == 0) {
                red_LED.on = !red_LED.on;
                json["status"] = red_LED.on? "on": "off";
            } 
            else if (strcmp(json["led"], "blue_LED") == 0) {
                blue_LED.on = !blue_LED.on;
                json["status"] = blue_LED.on? "on": "off";
            }
            else
            {   Serial.println("Unkown Event Message");
            }   
            // Sync Devices
            char syncData[35];
            ws.textAll(syncData, serializeJson(json, syncData));
        }
    }
}

// Server
void initWebServer() {
    // Web Socket
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    
    // Web Server
    server.on("/", onRootRequest);
    server.serveStatic("/", SPIFFS, "/");
    server.begin();
}



// Initialization
void setup() {
    // Define LED Outputs
    pinMode(message_LED.pin, OUTPUT);
    pinMode(red_LED.pin, OUTPUT);
    pinMode(blue_LED.pin, OUTPUT);
    
    // Activate Serial Monitor 
    Serial.begin(115200); delay(500);
  
    // Start SPIFFS, WiFi, and WebServer
    initSPIFFS();
    //initWiFi();
    initAccessPoint();
    initWebServer();
}

// Main Loop
void loop() {
    // Start Up Success
    if(!message_LED.on) {
        message_LED.on = true;
        message_LED.update();
    }

    // Update LEDs
    red_LED.update();
    blue_LED.update();

    // Close Lingering WebSockets
    if(millis()%1000 == 0) {
        ws.cleanupClients(); 
    }
}