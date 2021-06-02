// Dependancies
#include <Arduino.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <math.h>
#include <esp_task_wdt.h>

//3 seconds WDT
#define WDT_TIMEOUT 3

#define WIFI_ERROR_BLINKS 1
#define SPIFFS_ERROR_BLINKS 2
#define SERIAL_ERROR_BLINKS 3
#define BLINK_CYCLE 1000
#define HTTP_PORT 80

// WiFi Credentials
const char *WIFI_SSID = "ESP32";
const char *WIFI_PASS = "jetsonucsd";
// Server Instances
AsyncWebServer server(HTTP_PORT);
AsyncWebSocket ws("/ws");


int mostRecentHrtbt = 0;
int timeout = 500; // millis

int direction_pin = 0;
int pwm_pin = 4;

//for home testing
int servoPin = direction_pin;
int escPin = pwm_pin;

int steering_chn = 3;
int throttle_chn = 4;

int throttle_PWMmax = 255;
int throttle_PWMmin = 0;
int steering_PWMright = 255;
int steering_PWMleft = 0;

// Led Component
struct LED {
    uint8_t pin;
    bool    on;
    int     pwm;

    void update() {
      ledcWrite(pin, on ? pwm : LOW);
      }
};

// LED Instances
LED message_LED = {MESSAGE_PIN, false };
LED pwm_LED = {RED_PIN, false };
LED steering_LED = {BLUE_PIN, false };

void setup() {
  Serial.begin(115200); 
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch


// //   wait for serial to start up
// while(!Serial) {
// }

  pinMode(escPin, OUTPUT);
  ledcSetup(throttle_chn, 5000, 8);
  ledcAttachPin(escPin, throttle_chn);

  pinMode(servoPin, OUTPUT);
  ledcSetup(steering_chn, 5000, 8);
  ledcAttachPin(servoPin, steering_chn);

  // Start SPIFFS, WiFi, and WebServer
  initSPIFFS();
  //initWiFi();
  initAccessPoint();
  initWebServer();

}

// WiFi Connection
void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Trying to connect [%s] ", WiFi.macAddress().c_str());
    // Blink While Connecting
    // while(WiFi.status() != WL_CONNECTED) {blink(message_LED, WIFI_ERROR_BLINKS);}
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


void pwm(float normalized_throttle,float normalized_steering){
  // pwm = normalized throttle(-1 to 1) * (pwmMax - pwm min) 
  
  int throttle_pwm = int(normalized_throttle*(throttle_PWMmax-throttle_PWMmin));
  int steering_pwm = int(normalized_steering*(steering_PWMright-steering_PWMleft));
  Serial.print("steering_pwm=");
   Serial.print(steering_pwm);
   Serial.print(" throttle_pwm=");
   Serial.println(throttle_pwm);


  ledcWrite(throttle_chn, throttle_pwm);
  ledcWrite(steering_chn, steering_pwm);
}

void loop() {
  String  payload;

  // do nothing while serial is disconnected
  while ( !Serial.available()  ){}

  if ( Serial.available() )
    esp_task_wdt_reset(); // reset watchdog timer if serial is connected
    payload = Serial.readStringUntil( '\n' ); // read payload from serial
  const uint8_t size = JSON_OBJECT_SIZE(1000);
  StaticJsonDocument<size> doc;
  // Serial.print(payload);

  DeserializationError   error = deserializeJson(doc, payload);
  if (error) {
    Serial.println(error.c_str()); 
    return;
  };
  float normalized_throttle = doc["throttle"];
  float normalized_steering = doc["steering"];
  pwm(normalized_throttle,normalized_steering);

  delay(20);
}
