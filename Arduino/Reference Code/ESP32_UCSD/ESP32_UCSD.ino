#include <WiFi.h>
#include <base64.h>
#include "ArduinoJson.h"
#include "Arduino.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

/* 
Open this in Arduino IDE. Follow this guide to install ESP32 support: 
https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/ 
*/

// Choose your ssid and password.

const char* ssid     = "UCSD148-02-ESP32";
const char* password = "ucsd14802";



//pins
int servoPin = 13;
int escPin = 12;

int steering_chn = 3;
int throttle_chn = 2;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void wifiSetup(){
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    
    server.begin();
}

void pwmSetup(){
    pinMode(servoPin, OUTPUT);
    pinMode(escPin, OUTPUT);
    ledcSetup(steering_chn, 60, 12);
    ledcSetup(throttle_chn, 60, 12);
    ledcAttachPin(servoPin, steering_chn);
    ledcAttachPin(escPin, throttle_chn);
}

void setup()
{
    //// DEBUG PRINTS, COMMENT OUT FOR ACTUAL RUN 
    // Serial.begin(115200);
    // Serial.println();
    // Serial.println("Configuring access point...");
    //Serial.print("AP IP address: ");
    // Serial.println(myIP);
    // Serial.println("Server started");
  
    wifiSetup();
    pwmSetup()

}

// Notifies all clients with new data.
void notifyClients(String msg) {
  ws.textAll(msg);
}

void command(int steering_pwm, int throttle_pwm){
  //Serial.println(steering_pwm);
  //Serial.println(throttle_pwm);
  ledcWrite(throttle_chn, throttle_pwm);
  ledcWrite(steering_chn, steering_pwm);
}

void on_msg_recv(String msg){
  StaticJsonDocument<300> dic;
  deserializeJson(dic, msg);
  int steering_pwm = dic["steering"];
  int throttle_pwm = dic["throttle"];
  command(steering_pwm, throttle_pwm);
}

void parseJSONSerial(){
    String  payload;
    while ( !Serial.available()  ){}
    if ( Serial.available() ){
        payload = Serial.readStringUntil( '\n' );
        }
    StaticJsonDocument<300> dic;
    deserializeJson(dic, msg);
 
    DeserializationError   error = deserializeJson(doc, payload);
    if (error) {
    Serial.println(error.c_str()); 
    return;
    }
    int steering_pwm = dic["steering"];
    int throttle_pwm = dic["throttle"];
    command(steering_pwm,throttle_pwm);
}

void loop(){
 WiFiClient client = server.available();   // listen for incoming clients

  if (client) {
    //Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) { 
      //Serial.println("Waiting for client to be available");                     

      while (client.available()){
        // Parses the incoming string frin webclient char by char
        // put each read char into currentLine, when /n is read, 
        // pass to on_msg_recv for processing, start new currentLine
        char c = client.read();
        currentLine += c;
        if (c == '\n'){
            on_msg_recv(currentLine);
            currentLine = "";
        }
      }

        parseJSONSerial()
      
    }
    // close the connection:
    client.stop();
    //Serial.println("Client Disconnected.");
  }
}