#include <ArduinoJson.h>
#include <math.h>
#include <esp_task_wdt.h>

//3 seconds WDT
#define WDT_TIMEOUT 3

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
}

void pwm(float normalized_throttle,float normalized_steering){
  // pwm = normalized throttle(-1 to 1) * (pwmMax - pwm min) 
  
  int throttle_pwm = int(normalized_throttle*(throttle_PWMmax-throttle_PWMmin)/2 + (throttle_PWMmax+throttle_PWMmin)/2);
  int steering_pwm = int(normalized_steering*(steering_PWMright-steering_PWMleft)/2 + (steering_PWMright+steering_PWMleft)/2);
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

  delay(10);
}
