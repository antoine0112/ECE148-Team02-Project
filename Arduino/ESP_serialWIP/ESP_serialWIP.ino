#include <ArduinoJson.h>
#include <math.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <ESP32Servo.h>

//3 seconds WDT
#define WDT_TIMEOUT 3

int mostRecentHrtbt = 0;
int timeout = 500; // millis

Servo pwmThrottle;
int servoPin = 0;
int escPin = 4;

int steering_chn = 3;

//throttle
int idle_throt = 1500;
int brake_throt = 1000;
int max_throt = 2000;

// Steering
int left = 84;
int mid = 116;
int right = 148;

void setup() {
  initIO(); // Initialize throttle
  Serial.begin(115200); 
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts


// //   wait for serial to start up
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

void initIO() {
  delay(1000);
  pinMode(escPin, OUTPUT);
  pwmThrottle.attach(escPin);
  pwmThrottle.writeMicroseconds(idle_throt);
  delay(1000);
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

void loop() {
  String  payload;
  esp_task_wdt_add(NULL);
  unsigned long begin = millis();
  unsigned long end = millis();

  while ( !Serial.available()) {
    end = millis();
  }
  if (end - begin >= 200) {
    breakSubRoutine();
  }
  if ( Serial.available() ) {
    begin = millis();
    esp_task_wdt_reset();
    payload = Serial.readStringUntil( '\n' );
  }
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
