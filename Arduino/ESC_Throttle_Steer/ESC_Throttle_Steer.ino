#include <ESP32Servo.h>

int servoPin = 0;
int escPin = 4;

int steering_chn = 3;
int throttle_chn = 4;

// Throttle
Servo pwmThrottle;
int throt = 1530;
int stop_throt = 1500;
int brake_throt = 1000;

// Steering
int left = 84;
int mid = 116;
int right = 148;

void setup() {
  initIO(); // Initialize throttle

  // Setup Serial
  Serial.begin(115200); 
  while(!Serial) {
  }

  // Initialize steering
  pinMode(servoPin, OUTPUT);
  ledcSetup(steering_chn, 300, 8);
  ledcAttachPin(servoPin, steering_chn);
  ledcWrite(steering_chn, mid);

  // Delay to calibrate ESC
  Serial.println("Turn on ESC");
  delay(7000);
  Serial.println("Ready to go");

}

void loop() {

  throt++;
  if (throt > 1550){
    // Steering
    delay(1000);
    ledcWrite(steering_chn, right);
    Serial.println("Right");
    delay(1000);
    ledcWrite(steering_chn, left);
    Serial.println("Left");
    delay(1000);
    ledcWrite(steering_chn, mid);
    Serial.println("Center");
    delay(1000);

    // Brake
    pwmThrottle.writeMicroseconds(brake_throt);
    delay(1000);

    // Reset throttle
    Serial.println("Resetting throttle");
    pwmThrottle.writeMicroseconds(stop_throt);
    throt = stop_throt;
    delay(1000);
    Serial.println("Throttle reset");
  }
  
  pwmThrottle.writeMicroseconds(throt);
  delay(50);
}

void initIO() {
  delay(1000);
  pinMode(escPin, OUTPUT);
  pwmThrottle.attach(escPin);
  pwmThrottle.writeMicroseconds(stop_throt);
  delay(1000);
}
