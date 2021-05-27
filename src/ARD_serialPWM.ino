#include <ArduinoJson.h>
#include <math.h>

int mostRecentHrtbt = 0;
int timeout = 500; // millis

int direction_pin = 7;
int pwm_pin = 6;

//for home testing
int servoPin = direction_pin;
int escPin = pwm_pin;

int steering_chn = 3;
int throttle_chn = 2;

int throttle_PWMmax = 255;
int throttle_PWMmin = 0;
int steering_PWMright = 1;
int steering_PWMleft = -1;

void setup() {
  Serial.begin(9600); 
  // Serial.begin(115200); 

//   wait for serial to start up
  while(!Serial) {
  }

  pinMode(direction_pin, OUTPUT);           // Set the direction pin output
  pinMode(pwm_pin, OUTPUT);                 // Set the pwm pin output


  // pinMode(escPin, OUTPUT);
  // ledcSetup(throttle_chn, 60, 12);
  // ledcAttachPin(servoPin, steering_chn);
}

void pwm(float normalized_throttle,float normalized_steering){
  // pwm = normalized throttle(-1 to 1) * (pwmMax - pwm min) + pwmMiddle
  int throttle_pwm = int(normalized_throttle*(throttle_PWMmax - throttle_PWMmin) + (throttle_PWMmax - throttle_PWMmin)/2);
  int steering_pwm = int(normalized_steering*(steering_PWMright - steering_PWMleft) + (steering_PWMright - steering_PWMleft)/2);

  //just for home testing: 
  // digitalWrite(direction_pin, steering_pwm);   // Set the motor in motion
  digitalWrite(direction_pin, 0);   // Set the motor in motion
  analogWrite(pwm_pin, throttle_pwm);              // Set the motor in motion
  Serial.print(" steering_pwm=");
  Serial.print(steering_pwm);
  Serial.print(" throttle_pwm=");
  Serial.println(throttle_pwm);


  // ledcWrite(throttle_chn, throttle_pwm);
  // ledcWrite(steering_chn, steering_pwm);
}

void loop() {
  int     size_ = 0;
  String  payload;
  while ( !Serial.available()  ){}
  if ( Serial.available() )
    payload = Serial.readStringUntil( '\n' );
  StaticJsonDocument<70> doc;

  DeserializationError   error = deserializeJson(doc, payload);
  if (error) {
    Serial.println(error.c_str()); 
    return;
  };
  float normalized_throttle = doc["throttle"];
  float normalized_steering = doc["steering"];
  pwm(normalized_throttle,normalized_steering);

  // brake the car if heartbeat
  // if (doc["heartbeat"] <= mostRecentHrtbt + timeout) {
    
  // }
  // // normal throttle and steering if normal
  // else {
  //   mostRecentHrtbt = doc["heartbeat"];

  //   }
  delay(20);
}