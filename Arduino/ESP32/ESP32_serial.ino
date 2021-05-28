#include <ArduinoJson.h>
#include <math.h>

int mostRecentHrtbt = 0;
int timeout = 500; // millis

int direction_pin = 0;
int pwm_pin = 2;

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

//   wait for serial to start up
  while(!Serial) {
  }

  // pinMode(direction_pin, OUTPUT);           // Set the direction pin output
  // pinMode(pwm_pin, OUTPUT);                 // Set the pwm pin output


  pinMode(escPin, OUTPUT);
  ledcSetup(throttle_chn, 5000, 8);
  ledcAttachPin(escPin, throttle_chn);

  pinMode(servoPin, OUTPUT);
  ledcSetup(steering_chn, 5000, 8);
  ledcAttachPin(servoPin, steering_chn);
}

void pwm(float normalized_throttle,float normalized_steering){
  // pwm = normalized throttle(-1 to 1) * (pwmMax - pwm min) 

  int throttle_pwm = int(normalized_throttle(throttle_PWMmax-throttle_PWMmin));
  int steering_pwm = int(normalized_steering(steering_PWMright-steering_PWMleft));
  Serial.print("steering_pwm=");
   Serial.print(steering_pwm);
   Serial.print(" throttle_pwm=");
   Serial.println(throttle_pwm);

  ledcWrite(throttle_chn, throttle_pwm);
  ledcWrite(steering_chn, steering_pwm);
}

void loop() {
  String  payload;
  while ( !Serial.available()  ){}
  if ( Serial.available() )
    payload = Serial.readStringUntil( '\n' );
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