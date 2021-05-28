#include <Servo.h>
Servo ESC;     // create servo object to control the ESC
int i;
void setup() {
  // Attach the ESC on pin 9
  ESC.attach(9,900,2000); // (pin, min pulse width, max pulse width in microseconds) 
  Serial.begin(9600);
  ESC.writeMicroseconds(900);    // Send the signal to the ESC
  delay(200);
  ESC.writeMicroseconds(1800);    // Send the signal to the ESC

}
void loop() {
  i = i + 1;
  Serial.println(i);
  ESC.writeMicroseconds(1800);    // Send the signal to the ESC
  delay(20);
//    ESC.writeMicroseconds(900);    // Send the signal to the ESC


//if(millis()<1000*30){
//  ESC.writeMicroseconds(1800);    // Send the signal to the ESC
//
//}else{
//    ESC.writeMicroseconds(900);    // Send the signal to the ESC
//
//}
}
