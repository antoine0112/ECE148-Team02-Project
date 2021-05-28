
#include <Wire.h>

int led_1 = 11;
int led_2 = 10;
int led_3 = 9;

int encA_addr = 2;
int encB_addr = 3;

int encA_val = 0;
int encB_val = 0;

void setup()
{
  Serial.begin(9600);
  
  pinMode(led_1,OUTPUT);
  pinMode(led_2,OUTPUT);
  pinMode(led_3,OUTPUT);

  pinMode(encA_addr,INPUT);
  pinMode(encB_addr,INPUT);

  encA_val = digitalRead(encA_addr);
}
 
 
void loop()
{
  Serial.print(millis());
  Serial.print(",");

  encA_val = digitalRead(encA_addr);
  Serial.print(" encA: ");
  Serial.print(encA_val);  

  encB_val = digitalRead(encB_addr);
  Serial.print(" encB: ");
  Serial.print(encB_val);  
  
  if(encA_val)  {digitalWrite(led_1,HIGH);}
  else {digitalWrite(led_1,LOW);}
  
  if(encB_val)  {digitalWrite(led_2,HIGH);}
  else {digitalWrite(led_2,LOW);}

  Serial.println();
}
