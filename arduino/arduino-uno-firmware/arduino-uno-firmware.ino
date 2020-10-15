#include <Servo.h>
Servo servo;
int pospx;

void setup()
{
  Serial.begin(9600);
  pinMode(4,OUTPUT);
  servo.attach(10);
  servo.write(100);
  delay(1000);
}

void loop()
{
    if (Serial.available()>0) 
    {
      digitalWrite(4,HIGH);
      pospx = Serial.parseInt();
      if(pospx>140) pospx=140;
      else if(pospx<65) pospx=65;
      servo.write(pospx);
      digitalWrite(4,LOW);
    }
    
}
