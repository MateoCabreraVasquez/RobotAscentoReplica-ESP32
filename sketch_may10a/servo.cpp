#include "servo.h"
#include <Arduino.h>

servo::servo(int iniAngle, int iniChannel):
  angle(iniAngle),
  channel(iniChannel) 
  {}

void servo::setAngle(int newAngle,char color){
  int pwm = 0;
  angle = newAngle;

  if(color == 'b'){
    pwm = mapDutyCycleAzul();
  }

  else if(color == 'r'){
    pwm = mapDutyCycleRojo();
  }

  ledcWrite(channel, pwm);
  
}

int servo::getAngle(){
  return angle;
  }

int servo::mapDutyCycleAzul(){
  int duty = map(angle,0,180,80,21);  //servoAzul
  return map(duty, 0, 100, 0, 255);
}

int servo::mapDutyCycleRojo(){
  int duty = map(angle,0,180,21,85);  //servoRojo
  return map(duty, 0, 100, 0, 255);
}