#include <Arduino.h>

#ifndef servo_h
#define servo_h

class servo {
  private:
    int angle;
    int channel;

  public:
    servo(int iniAngle, int iniChannel) : angle(iniAngle), channel(iniChannel) {}

    void setAngle(int newAngle, char color)
    {
      int pwm = 0;
      angle = newAngle;

      if (color == 'b')
      {
        pwm = mapDutyCycleAzul();
      }

      else if (color == 'r')
      {
        pwm = mapDutyCycleRojo();
      }

      ledcWrite(channel, pwm);
  
}

int getAngle(){
  return angle;
  }

int mapDutyCycleAzul(){
  int duty = map(angle,0,180,80,21);  //servoAzul
  return map(duty, 0, 100, 0, 255);
}

int mapDutyCycleRojo(){
  int duty = map(angle,0,180,21,85);  //servoRojo
  return map(duty, 0, 100, 0, 255);
}

};

#endif