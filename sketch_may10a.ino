#include "communication_manager.h"
#include "imu_manager.h"
#include "util.h"
#include <array>
#include <Arduino.h>
#include <string.h>
#include "control_velocity.h"
#include "control_trajectory.h"

ImuManager imu_manager;
QueueBuffer<float, 3> buffer;
ControlVelocity c;
ControlTrajectory a(11.0,11.0,11.0,11.0);

void setup(){

  float a=c.computeLeft(1);
   Serial.begin(115200);
   Serial.print(a);


   imu_manager.begin();
   imu_manager.calibrate();
   delay(1000);
}

void loop(){
  imu_manager.update();
  Serial.println(imu_manager.getPitch());
   


}



