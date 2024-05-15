#include "communication_manager.h"
#include "imu_manager.h"
#include "util.h"
#include <array>
#include <Arduino.h>
#include <string.h>
#include "control_velocity.h"
#include "control_trajectory.h"
#include "ascento.h"


ImuManager imu_manager;
QueueBuffer<float, 3> buffer;
ControlVelocity c;
ControlTrajectory a(11.0,11.0,11.0,11.0);

void setup(){
  Serial.begin(115200);
  imu_manager.begin();
  imu_manager.calibrate();

 
}

void loop(){
  imu_manager.update();
  if(!imu_manager.isWorking()){
  Serial.print(imu_manager.getAccelX());
  Serial.print(",");
  Serial.print(imu_manager.getAccelY());
  Serial.print(",");
  Serial.println(imu_manager.getAccelZ());
  }

}


