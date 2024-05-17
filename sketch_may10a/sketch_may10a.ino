#include "communication_manager.h"
#include "imu_manager.h"
#include "util.h"
#include <array>
#include <Arduino.h>
#include <string.h>
#include "control_velocity.h"
#include "control_trajectory.h"
#include "ascento.h"
#include "motor.h"


ImuManager imu_manager;
QueueBuffer<float, 3> buffer;
ControlVelocity c;
ascento A;
//ControlTrajectory a(11.0,11.0,11.0,11.0);

void setup(){
  Serial.begin(115200);
  imu_manager.begin();
  imu_manager.calibrate();

 
}

void loop(){
  imu_manager.update();
  if(!imu_manager.isWorking()){
  // Serial.print(imu_manager.getAccelX());
  // Serial.print(",");
  // Serial.print(imu_manager.getAccelY());
  // Serial.print(",");
  // Serial.println(imu_manager.getAccelZ());

  //Serial.println(imu_manager.getVelX());
  //Serial.print(imu_manager.getPitch());
  Serial.print(", ");
  Serial.println(imu_manager.getAccelX()); 
  Serial.println(imu_manager.getVelX());
  delay(200);
  }

}



