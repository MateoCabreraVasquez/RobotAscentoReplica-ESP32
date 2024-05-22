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
ascento A;
ControlVelocity controlVelocity;

  // wheels
  Motor motoRigth = Motor(
      32,   // pinPulseA
      33,   // pinPulseB
      2,    // modulePWMPin
      5,    // modulePWMPinNeg
      0,    // pwmChannel0
      1,    // pwmChannel1
      6538, // R,
      0.075     // radioWheel
  );

//ControlTrajectory a(11.0,11.0,11.0,11.0);

void setup(){
  Serial.begin(115200);
  imu_manager.begin();
  imu_manager.calibrate();
  // motoRigth.begin();
  // motoRigth.setVelocity(50);
  

}
long int duty=0;
float w = 8;

void loop(){
  imu_manager.update();

  if(!imu_manager.isWorking()){
  //     Serial.println(imu_manager.getPitch);
  Serial.print(imu_manager.getAccelX());
   Serial.print(", ");
  // Serial.print(imu_manager.getAccelY());
  // Serial.print(",");
  // Serial.println(imu_manager.getAccelZ());

  Serial.print(imu_manager.getVelX());
  Serial.print(", ");

  Serial.println(imu_manager.getPosX());

  // Serial.print(imu_manager.getAccelX());
  // Serial.print(", ");
  // Serial.println(imu_manager.getPitch());

  // Serial.println(imu_manager.getAccelX()); 
  // Serial.println(imu_manager.getVelX());
  }
}

long readLongFromSerial() {
  while (Serial.available() == 0) {
    // Wait for input
  }

  String inputString = "";
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();
    if (incomingByte == '\n' || incomingByte == '\r') {
      // End of input
      break;
    }
    inputString += incomingByte;
  }

  long value = inputString.toInt();
  return value;
}



