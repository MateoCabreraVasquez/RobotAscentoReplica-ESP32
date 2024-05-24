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
  32,     // pinPulseA
  33,     // pinPulseB
  2,      // modulePWMPin
  5,      // modulePWMPinNeg
  0,      // pwmChannel0
  1,      // pwmChannel1
  6538,   // R,
  15.99,
  -6.576,
  -7.715,
  0.9847,
  0.01531);


// wheels
Motor motorLeft = Motor(
  25,
  26,    // pinPulseB
  15,    // modulePWMPin
  18,    // modulePWMPinNeg
  2,     // pwmChannel0
  3,     // pwmChannel1
  4451,  // R,
  13.28,
  - 6.997,
  - 4.867,
  0.9404,
  0.05964);


//ControlTrajectory a(11.0,11.0,11.0,11.0);

void setup() {



  Serial.begin(115200);
  motoRigth.begin();
  motoRigth.setVelocity(6);
   motorLeft.begin();
  motorLeft.setVelocity(9);
  // motoRigth.setPidParameters(15.99,15.99,- 7.715,0.9847,0.01531);




  buffer.insert(1.0);
  buffer.insert(2.0);
  buffer.insert(3.0);
  buffer.insert(4.0);
}
long int duty = 0;
float w = 8;

void loop() {
  motoRigth.motorRun();
  motorLeft.motorRun();

  // float vel = motoRigth.getAngularVelocity();
  // //  Serial.println(vel);
  // float m = controlVelocity.computeRigth(5, vel);
  // motoRigth.setVelocity(m);
  // Serial.println(vel);
}


// //***************************************************** PWM Encoder + Puente H *****************************************************//
// int pinApulse = 25;  //Digital pin, need interrupt for count rising flanks
// int pinBpulse = 26;

// const long intervalEncoderAngle = 50;  //Delay for the Angle Data
// unsigned long previousMillisEncoderAngle = 0;

// volatile int n = 0;            //store the pulse
// volatile byte actualAB = 0;    //Actual Value of AB
// volatile byte previousAB = 0;  //previous Value of AB

// double P = 0;              //Relative Position in grades
// double R = 4451;           //Resolution of Encoder for a quadrupel precision
// double Pact = 0;           //Relative Position in grades
// double Pant = 0;           //Relative Position in grades
// const int resolution = 8;  // set PWM resolution
// int frecuency = 21000;
// int pwmChannel2 = 2; 
// int pwmChannel3 = 3;
// int modulePWMPinIZ = 15;
// int modulePWMPinNegIZ = 18;

// float pastError[2] = { 0, 0 };
// float pastOutput[2] = { 0, 0 };
// void setup() {
//   //Encoder
//   pinMode(pinApulse, INPUT);
//   pinMode(pinBpulse, INPUT);
//   attachInterrupt(digitalPinToInterrupt(pinApulse), pulseinterrupt, CHANGE);  // Change Flanks pulse A
//   attachInterrupt(digitalPinToInterrupt(pinBpulse), pulseinterrupt, CHANGE);  // Change Flanks pulse B
//   ledcSetup(pwmChannel2, frecuency, resolution);                               // define the PWM Setup
//   ledcSetup(pwmChannel3, frecuency, resolution);   
//   ledcAttachPin(modulePWMPinIZ, pwmChannel2);
//   ledcAttachPin(modulePWMPinNegIZ, pwmChannel3);
//   Serial.begin(115200);  // open a serial connection to your computer
// }
// long int duty = 0;
// float setPoint = 12;
// void loop() {
//   unsigned long currentMillisEncoderAngle = millis();  // Actual time Variable Angle
//   if (currentMillisEncoderAngle - previousMillisEncoderAngle >= intervalEncoderAngle) {
//     float delta = (currentMillisEncoderAngle - previousMillisEncoderAngle) / 1000.f;
//     previousMillisEncoderAngle = currentMillisEncoderAngle;
//     //Position
//     Pant = Pact;
//     Pact = (n * 360.0) / R;
//     double velocity = (3.14159 / 180.f) * (Pact - Pant) / delta;
//     float error = setPoint - velocity;
//     duty = (13.28 * error - 6.997 * pastError[0] - 4.867 * pastError[1] + 0.9404 * pastOutput[0] + 0.05964 * pastOutput[1]);
//     if (duty > 100) duty = 100;
//     Serial.print(duty);
//     Serial.print(" ");
//     pastError[1] = pastError[0];
//     pastError[0] = error;
//     pastOutput[1] = pastOutput[0];
//     pastOutput[0] = duty;

//     Serial.println(velocity);
//     if (duty < 0) {  //Si es negativo, se debe mandar uno de los PWM a cero y activar el otro
//       ledcWrite(pwmChannel2, 0);
//       ledcWrite(pwmChannel3, -1 * duty * 255.f / 100.f);
//     } else {
//       ledcWrite(pwmChannel3, 0);
//       ledcWrite(pwmChannel2, duty * 255.f / 100.f);
//     }
//   }
// }

// //Encoder
// void pulseinterrupt() {
//   previousAB = actualAB;
//   if (digitalRead(pinApulse)) bitSet(actualAB, 1);
//   else bitClear(actualAB, 1);
//   if (digitalRead(pinBpulse)) bitSet(actualAB, 0);
//   else bitClear(actualAB, 0);

//   //direction of movement
//   if (previousAB == 2 && actualAB == 0) n++;
//   if (previousAB == 0 && actualAB == 1) n++;
//   if (previousAB == 3 && actualAB == 2) n++;
//   if (previousAB == 1 && actualAB == 3) n++;

//   if (previousAB == 1 && actualAB == 0) n--;
//   if (previousAB == 3 && actualAB == 1) n--;
//   if (previousAB == 0 && actualAB == 2) n--;
//   if (previousAB == 2 && actualAB == 3) n--;
// }