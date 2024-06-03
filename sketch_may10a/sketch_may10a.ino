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
// float setPoint = 5;
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

#include <Arduino.h>
#include "motor.h"
#include "motor2.h"
#include "robot_position.h"
#include "control_trajectory.h"
#include "util.h"


const float R = 0.08;
const float L = 0.24;
const float wMaxR = 11.0;
const float wMaxL = 11.0;

Debug debug;


RobotPosition robotPosition = RobotPosition(R, L);
ControlTrajectory controlTrajectory = ControlTrajectory(R, L, wMaxR, wMaxL);

// const int N= 1000;
// float angR[N];
// float angL[N];
// float wL[N];
// float wR[N];



Motor motoRigth = Motor(
  32,      // pinPulseA
  33,      // pinPulseB
  2,       // modulePWMPin
  5,       // modulePWMPinNeg
  0,       // pwmChannel0
  1,       // pwmChannel1
  6505,    // R,
  15.99,   // u0
  -6.576,  // u1
  -7.715,  // u2
  0.9847,  // y0
  0.01531  // y1
);


Motor2 motoLeft = Motor2(
  25,      // pinPulseA
  26,      // pinPulseB
  15,      // modulePWMPin
  18,      // modulePWMPinNeg
  2,       // pwmChannel0
  3,       // pwmChannel1
  4484,    // R,
  13.28,   // u0
  -6.997,  // u1
  -4.867,  // u2
  0.9404,  // y0
  0.05964  // y1
);
void setup() {
  Serial.begin(115200);
  motoRigth.begin();
  motoRigth.setVelocity(4);
  // motoLeft.begin();
  // motoLeft.setVelocity(4);
}

const float xTarget = 10;
const float yTarget = 4;

// int ind = 0;
// unsigned int _previousMillisEncoderAngle = 0;
// unsigned int _intervalEncoderAngle = 10;

int step=0;
int previousMillisEncoderAngle=0;
float refer=0;

void loop() {

  unsigned long currentMillisEncoderAngle = millis();  // Actual time Variable Angle
  if (currentMillisEncoderAngle - previousMillisEncoderAngle >= 2) {
        step =step +1;
        previousMillisEncoderAngle=currentMillisEncoderAngle;
  }
  

  if(step%20==0){
      motoRigth.setVelocity(3);
      refer=3;
  }
  else if(step%10==0){
      motoRigth.setVelocity(8);
      refer=8;
  }

  // run motors
  motoRigth.motorRun();
  // motoLeft.motorRun();

  // get positions
  float velR = motoRigth.getAngularVelocity();
  float velL = motoLeft.getAngularVelocity();

  float posAngR = motoRigth.getAnglePos();
  float posAngL = motoLeft.getAnglePos();

debug.runDebug(velR,  refer,  posAngR, refer);



  // robotPosition.updatePosition(posAngR, posAngL);
  // float posRealY = robotPosition.getY();
  // float posRealX = robotPosition.getX();
  // float phiReal = robotPosition.getPhi();

  // // debug.runDebug(posRealX, posRealY, posRealY, posRealY);

  // controlTrajectory.update(xTarget, yTarget, posRealX, posRealY, phiReal);
  // float wRigth = controlTrajectory.getWRigth();
  // float wLeft = controlTrajectory.getWLeft();


  // motoRigth.setVelocity(wRigth);
  // motoLeft.setVelocity(wLeft);

  // unsigned long _currentMillisEncoderAngle = millis();  // Actual time Variable Angle
  // if (_currentMillisEncoderAngle - _previousMillisEncoderAngle >= _intervalEncoderAngle) {
  //   _previousMillisEncoderAngle = _currentMillisEncoderAngle;
  //   //   // update real position


  //   angR[ind] = posRealY;
  //   angL[ind] = posRealX;
  //   wR[ind] = phiReal;
  //   wL[ind] = posAngL;

  //   if (ind == N - 1) {
  //     for (int i = 0; i < N; i++) {
  //       Serial.print(angR[i]);
  //       Serial.print(" ");
  //       Serial.print(angL[i]);
  //       Serial.print(" ");
  //       Serial.print(wR[i]);
  //       Serial.print(" ");
  //       Serial.print(wL[i]);
  //       Serial.println();
  //     }
  //     ind = 0;
  //   }
  //   ind++;
  // }



  // // update trajectory
  // controlTrajectory.update(xTarget, yTarget, posRealX, posRealY, phiReal);
  // float wRigth = controlTrajectory.getWRigth();
  // float wLeft = controlTrajectory.getWLeft();

  // // set velocities
  // motoRigth.setVelocity(wRigth);
  // motoLeft.setVelocity(wLeft);









  // Serial.println(motoRigth.getAngularVelocity());
  // Serial.println(motoLeft.getAngularVelocity());
}
