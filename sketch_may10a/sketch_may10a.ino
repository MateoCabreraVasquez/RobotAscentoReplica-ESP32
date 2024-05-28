// //***************************************************** PWM Encoder + Puente H *****************************************************//
// int pinApulse = 32;  //Digital pin, need interrupt for count rising flanks
// int pinBpulse = 33;

// const long intervalEncoderAngle = 50;  //Delay for the Angle Data
// unsigned long previousMillisEncoderAngle = 0;

// volatile int n = 0;            //store the pulse
// volatile byte actualAB = 0;    //Actual Value of AB
// volatile byte previousAB = 0;  //previous Value of AB

// double P = 0;              //Relative Position in grades
// double R = 6538;           //Resolution of Encoder for a quadrupel precision
// double Pact = 0;           //Relative Position in grades
// double Pant = 0;           //Relative Position in grades
// const int resolution = 8;  // set PWM resolution
// int frecuency = 21000;
// int pwmChannel0 = 0;
// int pwmChannel1 = 1;
// int modulePWMPin = 2;
// int modulePWMPinNeg = 5;

// float pastError[2] = { 0, 0 };
// float pastOutput[2] = { 0, 0 };
// void setup() {
//   //Encoder
//   pinMode(pinApulse, INPUT);
//   pinMode(pinBpulse, INPUT);
//   attachInterrupt(digitalPinToInterrupt(pinApulse), pulseinterrupt, CHANGE);  // Change Flanks pulse A
//   attachInterrupt(digitalPinToInterrupt(pinBpulse), pulseinterrupt, CHANGE);  // Change Flanks pulse B
//   ledcSetup(pwmChannel0, frecuency, resolution);                               // define the PWM Setup
//   ledcSetup(pwmChannel1, frecuency, resolution);   
//   ledcAttachPin(modulePWMPin, pwmChannel0);
//   ledcAttachPin(modulePWMPinNeg, pwmChannel1);
//   Serial.begin(115200);  // open a serial connection to your computer
// }
// long int duty = 0;
// float setPoint = 11;
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

//     duty = (15.99 * error - 6.576 * pastError[0] - 7.715 * pastError[1] + 0.9847 * pastOutput[0] + 0.01531 * pastOutput[1]);
//     if (duty > 100) duty = 100;
//     Serial.print(duty);
//     Serial.print(" ");
//     pastError[1] = pastError[0];
//     pastError[0] = error;
//     pastOutput[1] = pastOutput[0];
//     pastOutput[0] = duty;

//     Serial.println(velocity);

//     if (duty < 0) {  //Si es negativo, se debe mandar uno de los PWM a cero y activar el otro
//       ledcWrite(pwmChannel0, 0);
//       ledcWrite(pwmChannel1, -1 * duty * 255.f / 100.f);
//     } else {
//       ledcWrite(pwmChannel1, 0);
//       ledcWrite(pwmChannel0, duty * 255.f / 100.f);
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
  // motoRigth.setVelocity(5);
  // motoLeft.begin();
  // motoLeft.setVelocity(5);
}

void loop() {
  motoRigth.motorRun();
  // motoLeft.motorRun();
  // Serial.println(motoRigth.getAngularVelocity());
  // Serial.println(motoLeft.getAngularVelocity());

}
