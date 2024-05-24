#include <Arduino.h>

#ifndef MOTOR_H
#define MOTOR_H


/**
 * @class Motor
 * @brief A class representing a motor controller.
 */
class Motor {

  // **************************************************************
  //                            PRIVATE
  // **************************************************************
private:

  // class parameters
  static int pinApulse;  //Digital pin, need interrupt for count rising flanks
  static int pinBpulse;
  float P;                   //Relative Position in grades
  float R;                   //Resolution of Encoder for a quadrupel precision

  const long intervalEncoderAngle = 50;  //Delay for the Angle Data
  unsigned long previousMillisEncoderAngle = 0;

  static int n;                     //store the pulse
  static volatile byte actualAB;    //Actual Value of AB
  static volatile byte previousAB;  //previous Value of AB


  float Pact = 0;            //Relative Position in grades
  float Pant = 0;            //Relative Position in grades
  const int resolution = 8;  // set PWM resolution
  int frecuency = 21000;
  int pwmChannel0;
  int pwmChannel1;
  int modulePWMPin;
  int modulePWMPinNeg;

  float u0 = 0, u1 = 0, u2 = 0, y0 = 0, y1 = 0;

  float pastError[2] = { 0, 0 };
  float pastOutput[2] = { 0, 0 };



  long int duty = 0;
  float setPoint = 7;



  // **************************************************************
  //                             PUBLIC
  // **************************************************************

public:
  const unsigned long _INTERVAL_ENCODER_ANGLE = 50; /**< Interval for encoder angle */
  const int _RESOLUTION = 8;                        /**< PWM resolution */
  const int _FREQUENCY = 21000;                     /**< PWM frequency */

  /**
     * @brief Constructor for Motor class.
     * @param pinPulseA Pin for pulse A.
     * @param pinPulseB Pin for pulse B.
     * @param modulePWMPin Module PWM pin.
     * @param modulePWMPinNeg Negative module PWM pin.
     * @param pwmChannel0 PWM channel 0.
     * @param pwmChannel1 PWM channel 1.
     * @param R Gear ratio.
     * @param radioWheel wheel readio.
     */
  Motor(int pinPulseA, int pinPulseB, int modulePWMPin, int modulePWMPinNeg, int pwmChannel0, int pwmChannel1, float _R, float _u0, float _u1, float _u2, float _y0, float _y1);


  /**
     * @brief Set the velocity.
     * @param duty The duty cycle.
     */
  void setVelocity(float velocity) {
    setPoint = velocity;
  }

  /**
     * @brief Get the velocity.
     * @return The velocity.
     */
  float getLinealVelocity();

  /**
     * @brief Get the angular velocity.
     * @return The angular velocity.
     */
  float getAngularVelocity();

  /**
     * @brief Get the angle position.
     * @return The angle position.
     */
  float getAnglePos();

  /**
     * @brief Get the lineal position.
     * @return The lineal position.
     */
  float getLinealPos();

  float setPidParameters(float u0, float u1, float u2, float y0, float y1);

  /**
     * @brief Initialize the motor.
     */
  void begin();

  /**
     * @brief Run the motor controller.
     */
  void motorRun();

  /**
     * @brief Static ISR for pulse interruption.
     */
  static void pulseinterrupt();
};


// int Motor::_pinApulse = 0;
// int Motor::_pinBpulse = 0;
// volatile byte Motor::_previousAB = 0;
// volatile byte Motor::_actualAB = 0;
// volatile int Motor::_n = 0;




Motor::Motor(int _pinPulseA, int _pinPulseB, int _modulePWMPin, int _modulePWMPinNeg, int _pwmChannel0, int _pwmChannel1, float _R, float _u0, float _u1, float _u2, float _y0, float _y1) {
  pinApulse = _pinPulseA;
  pinBpulse = _pinPulseB;
  modulePWMPin = _modulePWMPin;
  modulePWMPinNeg = _modulePWMPinNeg;
  R = _R;
  pwmChannel0 = _pwmChannel0;
  pwmChannel1 = _pwmChannel1;

  u0 = _u0;
  u1 = _u1;
  u2 = _u2;
  y0 = _y0;
  y1 = _y1;
}

// **************************************************************
//                     MANGER MOTOR  FUNCTIONS
// **************************************************************






float Motor::getLinealVelocity() {
  return 0.1;
}

float Motor::getAngularVelocity() {
  return 0.1;
}

float Motor::getAnglePos() {
  return 0.1;
}

float Motor::getLinealPos() {
  return 0.1;
}

float Motor::setPidParameters(float u0, float u1, float u2, float y0, float y1) {
}

// **************************************************************
//                          SETUP MOTOR
// **************************************************************

void Motor::begin() {
  //Encoder
  pinMode(pinApulse, INPUT);
  pinMode(pinBpulse, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinApulse), pulseinterrupt, CHANGE);  // Change Flanks pulse A
  attachInterrupt(digitalPinToInterrupt(pinBpulse), pulseinterrupt, CHANGE);  // Change Flanks pulse B
  ledcSetup(pwmChannel0, frecuency, resolution);                              // define the PWM Setup
  ledcSetup(pwmChannel1, frecuency, resolution);
  ledcAttachPin(modulePWMPin, pwmChannel0);
  ledcAttachPin(modulePWMPinNeg, pwmChannel1);
  Serial.begin(115200);  // open a serial connection to your computer
}


// **************************************************************
//                           MOTOR RUN
// **************************************************************

void Motor::motorRun() {
  unsigned long currentMillisEncoderAngle = millis();  // Actual time Variable Angle
  if (currentMillisEncoderAngle - previousMillisEncoderAngle >= intervalEncoderAngle) {
    float delta = (currentMillisEncoderAngle - previousMillisEncoderAngle) / 1000.f;
    previousMillisEncoderAngle = currentMillisEncoderAngle;
    //Position
    Pant = Pact;
    Pact = (n * 360.0) / R;

    double velocity = (3.14159 / 180.f) * (Pact - Pant) / delta;

    float error = setPoint - velocity;

    duty = (u0 * error + u1 * pastError[0] + u2 * pastError[1] + y0 * pastOutput[0] + y1 * pastOutput[1]);
    if (duty > 100) duty = 100;
    Serial.print(duty);
    Serial.print(" ");
    pastError[1] = pastError[0];
    pastError[0] = error;
    pastOutput[1] = pastOutput[0];
    pastOutput[0] = duty;

    Serial.println(velocity);

    if (duty < 0) {  //Si es negativo, se debe mandar uno de los PWM a cero y activar el otro
      ledcWrite(pwmChannel0, 0);
      ledcWrite(pwmChannel1, -1 * duty * 255.f / 100.f);
    } else {
      ledcWrite(pwmChannel1, 0);
      ledcWrite(pwmChannel0, duty * 255.f / 100.f);
    }
  }
}

// **************************************************************
//                      PULSE INTERRUPTION
// **************************************************************
int Motor::pinApulse = 32;
int Motor::pinBpulse = 33;
volatile byte Motor::actualAB = 0;
volatile byte Motor::previousAB = 0;
int Motor::n = 0;

void Motor::pulseinterrupt() {

  previousAB = actualAB;
  if (digitalRead(pinApulse)) bitSet(actualAB, 1);
  else bitClear(actualAB, 1);
  if (digitalRead(pinBpulse)) bitSet(actualAB, 0);
  else bitClear(actualAB, 0);

  //direction of movement
  if (previousAB == 2 && actualAB == 0) n++;
  if (previousAB == 0 && actualAB == 1) n++;
  if (previousAB == 3 && actualAB == 2) n++;
  if (previousAB == 1 && actualAB == 3) n++;

  if (previousAB == 1 && actualAB == 0) n--;
  if (previousAB == 3 && actualAB == 1) n--;
  if (previousAB == 0 && actualAB == 2) n--;
  if (previousAB == 2 && actualAB == 3) n--;
}


#endif
