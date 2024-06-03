#include <Arduino.h>

#ifndef MOTOR2_H
#define MOTOR2_H


/**
 * @class Motor
 * @brief A class representing a motor controller.
 */
class Motor2 {

  // **************************************************************
  //                            PRIVATE
  // **************************************************************
private:

  int ind=0;

  // class parameters
  static int _pinApulse;  //Digital pin, need interrupt for count rising flanks
  static int _pinBpulse;
  int _modulePWMPin;
  int _modulePWMPinNeg;
  int _pwmChannel0;
  int _pwmChannel1;
  float _R; //Resolution of Encoder for a quadrupel precision
  float _u0, _u1, _u2, _y0, _y1;
  // float _P;  //Relative Position in grades

  const long _intervalEncoderAngle = 2;  //Delay for the Angle Data
  unsigned long _previousMillisEncoderAngle = 0;

  static int _n;                     //store the pulse
  static volatile byte _actualAB;    //Actual Value of AB
  static volatile byte _previousAB;  //previous Value of AB

  float _Pact = 0;            //Relative Position in grades
  float _Pant = 0;            //Relative Position in grades
  const int _resolution = 8;  // set PWM resolution
  int _frecuency = 21000;
  float _velocity=0;
 


  float _pastError[2] = { 0, 0 };
  float _pastOutput[2] = { 0, 0 };


  int N=1000;
  float data[1000];
  float data2[1000];


  long int _duty = 0;
  float _setPoint = 7;

  float _xk1 = 0;
  float _xk2 = 0;
  float _xk3 = 0;

  float observer(float duty, float angle){

    float _xk1_1=_xk1;
    float _xk2_1=_xk2;
    float _xk3_1=_xk3;

    _xk1 =   0.662452316388307*_xk1_1   +0.002000000000000*_xk2_1        +0*_xk3_1      +0.000002547422*duty     +0.337547683611694*angle;
    _xk2 = -18.054154356557294*_xk1_1   +0.983547683612005*_xk2_1   +1.0000*_xk3_1    +0.002519443258367*duty    +18.054154356557294*angle;
    _xk3 =  -0.815011249997974*_xk1_1                    +0*_xk2_1  +1.0000*_xk3_1                     +0*duty    +0.815011249997974*angle;
    return 0*_xk1_1+1*_xk2_1+0*_xk3_1;

  }

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
  Motor2(int pinPulseA, int pinPulseB, int modulePWMPin, int modulePWMPinNeg, int pwmChannel0, int pwmChannel1, float R, float u0, float u1, float u2, float y0, float y1) {
    _pinApulse = pinPulseA;
    _pinBpulse = pinPulseB;
    _modulePWMPin = modulePWMPin;
    _modulePWMPinNeg = modulePWMPinNeg;
    _R = R;
    _pwmChannel0 = pwmChannel0;
    _pwmChannel1 = pwmChannel1;

    _u0 = u0;
    _u1 = u1;
    _u2 = u2;
    _y0 = y0;
    _y1 = y1;
  }

void setR(float R){
  _R=R;
}

  /**
     * @brief Set the velocity.
     * @param duty The duty cycle.
     */
  void setVelocity(float velocity) {
    _setPoint = velocity;
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
  float getAngularVelocity(){
    return _velocity;
  }

  /**
     * @brief Get the angle position.
     * @return The angle position.
     */
    float getAnglePos(){
    return _Pact;
  }

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






// **************************************************************
//                     MANGER MOTOR  FUNCTIONS
// **************************************************************






float Motor2::getLinealVelocity() {
  return 0.1;
}



float Motor2::getLinealPos() {
  return 0.1;
}

float Motor2::setPidParameters(float u0, float u1, float u2, float y0, float y1) {
}

// **************************************************************
//                          SETUP MOTOR
// **************************************************************

void Motor2::begin() {
  //Encoder
  pinMode(_pinApulse, INPUT);
  pinMode(_pinBpulse, INPUT);
  attachInterrupt(digitalPinToInterrupt(_pinApulse), pulseinterrupt, CHANGE);  // Change Flanks pulse A
  attachInterrupt(digitalPinToInterrupt(_pinBpulse), pulseinterrupt, CHANGE);  // Change Flanks pulse B
  ledcSetup(_pwmChannel0, _frecuency, _resolution);                              // define the PWM Setup
  ledcSetup(_pwmChannel1, _frecuency, _resolution);
  ledcAttachPin(_modulePWMPin, _pwmChannel0);
  ledcAttachPin(_modulePWMPinNeg, _pwmChannel1);
  Serial.begin(115200);  // open a serial connection to your computer
}


// **************************************************************
//                           MOTOR RUN
// **************************************************************

void Motor2::motorRun() {
  unsigned long _currentMillisEncoderAngle = millis();  // Actual time Variable Angle
  if (_currentMillisEncoderAngle - _previousMillisEncoderAngle >= _intervalEncoderAngle) {
    float _delta = (_currentMillisEncoderAngle - _previousMillisEncoderAngle) / 1000.f;
    _previousMillisEncoderAngle = _currentMillisEncoderAngle;
    //Position
    _Pant = _Pact;
    _Pact = (_n * 2*3.1416) / _R;

    // _velocity=(3.14159 / 180.f) * (_Pact - _Pant) / _delta;


    float _error = _setPoint - _velocity;

    _duty = (_u0 * _error + _u1 * _pastError[0] + _u2 * _pastError[1] + _y0 * _pastOutput[0] + _y1 * _pastOutput[1]);
    

    if (_duty > 100) _duty = 100;
    else if(_duty<-100)_duty = -100;

     _velocity=observer(_duty, _Pact);

//  data[ind]=_velocity;
//   data2[ind]=_duty;

//     if(ind<N)
//       ind ++;

//     if(ind==N){
//       for(int i=0; i<N;i++){
//         Serial.print(data[i]);
//         Serial.print("   ");
//         Serial.println(data2[i]);

//       }
//     }

    


    _pastError[1] = _pastError[0];
    _pastError[0] = _error;
    _pastOutput[1] = _pastOutput[0];
    _pastOutput[0] = _duty;


    if (_duty < 0) {  //Si es negativo, se debe mandar uno de los PWM a cero y activar el otro
      ledcWrite(_pwmChannel0, 0);
      ledcWrite(_pwmChannel1, -1 * _duty * 255.f / 100.f);
    } else {
      ledcWrite(_pwmChannel1, 0);
      ledcWrite(_pwmChannel0, _duty * 255.f / 100.f);
    }
  }
}

// **************************************************************
//                      PULSE INTERRUPTION
// **************************************************************
int Motor2::_pinApulse = 32;
int Motor2::_pinBpulse = 33;
volatile byte Motor2::_actualAB = 0;
volatile byte Motor2::_previousAB = 0;
int Motor2::_n = 0;

void Motor2::pulseinterrupt() {

  _previousAB = _actualAB;
  if (digitalRead(_pinApulse)) bitSet(_actualAB, 1);
  else bitClear(_actualAB, 1);
  if (digitalRead(_pinBpulse)) bitSet(_actualAB, 0);
  else bitClear(_actualAB, 0);

  //direction of movement
  if (_previousAB == 2 && _actualAB == 0) _n++;
  if (_previousAB == 0 && _actualAB == 1) _n++;
  if (_previousAB == 3 && _actualAB == 2) _n++;
  if (_previousAB == 1 && _actualAB == 3) _n++;

  if (_previousAB == 1 && _actualAB == 0) _n--;
  if (_previousAB == 3 && _actualAB == 1) _n--;
  if (_previousAB == 0 && _actualAB == 2) _n--;
  if (_previousAB == 2 && _actualAB == 3) _n--;
}


#endif
