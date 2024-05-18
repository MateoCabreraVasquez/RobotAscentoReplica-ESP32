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
    static int _pinApulse; /**< Pin for pulse A */
    static int _pinBpulse; /**< Pin for pulse B */

    int _modulePWMPin; /**< Module PWM pin */
    int _modulePWMPinNeg; /**< Negative module PWM pin */

    double _R; /**< Gear ratio */

    int _pwmChannel0; /**< PWM channel 0 */
    int _pwmChannel1; /**< PWM channel 1 */

    float _velocityAngular = 0.0; /**< Velocity */
    float _radioWheel = 0.0; /**< Radio wheel */

    // Inner parameters
    unsigned long _previousMillisEncoderAngle = 0;
    static int _n; /**< Store the pulse */

    static volatile byte _actualAB; /**< Actual value of AB */
    static volatile byte _previousAB; /**< Previous value of AB */

    double _posAngular = 0; /**< Current angle position */
    double _posAngularPrev = 0; /**< Previous angle position */

    long int _duty = 0; /**< Duty cycle */

    float pastError[2] = { 0, 0 }; /**< Array to store past errors */
    float pastOutput[2] = { 0, 0 }; /**< Array to store past outputs */



    // **************************************************************
    //                             PUBLIC 
    // **************************************************************

public:
    const unsigned long _INTERVAL_ENCODER_ANGLE = 50000; /**< Interval for encoder angle */
    const int _RESOLUTION = 8; /**< PWM resolution */
    const int _FREQUENCY = 21000; /**< PWM frequency */

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
    Motor(int pinPulseA, int pinPulseB, int modulePWMPin, int modulePWMPinNeg, int pwmChannel0, int pwmChannel1, float R, float radioWheel);


    /**
     * @brief Set the velocity.
     * @param duty The duty cycle.
     */	
    void setVelocity(int  duty);

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


int Motor::_pinApulse = 0;
int Motor::_pinBpulse = 0;
volatile byte Motor::_previousAB = 0; 
volatile byte Motor::_actualAB = 0; 
int Motor::_n=0;


Motor::Motor(int pinPulseA, int pinPulseB, int modulePWMPin, int modulePWMPinNeg, int pwmChannel0, int pwmChannel1, float R, float radioWheel) {
    _pinApulse = pinPulseA;
    _pinBpulse = pinPulseB;
    _modulePWMPin = modulePWMPin;
    _modulePWMPinNeg = modulePWMPinNeg;
    _R = R;
    _radioWheel = radioWheel;
    _pwmChannel0=pwmChannel0;
    _pwmChannel1=pwmChannel1;
}

// **************************************************************
//                     MANGER MOTOR  FUNCTIONS
// **************************************************************

void Motor::setVelocity(int  duty) {_duty = duty;}

float Motor::getLinealVelocity() {return _velocityAngular*_radioWheel;}

float Motor::getAngularVelocity() {return  _velocityAngular;}

float Motor::getAnglePos() {return _velocityAngular;}

float Motor::getLinealPos() {return _velocityAngular*_radioWheel;}


// **************************************************************
//                          SETUP MOTOR
// **************************************************************

void Motor::begin() {
    pinMode(_pinApulse, INPUT);
    pinMode(_pinBpulse, INPUT);
    attachInterrupt(digitalPinToInterrupt(_pinApulse), pulseinterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinBpulse), pulseinterrupt, CHANGE);
    ledcSetup(_pwmChannel0, _FREQUENCY, _RESOLUTION);                              
    ledcSetup(_pwmChannel1, _FREQUENCY, _RESOLUTION);   
    ledcAttachPin(_modulePWMPin, _pwmChannel0);
    ledcAttachPin(_modulePWMPinNeg, _pwmChannel1);
}


// **************************************************************
//                           MOTOR RUN
// **************************************************************

void Motor::motorRun(){
unsigned long currentMillisEncoderAngle = micros();  
    //Serial.println("1");
  if (currentMillisEncoderAngle - _previousMillisEncoderAngle >= _INTERVAL_ENCODER_ANGLE) {

    float delta = (currentMillisEncoderAngle - _previousMillisEncoderAngle) / 1000000.f;
    
    // encoder lecures
    _previousMillisEncoderAngle = currentMillisEncoderAngle;
    _posAngularPrev = _posAngular;
    _posAngular = (_n * 360.0) / _R;
    _velocityAngular = (3.14159 / 180.f) * (_posAngular - _posAngularPrev) / delta;

    // direction of movement
    if (_duty < 0) { 
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

void Motor::pulseinterrupt() {
       
        _previousAB = _actualAB;
        if (digitalRead(_pinApulse))
            bitSet(_actualAB, 1);
        else
            bitClear(_actualAB, 1);
        if (digitalRead(_pinBpulse))
            bitSet(_actualAB, 0);
        else
            bitClear(_actualAB, 0);

        // direction of movement
        if (_previousAB == 2 && _actualAB == 0)
            _n++;
        if (_previousAB == 0 && _actualAB == 1)
            _n++;
        if (_previousAB == 3 && _actualAB == 2)
            _n++;
        if (_previousAB == 1 && _actualAB == 3)
            _n++;

        if (_previousAB == 1 && _actualAB == 0)
            _n--;
        if (_previousAB == 3 && _actualAB == 1)
            _n--;
        if (_previousAB == 0 && _actualAB == 2)
            _n--;
        if (_previousAB == 2 && _actualAB == 3)
            _n--;

}


#endif
