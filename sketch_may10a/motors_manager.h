// #ifndef MOTORS_MANAGER_H
// #define MOTORS_MANAGER_H

// class MotorsManager{
// private:

//     const long _INTERVAL_ENCODER_ANGLE = 50;  /**<Delay for the Angle Data*/
//     unsigned long previousMillisEncoderAngle = 0;  /**<Previous millis for the Angle Data*/
    
//     volatile int n = 0;            /**< store the pulse */
//     volatile byte _actualAB = 0;    /**< Actual Value of AB*/
//     volatile byte _previousAB = 0;  /**< previous Value of AB*/


//     double P = 0;                         //Relative Position in grades
//     double R = 4451;  //Resolution of Encoder for a quadrupel precision
//     double Pact = 0;//Relative Position in grades
//     double Pant = 0;//Relative Position in grades
//     const int resolution = 8;  // set PWM resolution
//     int frecuency = 21000;

//     // CLASS PARAMETERS    
//     short _pinApulse = 0; /**< Pulse A  pin, need interrupt for count rising flanks*/
//     short _pinBpulse = 0; /**< Pulse B  pin, need interrupt for count rising flanks*/
//     double _R = 6538;
//     int _pwmChannel = 1;
//     int _modulePWMPin = 15;

//     float _pastError[2] = {0,0};
//     float _pastOutput[2] = {0,0};

//     long int duty = 0;  //  REVISAR
//     float setPoint = 11;



// //Encoder
// static void _pulseinterrupt() {
//   _previousAB = _actualAB;
//   if (digitalRead(_pinApulse)) bitSet(_actualAB, 1);
//   else bitClear(_actualAB, 1);
//   if (digitalRead(_pinBpulse)) bitSet(_actualAB, 0);
//   else bitClear(_actualAB, 0);

//   //direction of movement
//   if (_previousAB == 2 && _actualAB == 0) n++;
//   if (_previousAB == 0 && _actualAB == 1) n++;
//   if (_previousAB == 3 && _actualAB == 2) n++;
//   if (_previousAB == 1 && _actualAB == 3) n++;

//   if (_previousAB == 1 && _actualAB == 0) n--;
//   if (_previousAB == 3 && _actualAB == 1) n--;
//   if (_previousAB == 0 && _actualAB == 2) n--;
//   if (_previousAB == 2 && _actualAB == 3) n--;
// }





//     /* data */
// public:
//     void begin(){   
       
//         //Encoder
//         pinMode(_pinApulse, INPUT);
//         pinMode(_pinBpulse, INPUT);
//         attachInterrupt(digitalPinToInterrupt(_pinApulse), _pulseinterrupt, CHANGE);  // Change Flanks pulse A
//         attachInterrupt(digitalPinToInterrupt(_pinBpulse), _pulseinterrupt, CHANGE);   // Change Flanks pulse B
//         ledcSetup(_pwmChannel, frecuency, resolution);                                // define the PWM Setup
//         ledcAttachPin(_modulePWMPin, _pwmChannel);
                
//     }


//     void run(){
//         unsigned long currentMillisEncoderAngle = millis();  // Actual time Variable Angle
//         if (currentMillisEncoderAngle - previousMillisEncoderAngle >= _INTERVAL_ENCODER_ANGLE) {
//             float delta =  (currentMillisEncoderAngle - previousMillisEncoderAngle)/1000.f;
//             previousMillisEncoderAngle = currentMillisEncoderAngle;
//             //Position
//             Pant = Pact;
//             Pact = (n*360.0)/R;
//             double velocity = (3.14159/180.f)*(Pact - Pant)/delta;
//             float error = setPoint-velocity;
//             duty = (13.28*error-6.997*_pastError[0]-4.867*_pastError[1]+0.9404*_pastOutput[0]+0.05964*_pastOutput[1]);
//             if (duty > 100) duty = 100;
//             Serial.print(duty);
//             Serial.print(" ");
//             _pastError[1] = _pastError[0];
//             _pastError[0] = error;
//             _pastOutput[1] =_pastOutput[0];
//             _pastOutput[0] = duty;

//             Serial.println(velocity);

//             ledcWrite(_pwmChannel, duty*255.f/100.f);

//         }
//     }


//     float getSpeed(){
//         return 0.0;
//     }


//     void setSpeed(int speed){}


// };


// #endif