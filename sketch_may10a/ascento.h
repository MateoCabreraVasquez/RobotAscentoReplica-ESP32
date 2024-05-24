#include "control_trajectory.h"
#include "control_velocity.h"
#include "imu_manager.h"
#include <array>
#include "motor.h"
#include "differential_follower_model.h"

#ifndef ASCENTO_H
#define ASCENTO_H
class ascento
{
private:
    // parameters
    const float R = 0.016;
    const float L = 0.024;
    const float _wMaxRigtht = 11.0;
    const float _wMaxLeft = 11.0;
    const float _xReal = 0.0;
    const float _yReal = 0.0;
    const float _phiReal = 0.0;

    // control
    ControlTrajectory _controlTrajectory = ControlTrajectory(
        R,            //R
        L,            //L
        _wMaxRigtht,  //wMaxRigtht,
         _wMaxLeft    //wMaxLeft
        );

    ControlVelocity _controlVelocity;

    // imu
    ImuManager _imuManager;

    // // wheels
    // Motor motoRigth = Motor(
    //     32,   // pinPulseA
    //     33,   // pinPulseB
    //     2,    // modulePWMPin
    //     5,    // modulePWMPinNeg
    //     0,    // pwmChannel0
    //     1,    // pwmChannel1
    //     6538, // R,
    //     R     // radioWheel
    // );

    //   Motor motorLeft = Motor(
    //     25,   // pinPulseA
    //     26,   // pinPulseB
    //     15,   // modulePWMPin
    //     18,   // modulePWMPinNeg
    //     2,    // pwmChannel0
    //     3,    // pwmChannel1
    //     4451, // R,
    //     R     // radioWheel
    // );

    // differential model
    DifferentialFollowerModel _differentialFollowerModel = DifferentialFollowerModel(
        L, // L
        R  // R
    );



    // manager
    
    void _updatePositionParameters(){
        // COMPLEE ******************************
       
    }

    

public:

    void runGoToGoal(float xTarget, float yTarget){
        // set the new positions
        _updatePositionParameters();

        // compute the go to goal velocities
        std::array<float, 2> angularVelocities = _controlTrajectory.compute(xTarget, yTarget, this->_xReal, this->_yReal, this->_phiReal);

        // compute right and left velocities control
        // float wRight = _controlVelocity.computeRigth(angularVelocities[0]);
        // float wLeft = _controlVelocity.computeLeft(angularVelocities[1]);

     

    }

    void runControlEquilibrium (){

    }
};
#endif
