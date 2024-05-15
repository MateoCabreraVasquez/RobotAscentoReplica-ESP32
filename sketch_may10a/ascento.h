#include "control_trajectory.h"
#include "control_velocity.h"
#include "imu_manager.h"
#include <array>
#include "motors_manager.h"
#ifndef ASCENTO_H
#define ASCENTO_H
class ascento{
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
    ControlTrajectory _controlTrajectory = ControlTrajectory(R, L, _wMaxRigtht, _wMaxLeft);
    ControlVelocity _controlVelocity;

    // imu
    ImuManager _imuManager;

    // manager
    
    void _updatePositionParameters(){
        // COMPLEE ******************************
       
    }

    

public:
    ascento(/* args */);
    ~ascento();


    void runGoToGoal(float xTarget, float yTarget){

        // set the new positions
        _updatePositionParameters();
         
        // compute the go to goal velocities
        std::array<float, 2> angularVelocities = _controlTrajectory.compute(xTarget, yTarget, this->_xReal,  this->_yReal,  this->_phiReal);

        // compute right and left velocities control
       float wRight= _controlVelocity.computeRigth(angularVelocities[0]);
       float wLeft = _controlVelocity.computeLeft(angularVelocities[1]);

     

    }

    void runControlEquilibrium (){

    }


};
#endif



