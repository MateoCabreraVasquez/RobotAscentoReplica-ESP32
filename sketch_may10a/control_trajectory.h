#include <array>
#include "util.h"

#ifndef CONTROL_TRAJECTORY_H
#define CONTROL_TRAJECTORY_H

/**
 * @brief A class for controlling the trajectory of a vehicle.
 */
class ControlTrajectory {
private:
  // ***************** PARAMETERS *****************

  // CONST
  const float R;           /**< Wheel radius */
  const float L;           /**< Distance between wheels */
  const float _wMaxRigtht; /**< Maximum right wheel velocity */
  const float _wMaxLeft;   /**< Maximum left wheel velocity */

  int timeDeley = 50;

#define N2 4
  float DfistBuffer[N2];
  float DfistBuffer2[N2];
  float DfistBuffer3[N2];
  float DfistBuffer4[N2];

  float Dfist = 0;
  int ind = 0;

  float _wRight = 0;
  float _wLeft = 0;


  // UTIL
  Saturator saturator; /**< Saturator object for saturation */

  // ***************** FUNCTIONS *****************

  /**
         * @brief Proportional controller.
         * @param value The input value.
         * @param P The proportional gain.
         * @return The controlled value.
         */
  float _pController(float value, float P) {
    return value * P;
  }

  /**
         * @brief Get the target angle.
         * @param xTarget Target x-coordinate.
         * @param yTarget Target y-coordinate.
         * @param xReal Current x-coordinate.
         * @param yReal Current y-coordinate.
         * @return The target angle.
         */
  float _getAngleTarget(float xTarget, float yTarget, float xReal, float yReal) {
    float deltaX = xTarget - xReal;
    float deltaY = yTarget - yReal;
    float ang = atan2(deltaY, deltaX);
    if (ang < 0)
        ang= 2*3.1416 + ang;
    if (ang > 2*3.1416)
        ang= ang -2*3.1416;
    return ang;
  }

  /**
         * @brief Get the distance to the goal.
         * @param xTarget Target x-coordinate.
         * @param yTarget Target y-coordinate.
         * @param xReal Current x-coordinate.
         * @param yReal Current y-coordinate.
         * @return The distance to the goal.
         */
  float _getDistanceToGoal(float xTarget, float yTarget, float xReal, float yReal) {
    float deltaX = xTarget - xReal;
    float deltaY = yTarget - yReal;
    return sqrt(deltaX * deltaX + deltaY * deltaY);
  }

  /**
         * @brief Control the angle.
         * @param phiTarget Target angle.
         * @param phiReal Current angle.
         * @return The controlled angular velocity.
         */
  float _controlAngle(float phiTarget, float phiReal) {
    float error = phiTarget - phiReal;
    float p = 10;
    return _pController(error, p);
  }

  /**
         * @brief Saturate the linear velocity.
         * @param wControl Controlled angular velocity.
         * @param velControl Controlled velocity.
         * @return The saturated linear velocity.
         */
  float _saturateLinearVelocity(float wControl, float velControl) {
    float vMaxRigtht = _wMaxRigtht * R - (wControl * L) / 2;
    float vMaxLeft = _wMaxLeft * R + (wControl * L) / 2;

    float vMax = 0.0;
    if (vMaxRigtht > vMaxLeft) {
      vMax = vMaxLeft;
    } else {
      vMax = vMaxRigtht;
    }

    if (vMax < 0.1) {
      vMax = 0.1;
    }

    return saturator.compute(velControl, 0, vMax);
  }

  /**
         * @brief Convert linear velocity to angular velocity.
         * @param v Linear velocity.
         * @param w Angular velocity.
         * @param isRight Flag indicating right wheel.
         * @return The angular velocity.
         */
  float _velToAngVel(float v, float w, bool isRight) {
    float vel = 0.0;
    if (isRight) {
      vel = v + w * L / 2;
    } else {
      vel = v - w * L / 2;
    }
    return vel / R;
  }


public:

  /**
         * @brief Construct a new Control Trajectory object.
         * 
         * @param R Wheel radius.
         * @param L Distance between wheels.
         * @param wMaxRigtht Maximum right wheel velocity.
         * @param wMaxLeft Maximum left wheel velocity.
         */
  ControlTrajectory(float R_, float L_, float wMaxRigtht, float wMaxLeft)
    : R(R_), L(L_), _wMaxRigtht(wMaxRigtht), _wMaxLeft(wMaxLeft) {}



  float getWRigth() {
    return _wRight;
  }

  float getWLeft() {
    return _wLeft;
  }


  /**
         * @brief Compute the control values for the trajectory.
         * @param xTarget Target x-coordinate.
         * @param yTarget Target y-coordinate.
         * @param xReal Current x-coordinate.
         * @param yReal Current y-coordinate.
         * @param phiReal Current angle.
         * @return An array containing the control values for right and left wheels.
         */
  unsigned long _previousMillisEncoderAngle = 0;
  unsigned long _intervalEncoderAngle = 10;

  void update(float xTarget, float yTarget, float xReal, float yReal, float phiReal) {

    // get the variables to reach the target
    float phiTarget = _getAngleTarget(xTarget, yTarget, xReal, yReal);
    float Dfist = _getDistanceToGoal(xTarget, yTarget, xReal, yReal);


    // P controller for the velocity
    float vControl = 4;

    // P controller for the angle
    float wControl = _controlAngle(phiTarget, phiReal);

    // saturate the linear velocity
    float vSatured = _saturateLinearVelocity(wControl, vControl);

    if(Dfist<0.05){
     _wRight = 0;
     _wLeft = 0;

    }
    else{
      _wRight = _velToAngVel(vSatured, wControl, true);
     _wLeft = _velToAngVel(vSatured, wControl, false);

    }

      

    // get the angular velocity for the right and left wheel




    // DfistBuffer[ind] = _wRight;
    // DfistBuffer2[ind] = _wLeft;
    // DfistBuffer3[ind] = wControl;
    // DfistBuffer4[ind] = vSatured;

    // unsigned long _currentMillisEncoderAngle = millis();  // Actual time Variable Angle
    // if (_currentMillisEncoderAngle - _previousMillisEncoderAngle >= _intervalEncoderAngle) {
    //   _previousMillisEncoderAngle = _currentMillisEncoderAngle;
    //   if (ind == N2 - 1) {
    //     for (int i = 0; i < N2; i++) {
    //       Serial.print(DfistBuffer[i]);
    //       Serial.print("  ");
    //       Serial.print(DfistBuffer2[i]);
    //       Serial.print("  ");
    //       Serial.print(DfistBuffer3[i]);
    //       Serial.print("  ");
    //       Serial.print(DfistBuffer4[i]);
    //       Serial.println();
    //     }
    //     ind = 0;
    //   }
    //   ind++;
    // }
  }
};

#endif