#include <array>
#include "util.h"

#ifndef ROBOT_POSITION_H
#define ROBOT_POSITION_H


int _previousMillisEncoderAngle = 0;
int _intervalEncoderAngle = 2;

class RobotPosition {

private:
  float yPrev = 0;
  float xPrev = 0;
  float phiPrev = 0;

  float L;
  float R;

  // Debug debug;

public:


  RobotPosition( float Ri,float Li) {
    L = Li;
    R = Ri;
  }

  float getY() {
    return yPrev;
  }

  float getX() {
    return xPrev;
  }

  float getPhi() {
    return phiPrev;
  }

  float thetaRPrev_ = 0;
  float thetaLPrev_ = 0;
  float prevTime = 0;


  float x = 0;
  float y = 0;
  float phi = 0;


  void updatePosition2(float wR2, float wL2) {
    unsigned long _currentMillisEncoderAngle = millis();  // Actual time Variable Angle
    if (_currentMillisEncoderAngle - _previousMillisEncoderAngle >= _intervalEncoderAngle) {
      float dt = (_currentMillisEncoderAngle - _previousMillisEncoderAngle) / 1000.0f;
      _previousMillisEncoderAngle = _currentMillisEncoderAngle;

      float delta_x = R * (wR2 + wL2) * cos(phiPrev) * dt / 2.0f;
      float delta_y = R * (wR2 + wL2) * sin(phiPrev) * dt / 2.0f;
      float delta_phi = R * (wR2 - wL2) * dt / L;

      xPrev = xPrev + delta_x;
      yPrev = yPrev + delta_y;

      phiPrev = phiPrev + delta_phi;
    }
  }




  void updatePosition(float thetaR, float thetaL, float phi_re) {

    unsigned long _currentMillisEncoderAngle = millis();  // Actual time Variable Angle
    if (_currentMillisEncoderAngle - _previousMillisEncoderAngle >= _intervalEncoderAngle) {
      _previousMillisEncoderAngle = _currentMillisEncoderAngle;

      // Distance Calculation
      float Dr = R * (thetaR - thetaRPrev_);
      float Dl = R * (thetaL - thetaLPrev_);
      float Dc = (Dr + Dl) / 2.0f;

      // Position Update
      float yComp = Dc * sin(phiPrev);
      float xComp=  Dc * cos(phiPrev);

      // debug.runDebug(thetaR, thetaL,yPrev,xPrev );

      float y = yPrev +yComp;
      float x = xPrev +xComp;

      // Orientation Update
      float phi = phiPrev + ((Dr - Dl) / L);  // Apply deltaTime






      // Normalize the angle to be within [-pi, pi)
      if (phi < -2*3.1416) {
        phi += 2 * 3.1416;
      } else if (phi >= 2*3.1416) {
        phi -= 2 * M_PI;
      }

      phiPrev = phi;
      // debug.runDebug(phiPrev, thetaRPrev_,phiPrev , phi_re);

      // Update previous values
      thetaRPrev_ = thetaR;
      thetaLPrev_ = thetaL;
      yPrev = y;
      xPrev = x;
    }
  }
};

#endif
