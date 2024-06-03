#include <array>
#include "util.h"

#ifndef ROBOT_POSITION_H
#define ROBOT_POSITION_H


int _previousMillisEncoderAngle = 0;
int _intervalEncoderAngle = 10;

class RobotPosition {

private:
  float yPrev = 0;
  float xPrev = 0;
  float phiPrev = 0;

  float L;
  float R;

  // Debug debug;

public:


  RobotPosition(float Li, float Ri) {
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

  float thetaRPrev = 0;
  float thetaLPrev = 0;
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


  void updatePosition(float thetaR, float thetaL) {

    unsigned long _currentMillisEncoderAngle = millis();  // Actual time Variable Angle
    if (_currentMillisEncoderAngle - _previousMillisEncoderAngle >= _intervalEncoderAngle) {
      _previousMillisEncoderAngle = _currentMillisEncoderAngle;

      // Distance Calculation
      float Dr = R * (thetaR - thetaRPrev);
      float Dl = R * (thetaL - thetaLPrev);
      float Dc = (Dr + Dl) / 2.0f;

      // Position Update
      float y = yPrev + Dc * sin(phiPrev);
      float x = xPrev + Dc * cos(phiPrev);

      // Orientation Update
      float phi = phiPrev + ((Dr - Dl) / L);  // Apply deltaTime




      // Normalize the angle to be within [-pi, pi)
      if (phi < -3.1416) {
        phi += 2 * 3.1416;
      } else if (phi >= 3.1416) {
        phi -= 2 * M_PI;
      }

      phiPrev = phi;

      // Update previous values
      thetaRPrev = thetaR;
      thetaLPrev = thetaL;
      yPrev = y;
      xPrev = x;
    }
  }
};

#endif
