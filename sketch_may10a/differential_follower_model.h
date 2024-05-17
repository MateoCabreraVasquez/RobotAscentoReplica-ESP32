/**
 * @file DifferentialFollowerModel.h
 * @brief A class for modeling a differential follower robot.
 */

#include <Arduino.h>
#include <math.h>

#ifndef DIFFERENTIAL_FOLLOWER_MODEL_H
#define DIFFERENTIAL_FOLLOWER_MODEL_H

/**
 * @class DifferentialFollowerModel
 * @brief Class to represent the state and behavior of a differential follower robot.
 */
class DifferentialFollowerModel {
private:
    float L = 0.0; /**< Distance between wheels */
    float R = 0.0; /**< Wheel radius */
    float _dtPrev = 0.0; /**< Previous time step */
    float _phiReal = 0.0; /**< Real angle */
    float _xReal = 0.0; /**< Real x-coordinate */
    float _yReal = 0.0; /**< Real y-coordinate */

public:
    /**
     * @brief Constructor for DifferentialFollowerModel.
     * @param L Distance between wheels.
     * @param R Wheel radius.
     */
    DifferentialFollowerModel(float L, float R): L(L), R(R) {}

    /**
     * @brief Updates the position parameters of the robot.
     * @param wRightWheel Angular velocity of the right wheel.
     * @param wLeftWheel Angular velocity of the left wheel.
     */
    void updatePositionParameters(float wRightWheel, float wLeftWheel) {
        // get dt
        float time = millis() / 1000.0;
        float dt = time - _dtPrev;
        _dtPrev = time;

        // get absolute velocities
        float v = ((wRightWheel + wLeftWheel) * R) / 2.0;
        float w = ((wRightWheel - wLeftWheel) * R) / L;

        // get the angle
        _phiReal = _phiReal + w * dt;

        // linear velocities
        float velX = v * cos(_phiReal);
        float velY = v * sin(_phiReal);

        // positions
        _xReal = _xReal + velX * dt;
        _yReal = _yReal + velY * dt;
    }

    /**
     * @brief Gets the real x-coordinate of the robot.
     * @return Real x-coordinate.
     */
    float getRealX() {
        return _xReal;
    }

    /**
     * @brief Gets the real y-coordinate of the robot.
     * @return Real y-coordinate.
     */
    float getRealY() {
        return _yReal;
    }
};

#endif // DIFFERENTIAL_FOLLOWER_MODEL_H
