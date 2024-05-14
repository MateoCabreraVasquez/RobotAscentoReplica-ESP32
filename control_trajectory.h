#include <array>
#include "util.h"

#ifndef CONTROL_TRAJECTORY_H
#define CONTROL_TRAJECTORY_H

/**
 * @brief A class for controlling the trajectory of a vehicle.
 */
class ControlTrajectory{
    private:
        // ***************** PARAMETERS *****************

        // CONST
        const float R = 0.0; /**< Wheel radius */
        const float L = 0.0; /**< Distance between wheels */
        const float _wMaxRigtht = 11.0; /**< Maximum right wheel velocity */
        const float _wMaxLeft = 11.0; /**< Maximum left wheel velocity */

        // UTIL
        Saturator saturator; /**< Saturator object for saturation */

        // ***************** FUNCTIONS *****************

        /**
         * @brief Proportional controller.
         * @param value The input value.
         * @param P The proportional gain.
         * @return The controlled value.
         */
        float _pController(float value, float P){
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
        float _getAngleTarget(float xTarget, float yTarget, float xReal, float yReal){
            float deltaX = xTarget - xReal;
            float deltaY = yTarget - yReal;
            return atan2(deltaY, deltaX);
        }

        /**
         * @brief Get the distance to the goal.
         * @param xTarget Target x-coordinate.
         * @param yTarget Target y-coordinate.
         * @param xReal Current x-coordinate.
         * @param yReal Current y-coordinate.
         * @return The distance to the goal.
         */
        float _getDistanceToGoal(float xTarget, float yTarget, float xReal, float yReal){
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
        float _controlAngle(float phiTarget, float phiReal){
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
        float _saturateLinearVelocity( float wControl, float velControl){
            float vMaxRigtht = _wMaxRigtht * R - (wControl * L) / 2;
            float vMaxLeft = _wMaxLeft * R + (wControl * L) / 2;

            float vMax = 0.0;
            if (vMaxRigtht > vMaxLeft){
                vMax = vMaxLeft;
            }
            else{
                vMax = vMaxRigtht;
            }

            if (vMax < 0.1){
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
        float _velToAngVel(float v, float w, bool isRight){
            float vel = 0.0;
            if (isRight){
                 vel = v + w * L / 2;
            }
            else{
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
        ControlTrajectory(float R, float L, float wMaxRigtht, float wMaxLeft) : R(R), L(L), _wMaxRigtht(wMaxRigtht), _wMaxLeft(wMaxLeft){}


        /**
         * @brief Compute the control values for the trajectory.
         * @param xTarget Target x-coordinate.
         * @param yTarget Target y-coordinate.
         * @param xReal Current x-coordinate.
         * @param yReal Current y-coordinate.
         * @param phiReal Current angle.
         * @return An array containing the control values for right and left wheels.
         */
        std::array<float, 2> cumpute(float xTarget, float yTarget, float xReal, float yReal, float phiReal){
            
            // get the variables to reach the target
            float phiTarget = _getAngleTarget(xTarget, yTarget, xReal, yReal);
            float Dfist= _getDistanceToGoal(xTarget, yTarget, xReal, yReal);

            // P controller for the velocity
            float vControl = _pController(Dfist, 0.5);

            // P controller for the angle
            float wControl = _controlAngle(phiTarget, phiReal);

            // saturate the linear velocity
            float vSatured = _saturateLinearVelocity(wControl, vControl);

            // get the angular velocity for the right and left wheel
            float wRight = _velToAngVel(vSatured, wControl, true);
            float wLeft = _velToAngVel(vSatured, wControl, false);

            // add the values to the array and return it
            std::array<float, 2> res = {wRight, wLeft};
            return res;
        }

};

#endif