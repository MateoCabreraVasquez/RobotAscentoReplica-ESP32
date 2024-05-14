#include "Arduino.h"
#include "MPU9250.h"

#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

/**
 * @brief A class for managing IMU (Inertial Measurement Unit) data.
 */
class ImuManager{

private:
    MPU9250 _mpu; /**< MPU9250 object for IMU communication */

    float _pitch = 0.0; /**< Pitch angle */
    float _roll = 0.0; /**< Roll angle */
    float _yaw = 0.0; /**< Yaw angle */

    float _eulerX = 0.0; /**< Euler angle X */
    float _eulerY = 0.0; /**< Euler angle Y */
    float _eulerZ = 0.0; /**< Euler angle Z */

    float _angAccelX = 0.0; /**< Angular acceleration X */
    float _angAccelY = 0.0; /**< Angular acceleration Y */
    float _angAccelZ = 0.0; /**< Angular acceleration Z */

    float _linAccelX = 0.0; /**< Linear acceleration X */
    float _linAccelY = 0.0; /**< Linear acceleration Y */
    float _linAccelZ = 0.0; /**< Linear acceleration Z */

    const float _pitchOffset = 0.0; /**< Pitch offset */
    const float _rollOffset = 0.0; /**< Roll offset */
    const float _yawOffset = 0.0; /**< Yaw offset */

    const float _eulerXOffset = 0.0; /**< Euler angle X offset */
    const float _eulerYOffset = 0.0; /**< Euler angle Y offset */
    const float _eulerZOffset = 0.0; /**< Euler angle Z offset */

    const float _angAccelXOffset = 0.0; /**< Angular acceleration X offset */
    const float _angAccelYOffset = 0.0; /**< Angular acceleration Y offset */
    const float _angAccelZOffset = 0.0; /**< Angular acceleration Z offset */

    const float _linAccelXOffset = 0.0; /**< Linear acceleration X offset */
    const float _linAccelYOffset = 0.0; /**< Linear acceleration Y offset */
    const float _linAccelZOffset = 0.0; /**< Linear acceleration Z offset */

public:
    /**
     * @brief Initializes the IMU Manager and connects to MPU9250 sensor.
     */
    void begin(){
        Wire.begin();
        delay(2000); 
    
        if (!(_mpu.setup(0x68))) {  // Change to your own address
            while (1) {
                Serial.println("MPU connection failed");
                delay(5000);
            }
        }
    }


    
    /**
     * @brief Update IMU data.
     * 
     * This function updates the IMU data if available.
     * 
     * @note This function should be called periodically to update the IMU data.
     */
    void update(){

        if (_mpu.update()) {
            static uint32_t prev_ms = millis();
            if (millis() > prev_ms + 25) {
                prev_ms = millis();

                this->_pitch = _mpu.getPitch();
                this->_roll = _mpu.getRoll();
                this->_yaw = _mpu.getYaw();

                this->_eulerX = _mpu.getEulerX();
                this->_eulerY = _mpu.getEulerY();
                this->_eulerZ = _mpu.getEulerZ();

                this->_angAccelX = _mpu.getAccX();
                this->_angAccelY = _mpu.getAccY();
                this->_angAccelZ = _mpu.getAccZ();

                this->_linAccelX = _mpu.getLinearAccX();
                this->_linAccelY = _mpu.getLinearAccY();
                this->_linAccelZ = _mpu.getLinearAccZ();
            }
        }
    }



    /**
     * @brief Calibrates the MPU9250 sensor.
     */
    void calibrate(){
      Serial.println("Calib. will start in 5sec.");
      Serial.println("Leave the device on a flat plane.");
      _mpu.verbose(true);
      delay(5000);
      _mpu.calibrateAccelGyro();
      _mpu.verbose(false);
    }



    // *********** PITCH ROLL YAW *********** //

    /**
     * @brief Get pitch angle from MPU9250 sensor.
     * @return Pitch angle.
     */
    float getPitch(){
        return this->_pitch + this->_pitchOffset;
    }

    /**
     * @brief Get roll angle from MPU9250 sensor.
     * @return Roll angle.
     */
    float getRoll(){
        return this->_roll + this->_rollOffset;
    }

    /**
     * @brief Get yaw angle from MPU9250 sensor.
     * @return Yaw angle.
     */
    float getYaw(){
        return this->_yaw + this->_yawOffset;
    }




    // ************* EULER ANGLES ************* //

    /**
     * @brief Get Euler X angle from MPU9250 sensor.
     * @return Euler X angle.
     */
    float getEulerX(){
        return _mpu.getEulerX() + this->_eulerXOffset;
    }

    /**
     * @brief Get Euler Y angle from MPU9250 sensor.
     * @return Euler Y angle.
     */
    float getEulerY(){
        return _mpu.getEulerY() + this->_eulerYOffset;
    }

    /**
     * @brief Get Euler Z angle from MPU9250 sensor.
     * @return Euler Z angle.
     */
    float getEulerZ(){
        return _mpu.getEulerZ() + this->_eulerZOffset;
    }




    // ************* ANGULAR ACCELERATION ************* //

    /**
     * @brief Get angular acceleration along X-axis from MPU9250 sensor.
     * @return Angular acceleration along X-axis.
     */
    float getAngAccelX(){
        return _mpu.getAccX() + this->_angAccelXOffset;
    }

    /**
     * @brief Get angular acceleration along Y-axis from MPU9250 sensor.
     * @return Angular acceleration along Y-axis.
     */
    float getAngAccelY(){
        return _mpu.getAccY() + this->_angAccelYOffset;
    }

    /**
     * @brief Get angular acceleration along Z-axis from MPU9250 sensor.
     * @return Angular acceleration along Z-axis.
     */
    float getAngAccelZ(){
        return _mpu.getAccZ() + this->_angAccelZOffset;
    }




    // ************* LINEAR ACCELERATION ************* //

    /**
     * @brief Get linear acceleration along X-axis from MPU9250 sensor.
     * @return Linear acceleration along X-axis.
     */
    float getLinAccelX(){
        return _mpu.getLinearAccX() + this->_linAccelXOffset;
    }

    /**
     * @brief Get linear acceleration along Y-axis from MPU9250 sensor.
     * @return Linear acceleration along Y-axis.
     */
    float getLinAccelY(){
        return _mpu.getLinearAccY() + this->_linAccelYOffset;
    }

    /**
     * @brief Get linear acceleration along Z-axis from MPU9250 sensor.
     * @return Linear acceleration along Z-axis.
     */
    float getLinAccelZ(){
        return _mpu.getLinearAccZ() + this->_linAccelZOffset;
    }


};
#endif





