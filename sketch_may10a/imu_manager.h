#include "Arduino.h"
#include "MPU9250.h"

#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

/**
 * @brief A class for managing IMU (Inertial Measurement Unit) data.
 */
class ImuManager{

private:

    bool _isWorking = false;
    MPU9250 _mpu; /**< MPU9250 object for IMU communication */

    float _pitch = 0.0; /**< Pitch angle */
    float _roll = 0.0; /**< Roll angle */
    float _yaw = 0.0; /**< Yaw angle */

    float _eulerX = 0.0; /**< Euler angle X */
    float _eulerY = 0.0; /**< Euler angle Y */
    float _eulerZ = 0.0; /**< Euler angle Z */

    float _linAccelX = 0.0; /**< Linear acceleration X */
    float _linAccelY = 0.0; /**< Linear acceleration Y */
    float _linAccelZ = 0.0; /**< Linear acceleration Z */

    // offsets
    const float _pitchOffset = 0.0; /**< Pitch offset */
    const float _rollOffset = 0.0; /**< Roll offset */
    const float _yawOffset = 0.0; /**< Yaw offset */

    const float _eulerXOffset = 0.0; /**< Euler angle X offset */
    const float _eulerYOffset = 0.0; /**< Euler angle Y offset */
    const float _eulerZOffset = 0.0; /**< Euler angle Z offset */

    const float _linAccelXOffset = 0.0; /**< Linear acceleration X offset */
    const float _linAccelYOffset = 0.0; /**< Linear acceleration Y offset */
    const float _linAccelZOffset = 0.0; /**< Linear acceleration Z offset */

    const  float _accelBounds = 0.4/9.8; /**< Acceleration bounds 0.4ms */
    const  float _angleBounds = 4; /**< Angle bounds 4 degrees*/

    
    // linear velocity and position
    float _dt = 0.0; /**< Time step */


    float _preLinAccelX = 0.0; /**< Linear acceleration X */
    float _preLinAccelY = 0.0; /**< Linear acceleration Y */

    float _preVelX = 0.0; /**< Linear velocity X */
    float _preVely = 0.0; /**< Linear velocity X */


    float _velX = 0.0; /**< Linear velocity X */
    float _velY = 0.0; /**< Linear velocity Y */

    float _posX = 0.0; /**< Linear position X */
    float _posY = 0.0; /**< Linear position Y */

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
        _isWorking=true;
        if (_mpu.update()) {

            static uint32_t prev_ms = millis();
            if (millis() > prev_ms + 25) {
                prev_ms = millis();

                // this->_linAccelX = _mpu.getAccX();
                // this->_linAccelY = _mpu.getAccY();
                // this->_linAccelZ = _mpu.getAccZ();

                // get linear acceleration
                float accMpuX = _mpu.getAccX();
                float accMpuY = _mpu.getAccY();
                float accMpuZ = _mpu.getAccZ();

                Serial.println(accMpuX);
                Serial.println(abs(accMpuX - _linAccelX));


                // get angles
                float pitch = _mpu.getPitch();
                float roll = _mpu.getRoll();
                float yaw = _mpu.getYaw();

                // get euler angles
                float eulerX = _mpu.getEulerX();
                float eulerY = _mpu.getEulerY();
                float eulerZ = _mpu.getEulerZ();

                // aceleration
                if (abs(accMpuX - _linAccelX) > _accelBounds)
                    this->_linAccelX = accMpuX;	 

                if (abs(accMpuY - _linAccelY) > _accelBounds)
                    this->_linAccelY = accMpuY;

                if (abs(accMpuZ - _linAccelZ) > _accelBounds)
                    this->_linAccelZ = accMpuZ;

                // pitch roll yaw
                if (abs(pitch - _pitch) > _angleBounds)
                    this->_pitch = pitch;

                if (abs(roll - _roll) > _angleBounds)
                    this->_roll = roll;

                if (abs(yaw - _yaw) > _angleBounds)
                    this->_yaw = yaw;

                // euler angles
                if (abs(eulerX - _eulerX) > _angleBounds)
                    this->_eulerX = eulerX;

                if (abs(eulerY - _eulerY) > _angleBounds)
                    this->_eulerY = eulerY;

                if (abs(eulerZ - _eulerZ) > _angleBounds)
                    this->_eulerZ = eulerZ;

                _isWorking=false;

            }
        }
    }

    bool isWorking(){
      return _isWorking;
    }

    void updateLinearVelAndPos(){
        //_dt = (float)(millis() - prev_ms) * 0.001 - _dt;
        
        this->_velX += this->_linAccelX * _dt;
        this->_velY += this->_linAccelY * _dt;

        this->_posX += this->_velX * _dt;
        this->_posY += this->_velY * _dt;


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
        return this->_eulerX + this->_eulerXOffset;
    }

    /**
     * @brief Get Euler Y angle from MPU9250 sensor.
     * @return Euler Y angle.
     */
    float getEulerY(){
        return this->_eulerY + this->_eulerYOffset;
    }

    /**
     * @brief Get Euler Z angle from MPU9250 sensor.
     * @return Euler Z angle.
     */
    float getEulerZ(){
        return  this->_eulerZ + this->_eulerZOffset;
    }




    // ************* LINEAR ACELERATION************* //

    /**
     * @brief Get Linear acceleration along X-axis from MPU9250 sensor.
     * @return Linear acceleration along X-axis.
     */
    float getAccelX(){
        return  this->_linAccelX*9.7808 + this->_linAccelXOffset;
    }

    /**
     * @brief Get Linear acceleration along Y-axis from MPU9250 sensor.
     * @return Linear acceleration along Y-axis.
     */
    float getAccelY(){
        return  this->_linAccelY*9.7808 + this->_linAccelYOffset;
    }

    /**
     * @brief Get Linear acceleration along Z-axis from MPU9250 sensor.
     * @return Linear acceleration along Z-axis.
     */
    float getAccelZ(){
        return  this->_linAccelZ*9.7808 + this->_linAccelZOffset;
    }




};
#endif





