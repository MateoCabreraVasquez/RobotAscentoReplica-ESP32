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

    float _G = 9.7808; /**< Gravety constant */
    float _dt = 0; 

    float _pitch = 0.0; /**< Pitch angle */
    float _roll = 0.0; /**< Roll angle */
    float _yaw = 0.0; /**< Yaw angle */

    float _eulerX = 0.0; /**< Euler angle X */
    float _eulerY = 0.0; /**< Euler angle Y */
    float _eulerZ = 0.0; /**< Euler angle Z */

    float _linAccelX = 0.0; /**< Linear acceleration X */
    float _linAccelY = 0.0; /**< Linear acceleration Y */
    float _linAccelZ = 0.0; /**< Linear acceleration Z */

    float _linAccelPrevX = 0.0; /**< Linear acceleration X */
    float _linAccelPrevY = 0.0; /**< Linear acceleration Y */

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

    const float _accelBounds = 0.4/9.8; /**< Acceleration bounds 0.4ms */
    const float _angleBounds = 4; /*< Angle bounds 4 degrees/

    // linear velocity and position
    float _dt = 0.0; /**< Time step */

    float _velX = 0.0; /**< Linear velocity X */
    float _velY = 0.0; /**< Linear velocity Y */

    float _posX = 0.0; /**< Linear position X */
    float _posY = 0.0; /**< Linear position Y */

    uint32_t _prevUpdateTime = 0; /**< Previous update time in milliseconds */

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
        _prevUpdateTime = millis();  // Initialize previous update time
    }

    /**
     * @brief Update IMU data.
     * 
     * This function updates the IMU data if available.
     * 
     * @note This function should be called periodically to update the IMU data.
     */
    void update(){
        _isWorking = true;
        if (_mpu.update()) {
            uint32_t currentTime = millis();
            _dt = (currentTime - _prevUpdateTime) / 1000.0;  // Convert to seconds
            _prevUpdateTime = currentTime;

            // this->_linAccelX = _mpu.getAccX();
            // this->_linAccelY = _mpu.getAccY();
            // this->_linAccelZ = _mpu.getAccZ();
            _linAccelPrevX =_linAccelX;
            _linAccelPrevY =_linAccelX;
            
            // get angles
            float pitch = _mpu.getPitch()*PI/180;
            float roll = _mpu.getRoll()*PI/180;
            float yaw = _mpu.getYaw()*PI/180;

            // get linear acceleration
            float accMpuX = _G*_mpu.getAccX();
            float accMpuY = _G*_mpu.getAccY();
            float accMpuZ = _G*_mpu.getAccZ();

            Serial.println(accMpuX);
            accMpuX = (-_G*cos(PI/2-_pitch) + accMpuX)*cos(pitch);
            accMpuY = (-_G*cos(PI/2-_roll) + accMpuY)*cos(roll);
            accMpuZ = _G*_mpu.getAccZ();
        

            // get euler angles
            float eulerX = _mpu.getEulerX();
            float eulerY = _mpu.getEulerY();
            float eulerZ = _mpu.getEulerZ();

            // aceleration
            if (abs(accMpuX - _linAccelX) > _accelBounds)
                _linAccelX = accMpuX;  

            if (abs(accMpuY - _linAccelY) > _accelBounds)
                _linAccelY = accMpuY;

            if (abs(accMpuZ - _linAccelZ) > _accelBounds)
                _linAccelZ = accMpuZ;

            // pitch roll yaw
            if (abs(pitch - _pitch) > _angleBounds)
                _pitch = pitch;

            if (abs(roll - _roll) > _angleBounds)
                _roll = roll;

            if (abs(yaw - _yaw) > _angleBounds)
                _yaw = yaw;

            // euler angles
            if (abs(eulerX - _eulerX) > _angleBounds)
                _eulerX = eulerX;

            if (abs(eulerY - _eulerY) > _angleBounds)
                _eulerY = eulerY;

            if (abs(eulerZ - _eulerZ) > _angleBounds)
                _eulerZ = eulerZ;

            // Update linear velocity and position
            updateLinearVelAndPos();

            _isWorking = false;
        }
    }

    bool isWorking(){
        return _isWorking;
    }

    /**
     * @brief Update linear velocity and position based on current acceleration.
     */
    void updateLinearVelAndPos(){
        _velX += (_linAccelX)* _dt;
        _velY += (_linAccelY-_linAccelPrevY) * _dt;

        _posX += _velX * _dt;
        _posY += _velY * _dt;
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

    // **** PITCH ROLL YAW **** //

    /**
     * @brief Get pitch angle from MPU9250 sensor.
     * @return Pitch angle.
     */
    float getPitch(){
        return _pitch + _pitchOffset;
    }

    /**
     * @brief Get roll angle from MPU9250 sensor.
     * @return Roll angle.
     */
    float getRoll(){
        return _roll + _rollOffset;
    }

    /**
     * @brief Get yaw angle from MPU9250 sensor.
     * @return Yaw angle.
     */
    float getYaw(){
        return _yaw + _yawOffset;
    }

    // ***** EULER ANGLES ***** //

    /**
     * @brief Get Euler X angle from MPU9250 sensor.
     * @return Euler X angle.
     */
    float getEulerX(){
        return _eulerX + _eulerXOffset;
    }

    /**
     * @brief Get Euler Y angle from MPU9250 sensor.
     * @return Euler Y angle.
     */
    float getEulerY(){
        return _eulerY + _eulerYOffset;
    }

    /**
     * @brief Get Euler Z angle from MPU9250 sensor.
     * @return Euler Z angle.
     */
    float getEulerZ(){
        return _eulerZ + _eulerZOffset;
    }

    // ***** LINEAR ACCELERATION ***** //

    /**
     * @brief Get linear acceleration along X-axis from MPU9250 sensor.
     * @return Linear acceleration along X-axis.
     */
    float getAccelX(){
        return _linAccelX + _linAccelXOffset;
    }

    /**
     * @brief Get linear acceleration along Y-axis from MPU9250 sensor.
     * @return Linear acceleration along Y-axis.
     */
    float getAccelY(){
        return _linAccelY + _linAccelYOffset;
    }

    /**
     * @brief Get linear acceleration along Z-axis from MPU9250 sensor.
     * @return Linear acceleration along Z-axis.
     */
    float getAccelZ(){
        return _linAccelZ  + _linAccelZOffset;
    }

    /**
     * @brief Get linear velocity along X-axis.
     * @return Linear velocity along X-axis.
     */
    float getVelX(){
        return _velX;
    }

    /**
     * @brief Get linear velocity along Y-axis.
     * @return Linear velocity along Y-axis.
     */
    float getVelY(){
        return _velY;
    }

    /**
     * @brief Get linear position along X-axis.
     * @return Linear position along X-axis.
     */
    float getPosX(){
        return _posX;
    }

    /**
     * @brief Get linear position along Y-axis.
     * @return Linear position along Y-axis.
     */
    float getPosY(){
        return _posY;
    }
};

#endif


