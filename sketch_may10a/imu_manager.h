#include <mat.h>
#include "Arduino.h"
#include "MPU9250.h"

#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

/**
 * @brief A class for managing IMU (Inertial Measurement Unit) data.
 */
class ImuManager {

private:

  bool _isWorking = false;
  MPU9250 _mpu; /**< MPU9250 object for IMU communication */

  float _G = 9.8;
  float _pitch = 0.0; /**< Pitch angle */
  float _roll = 0.0;  /**< Roll angle */
  float _yaw = 0.0;   /**< Yaw angle */

  float _compassHeading = 0.0; /**< Compass heading */

  float _linAccelX = 0.0; /**< Linear acceleration X */
  float _linAccelY = 0.0; /**< Linear acceleration Y */
  float _linAccelZ = 0.0; /**< Linear acceleration Z */

  // offsets
  const float _pitchOffset = 0.0; /**< Pitch offset */
  const float _rollOffset = 0.0;  /**< Roll offset */
  const float _yawOffset = 0.0;   /**< Yaw offset */

  const float _compassHeadingOffset = 0.0; /**< Compass offset */

  const float _linAccelXOffset = 0.0; /**< Linear acceleration X offset */
  const float _linAccelYOffset = 0.0; /**< Linear acceleration Y offset */
  const float _linAccelZOffset = 0.0; /**< Linear acceleration Z offset */

  const float _accelBounds = 0.5 / 9.8; /**< Acceleration bounds 0.4ms */
  const float _angleBounds = 0.1;       /**< Angle bounds 4 degrees*/


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
  void begin() {
    Wire.begin();
    delay(2000);

    if (!(_mpu.setup(0x68))) {  // Change to your own address
      while (1) {
        Serial.println("MPU connection failed");
        delay(5000);
      }
    }

    _mpu.setFilterIterations(20);
  }



  /**
     * @brief Update IMU data.
     * 
     * This function updates the IMU data if available.
     * 
     * @note This function should be called periodically to update the IMU data.
     */
  void update() {
    _isWorking = true;
    if (_mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 12) {

        // get angles
        float pitch = _mpu.getPitch() * PI / 180.0f;
        float roll = _mpu.getRoll() * PI / 180.0f;
        float yaw = _mpu.getYaw() * PI / 180.0f;

        // get linear acceleration
        float a = _mpu.getAccX();
        float accMpuX = _G * a;
        float accMpuY = _G * _mpu.getAccY();
        float accMpuZ = _G * _mpu.getAccZ();


        accMpuX = (-_G * sin(pitch) + accMpuX) * cos(pitch);
        accMpuY = (-_G * cos(PI / 2 - roll) + accMpuY) * cos(roll);
        accMpuZ = _G * _mpu.getAccZ();

        

        if(abs(accMpuX)<0.1)accMpuX=0;
        if(abs(accMpuY)<0.1)accMpuY=0;
  
        if(abs(accMpuX)<0.1 && abs(accMpuY)<0.1 && abs(pitch)<0.3 && abs(roll)<0.3){_velX=0;_velY=0;};

        // Serial.print(pitch);
        // Serial.print(" ,");
        // Serial.print(a);
        // Serial.print(" ,");
        // Serial.println(accMpuX);



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


        updateLinearVelAndPos();

        prev_ms = millis();
        _isWorking = false;
      }
    }
  }



  bool isWorking() {
    return _isWorking;
  }

  float prev_time=0;
  void updateLinearVelAndPos() {
    float dt = (float)((millis() - prev_time)/1000.0f);
    // Serial.print("dt ");
    // Serial.println(dt);

    this->_velX += this->_linAccelX * dt;
    this->_velY += this->_linAccelY * dt;

    this->_posX += this->_velX * dt;
    this->_posY += this->_velY * dt;
    prev_time=millis();
  }


  /**
     * @brief Calibrates the MPU9250 sensor.
     */
  void calibrate() {
    Serial.println("Calib. will start in 5sec.");
    Serial.println("Leave the device on a flat plane.");
    _mpu.verbose(true);
    delay(5000);
    _mpu.calibrateAccelGyro();
    _mpu.verbose(false);
  }

  // *********** POS VEL *********** //

 float getPosX() {
    return this->_posX;
  }

  float getPosY() {
    return this->_posY;
  }

  float getVelX() {
    return this->_velX;
  }

  float getVelY() {
    return this->_velY;
  }

  // ************* COMPASS ************* //



  // *********** PITCH ROLL YAW *********** //

  /**
     * @brief Get pitch angle from MPU9250 sensor.
     * @return Pitch angle.
     */
  float getPitch() {
    return this->_pitch + this->_pitchOffset;
  }

  /**
     * @brief Get roll angle from MPU9250 sensor.
     * @return Roll angle.
     */
  float getRoll() {
    return this->_roll + this->_rollOffset;
  }

  /**
     * @brief Get yaw angle from MPU9250 sensor.
     * @return Yaw angle.
     */
  float getYaw() {
    return this->_yaw + this->_yawOffset;
  }


  // ************* LINEAR ACELERATION************* //

  /**
     * @brief Get Linear acceleration along X-axis from MPU9250 sensor.
     * @return Linear acceleration along X-axis.
     */
  float getAccelX() {
    return this->_linAccelX + this->_linAccelXOffset;
  }

  /**
     * @brief Get Linear acceleration along Y-axis from MPU9250 sensor.
     * @return Linear acceleration along Y-axis.
     */
  float getAccelY() {
    return this->_linAccelY + this->_linAccelYOffset;
  }

  /**
     * @brief Get Linear acceleration along Z-axis from MPU9250 sensor.
     * @return Linear acceleration along Z-axis.
     */
  float getAccelZ() {
    return this->_linAccelZ + this->_linAccelZOffset;
  }
};
#endif
