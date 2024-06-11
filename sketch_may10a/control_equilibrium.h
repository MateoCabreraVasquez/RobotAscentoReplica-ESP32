
#ifndef CONTROL_EQUILIBRIUM_H
#define CONTROL_EQUILIBRIUM_H

class ControlEquilibrium{

private :
// PID parameters
float kp = 4;
float ki = 0.2;
float kd = 2;

// PID variables
float setpoint = 0.0;  // Target angle (equilibrium position)
float current_angle = 0.0;  // This should be read from a sensor
float previous_error = 0.0;
float integral = 0.0;

// Time variables
unsigned long last_time = 0;
float elapsed_time;

float computePID(float current_value, float target_value) {
    float error = target_value - current_value;
    integral += error * elapsed_time;
    float derivative = (error - previous_error) / elapsed_time;
    // float output = kp * error;
    // Serial.println(error);
     float output = kp * error + ki * integral + kd * derivative;
    previous_error = error;
    return output;
}

public:

float run( float current_angle,float setpoint) {
    unsigned long current_time = micros();
    elapsed_time = (current_time - last_time) / 1000000.0f;  // Convert to seconds

    // Compute the PID output
    float wheel_angle_velocity = computePID(current_angle, setpoint);

    // Ensure the angular velocity does not exceed 11
    if (wheel_angle_velocity > 11) {
        wheel_angle_velocity = 11;
    } else if (wheel_angle_velocity < -11) {
        wheel_angle_velocity = -11;
    }

    // Update the previous time and angle for the next iteration
    last_time = current_time;

    // Serial.print(current_angle);
    // Serial.print(", ");
    // Serial.print(wheel_angle_velocity);
    // Serial.print(", ");




    return wheel_angle_velocity;
}

};
#endif








