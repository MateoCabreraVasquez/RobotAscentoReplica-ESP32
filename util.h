#include <iostream>
#include <array>

#ifndef UTIL_H
#define UTIL_H

/**
 * @brief A class for saturating values between a minimum and maximum.
 */
class Saturator{
public:
    /**
     * @brief Compute the saturated value within the given range.
     * 
     * @param value The value to saturate.
     * @param min The minimum value.
     * @param max The maximum value.
     * @return The saturated value.
     */
    float compute(float value, float min, float max){
        if(value > max){
            return max;
        }else if(value < min){
            return min;
        }else{
            return value;
        }
    }
};

/**
 * @brief A class for a PID controller.
 */
class PidController {
private:
    double Kp; /**< Proportional gain */
    double Ki; /**< Integral gain */
    double Kd; /**< Derivative gain */
    double prevError; /**< Previous error */
    double integral; /**< Integral sum */

public:
    /**
     * @brief Construct a new Pid Controller object.
     * 
     * @param p Proportional gain.
     * @param i Integral gain.
     * @param d Derivative gain.
     */
    PidController(double p, double i, double d) : Kp(p), Ki(i), Kd(d), prevError(0), integral(0) {}

    /**
     * @brief Compute the control output.
     * 
     * @param setpoint The desired setpoint.
     * @param current The current value.
     * @return The control output.
     */
    double compute(double setpoint, double current) {
        double error = setpoint - current;
        integral += error;
        double derivative = error - prevError;

        double output = Kp * error + Ki * integral + Kd * derivative;

        prevError = error;

        return output;
    }
};

/**
 * @brief A class for a queue buffer.
 * 
 * @tparam T The type of elements stored in the buffer.
 * @tparam N The size of the buffer.
 */
template<typename T, size_t N>
class QueueBuffer {
public:
    /**
     * @brief Insert an element into the buffer.
     * 
     * @param element The element to insert.
     */
    void insert(const T& element) {
        for (int i = N - 1; i >= 1; i--){
            data[i] = data[i - 1];  // Shift elements to make space for the new element
        }
        data[0] = element; // Insert the new element at the front
    }

    /**
     * @brief Convert the buffer to an array.
     * 
     * @return An array containing the elements of the buffer.
     */
    std::array<T, N> toArray() const {
        return data;
    }

private:
    std::array<T, N> data; /**< Array to store elements */
};

#endif
