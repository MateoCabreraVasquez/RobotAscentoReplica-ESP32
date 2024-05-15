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
 * @brief A class for implementing a zero-order holder.
 */
class ZeroOrderHolder
{
private:
    float _lastValue = 0.0; /**< Last stored value */
    float _lastTime = 0.0; /**< Last time value */
    const float _period = 1; /**< Sampling period (1 ms by default) */

public:
    /**
     * @brief Compute the output of the zero-order holder.
     * 
     * @param newValue The new input value.
     * @return The output value of the zero-order holder.
     */
    float Compute(float newValue){
        // Check if the sampling period has elapsed
        if (_lastTime + _period <=  millis()){
            _lastValue = newValue; // Update the last value
            _lastTime = millis(); // Update the last time
        }
        
        return _lastValue; // Return the last stored value
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


/**
 * @brief A class representing a second-order transfer function in discrete time.
 */
class TransferFunctionOrderTwo {
private:
    CircularBuffer<float, 2> _prevOutputs; /**< Circular buffer for storing previous outputs */
    CircularBuffer<float, 1> _prevInputs; /**< Circular buffer for storing previous inputs */
    float _num0; /**< Numerator coefficient 0 */
    float _den0; /**< Denominator coefficient 0 */
    float _den1; /**< Numerator coefficient 1 */


public:
    /**
     * @brief Construct a new Transfer Function Order Two object.
     * 
     * @param num0 Numerator coefficient 0.
     * @param num1 Numerator coefficient 1.
     * @param den0 Denominator coefficient 0.
     */
    TransferFunctionOrderTwo( float num0, float den0, float den1) :  _num0(num0), _den0(den0),_den1(den1) {}

    /**
     * @brief Compute the output of the transfer function.
     * 
     * @param value The current input value.
     * @return The output value.
     */
    float compute(float value) {
        _prevInputs.insert(value);
        std::array<float, 1> u = _prevInputs.toArray();
        std::array<float, 2> Y = _prevOutputs.toArray();
        float y = _num0 * u[0] -_den1 * Y[1] - _den0 * Y[0];
        _prevOutputs.insert(y);
        return y;
    }
};


#endif
