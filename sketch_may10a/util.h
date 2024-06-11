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
    QueueBuffer<float, 2> _prevOutputs; /**< Circular buffer for storing previous outputs */
    QueueBuffer<float, 1> _prevInputs; /**< Circular buffer for storing previous inputs */
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

class ComplementaryFilter3{


    private: 
        const float alpha1 = 0.6;  // Weight for the first measure
        const float alpha2 = 0.3;  // Weight for the second measure
        const float alpha3 = 0.1;  // Weight for the third measure


    public:
        float compute(float measure1, float measure2, float measure3){
            return alpha1 * measure1 + alpha2 * measure2 + alpha3 * measure3;
        }

};



class Debug {

    private:
    const int N = 3000; 
    float a1[3000];
    float a2[3000];
    float a3[3000];
    float a4[3000];
    int ind = 0;
    unsigned long _previousMillisEncoderAngle = 0;
    unsigned long _intervalEncoderAngle = 10;


    public:

    void runDebug(float x1,float x2, float x3, float x4){
        unsigned long _currentMillisEncoderAngle = millis();  // Actual time Variable Angle
        if (_currentMillisEncoderAngle - _previousMillisEncoderAngle >= _intervalEncoderAngle) {
            _previousMillisEncoderAngle = _currentMillisEncoderAngle;

        a1[ind] = x1;
        a2[ind] = x2;
        a3[ind] = x3;
        a4[ind] = x4;

        if (ind == N - 1) {
            for (int i = 0; i < N; i++) {
            Serial.print(a1[i], 4);
            Serial.print(" ");
            Serial.print(a2[i], 4);
            Serial.print(" ");
            Serial.print(a3[i], 4);
            Serial.print(" ");
            Serial.print(a4[i], 4);
            Serial.println();
        }
        ind = 0;
      }
    ind++;

    }
    }

};

#endif
