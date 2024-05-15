#include "util.h"

#ifndef CONTROL_VELOCITY_H
#define CONTROL_VELOCITY_H

/**
 * @brief A class for controlling velocity.
 */
class ControlVelocity{
private:

    // ***************** PARAMETERS *****************

    // UTIL
    Saturator _saturator; /**< Saturator object for saturation */

    // CONST
    const float Ts = 0.0; /**< Sampling time */

    // PID Right
    QueueBuffer<float, 3> _pidRighEntries; /**< Buffer for PID right entries */
    QueueBuffer<float, 2> _pdiRightOutputs; /**< Buffer for PID right outputs */

    // PID Left
    QueueBuffer<float, 3> _pidLeftEntries; /**< Buffer for PID left entries */
    QueueBuffer<float, 2>  _pdiLeftOutputs; /**< Buffer for PID left outputs */

    // ZERO ORDER HOLD
    ZeroOrderHolder _zeroOrderHolderRigth; /**< Zero order holder for right wheel */
    ZeroOrderHolder _zeroOrderHolderLeft; /**< Zero order holder for left wheel */

    // TRANSFER FUNCTION
    TransferFunctionOrderTwo _tranferFunRigth; /**< Transfer function for right wheel */
    TransferFunctionOrderTwo _tranferFunLeft; /**< Transfer function for left wheel */

    // ***************** FUNCTIONS *****************

    // RIGHT
    /**
     * @brief Compute the PID control output for the right wheel.
     * 
     * @param value The input value.
     * @return The PID control output.
     */
    float _pidRight(float value){
        _pidRighEntries.insert(value);
        std::array<float, 3> u =_pidRighEntries.toArray();
        std::array<float, 2> Y =_pdiRightOutputs.toArray();

        float y = 15.99 * u[0] - 6.576 * u[1] - 7.715 * u[2] + 0.9847 * Y[0] + 0.01531 * Y[1];
        _pdiRightOutputs.insert(y);
        return y;
    }


    // LEFT
    /**
     * @brief Compute the PID control output for the left wheel.
     * 
     * @param value The input value.
     * @return The PID control output.
     */
    float _pidLeft(float value){
        _pidLeftEntries.insert(value);
        std::array<float, 3> u =_pidLeftEntries.toArray();
        std::array<float, 2> Y =_pdiLeftOutputs.toArray();
        float y = 13.28 * u[0] - 6.997 * u[1] - 4.867 * u[2] + 0.9404 * Y[0] + 0.05964 * Y[1];
        _pdiLeftOutputs.insert(y);
        return y;
    }


    /**
     * @brief Apply zero-order hold.
     * 
     * @param value The input value.
     * @return The output value after applying zero-order hold.
     */
    float _zeroOderHold(float value){
        return value;
    }


    /**
     * @brief Apply a transfer function.
     * 
     * @param value The input value.
     * @param num Numerator coefficient.
     * @param den1 Denominator coefficient 1.
     * @param den2 Denominator coefficient 2.
     * @return The output value after applying the transfer function.
     */
    float _transferFunction(float value, float num, float den1, float den2){
        return value;
    }




public:
    // RIGHT
    /**
     * @brief Compute the control output for the right wheel.
     * 
     * @param value The input value.
     * @return The control output for the right wheel.
     */
    float computeRigth(float value){

        float res = _pidRight(value);

        float resatured1 = _saturator.compute(res, -100, 100);

        float resZeroOrderHold = _zeroOrderHolderRigth.Compute(resatured1);

        float resTransferFunction = _transferFunction(resZeroOrderHold, 1, 1, 1);

        float resSatured2 = _saturator.compute(resTransferFunction, -11, 11);

        return resSatured2;
    }

    // LEFT
    /**
     * @brief Compute the control output for the left wheel.
     * 
     * @param value The input value.
     * @return The control output for the left wheel.
     */
    float computeLeft(float value){

        float res = _pidLeft(value);

        float resatured1 = _saturator.compute(res, -100, 100);

        float resZeroOrderHold = _zeroOrderHolderLeft.Compute(resatured1);

        float resTransferFunction = _transferFunction(resZeroOrderHold, 1, 1, 1);

        float resSatured2 = _saturator.compute(resTransferFunction, -11, 11);

        return resSatured2;
    }
    
};

#endif
