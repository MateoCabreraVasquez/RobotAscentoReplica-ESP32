#include <Arduino.h>
#include <array.h>

#ifndef CONTROL_TRAJECTORY_H
#define CONTROL_TRAJECTORY_H
class ControlTrajectory2{
    std::array<float, 2> _pController(std::array<float, 2> value, std::array<float, 2> P){
        std::array<float, 2> result;
        result[0] = value[0] * P[0];
        result[1] = value[1] * P[1];
        return result;
    }

    private:

    

    public:
        
    


};

#endif