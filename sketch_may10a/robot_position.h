#include <array>

#ifndef ROBOT_POSITION_H
#define ROBOT_POSITION_H

class RobotPosition {

    private:
        float yPrev = 0;
        float xPrev = 0;
        float phiPrev = 0;

        float L=0;
        float R=0;
    
    public:
        RobotPosition(float Li, float Ri) {
            L=Li;
            R=Ri;
        }
        std::array<float, 3> calculatePosition(float thetaR, float thetaL) { 
            float Dr = R * thetaR;
            float Dl = R * thetaL;
            float Dc = (Dr + Dl) / 2;
            
            float y = yPrev + Dc * cos(phiPrev);
            float x = xPrev + Dc * sin(phiPrev);
            float phi = phiPrev + (Dr - Dl) / L;

            // Actualizar valores anteriores
            yPrev = y;
            xPrev = x;
            phiPrev = phi;
        
            return {x, y, phi};
        }
};

#endif
