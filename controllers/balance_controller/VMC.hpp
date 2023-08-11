#pragma once
/*
                tp
                ||  x,y
             // |  \\
         l2 //  |   \\  l3
           //   |    \\
          //    |     \\
         //     |      \\
        //phi2  l0      \\phi3
        \\      |       //
      l1 \\   l5|phi0  // l4
          \\phi1---- -//phi4
          t1          t2
*/

namespace VMC
{
const struct
{
    float l1 = 0.15f;
    float l2 = 0.25f;
    float l3 = 0.25f;
    float l4 = 0.15f;
    float l5 = 0.11f;
} legLength;

struct MotorState
{
    float T1;       // left motor torque
    float phi1;     // left motor angle
    float phi1Dot;  // rad/s
    float T2;       // right motor torque
    float phi4;     // right motor angle
    float phi4Dot;  // rad/s
};

struct VirtualToeState
{
    float phi0;  // angle of virtual toe
    float l0;    // length of virtual toe
    float F;     // force
    float Tp;    // torque
    float xcDot;
    float ycDot;
    float phi1;
    float phi2;
    float phi3;
    float phi4;
};

VirtualToeState calcVirtualToeState(const MotorState &motorState,
                                    const float &dt);

void getMotorTorque(const VirtualToeState &toeState,
                    float &T1,
                    float &T2,
                    const float &F,
                    const float &Tp);

float getSupportForce(const float &F,
                      const float &Tp,
                      const float &l0,
                      const float &theta,
                      const float &thetaDot,
                      const float &thetaDotDot,
                      const float &bodyAccel,
                      const float &l0Dot,
                      const float &l0DotDot);

}  // namespace VMC
