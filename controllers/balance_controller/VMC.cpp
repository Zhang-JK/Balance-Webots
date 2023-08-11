#include "VMC.hpp"

#include "math.h"

namespace VMC
{

VirtualToeState calcVirtualToeState(const MotorState &motorState,
                                    const float &dt)
{
    VirtualToeState virtualState;

    float x_DB = cosf(motorState.phi4) * legLength.l4 + legLength.l5 -
                 cosf(motorState.phi1) * legLength.l1;  // x_D - x_B
    float y_DB = sinf(motorState.phi4) * legLength.l4 -
                 sinf(motorState.phi1) * legLength.l1;  // y_D - y_B
    float A0 = 2 * legLength.l2 * x_DB;
    float B0 = 2 * legLength.l2 * y_DB;
    float C0 = legLength.l2 * legLength.l2 - legLength.l3 * legLength.l3 +
               x_DB * x_DB + y_DB * y_DB;
    // float A1 = 2 * legLength.l3 * x_DB;
    // float B1 = 2 * legLength.l3 * y_DB;
    virtualState.phi1 = motorState.phi1;
    virtualState.phi4 = motorState.phi4;
    float phi2        = virtualState.phi2 =
        2 * atan2f((B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));

    // float phi3 =
    //     2 * atan2f((B1 + sqrtf(powf(A1, 2) + powf(B1, 2) - powf(C0, 2))),
    //                (A1 + C0));  // ?

    float xC = legLength.l1 * cosf(motorState.phi1) + legLength.l2 * cosf(phi2);
    float yC = legLength.l1 * sinf(motorState.phi1) + legLength.l2 * sinf(phi2);

    virtualState.l0 = sqrtf(powf(xC - legLength.l5 / 2, 2) + powf(yC, 2));

    float phi0 = virtualState.phi0 = atan2f(yC, xC - legLength.l5 / 2);
    float phi1                     = motorState.phi1;
    // float phi2 = asinf((virtualState.l0 * sinf(virtualState.phi0) -
    //                     legLength.l1 * sinf(motorState.phi1)) /
    //                    legLength.l2);
    float phi3 = virtualState.phi3 =
        static_cast<float>(M_PI) -
        asinf((virtualState.l0 * sinf(virtualState.phi0) -
               legLength.l4 * sinf(motorState.phi4)) /
              legLength.l3);
    float phi4 = motorState.phi4;

    float sinphi32 = sinf(phi3 - phi2);
    float sinphi12 = sinf(phi1 - phi2);
    float sinphi34 = sinf(phi3 - phi4);
    // [ a b ]
    // [ c d ]
    float a = legLength.l1 * sinf(phi0 - phi3) * sinphi12 / sinphi32;
    float b = legLength.l1 * cosf(phi0 - phi3) * sinphi12 / virtualState.l0 *
              sinphi32;
    float c = legLength.l4 * sinf(phi0 - phi2) * sinphi34 / sinphi32;
    float d = legLength.l4 * cosf(phi0 - phi2) * sinphi34 / virtualState.l0 *
              sinphi32;

    float det       = a * d - b * c;
    virtualState.F  = (motorState.T1 * d - motorState.T2 * b) / det;
    virtualState.Tp = (motorState.T2 * a - motorState.T1 * c) / det;

    float a1 = -legLength.l1 * sinphi12 * sinf(phi3) / sinphi32;
    float b1 = -legLength.l4 * sinphi34 * sinf(phi2) / sinphi32;
    float c1 = legLength.l1 * sinphi12 * cosf(phi3) / sinphi32;
    float d1 = legLength.l4 * sinphi34 * cosf(phi2) / sinphi32;

    virtualState.xcDot = a1 * motorState.phi1Dot + b1 * motorState.phi4Dot;
    virtualState.ycDot = c1 * motorState.phi1Dot + d1 * motorState.phi4Dot;

    return virtualState;
}

void getMotorTorque(const VirtualToeState &toeState,
                    float &T1,
                    float &T2,
                    const float &F,
                    const float &Tp)
{
    float sinphi12 = sinf(toeState.phi1 - toeState.phi2);
    float sinphi32 = sinf(toeState.phi3 - toeState.phi2);
    float sinphi34 = sinf(toeState.phi3 - toeState.phi4);

    float a = legLength.l1 * sinf(toeState.phi0 - toeState.phi3) * sinphi12 /
              sinphi32;
    float b = legLength.l1 * cosf(toeState.phi0 - toeState.phi3) * sinphi12 /
              toeState.l0 * sinphi32;
    float c = legLength.l4 * sinf(toeState.phi0 - toeState.phi2) * sinphi34 /
              sinphi32;
    float d = legLength.l4 * cosf(toeState.phi0 - toeState.phi2) * sinphi34 /
              toeState.l0 * sinphi32;

    T1 = a * F + b * Tp;
    T2 = c * F + d * Tp;
}

float getSupportForce(const float &F,
                      const float &Tp,
                      const float &l0,
                      const float &theta,
                      const float &thetaDot,
                      const float &thetaDotDot,
                      const float &bodyAccel,
                      const float &l0Dot,
                      const float &l0DotDot)
{
    const float mw = 1.567f;  // wheel mass
    const float g  = 9.8f;    // gravity

    float cosTheta = cosf(theta);
    float sinTheta = sinf(theta);

    float P = F * cosTheta + Tp * sinTheta / l0;

    float zwDotDot =
        bodyAccel - l0DotDot * cosTheta + 2 * l0Dot * thetaDot * sinTheta +
        l0 * thetaDotDot * sinTheta + l0 * thetaDot * thetaDot * cosTheta;

    return P + mw * g + mw * zwDotDot;
}
}  // namespace VMC