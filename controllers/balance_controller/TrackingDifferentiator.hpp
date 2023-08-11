/**
 * @file TrackingDifferentiator.hpp
 * @author your name (you@domain.com)
 * @brief  TrackingDifferentiator Inplementation
 * @version 0.1
 * @date 2023-03-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include "math.h"
#include "stdint.h"


#define SIGN(x) (x >= 0.0f ? 1.0f : -1.0f)
template <typename T = float>
class TrackingDifferentiator
{
   public:
    TrackingDifferentiator(float r_, float h0_) : r(r_), h0(h0_), x(0), dx(0), ddx(0), lastdx(0), lastddx(0){};
#ifdef USE_FREERTOS
    T TDCalculate(T input_)
#else
    T TDCalculate(T input_, T dt_)
#endif
    {
#ifdef USE_FREERTOS

        // TickType_t t    = xTaskGetTickCount();
        // TickType_t diff = t - lastTick;
        // dt              = diff / (float)configTICK_RATE_HZ;
#endif
        dt = dt_;

        static float d, a0, y, a1, a2, a, fhan;
        if (dt > 0.5f)
            return 0;
        input = input_;
        d     = r * pow(h0, 2);
        a0    = dx * h0;
        y     = x - input + a0;
        a1    = sqrt(d * (d + 8 * fabs(y)));
        a2    = a0 + SIGN(y) * (a1 - d) / 2;
        a     = (a0 + y) * (SIGN(y + d) - SIGN(y - d)) / 2 + a2 * (1 - (SIGN(y + d) - SIGN(y - d)) / 2);
        fhan  = -r * a / d * (SIGN(a + d) - SIGN(a - d)) / 2 - r * SIGN(a) * (1 - (SIGN(a + d) - SIGN(a - d)) / 2);
        ddx   = fhan;
        dx += (0.5 * ddx + 0.5 * lastddx) * dt;
        x += (0.5 * dx + 0.5 * lastdx) * dt;
        lastdx  = dx;
        lastddx = ddx;

        return x;
    }

   private:
    T input;

    T h0;
    T r;

    T x;
    T dx;
    T ddx;

    T lastdx;
    T lastddx;
    T dt;
};