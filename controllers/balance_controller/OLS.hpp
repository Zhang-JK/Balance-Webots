/**
 * @file OLS.hpp
 * @author your name (you@domain.com)
 * @brief  ordinary least squares approximation approach to solve the derivative
 * of a function
 * @version 0.1
 * @date 2023-03-28
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "math.h"
#include "stdint.h"
#include "string.h"

// If the order of function is less than 3, the OLS method is not suitable.
template <uint16_t order, typename T = float>
class OLSMethod
{
   public:
    OLSMethod() : count(0), k(0), b(0), standardDeviation(0)
    {
        for (uint16_t i = 0; i < order; i++)
        {
            x[i] = 0;
            y[i] = 0;
        }
        for (uint16_t i = 0; i < 4; i++)
        {
            t[i] = 0;
        }
    }

    void update(T y_, T deltaX)  // the time interval between the new data and
                                 // the old data is deltaX
    {
        static float temp = 0;
        temp              = this->x[1];
        for (uint16_t i = 1; i < order; i++)
        {
            x[i] = x[i + 1] - temp;
            y[i] = y[i + 1];
        }
        this->x[order - 1] = this->x[order - 2] + deltaX;
        this->y[order - 1] = y_;
        if (count < order)
        {
            count++;
        }
        memset((void *)t, 0, sizeof(float) * 4);
        for (uint16_t i = order - count; i < order; ++i)
        {
            t[0] += x[i] * x[i];
            t[1] += x[i];
            t[2] += x[i] * y[i];
            t[3] += y[i];
        }
        k = (t[2] * order - t[1] * t[3]) / (t[0] * order - t[1] * t[1]);
        b = (t[0] * t[3] - t[1] * t[2]) / (t[0] * order - t[1] * t[1]);
        standardDeviation = 0;
        for (uint16_t i = order - count; i < order; ++i)
        {
            standardDeviation += fabsf(k * x[i] + b - y[i]);
        }
        standardDeviation /= order;
    }
    float derivative(T y_, T deltaX)
    {
        static float temp = 0;
        temp              = this->x[1];
        for (uint16_t i = 0; i < order - 1; ++i)
        {
            this->x[i] = this->x[i + 1] - temp;
            this->y[i] = this->y[i + 1];
        }
        this->x[order - 1] = this->x[order - 2] + deltaX;
        this->y[order - 1] = y_;

        if (this->count < order)
        {
            this->count++;
        }

        memset((void *)this->t, 0, sizeof(float) * 4);
        for (uint16_t i = order - this->count; i < order; ++i)
        {
            this->t[0] += this->x[i] * this->x[i];
            this->t[1] += this->x[i];
            this->t[2] += this->x[i] * this->y[i];
            this->t[3] += this->y[i];
        }

        this->k = (this->t[2] * order - this->t[1] * this->t[3]) /
                  (this->t[0] * order - this->t[1] * this->t[1]);
        this->b = (t[0] * t[3] - t[1] * t[2]) / (t[0] * order - t[1] * t[1]);
        this->standardDeviation = 0;
        for (uint16_t i = order - this->count; i < order; ++i)
        {
            this->standardDeviation +=
                fabsf(this->k * this->x[i] + this->b - this->y[i]);
        }
        this->standardDeviation /= order;
        if (initCnt < order + 1)
        {
            initCnt++;
            return 0;
        }
        else
        {
            return this->k;
        }
    }

   private:
    // uint16_t order;
    uint32_t count;
    T x[order];
    T y[order];

    T k;
    T b;
    T standardDeviation;

    T t[4];

    int initCnt = 0;
};