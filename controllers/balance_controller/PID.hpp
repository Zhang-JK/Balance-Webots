/**
 * @file PID.hpp
 * @author JIANG Yicheng RM2023
 * @brief
 * @version 2.0
 * @date 2023-03-12
 *
 * @copyright Copyright (c) 2023
 */

#pragma once

#include "AppConfig.h"
/*disable clang-format*/
#include "FreeRTOS.h"
#include "main.h"

#ifndef PID_DEFAULT_TIMEOUT
#define PID_DEFAULT_TIMEOUT pdMS_TO_TICKS(100)
#endif

#ifndef PID_DEFAULT_ALPHA
#define PID_DEFAULT_ALPHA 0.2f
#endif

#ifndef PID_DEFAULT_MAX_OUTPUT
#define PID_DEFAULT_MAX_OUTPUT 10000.0f
#endif

namespace Core
{
namespace Control
{
class PID
{
   public:
    struct Param
    {
        /* PID parameters */
        float KPonError;        // Kp
        float KIonError;        // Ki
        float KDonMeasurement;  // Kd

        /**
         * @brief Kp on measurement
         * @remark This is used to reduce or cancel the unavoidable overshoot caused by integral term
         * @note This is not a standard PID parameter. If you do not know what it is, set it to 0.
         */
        float KPonMeasurement;

        /**
         * @brief Kd on target
         * @remark It will make the derivative term more sensitive to the change of the target value, which may introduce more noise to the system.
         */
        float KDonTarget;

        /* output limit */
        float integralMax;  // integral upper limit
        float integralMin;  // integral lower limit

        float outputMax;  // output upper limit
        float outputMin;  // output lower limit

        /* for derivative term low pass filter */
        float alpha;

        /**
         * @brief Timeout term
         * @remark If the time between two updates is too large, the controller may generate crazy output. This term monitors the time between two
         * updates and reset controller once the time between two updates is too large.
         */
        TickType_t timeout;

        /**
         * @brief Construct a new Param object
         *
         * @param kPonError         Kp
         * @param kIonError         Ki
         * @param kDonMeasurement   Kd
         * @param integralLimit     abs integral upper limit
         * @param outputLimit       abs output upper limit
         * @param kPonMeasurement   Kp on measurement, used to reduce or cancel the unavoidable overshoot caused by integral term
         * @param kDonTarget        Kd on target, make the derivative term more sensitive to the change of the target value
         * @param alpha             alpha for derivative term low pass filter
         * @param timeout           timeout term
         */
        Param(float kPonError,
              float kIonError       = 0.0f,
              float kDonMeasurement = 0.0f,
              float integralLimit   = PID_DEFAULT_MAX_OUTPUT,
              float outputLimit     = PID_DEFAULT_MAX_OUTPUT,
              float kPonMeasurement = 0.0f,
              float kDonTarget      = 0.0f,
              float alpha           = PID_DEFAULT_ALPHA,
              TickType_t timeout    = PID_DEFAULT_TIMEOUT);

        Param(const Param &_param);

       private:
        /**
         * @brief This function is used to validate the parameters, to prevent SB from setting a crazy parameter during runtime.
         * @note This function will assert the parameters, which may cause assertFailed.
         */
        friend class PID;
        void validate();
    };

    struct State
    {
        float target;           // target value, set by user during update
        float measurement;      // measurement value, get from user during update
        float error;            // error = target - measurement
        float pOut;             // Proportional term output on error
        float iOut;             // Integral term output
        float dOut;             // Derivative term output
        float output;           // PID output
        TickType_t lastUpdate;  // last update time, used to perform timeout check

       private:
        /**
         * @brief Do not allow user to construct a PID::State object
         * @remark This is a private constructor, which means that only friend class can construct a PID::State object.
         * @note Prevent SB from constructing a PID::State object and keep asking why it does not update.
         */
        friend class PID;
        State();
        // clang-format off
        State(const State &state) = delete;
        State &operator=(const State &state) = delete;
        // clang-format on
    };

    /* constructor */
    PID(const Param param);

    /**
     * @brief update the pid controller
     *
     * @param target        target set value
     * @param current       current feedback value
     * @param derivative    derivative value given by user
     */
    float operator()(float target, float current);
    float operator()(float target, float current, float derivative);

    /**
     * @brief feed forward control
     *
     * @param deltaFeedForward the change of feed forward value, will be added to the internal term
     */
    PID &operator+=(float deltaFeedForward);
    PID &operator-=(float deltaFeedForward);
    PID &operator*=(float ratio);
    void setDeltaFeedForward(float deltaFeedForward);

    /**
     * @brief bumpless switch between two pid controllers
     *
     * @note    -   when we switch between two pid controllers, the output starting point of the new controller can be set to the output of the old
     *          -   controller to avoid the bump during switching.
     *
     * @usage:  if we want to switch from pid1 to pid2, the following codes are equivalent:
     *          -   pid2 << pid1;
     *          -   pid2 << pid1.getLastOutput();
     *          -   pid2.setOutput(pid1.getLastOutput());
     *
     * @param output
     */
    PID &operator<<(float output);
    PID &operator<<(const PID &pid);
    float getLastOutput() const;
    void setOutput(float output);
    void setKp(float kPonError) { param.KPonError = kPonError; }
    void setKi(float kIonError) { param.KIonError = kIonError; }
    void setKd(float kDonMeasurement) { param.KDonMeasurement = kDonMeasurement; }

    /**
     * @brief reset the controller
     * @note this function will automatically be called when the time between two updates exceeds the timeout term
     */
    void reset();

    /**
     * @brief Get pid parameters
     * @note user can modify the parameters on the fly using this api
     * @return Param&
     */
    Param &getParam();

    /**
     * @brief Get pid state
     * @note user can read the state of the controller (but cannot modify) using this api
     * @return const State&
     */
    const State &getState() const;

   private:
    Param param;  // PID parameters
    State state;  // PID state
};
}  // namespace Control
}  // namespace Core
