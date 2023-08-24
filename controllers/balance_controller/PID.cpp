#include "PID.hpp"

#include "math.h"

#define PID_ABS(x) ((x) > 0 ? (x) : -(x))
#define PID_CLAMP(x, min, max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))
#define PID_CLAMP_ABS(x, max) ((x) > (max) ? (max) : ((x) < -(max) ? -(max) : (x)))
#define PID_ASSERT(x) 
// #define PID_GET_TIME() xTaskGetTickCount()
#define PID_IS_DATA_VALID(x) ((x) == (x))

#define PID_MIN_TIMEOUT 2

#define TickType_t uint32_t

namespace Core
{
namespace Control
{
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
PID::Param::Param(float kPonError_,
                  float kIonError_,
                  float kDonMeasurement_,
                  float integralLimit_,
                  float outputLimit_,
                  float kPonMeasurement_,
                  float kDonTarget_,
                  float alpha_,
                  TickType_t timeout_)
    : KPonError(kPonError_),
      KIonError(kIonError_),
      KDonMeasurement(kDonMeasurement_),
      KPonMeasurement(kPonMeasurement_),
      KDonTarget(kDonTarget_),
      integralMax(integralLimit_),
      integralMin(-integralLimit_),
      outputMax(outputLimit_),
      outputMin(-outputLimit_),
      alpha(alpha_),
      timeout(timeout_)
{
    this->validate();
}

PID::Param::Param(const PID::Param &_param)
{
    this->KPonError       = _param.KPonError;
    this->KIonError       = _param.KIonError;
    this->KDonMeasurement = _param.KDonMeasurement;
    this->KPonMeasurement = _param.KPonMeasurement;
    this->KDonTarget      = _param.KDonTarget;
    this->integralMax     = _param.integralMax;
    this->integralMin     = _param.integralMin;
    this->outputMax       = _param.outputMax;
    this->outputMin       = _param.outputMin;
    this->alpha           = _param.alpha;
    this->timeout         = _param.timeout;
    this->validate();
}

void PID::Param::validate()
{
    PID_ASSERT(this->KPonError >= 0.0f);
    PID_ASSERT(this->KIonError >= 0.0f);
    PID_ASSERT(this->KDonMeasurement >= 0.0f);
    PID_ASSERT(this->KPonMeasurement >= 0.0f);
    PID_ASSERT(this->KDonTarget >= 0.0f);
    PID_ASSERT(this->integralMax >= this->integralMin);
    PID_ASSERT(this->outputMax >= this->outputMin);
    PID_ASSERT(this->alpha >= 0.0f && this->alpha <= 1.0f);
    PID_ASSERT(this->timeout >= PID_MIN_TIMEOUT);
}

/**
 * @brief Construct a new PID::State::State object
 *
 */
PID::State::State() : target(0.0f), measurement(0.0f), error(0.0f), pOut(0.0f), iOut(0.0f), dOut(0.0f), output(0.0f), lastUpdate(0) {}

/* constructor */
PID::PID(const Param param_) : param(param_), state() {}

/**
 * @brief update the pid controller
 *
 * @param target        target set value
 * @param measurement       measurement feedback value
 * @param derivative    derivative value given by user
 */
float PID::operator()(float target, float measurement)
{
    /* validate parameters */
    this->param.validate();

    /* validate input data */
    if (!PID_IS_DATA_VALID(target) || !PID_IS_DATA_VALID(measurement))
    {
        this->reset();
        return this->state.output;
    }

    /* handle dt */
    TickType_t now = PID_GET_TIME();

    float dt = (float)(now - this->state.lastUpdate);
    if (dt > this->param.timeout)
    {
        this->reset();
    }
    else if (dt <= 0.0f)
    {
        return this->state.output;
    }
    dt /= (float)configTICK_RATE_HZ;

    this->state.lastUpdate = now;

    /* calculate error */

    float deltaTarget       = target - this->state.target;
    float deltaMeasurement  = measurement - this->state.measurement;
    this->state.target      = target;
    this->state.measurement = measurement;
    this->state.error       = target - measurement;

    /* calculate p on error */
    float pOut = this->param.KPonError * this->state.error;

    /* calculate integral */
    float iOut = this->state.iOut + this->param.KIonError * this->state.error * dt;

    /* calculate derivative */
    this->state.dOut = PID_CLAMP_ABS(
        this->state.dOut +
            ((-this->param.KDonMeasurement * deltaMeasurement + this->param.KDonTarget * deltaTarget) / dt - this->state.dOut) * this->param.alpha,
        this->param.outputMax - this->param.outputMin);

    /* apply p on measurement */
    iOut -= this->param.KPonMeasurement * deltaMeasurement;

    /* clamp p on error output */
    float output     = PID_CLAMP(pOut + this->state.dOut, this->param.outputMin, this->param.outputMax);
    this->state.pOut = output - this->state.dOut;

    /* clamp integral output */
    this->state.iOut =
        PID_CLAMP(PID_CLAMP(iOut, this->param.outputMin - output, this->param.outputMax - output), this->param.integralMin, this->param.integralMax);

    /* calculate output */
    this->state.output = PID_CLAMP(output + iOut, this->param.outputMin, this->param.outputMax);

    return this->state.output;
}

float PID::operator()(float target, float measurement, float derivative)
{
    /* validate parameters */
    this->param.validate();

    /* validate input data */
    if (!PID_IS_DATA_VALID(target) || !PID_IS_DATA_VALID(measurement))
    {
        this->reset();
        return this->state.output;
    }

    /* handle dt */
    TickType_t now = PID_GET_TIME();
    if (now - this->state.lastUpdate > this->param.timeout)
    {
        this->reset();
    }
    float dt               = (float)(now - this->state.lastUpdate) / (float)configTICK_RATE_HZ;
    this->state.lastUpdate = now;

    /* calculate error */
    float deltaMeasurement  = measurement - this->state.measurement;
    this->state.target      = target;
    this->state.measurement = measurement;
    this->state.error       = target - measurement;

    /* calculate p on error */
    float pOut = this->param.KPonError * this->state.error;

    /* calculate integral */
    float iOut = this->state.iOut + this->param.KIonError * this->state.error * dt;

    /* calculate derivative */
    this->state.dOut = PID_CLAMP_ABS(derivative, this->param.outputMax - this->param.outputMin);

    /* apply p on measurement */
    iOut -= this->param.KPonMeasurement * deltaMeasurement;

    /* clamp p on error output */
    float output     = PID_CLAMP(pOut + this->state.dOut, this->param.outputMin, this->param.outputMax);
    this->state.pOut = output - this->state.dOut;

    /* clamp integral output */
    this->state.iOut =
        PID_CLAMP(PID_CLAMP(iOut, this->param.outputMin - output, this->param.outputMax - output), this->param.integralMin, this->param.integralMax);

    /* calculate output */
    this->state.output = PID_CLAMP(output + iOut, this->param.outputMin, this->param.outputMax);

    return this->state.output;
}

/**
 * @brief feed forward control
 *
 * @param deltaFeedForward the change of feed forward value, will be added to the internal term
 */
PID &PID::operator+=(float deltaFeedForward)
{
    this->state.iOut += deltaFeedForward;
    this->state.iOut = PID_CLAMP(this->state.iOut, this->param.integralMin, this->param.integralMax);
    return *this;
}

PID &PID::operator-=(float deltaFeedForward) { return *this += -deltaFeedForward; }

PID &PID::operator*=(float ratio)
{
    this->state.output *= ratio;
    this->state.iOut *= ratio;
    return *this;
}

void PID::setDeltaFeedForward(float deltaFeedForward) { *this += deltaFeedForward; }

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
PID &PID::operator<<(float output) { return *this += output - this->state.output; }

PID &PID::operator<<(const PID &pid) { return *this << pid.state.output; }

float PID::getLastOutput() const { return this->state.output; }

void PID::setOutput(float output) { *this << output; }

/**
 * @brief reset the controller
 * @note this function will automatically be called when the time between two updates exceeds the timeout term
 */
void PID::reset()
{
    this->state.target      = 0.0f;
    this->state.measurement = 0.0f;
    this->state.error       = 0.0f;
    this->state.pOut        = 0.0f;
    this->state.iOut        = 0.0f;
    this->state.dOut        = 0.0f;
    this->state.output      = 0.0f;
    this->state.lastUpdate  = PID_GET_TIME();
}

/**
 * @brief Get pid parameters
 * @note user can modify the parameters on the fly using this api
 * @return Param&
 */
PID::Param &PID::getParam() { return this->param; }

/**
 * @brief Get pid state
 * @note user can read the state of the controller (but cannot modify) using this api
 * @return const State&
 */
const PID::State &PID::getState() const { return this->state; }

}  // namespace Control
}  // namespace Core