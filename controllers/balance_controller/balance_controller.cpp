#include <webots/Robot.hpp>
#include <webots/supervisor.hpp>
#include <webots/keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/PositionSensor.hpp>
// #include "PID.h"
#include <windows.h>

#include "VMC.hpp"
#include "OLS.hpp"

#define TIME_STEP 8

#define getPosition getPositionSensor()->getValue

/* Controller parameters */
#define SAMPLE_TIME_S 1.0f

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace VMC;

int main(int argc, char **argv)
{
    Supervisor *robot = new Supervisor();
    Keyboard *keyboard = new Keyboard();
    keyboard->enable(TIME_STEP);

    Gyro *gyro = robot->getGyro("gyro");
    Accelerometer *accelerometer = robot->getAccelerometer("accelerometer");
    InertialUnit *imu = robot->getInertialUnit("inertial unit");
    gyro->enable(TIME_STEP);
    accelerometer->enable(TIME_STEP);
    imu->enable(TIME_STEP);

    Motor *LF = robot->getMotor("LF_motor");
    Motor *LB = robot->getMotor("LB_motor");
    Motor *L = robot->getMotor("L_motor");

    Motor *RF = robot->getMotor("RF_motor");
    Motor *RB = robot->getMotor("RB_motor");
    Motor *R = robot->getMotor("R_motor");

    LF->getPositionSensor()->enable(TIME_STEP);
    LF->enableTorqueFeedback(TIME_STEP);
    LB->getPositionSensor()->enable(TIME_STEP);
    LB->enableTorqueFeedback(TIME_STEP);
    L->getPositionSensor()->enable(TIME_STEP);

    RF->getPositionSensor()->enable(TIME_STEP);
    RF->enableTorqueFeedback(TIME_STEP);
    RB->getPositionSensor()->enable(TIME_STEP);
    RB->enableTorqueFeedback(TIME_STEP);
    R->getPositionSensor()->enable(TIME_STEP);

    // Motor *leftWheel = robot->getMotor("motorLeft");
    // get a handler to the motors and set target position to infinity (speed control)
    // Motor *leftMotor = robot->getMotor("motorLeft");
    // Motor *rightMotor = robot->getMotor("motorRight");
    // leftMotor->setPosition(INFINITY);
    // rightMotor->setPosition(INFINITY);
    // leftMotor->setVelocity(0);
    // rightMotor->setVelocity(0);
    // InertialUnit *inertial = robot->getInertialUnit("inertial");
    // inertial->enable(TIME_STEP);

    // set up the motor speeds at 10% of the MAX_SPEED.
    // PIDController pid = {PID_KP, PID_KI, PID_KD,
    //                      PID_TAU,
    //                      PID_LIM_MIN, PID_LIM_MAX,
    //                      PID_LIM_MIN_INT, PID_LIM_MAX_INT,
    //                      SAMPLE_TIME_S};

    // PIDController_Init(&pid);
    // float setpoint = 0.0f;
    // double out = 0.0f;
    // while (robot->step(TIME_STEP) != -1)
    // {
    //     double measurement = inertial->getRollPitchYaw()[0];
    //     switch (keyboard->getKey())
    //     {
    //     case Keyboard::UP:
    //         setpoint = 0.1;
    //         break;
    //     case Keyboard::DOWN:
    //         setpoint = -0.1;
    //         break;
    //     default:
    //         setpoint = 0.0f;
    //     }
    //     PIDController_Update(&pid, setpoint, measurement);
    //     leftMotor->setVelocity(-pid.out);
    //     rightMotor->setVelocity(-pid.out);
    //     printf("setpoint:%f  roll:%f    Velocity:%f    out:%f \n",
    //            setpoint, measurement, leftMotor->getVelocity(), -pid.out);
    // };

    OLSMethod<7> olsLPhi1;
    OLSMethod<7> olsLPhi4;
    OLSMethod<7> olsRPhi1;
    OLSMethod<7> olsRPhi4;

    OLSMethod<7> olsLSpeed;
    OLSMethod<7> olsRSpeed;

    OLSMethod<7> olsLToe;
    OLSMethod<7> olsRToe;

    float euler[3] = {};
    float gyroRates[3] = {};
    float accel[3] = {};

    float motorSpeed[2] = {};
    float imuSpeed = 0.0f;
    float currentStatesL[6] = {};
    float currentStatesR[6] = {};
    float targetStates[6] = {};
    float lqrK[2][6];
    VirtualToeState lLegVirtualToeState;
    VirtualToeState rLegVirtualToeState;
    MotorState lLegMotorState;
    MotorState rLegMotorState;

    while (robot->step(TIME_STEP) != -1)
    {
        /* update leg motor states */
        lLegMotorState.T1 = LF->getTorqueFeedback();
        lLegMotorState.phi1 = LF->getPosition() + 0.38f + M_PI;
        lLegMotorState.phi1Dot = olsLPhi1.derivative(lLegMotorState.phi1, TIME_STEP / 1000.0f);
        lLegMotorState.T2 = -LB->getTorqueFeedback();
        lLegMotorState.phi4 = -LB->getPosition() - 0.38f;
        lLegMotorState.phi4Dot = olsLPhi4.derivative(lLegMotorState.phi4, TIME_STEP / 1000.0f);

        rLegMotorState.T1 = -RF->getTorqueFeedback();
        rLegMotorState.phi1 = -RF->getPosition() + 0.38f + M_PI;
        rLegMotorState.phi1Dot = olsRPhi1.derivative(rLegMotorState.phi1, TIME_STEP / 1000.0f);
        rLegMotorState.T2 = RB->getTorqueFeedback();
        rLegMotorState.phi4 = RB->getPosition() - 0.38f;
        rLegMotorState.phi4Dot = olsRPhi4.derivative(rLegMotorState.phi4, TIME_STEP / 1000.0f);

        /* update VMC */
        lLegVirtualToeState = calcVirtualToeState(lLegMotorState,
                                                  TIME_STEP / 1000.0f);
        rLegVirtualToeState = calcVirtualToeState(rLegMotorState,
                                                  TIME_STEP / 1000.0f);

        /* update imu */
        euler[0] = imu->getRollPitchYaw()[0];
        euler[1] = imu->getRollPitchYaw()[1];
        euler[2] = imu->getRollPitchYaw()[2];
        gyroRates[0] = gyro->getValues()[0];
        gyroRates[1] = gyro->getValues()[1];
        gyroRates[2] = gyro->getValues()[2];
        accel[0] = accelerometer->getValues()[0];
        accel[1] = accelerometer->getValues()[1];
        accel[2] = accelerometer->getValues()[2];

        /* estimate speed */
        motorSpeed[0] = olsLSpeed.derivative(L->getPosition(), TIME_STEP / 1000.0f);
        motorSpeed[1] = olsRSpeed.derivative(R->getPosition(), TIME_STEP / 1000.0f);
        float speed = (motorSpeed[0] + motorSpeed[1]) / 2.0f;

        /* update status */
        currentStatesL[0] = lLegVirtualToeState.phi1 + euler[1];
        currentStatesL[1] = olsLToe.derivative(currentStatesL[0], TIME_STEP / 1000.0f);
        currentStatesL[2] = 0.0f;
        currentStatesL[3] = speed;
        currentStatesL[4] = euler[1];
        currentStatesL[5] = gyroRates[1];

        // currentStates[0] = gyro->getValues()[0]; // leg
        // currentStates[1] = gyro->getValues()[1];
        // currentStates[2] = gyro->getValues()[2]; // speed
        // currentStates[3] = accelerometer->getValues()[0];
        // currentStates[4] = accelerometer->getValues()[1]; // body
        // currentStates[5] = accelerometer->getValues()[2];

        // LF->setPosition(-0.38f);
        // LB->setPosition(-0.38f);
        // RF->setPosition(0.38f);
        // RB->setPosition(0.38f);
        LF->setPosition(0);
        LB->setPosition(0);
        RF->setPosition(0);
        RB->setPosition(0);
        // LF->setTorque(-0.5f);
        // LB->setTorque(-0.5f);
        // RF->setTorque(0.5f);
        // RB->setTorque(0.5f);
        // R->setTorque(-1.0f);
        // float L_pos = L->getPosition();

        printf("%f %f %f\n", speed, currentStatesL[4], currentStatesL[5]);
        // printf("%f %f %f %f\n", RF_pos, RB_pos, LF_pos, LB_pos);
        // printf("%f %f %f %f\n", lLegVirtualToeState.l0, lLegVirtualToeState.phi0, rLegVirtualToeState.l0, rLegVirtualToeState.phi0);
        // printf("%f %f %f %f\n", lLegMotorState.phi1, lLegMotorState.phi4,rLegMotorState.phi1, rLegMotorState.phi4);
        // printf("%f %f %f\n", inertial->getRollPitchYaw()[0], inertial->getRollPitchYaw()[1], inertial->getRollPitchYaw()[2]);
    }
    delete robot;

    return 0;
}