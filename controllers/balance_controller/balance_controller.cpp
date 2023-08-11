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
#define TIME_STEP 8

#define getPosition getPositionSensor()->getValue

/* Controller parameters */
#define SAMPLE_TIME_S 1.0f

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv)
{
    Supervisor *robot = new Supervisor();
    Keyboard *keyboard = new Keyboard();
    keyboard->enable(TIME_STEP);

    Gyro *gyro = robot->getGyro("gyro");
    Accelerometer *accelerometer = robot->getAccelerometer("accelerometer");
    InertialUnit *inertial = robot->getInertialUnit("inertial unit");
    gyro->enable(TIME_STEP);
    accelerometer->enable(TIME_STEP);
    inertial->enable(TIME_STEP);

    Motor *LF = robot->getMotor("LF_motor");
    Motor *LB = robot->getMotor("LB_motor");
    Motor *L = robot->getMotor("L_motor");

    Motor *RF = robot->getMotor("RF_motor");
    Motor *RB = robot->getMotor("RB_motor");
    Motor *R = robot->getMotor("R_motor");

    LF->getPositionSensor()->enable(TIME_STEP);
    LB->getPositionSensor()->enable(TIME_STEP);
    L->getPositionSensor()->enable(TIME_STEP);

    RF->getPositionSensor()->enable(TIME_STEP);
    RB->getPositionSensor()->enable(TIME_STEP);
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

    while (robot->step(TIME_STEP) != -1)
    {
        float LF_pos = LF->getPosition();
        float LB_pos = LB->getPosition();
        float RF_pos = RF->getPosition();
        float RB_pos = RB->getPosition();

        LF->setPosition(-0.38f);
        LB->setPosition(-0.38f);
        RF->setPosition(0.38f);
        RB->setPosition(0.38f);
        // LF->setTorque(-0.5f);
        // LB->setTorque(-0.5f);
        // RF->setTorque(0.5f);
        // RB->setTorque(0.5f);
        // R->setTorque(-1.0f);
        // float L_pos = L->getPosition();

        printf("%f %f %f %f\n", RF_pos, RB_pos, LF_pos, LB_pos);
    }
    delete robot;

    return 0;
}