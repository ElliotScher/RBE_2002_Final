#pragma once

#include <Arduino.h>
#include "event_timer.h"

class Chassis
{
protected:
    /**
     * Kinematic parameters default to the spec sheet from Pololu. You'll need to fine
     * tune them.
     */
    const float ROBOT_RADIUS = 14.7 / 2;
    const float LEFT_TICKS_PER_CM = 1440.0 / (3.1416 * 7.0);
    const float RIGHT_TICKS_PER_CM = 1440.0 / (3.1416 * 7.0);

    /**
     * You can change the control loop period, but you should use multiples of 4 ms to 
     * avoid rounding errors.
     */
    const uint16_t CONTROL_LOOP_PERIOD_MS = 20;

protected:
    /**
     * loopFlag is used to tell the program when to update. It is set when Timer4
     * overflows (see InitializeMotorControlTimer). Some of the calculations are too
     * lengthy for an ISR, so we set a flag and use that to key the calculations.
     * 
     * Note that we use in integer so we can see if we've missed a loop. If loopFlag is
     * more than 1, then we missed a cycle.
     */
    static uint8_t loopFlag;

public:
    Chassis(void) {}
    void InititalizeChassis(void)
    {
        InitializeMotorControlTimer();
        InitializeMotors();
    }

    void SetMotorKp(float kp);
    void SetMotorKi(float ki);
    void SetMotorKd(float kd);

    bool CheckChassisTimer(void);

    static void Timer4OverflowISRHandler(void);

public:
    void Stop(void) {SetMotorEfforts(0, 0);}
    void SetTwist(float fwdSpeed, float rotSpeed);
    void SetWheelSpeeds(float, float);
    void UpdateMotors(void);

protected:
    /**
     * Initialization and Setup routines
     */
    void InitializeMotorControlTimer(void);
    void InitializeMotors(void);
    
    void SetMotorEfforts(int16_t, int16_t);
};
