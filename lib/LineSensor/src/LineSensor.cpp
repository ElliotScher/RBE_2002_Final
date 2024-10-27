#include "LineSensor.h"

#define DARK_THRESHOLD 500;

float kp = 0.005;
float kd = 0.005;

int16_t prevError = 0;

void LineSensor::Initialize(void)
{
    pinMode(A0, INPUT);
    pinMode(A4, INPUT);
}

bool LineSensor::CheckIntersection(void)
{
    bool retVal = false;

    bool isLeftDark = analogRead(leftSensorPin) > DARK_THRESHOLD;
    bool isRightDark = analogRead(rightSensorPin) > DARK_THRESHOLD;

    bool onIntersection = isLeftDark && isRightDark;
    if(onIntersection && !prevOnIntersection) retVal = true;

    prevOnIntersection = onIntersection;

    return false;
}

float LineSensor::CalcError(void) 
{ 
    return (analogRead(leftSensorPin) - analogRead(rightSensorPin)); 
}

int16_t LineSensor::CalcEffort(void) {
    int16_t effort = (kp * CalcError()) + (kd * (CalcError() - prevError));
    prevError = CalcError();
    return effort;
}

void LineSensor::setKp(float newKp) {
    kp = newKp;
}

void LineSensor::setKd(float newKd) {
    kd = newKd;
}