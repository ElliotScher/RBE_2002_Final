#include "LineSensor.h"

#define DARK_THRESHOLD 500;

void LineSensor::Initialize(void)
{
    pinMode(A0, INPUT);
    pinMode(A4, INPUT);
}

int16_t LineSensor::CalcError(void) 
{ 
    return analogRead(rightSensorPin) - analogRead(leftSensorPin); 
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