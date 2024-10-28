#include "LineSensor.h"

#define DARK_THRESHOLD 500;

void LineSensor::Initialize(void)
{
    pinMode(lineSenor1Pin, INPUT);
    pinMode(lineSenor2Pin, INPUT);
    pinMode(lineSenor3Pin, INPUT);
    pinMode(lineSenor4Pin, INPUT);
    pinMode(lineSenor5Pin, INPUT);
    pinMode(lineSenor6Pin, INPUT);
}

int16_t LineSensor::CalcError(void) 
{ 
    //0 is center of sensor on center of line
    return 3*analogRead(lineSenor1Pin) + analogRead(lineSenor2Pin) + analogRead(lineSenor3Pin) - analogRead(lineSenor4Pin) - analogRead(lineSenor5Pin) - 3*analogRead(lineSenor6Pin);
}
    

bool LineSensor::CheckIntersection(void)
{
    bool retVal = false;

    bool isLeftDark = analogRead(lineSenor1Pin) > DARK_THRESHOLD;
    bool isRightDark = analogRead(lineSenor6Pin) > DARK_THRESHOLD;

    bool onIntersection = isLeftDark && isRightDark;
    if(onIntersection && !prevOnIntersection) retVal = true;

    prevOnIntersection = onIntersection;

    return retVal;
}