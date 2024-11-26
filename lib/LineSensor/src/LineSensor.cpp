#include "LineSensor.h"

#define LIGHT_THRESHOLD 600;

void LineSensor::Initialize(void)
{
    pinMode(lineSenor1Pin, INPUT);
    pinMode(lineSenor2Pin, INPUT);
    pinMode(lineSenor3Pin, INPUT);
    pinMode(lineSenor4Pin, INPUT);
    pinMode(lineSenor5Pin, INPUT);
    pinMode(lineSenor6Pin, INPUT);
}

// SENSOR 6 DOES NOT WORK

//float lineSensorWeight = 0.5;

int16_t LineSensor::CalcError(void) 
{ 
    //0 is center of sensor on center of line

    //-1 for black line on white background
    //+1 for white line on black background
    return 1*(analogRead(lineSenor1Pin) + analogRead(lineSenor2Pin) + analogRead(lineSenor3Pin) - analogRead(lineSenor4Pin) - analogRead(lineSenor5Pin) - analogRead(lineSenor6Pin));
}
    

bool LineSensor::CheckIntersection(void)
{
    bool retVal = false; 

    bool isLeftLight = analogRead(lineSenor2Pin) < LIGHT_THRESHOLD;
    bool isRightLight = analogRead(lineSenor4Pin) < LIGHT_THRESHOLD;

    bool onIntersection = isLeftLight && isRightLight;

    // Serial.print("\t PREV: ");
    // Serial.print("\t" + (String) prevOnIntersection);
    // Serial.print("\t CURRENT: ");
    // Serial.println("\t"+ (String) onIntersection);

    // Serial.print("\t LEFT1: ");
    // Serial.print(analogRead(lineSenor1Pin));
    // Serial.print("\t LEFT2: ");
    // Serial.print(analogRead(lineSenor2Pin));
    // Serial.print("\t LEFT3: ");
    // Serial.print(analogRead(lineSenor3Pin));
    // Serial.print("\t RIGHT4: ");
    // Serial.print(analogRead(lineSenor4Pin));
    // Serial.print("\t RIGHT5: ");
    // Serial.print(analogRead(lineSenor5Pin));
    // Serial.print("\t RIGHT6: ");
    // Serial.println(analogRead(lineSenor6Pin));

    if(onIntersection && !prevOnIntersection) {
        Serial.println("\nNEW INTERSECTION!");       
        retVal = true;
    }
    prevOnIntersection = onIntersection;

    return retVal;
}