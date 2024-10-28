#pragma once

#include <Arduino.h>

//from left to right
#define LINE_SENSOR1 A5
#define LINE_SENSOR2 A4
#define LINE_SENSOR3 A3
#define LINE_SENSOR4 A2
#define LINE_SENSOR5 A1
#define LINE_SENSOR6 A0

class LineSensor
{
protected:
    uint8_t lineSenor1Pin = LINE_SENSOR1;
    uint8_t lineSenor2Pin = LINE_SENSOR2;
    uint8_t lineSenor3Pin = LINE_SENSOR3;
    uint8_t lineSenor4Pin = LINE_SENSOR4;
    uint8_t lineSenor5Pin = LINE_SENSOR5;
    uint8_t lineSenor6Pin = LINE_SENSOR6;

    bool prevOnIntersection = false;

public:
    LineSensor(void) {}
    void Initialize(void);
    int16_t CalcError(void);
    bool CheckIntersection(void);
};