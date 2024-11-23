/**
 * robot-remote.cpp implements features of class Robot that are related to processing
 * remote control commands. It also manages modes. You might want to trim away some of 
 * the code that isn't needed later in the term.
 */
#include "robot.h"

#include <ir_codes.h>
#include <IRdecoder.h>

/**
 * IRDecoder decoder is declared extern in IRdecoder.h (for ISR purposes), 
 * so we _must_ name it decoder.
 */
#define IR_PIN 17
IRDecoder decoder(IR_PIN);

void Robot::HandleKeyCode(int16_t keyCode)
{ 
    // Serial.println(keyCode);

    // Regardless of current state, if ENTER is pressed, go to idle state
    if(keyCode == STOP_MODE) EnterIdleState();

    // The SETUP key is used for tuning motor gains
    else if(keyCode == SETUP_BTN)
    {
        if(robotCtrlMode == CTRL_SETUP) {EnterTeleopMode(); EnterIdleState();}
        else {EnterSetupMode(); EnterIdleState();}
    }

    // If PLAY is pressed, it toggles control mode (setup -> teleop)
    else if(keyCode == PLAY_PAUSE)
    {
        if(robotCtrlMode == CTRL_AUTO) {EnterTeleopMode(); EnterIdleState();}
        else if(robotCtrlMode == CTRL_TELEOP) {EnterAutoMode(); EnterIdleState();}
    }

    /**
     * AUTO commands
     */
    if(robotCtrlMode == CTRL_AUTO)
    {
        switch(keyCode)
        {
            case REWIND:
                EnterSearchingState();
                keyString = "";
                break;
            case NUM_1:
            case NUM_2:
            case NUM_3:
                keyString += (char)(keyCode + 33);
                break;
            case NUM_4:
            case NUM_5:
            case NUM_6:
                keyString += (char)(keyCode + 32);
                break;
            case NUM_7:
            case NUM_8:
            case NUM_9:
                keyString += (char)(keyCode + 31);
                break;
            case NUM_0_10:
                keyString += '0';
                break;
            case UP_ARROW:
                // EnterTurn(0.0);
                jTarget++;
                break;
            case RIGHT_ARROW:
                EnterTurn(-90.0);
                break;
            case DOWN_ARROW:
                EnterTurn(180.0);
                break;
            case LEFT_ARROW:
                EnterTurn(90.0);
                break;
            case ENTER_SAVE:
                chassis.SetTwist(0, 0);
                break;
        }
    }

    /**
     * TELEOP commands
     */
    else if(robotCtrlMode == CTRL_TELEOP)
    {
        switch(keyCode)
        {
            case UP_ARROW:
                chassis.SetTwist(25, 0);
                break;
            case RIGHT_ARROW:
                chassis.SetTwist(0, -0.25);
                break;
            case DOWN_ARROW:
                chassis.SetTwist(-5, 0);
                break;
            case LEFT_ARROW:
                chassis.SetTwist(0, 0.25);
                break;
            case ENTER_SAVE:
                chassis.SetTwist(0, 0);
                break;
        }
    }

    /**
     * SETUP mode
     */
    else if(robotCtrlMode == CTRL_SETUP)
    {
        float k = 0;
        switch(keyCode)
        {
            case VOLminus:
                k = keyString.toInt() / 100.0;
                Serial.print("Kp = "); Serial.println(k);
                chassis.SetMotorKp(k);
                keyString = "";
                break;
            case PLAY_PAUSE:
                k = keyString.toInt() / 100.0;
                Serial.print("Ki = "); Serial.println(k);
                chassis.SetMotorKi(k);
                keyString = "";
                break;
            case VOLplus:
                k = keyString.toInt() / 100.0;
                Serial.print("Kd = "); Serial.println(k);
                chassis.SetMotorKd(k);
                keyString = "";
                break;
            case UP_ARROW:
                if(!keyString.length()) chassis.SetWheelSpeeds(60, 60);
                break;
            case DOWN_ARROW:
                if(!keyString.length()) chassis.SetWheelSpeeds(20, 20);
                break;
            case ENTER_SAVE:
                keyString = "";
                chassis.SetWheelSpeeds(0, 0);
                break;
            case NUM_1:
            case NUM_2:
            case NUM_3:
                keyString += (char)(keyCode + 33);
                break;
            case NUM_4:
            case NUM_5:
            case NUM_6:
                keyString += (char)(keyCode + 32);
                break;
            case NUM_7:
            case NUM_8:
            case NUM_9:
                keyString += (char)(keyCode + 31);
                break;
            case NUM_0_10:
                keyString += '0';
                break;
        }
    }
    
}

void Robot::EnterTeleopMode(void)
{
    Serial.println("-> TELEOP");
    robotCtrlMode = CTRL_TELEOP;
}

void Robot::EnterAutoMode(void)
{
    Serial.println("-> AUTO");
    robotCtrlMode = CTRL_AUTO;
}

void Robot::EnterSetupMode(void)
{
    Serial.println("-> SETUP");
    robotCtrlMode = CTRL_SETUP;
}