#pragma once
#include "chassis.h"
#include <LineSensor.h>
#include <LSM6.h>
#include <apriltagdatum.h>
#include <openmv.h>
#include <servo32u4.h>
#include <HX711.h>

class Robot
{
protected:
    /**
     * We define some modes for you. SETUP is used for adjusting gains and so forth. Most
     * of the activities will run in AUTO. You shouldn't need to mess with these.
     */
    enum ROBOT_CTRL_MODE
    {
        CTRL_TELEOP,
        CTRL_AUTO,
        CTRL_SETUP,
    };
    ROBOT_CTRL_MODE robotCtrlMode = CTRL_TELEOP;

    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE, 
        ROBOT_LINING,
        ROBOT_TURNING,
        ROBOT_CLIMBING,
        ROBOT_SEARCHING,
        ROBOT_APPROACHING,
        ROBOT_DEAD_RECKONING,
        ROBOT_LIFTING,
        ROBOT_WEIGHING
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    /* Define the chassis*/
    Chassis chassis;

    /* Line sensor */
    LineSensor lineSensor;

    /* To add later: rangefinder, camera, etc.*/

    // For managing key presses
    String keyString;

    /**
     * The LSM6 IMU that is included on the Romi. We keep track of the orientation through
     * Euler angles (roll, pitch, yaw).
     */
    LSM6 imu;
    LSM6::vector<float> prevEulerAngles;
    LSM6::vector<float> eulerAngles;
    float predictedPitchAngle = 0;

    /* targetHeading is used for commanding the robot to turn */
    float targetHeading;

    /* baseSpeed is used to drive at a given speed while, say, line following.*/
    float baseSpeed = 0;

    /* previous line following error*/
    float prevLineError = 0;

    float targetDirection = 0;
    uint8_t iGrid = 0, jGrid = 0;
    uint8_t iTarget = 0, jTarget = 0;

    enum DIRECTION {
        NORTH, EAST, SOUTH, WEST
    };
    DIRECTION direction = NORTH;

    OpenMV openMV;
    AprilTagDatum tag;

    float approachturnkp = 0.025;
    float approachdrivekp = 0.75;
    EventTimer approachTimer;
    EventTimer deadReckonTimer;

    Servo32U4Pin5 servo;
    HX711<6, 13> amplifier;
    int32_t amplifierReading;
    int32_t previousAmplifierReading;


    int readingCount = 0;
    float readingSum = 0;
    
public:
    Robot(void) {keyString.reserve(8);} //reserve some memory to avoid reallocation
    void InitializeRobot(void);
    void RobotLoop(void);

protected:
    /* For managing IR remote key presses*/
    void HandleKeyCode(int16_t keyCode);

    /* State changes */    
    void EnterIdleState(void);

    /* Mode changes */
    void EnterTeleopMode(void);
    void EnterAutoMode(void);
    void EnterSetupMode(void);

    /**
     * Line following and navigation routines.
     */
    void EnterLineFollowing(float speed);
    void LineFollowingUpdate(void);

    bool CheckIntersection(void) {return lineSensor.CheckIntersection();}
    void HandleIntersection(void);

    void EnterTurn(float angleInDeg);
    bool CheckTurnComplete(void);
    void HandleTurnComplete(void);

    void EnterClimbing(float speed);
    bool CheckClimbComplete(void);
    void HandleClimbComplete(void);

    /* IMU routines */
    void HandleOrientationUpdate(void);

    /* For commanding the lifter servo */
    void SetLifter(uint16_t pulseLengthUS) {servo.setTargetPos(pulseLengthUS);}
    bool CheckLiftComplete(void);
    void HandleLiftComplete(void);

    /**
     * For the camera alignment routine
     */
    void HandleAprilTag(const AprilTagDatum& tag);
    void EnterSearchingState(void);
    void EnterApproachingState(void);
    bool CheckApproachComplete(int headingTolerance, int distanceTolerance);
    void HandleApproachTimerStop(void);

    float getWeight() { return ((readingSum / (float)readingCount) + 579833.678571429) / 659.133857142857; }

    void HandleDeadReckoningTimerStop(void);
};
