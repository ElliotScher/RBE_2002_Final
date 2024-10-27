#pragma once
#include "chassis.h"
#include <LineSensor.h>
#include <LSM6.h>

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

    /* targetHeading is used for commanding the robot to turn */
    float targetHeading;

    /* baseSpeed is used to drive at a given speed while, say, line following.*/
    float baseSpeed = 0;

    /* previous line following error*/
    float prevLineError = 0;
    
    /**
     * For tracking the motion of the Romi. We keep track of the intersection we came
     * from and the one we're headed to. You'll program in the map in handleIntersection()
     * and other functions.
     */
    enum INTERSECTION {NODE_START, NODE_1, NODE_2, NODE_3,};
    INTERSECTION nodeFrom = NODE_START;
    INTERSECTION nodeTo = NODE_1;
    
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

    /* IMU routines */
    void HandleOrientationUpdate(void);

    /* For commanding the lifter servo */
    void SetLifter(uint16_t position);
};
