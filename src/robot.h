#pragma once
#include "chassis.h"
#include <LineSensor.h>
#include <LSM6.h>
#include <openmv.h>
#include <servo32u4.h>
#include <HX711.h>
#include <Romi32U4Buzzer.h>

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

    enum ROBOT_SUPERSTATE 
    {
        ROBOT_NAVIGATING,
        ROBOT_COLLECTING,
        ROBOT_DELIVERING,
        ROBOT_SUPER_IDLE
    };
    ROBOT_SUPERSTATE robotSuperState = ROBOT_SUPER_IDLE;

    enum ROBOT_SUBSTATE 
    {
        ROBOT_IDLE, 
        ROBOT_LINING,
        ROBOT_TURNING,
        ROBOT_DEAD_RECKONING,
        ROBOT_SEARCHING,
        ROBOT_APPROACHING,
        ROBOT_LIFTING,
        ROBOT_LOWERING,
        ROBOT_WEIGHING,
        ROBOT_CENTERING,
        ROBOT_RETURNING,
        ROBOT_CLIMBING,
        ROBOT_DESCENDING
    };
    ROBOT_SUBSTATE robotSubState = ROBOT_IDLE;

    /* Define the chassis*/
    Chassis chassis;

    /* Line sensor */
    LineSensor lineSensor;

    /* To add later: rangefinder, camera, etc.*/
    OpenMV camera;

    //Servo
    Servo32U4Pin5 servo;

    //buzzer
    PololuBuzzer buzzer;

    //Load cell
    static const uint8_t HX711_CLK_PIN = 6;
    static const uint8_t HX711_DAT_PIN = 13;
    HX711<HX711_CLK_PIN, HX711_DAT_PIN> loadCell;

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

    bool hasBucket;
    int deadReckoningMillis;
    int centeringMillis;

    int currentI = 0, currentJ = 0;
    int targetI = 2, targetJ = 0;

    int destination = 3;

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
    //IR REMOTE
    void HandleKeyCode(int16_t keyCode);

    //IDLE STATE   
    void EnterIdleState(void);

    //Mode changes
    void EnterTeleopMode(void);
    void EnterAutoMode(void);
    void EnterSetupMode(void);

    //SUPERSTATES
    void EnterNav(void);
    void EnterCollect(void);
    void EnterDeliver(void);

    //Line following and navigation routines.
    void EnterLining(float speed);
    void LiningUpdate(void);
    
    void DriveAfterIntersection(void);
    bool CheckIntersection(void) {return lineSensor.CheckIntersection();}
    void HandleIntersection(void);

    void EnterTurn(int8_t target);
    void TurnUpdate(void);
    bool CheckTurnComplete(void);
    void HandleTurnComplete(void);

    void SetTargetI(int i);
    void SetTargetJ(int j);
    
    void EnterDeadReckon(int milliseconds);
    bool CheckDeadReckonComplete(void);

    /* IMU routines */
    void HandleOrientationUpdate(void);

    //CAMERA
    void FindAprilTags(void);

    /* For commanding the lifter servo */
    void SetLifter(uint16_t position);

    //NEW STATES
    void EnterSearch(void);
    bool CheckSearchComplete(void);

    void EnterApproach(void);
    void ApproachUpdate(void);
    bool CheckApproachComplete(void);

    void EnterLifting(void);
    bool CheckLiftComplete(void);

    void EnterLowering(void);
    bool CheckLowerComplete(void);

    void EnterWeighing(void);
    void WeighUpdate(void);
    bool CheckWeighComplete(void);

    void EnterCentering(int milliseconds);
    void CenteringUpdate(void);
    bool CheckCenteringComplete(void);
    void HandleCenteringComplete(void);

    void EnterReturning(void);
    void ReturnUpdate(void);
    bool CheckReturningComplete(void);
    void HandleReturningComplete(void);

    void ResetCurrents(void);

    void EnterClimbing(void);
    void ClimbingUpdate(void);
    bool CheckClimbingComplete(void);
    void HandleClimbingComplete(void);

    void EnterDescending(void);
    void DescendUpdate(void);
    bool CheckDescendComplete(void);
    void HandleDescendComplete(void);

    void BuzzerUpdate(void);
    void PlayTone(int);
};
