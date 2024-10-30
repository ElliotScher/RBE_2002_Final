#include "robot.h"
#include <IRdecoder.h>

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    /**
     * Initialize the IR decoder. Declared extern in IRdecoder.h; see robot-remote.cpp
     * for instantiation and setting the pin.
     */
    decoder.init();

    /**
     * Initialize the IMU and set the rate and scale to reasonable values.
     */
    imu.init();

    /**
     * TODO: Add code to set the data rate and scale of IMU (or edit LSM6::setDefaults())
     */

    // The line sensor elements default to INPUTs, but we'll initialize anyways, for completeness
    lineSensor.Initialize();
}

void Robot::EnterIdleState(void)
{
    Serial.println("-> IDLE");
    chassis.Stop();
    keyString = "";
    robotState = ROBOT_IDLE;
}

/**
 * Functions related to the IMU (turning; ramp detection)
 */
void Robot::EnterTurn(float angleInDeg)
{
    Serial.println(" -> TURN");
    robotState = ROBOT_TURNING;

    /**
     * TODO: Add code to initiate the turn and set the target
     */
}

bool Robot::CheckTurnComplete(void)
{
    bool retVal = false;

    /**
     * TODO: add a checker to detect when the turn is complete
     */

    return retVal;
}

void Robot::HandleTurnComplete(void)
{
    /**
     * TODO: Add code to handle the completed turn
     */
}

/**
 * Here is a good example of handling information differently, depending on the state.
 * If the Romi is not moving, we can update the bias (but be careful when you first start up!).
 * When it's moving, then we update the heading.
 */
void Robot::HandleOrientationUpdate(void)
{
    prevEulerAngles = eulerAngles;
    if(robotState == ROBOT_IDLE)
    {
        // TODO: You'll need to add code to LSM6 to update the bias
        imu.updateGyroBias();
    }
    else // update orientation
    {
        eulerAngles.z += imu.g.z * (1.0 / imu.SIGMA) * (1.0 / imu.gyroODR);
    }
    prevTime = currentTime;

#ifdef __IMU_DEBUG__
    Serial.println(eulerAngles.z);
#endif
}

/**
 * Functions related to line following and intersection detection.
 */
void Robot::EnterLineFollowing(float speed) 
{
    Serial.println(" -> LINING"); 
    baseSpeed = speed; 
    robotState = ROBOT_LINING;
}

void Robot::LineFollowingUpdate(void)
{
    if(robotState == ROBOT_LINING) 
    {

        chassis.SetTwist(baseSpeed, lineSensor.CalcEffort());
    }
}

/**
 * As coded, HandleIntersection will make the robot drive out 3 intersections, turn around,
 * and stop back at the start. You will need to change the behaviour accordingly.
 */
void Robot::HandleIntersection(void)
{
    Serial.print("X: ");
    if(robotState == ROBOT_LINING) 
    {
        switch(nodeTo)
        {
            case NODE_START:
                if(nodeFrom == NODE_1)
                    EnterIdleState();
                break;
            case NODE_1:
                // By default, we'll continue on straight
                if(nodeFrom == NODE_START) 
                {
                    nodeTo = NODE_2;
                }
                else if(nodeFrom == NODE_2)
                {
                    nodeTo = NODE_START;
                }
                nodeFrom = NODE_1;
                break;
            case NODE_2:
                // By default, we'll continue on straight
                if(nodeFrom == NODE_1) 
                {
                    nodeTo = NODE_3;
                }
                else if(nodeFrom == NODE_3)
                {
                    nodeTo = NODE_1;
                }
                nodeFrom = NODE_2;
                break;
            case NODE_3:
                // By default, we'll bang a u-ey
                if(nodeFrom == NODE_2) 
                {
                    nodeTo = NODE_2;
                    nodeFrom = NODE_3;
                    EnterTurn(180);
                }
                break;
            default:
                break;
        }
        Serial.print(nodeFrom);
        Serial.print("->");
        Serial.print(nodeTo);
        Serial.print('\n');
    }
}

void Robot::RobotLoop(void) 
{
    /**
     * The main loop for your robot. Process both synchronous events (motor control),
     * and asynchronous events (IR presses, distance readings, etc.).
    */

    /**
     * Handle any IR remote keypresses.
     */
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);

    /**
     * Check the Chassis timer, which is used for executing motor control
     */
    if(chassis.CheckChassisTimer())
    {
        // add synchronous, pre-motor-update actions here
        if(robotState == ROBOT_LINING)
        {
            LineFollowingUpdate();
        }

        chassis.UpdateMotors();

        // add synchronous, post-motor-update actions here

    }

    /**
     * Check for any intersections
     */
    if(lineSensor.CheckIntersection()) HandleIntersection();
    if(CheckTurnComplete()) HandleTurnComplete();

    /**
     * Check for an IMU update
     */
    if(imu.checkForNewData())
    {
        HandleOrientationUpdate();
    }
}

