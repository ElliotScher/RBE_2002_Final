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
    imu.setGyroDataOutputRate(LSM6::ODR13);
    imu.setFullScaleGyro(LSM6::GYRO_FS500);

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
    targetDirection = angleInDeg;

    float angleDifference = targetDirection - eulerAngles.z;

    // Normalize angleDifference to the range [-180, 180]
    if (angleDifference > 180.0) {
        angleDifference -= 360.0;
    } else if (angleDifference < -180.0) {
        angleDifference += 360.0;
    }

    // Turn left or right based on the shortest path
    if (angleDifference < 0) {
        chassis.SetTwist(0, -1.0); // Turn left
    } else if (angleDifference > 0) {
        chassis.SetTwist(0, 1.0);  // Turn right
    }
}

bool Robot::CheckTurnComplete(void)
{
    bool retVal = false;

    float error = abs(targetDirection - eulerAngles.z);

    if (robotState == ROBOT_TURNING) {
        if (error < 5.0) {
            retVal = true;
        }
    }
    
    return retVal;
}

void Robot::HandleTurnComplete(void)
{
    chassis.SetTwist(0, 0);
    robotState = ROBOT_LINING;
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
        eulerAngles.z += (LSM6::CTRL2_G * (imu.g.z - imu.gyroBias.z) * (1.0 / imu.gyroODR)) / 1000.0;

        if (eulerAngles.z > 180.0) {
            eulerAngles.z -= 360.0;
        }
        if (eulerAngles.z < -180.0) {
            eulerAngles.z += 360.0;
        }
    }

#ifdef __IMU_DEBUG__
    Serial.print(">Yaw: ");
    Serial.print(eulerAngles.z);
    Serial.println("\n");
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
    float angle = eulerAngles.z;
    if (angle >= -45 && angle <= 45) {
        jGrid++;  // Facing "north," increment j
    } else if (angle > 45 && angle < 135) {
        iGrid++;  // Facing "east," increment i
    } else if ((angle >= 135 && angle <= 180) || (angle <= -135 && angle >= -180)) {
        jGrid--;  // Facing "south," decrement j
    } else if (angle > -135 && angle < -45) {
        iGrid--;  // Facing "west," decrement i
    }
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

