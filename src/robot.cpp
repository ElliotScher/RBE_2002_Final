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
    imu.setGyroDataOutputRate(LSM6::ODR104);
    imu.setFullScaleGyro(LSM6::GYRO_FS245);

    // The line sensor elements default to INPUTs, but we'll initialize anyways, for completeness
    lineSensor.Initialize();
    Serial1.begin(115200);
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
    float observedPitchAngle;
    prevEulerAngles = eulerAngles;
    if(robotState == ROBOT_IDLE)
    {
        // TODO: You'll need to add code to LSM6 to update the bias
        imu.updateGyroBias();
    }
    else // update orientation
    {
        observedPitchAngle = atan2(-imu.a.x, imu.a.z) * (180.0 / M_PI);

        predictedPitchAngle = eulerAngles.y + ((imu.mdpsPerLSB) * (imu.g.y - imu.gyroBias.y) * (1.0 / imu.gyroODR)) / 1000.0;

        imu.gyroBias.y = imu.previousGyroBias.y - imu.EPSILON / ((imu.mdpsPerLSB / 1000.0) * (1.0 / imu.gyroODR)) * (observedPitchAngle - predictedPitchAngle);

        eulerAngles.y = predictedPitchAngle + imu.KAPPA * (observedPitchAngle - predictedPitchAngle);










        eulerAngles.z += (imu.mdpsPerLSB * (imu.g.z - imu.gyroBias.z) * (1.0 / imu.gyroODR)) / 1000.0;

        if (eulerAngles.z > 180.0) {
            eulerAngles.z -= 360.0;
        }
        if (eulerAngles.z < -180.0) {
            eulerAngles.z += 360.0;
        }

        imu.previousGyroBias = imu.gyroBias;
    }

#ifdef __IMU_DEBUG__
    // Serial.print(">Yaw: ");
    // Serial.print(eulerAngles.z);
    // Serial.println("\n");

    Serial.print(">Predicted:");
    Serial.println(predictedPitchAngle);

    Serial.print(">Observed:");
    Serial.println(observedPitchAngle);

    Serial.print(">Filtered:");
    Serial.println(eulerAngles.y);

    Serial.print(">Gyro Bias:");
    Serial.println(imu.gyroBias.y);
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
    if (eulerAngles.y < -7.0) {
        EnterClimbing(10);
    }
}

void Robot::HandleClimbComplete(void)
{
    chassis.SetTwist(0, 0);
    digitalWrite(13, LOW);
    robotState = ROBOT_IDLE;
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
        direction = NORTH;
    } else if (angle > 45 && angle < 135) {
        iGrid++;  // Facing "east," increment i
        direction = EAST;
    } else if ((angle >= 135 && angle <= 180) || (angle <= -135 && angle >= -180)) {
        jGrid--;  // Facing "south," decrement j
        direction = SOUTH;
    } else if (angle > -135 && angle < -45) {
        iGrid--;  // Facing "west," decrement i
        direction = WEST;
    }

    if (jGrid < jTarget && direction == NORTH) {
        EnterLineFollowing(10);
    } else if (jGrid == jTarget && direction == NORTH) {
        EnterTurn(180);
    } else if (jGrid == jTarget && direction == SOUTH) {
        EnterLineFollowing(10);
    } else if (jGrid > 0 && direction == SOUTH) {
        EnterLineFollowing(10);
    } else if (jGrid == 0) {
        chassis.Stop();
        robotState = ROBOT_IDLE;
    }
}

void Robot::EnterClimbing(float speed) 
{
        Serial.println(" -> CLIMBING");
        baseSpeed = speed; 
        robotState = ROBOT_CLIMBING;
        digitalWrite(13, HIGH);
}

bool Robot::CheckClimbComplete(void) {
    if (robotState == ROBOT_CLIMBING) {
        return eulerAngles.y < 0.0 && eulerAngles.y > -5.0;
    }
    return false;
}

void Robot::HandleAprilTag(const AprilTagDatum& tag)
{
    approachTimer.start(1000);
    Serial.print("Tag: ");
    Serial.print(tag.id); Serial.print('\t');
    Serial.print(tag.cx); Serial.print('\t');
    Serial.print(tag.cy); Serial.print('\t');
    Serial.print(tag.h); Serial.print('\t');
    Serial.print(tag.w); Serial.print('\t');
    Serial.print(tag.rot); Serial.print('\t');
    Serial.print('\n');

    if (robotState != ROBOT_APPROACHING) {
        EnterApproachingState();
    } else if (robotState == ROBOT_APPROACHING) {
        chassis.SetTwist(approachdrivekp * -(70 - tag.h), approachturnkp * -(80 - tag.cx));
    }

    if (CheckApproachComplete(5, 5)) {
        digitalWrite(13, LOW);
        EnterIdleState();
        delay(1000);
    }
}

void Robot::HandleTimerStop() {
    chassis.SetTwist(0, 0);
}

void Robot::EnterSearchingState(void)
{
    Serial.println(" -> SEARCHING");
    robotState = ROBOT_SEARCHING;
    chassis.SetTwist(0, 0.5);
}
void Robot::EnterApproachingState(void)
{
    Serial.println(" -> APPROACHING");
    robotState = ROBOT_APPROACHING;
    digitalWrite(13, HIGH);
}

bool Robot::CheckApproachComplete(int headingTolerance, int distanceTolerance)
{
    return tag.h > 70 - distanceTolerance && tag.h < 70 + distanceTolerance && tag.cx > 80 - headingTolerance && tag.cx < 80 + headingTolerance;
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
    if(CheckClimbComplete()) HandleClimbComplete();
    if(openMV.checkUART(tag)) HandleAprilTag(tag);
    if (approachTimer.checkExpired()) HandleTimerStop();

    /**
     * Check for an IMU update
     */
    if(imu.checkForNewData())
    {
        HandleOrientationUpdate();
    }
}

