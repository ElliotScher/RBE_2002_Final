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
int8_t currDirection = 0; //EAST
int8_t targetDirection;

void Robot::EnterTurn(void) // 1: 90 degree left, 2: u-ey, 3: 270 degree left/90 right
{
    Serial.println(" -> TURN");
    robotState = ROBOT_TURNING;

    /**
     * TODO: Add code to initiate the turn and set the target
     */
    if ((targetDirection - currDirection + 4) % 4 < 2){
        chassis.SetTwist(0, 1);
    } else {
        chassis.SetTwist(0,-1);
    }
}

bool Robot::CheckTurnComplete(void)
{
     /**
     * TODO: add a checker to detect when the turn is complete
     */

    if (abs(targetDirection * 90 - fmod(eulerAngles.z + 3600, 360)) < 1){
        return true;
    }

    return false;
}

void Robot::HandleTurnComplete(void)
{
    /**
     * TODO: Add code to handle the completed turn
     */
    chassis.SetWheelSpeeds(0,0);
    currDirection = targetDirection;
    robotState = ROBOT_IDLE;
}

/**
 * Here is a good example of handling information differently, depending on the state.
 * If the Romi is not moving, we can update the bias (but be careful when you first start up!).
 * When it's moving, then we update the heading.
 */

void Robot::HandleOrientationUpdate(void)
{
    // Update previous Euler angles
    prevEulerAngles = eulerAngles; 

    if (robotState == ROBOT_IDLE) {

        // Update gyro bias in the IMU
        imu.updateGyroBias();
        //Serial.print("BIAS___");

    } else { // Update orientation

        // Read the gyro data from the IMU
        imu.readGyro();
        //Serial.print("ANGLE___");

        // Update Euler angles, applying bias correction for each axis
        eulerAngles.x += (imu.g.x - imu.gyroBias.x) * (imu.mdpsPerLSB/1000) / imu.gyroODR;
        eulerAngles.y += (imu.g.y - imu.gyroBias.y) * (imu.mdpsPerLSB/1000) / imu.gyroODR;
        eulerAngles.z += (imu.g.z - imu.gyroBias.z) * (imu.mdpsPerLSB/1000) / imu.gyroODR;
    }

#ifdef __IMU_DEBUG__

        // Serial.print("\t eulerZ: ");
        // Serial.print(eulerAngles.z);
        // Serial.print("\tTimestamp: ");
        // Serial.print(millis());

        // Serial.print("\tROBOT STATE: ");
        // Serial.println(robotState);    

        // Serial.print("adjusted x: ");
        // Serial.print(imu.g.x - imu.gyroBias.x);
        // Serial.print("\t eadjusted y: ");
        // Serial.print(imu.g.y - imu.gyroBias.y);
        // Serial.print("\t adjusted xulerZ: ");
        // Serial.print(imu.g.z - imu.gyroBias.z);
        // Serial.print("\tTimestamp: ");
        // Serial.println(millis());

        // Serial.print("adjusted x: ");
        // Serial.print(imu.g.x);
        // Serial.print("\t eadjusted y: ");
        // Serial.print(imu.g.y - imu.gyroBias.y);
        // Serial.print("\t adjusted xulerZ: ");
        // Serial.print(imu.g.z - imu.gyroBias.z);
        // Serial.print("\tTimestamp: ");
        // Serial.println(millis());
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

float KpLine = 0.0003;
float KdLine = 0.00008;
float prevLineError;

void Robot::LineFollowingUpdate(void)
{
    if(robotState == ROBOT_LINING) 
    {
        // TODO: calculate the error in CalcError(), calc the effort, and update the motion
        int16_t lineError = lineSensor.CalcError();
        float deltaLineError = lineError - prevLineError;
        float turnEffort = KpLine*lineError + KdLine*deltaLineError;
        prevLineError = lineError;

        chassis.SetTwist(baseSpeed, turnEffort);
    }
}

float deadReckonTime, prevTimestamp;

void Robot::EnterDeadReckon(void){
    robotState = ROBOT_DEAD_RECKONING;
    chassis.SetTwist(5,0);
    deadReckonTime = 0;
    prevTimestamp = millis();
}

bool Robot::CheckDeadReckonComplete(void){
    deadReckonTime += millis() - prevTimestamp;
    prevTimestamp = millis();
    //Serial.println(deadReckonTime);

    if (deadReckonTime > 1500){
        robotState = ROBOT_IDLE;
        return true;
    }

    return false;
}

/**
 * As coded, HandleIntersection will make the robot drive out 3 intersections, turn around,
 * and stop back at the start. You will need to change the behaviour accordingly.
 */

int currentI = 0, currentJ = 0;         // Current grid position 
int targetI = 2, targetJ = 0;           // Target grid position
bool reverseMode = false;               // Mode tracking forward or reverse

void Robot::SetTarget(int i){
    targetI = i;
    currentI = 0;
    currentJ = 0;  
    currDirection = 0;
    eulerAngles.x = 0, eulerAngles.y = 0, eulerAngles.z = 0;
    reverseMode = false;
}

void Robot::HandleIntersection(void)
{
    if (!reverseMode) {
        // Forward mode logic                                    
        if (currDirection == 0) { //east
            currentI++;
            if (currentI >= targetI && currentJ >= targetJ) {
                targetDirection = 2; //turn to west
                EnterDeadReckon();
                reverseMode = true;
            } else if (currentI >= targetI) {
                targetDirection = 1; //turn to north
                EnterDeadReckon();    
            }               
        } else { //north
            currentJ++;
            if (currentI >= targetI && currentJ >= targetJ) {
                targetDirection = 3; // turn to south
                EnterDeadReckon();
                reverseMode = true;
            }                   
        }
    } else {
        // Reverse mode logic       
        if (currDirection == 2) { // west
            currentI--;  
            if (currentI <= 0 && currentJ <= 0) {
                EnterIdleState(); //stop when back to 0,0
                Serial.println("STOP");
                Serial.end();     
            }                
        } else if (currDirection == 3) { // south
            currentJ--;
            if (currentJ <= 0) {
                targetDirection = 2; // turn to west
                EnterDeadReckon();  
            }                   
        }
    }
}

void Robot::RobotLoop(void) 
{
    /**
     * The main loop for your robot. Process both synchronous events (motor control),
     * and asynchronous events (IR presses, distance readings, etc.).
    */

    // Serial.print("\t eulerZ: ");
    // Serial.print(eulerAngles.z);
    // Serial.print("\tTimestamp: ");
    // Serial.println(millis());

        // Serial.print("Position: (");
        // Serial.print(currentI);
        // Serial.print(", ");
        // Serial.print(currentJ);
        // Serial.print(")\tCurrent Direction: ");
        // Serial.print(currDirection);
        // Serial.print("\tTarget Direction: ");
        // Serial.print(targetDirection);
        // Serial.print("\tTargetI: ");
        // Serial.print(targetI);
        // Serial.print("\tREVERSE? ");
        // Serial.println(reverseMode);

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

        if (robotState == ROBOT_TURNING){
            if(CheckTurnComplete()) {
                HandleTurnComplete();
                EnterLineFollowing(15);
            }
        }

        if (robotState == ROBOT_DEAD_RECKONING){
            if(CheckDeadReckonComplete()){
                EnterTurn();
            }
        }
        chassis.UpdateMotors();

        // add synchronous, post-motor-update actions here

    }

    /**
     * Check for any intersections
     */
    if(lineSensor.CheckIntersection()) HandleIntersection();

    /**
     * Check for an IMU update
     */

    digitalWrite(12, HIGH);

    if(imu.checkForNewData())
    {
        HandleOrientationUpdate();
    }

    digitalWrite(12, LOW);
}

