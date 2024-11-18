#include "robot.h"
#include <IRdecoder.h>
#include <openmv.h>


void Robot::InitializeRobot(void)
{
    Serial.begin(9600);
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

int8_t currDirection = 0; //EAST
int8_t targetDirection;

void Robot::EnterTurn(void) // 1: 90 degree left, 2: u-ey, 3: 270 degree left/90 right
{
    Serial.println(" -> TURN");
    robotState = ROBOT_TURNING;

    if ((targetDirection - currDirection + 4) % 4 < 2){
        chassis.SetTwist(0, 1);
    } else {
        chassis.SetTwist(0,-1);
    }
}

bool Robot::CheckTurnComplete(void)
{
    if (abs(targetDirection * 90 - fmod(eulerAngles.z + 3600, 360)) < 1){
        return true;
    }

    return false;
}

void Robot::HandleTurnComplete(void)
{
    chassis.SetWheelSpeeds(0,0);
    currDirection = targetDirection;
    robotState = ROBOT_IDLE;
}

float accelBiasX, accelBiasY, accelBiasZ, zetaAccel = 0.9;
float predictedPitchAngle, observedPitchAngle, correctionPitchAngle, kappa = 0.1;

void Robot::HandleOrientationUpdate(void)
{
    // Update previous Euler angles
    prevEulerAngles = eulerAngles;

    if (robotState == ROBOT_IDLE) {

        // Update gyro bias in the IMU
        imu.updateGyroBias();

        accelBiasX = zetaAccel * accelBiasX + (1 - zetaAccel) * imu.a.x;
        accelBiasY = zetaAccel * accelBiasY + (1 - zetaAccel) * imu.a.y;

    } else { // Update orientation

        // Read the gyro data from the IMU
        imu.readGyro();

        // Update Euler angles, applying bias correction for each axis
        eulerAngles.x += (imu.g.x - imu.gyroBias.x) * (imu.mdpsPerLSB/1000) / imu.gyroODR;
        eulerAngles.z += (imu.g.z - imu.gyroBias.z) * (imu.mdpsPerLSB/1000) / imu.gyroODR;

        predictedPitchAngle += ((imu.mdpsPerLSB/1000) / imu.gyroODR) * (imu.g.y - imu.gyroBias.y);
        //imu.gyroBias.y -= imu.EPSILON / ((imu.mdpsPerLSB / 1000.0) * (1.0 / imu.gyroODR)) * (observedPitchAngle - predictedPitchAngle);
        
        observedPitchAngle = atan2((float) -imu.a.x, (float) imu.a.z - accelBiasX) * 360 / (2*3.14159);
        correctionPitchAngle = predictedPitchAngle + kappa*(observedPitchAngle-predictedPitchAngle);
    }

#ifdef __IMU_DEBUG__

#endif
}

void Robot::EnterLining(float speed) 
{
    Serial.println(" -> LINING"); 
    baseSpeed = speed; 
    robotState = ROBOT_LINING;

}

float KpLine = 0.0008;
float KdLine = 0.00008;
float prevLineError, prevCorrected;
bool onAngle = false;
float angleThreshold = -7, hysteresisBand = 3;

void Robot::LiningUpdate(void)
{
    if(robotState == ROBOT_LINING) 
    {
        // TODO: calculate the error in CalcError(), calc the effort, and update the motion
        int16_t lineError = lineSensor.CalcError();
        float deltaLineError = lineError - prevLineError;
        float turnEffort = KpLine*lineError + KdLine*deltaLineError;
        prevLineError = lineError;
        //Serial.println(turnEffort);

        chassis.SetTwist(baseSpeed, turnEffort);

        if (!onAngle){
            if(correctionPitchAngle < angleThreshold - hysteresisBand){
                onAngle = true;
                Serial.println("ON ANGLE \n\n\n\n\n\n\n\n\n\n");
                digitalWrite(30, LOW);
            }
        } else {
            if(correctionPitchAngle > angleThreshold + hysteresisBand){
                onAngle = false;
                Serial.println("ON FLAT \n\n\n\n\n\n\n\n\n\n");
                targetDirection = 2;
                EnterTurn();
                digitalWrite(30, HIGH);
            }
        }
        Serial.print("pitch: ");
        Serial.println(correctionPitchAngle);

        prevCorrected = correctionPitchAngle;
    }
}

float deadReckonTime, prevTimestamp;

void Robot::EnterDeadReckon(void){
    robotState = ROBOT_DEAD_RECKONING;
    Serial.println(" -> DEAD RECKONING");
    chassis.SetTwist(-10,0);
    deadReckonTime = 0;
    prevTimestamp = millis();
}

bool Robot::CheckDeadReckonComplete(void){
    deadReckonTime += millis() - prevTimestamp;
    prevTimestamp = millis();

    if (deadReckonTime > 1400){
        digitalWrite(30, HIGH);
        return true;
    }
    return false;
}

int currentI = 0, currentJ = 0;
int targetI = 200, targetJ = 0;
bool reverseMode = false;

void Robot::SetTargetI(int i){
    targetI = i;
    currentI = 0;
    currentJ = 0;  
    currDirection = 0;
    eulerAngles.x = 0, eulerAngles.y = 0, eulerAngles.z = 0;
    reverseMode = false;
}

void Robot::SetTargetJ(int j){
    targetJ = j;
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

AprilTagDatum tag;
bool tagFound = false;
bool tagLost = false;
uint16_t lastTagCounter;
void Robot::FindAprilTags(void)
{
    if (camera.checkUART(tag) && tag.id == 0) {
        tagFound = true;
        lastTagCounter = 0;
    } else {
        tagFound = false;
    }
}

bool callFlag;

void Robot::EnterSearch(void){
    robotState = ROBOT_SEARCHING;
    Serial.println(" -> SEARCHING");
    chassis.SetTwist(0,0.7);
    tagFound = false;
    tagLost = false;
    callFlag = true;
}

bool Robot::CheckSearchComplete(void){
    FindAprilTags();
    if (abs((float) tag.cx - 100.0) < 30){
        return tagFound;
    } else {
        return false;
    }
}

void Robot::EnterApproach(void){
    robotState = ROBOT_APPROACHING;
    Serial.println(" -> APPROACHING");
    digitalWrite(30, LOW);
}

float KpSpeed = 1, KpTwist = 0.02, KdTwist = 0.08;

void Robot::ApproachUpdate(void){
    FindAprilTags();
    static float prevTwistError = 0;
    float speed = (tag.h - 65.0) * KpSpeed;
    float twistError = (float) tag.cx - 100.0;
    float twist = twistError * KpTwist + (twistError - prevTwistError) * KdTwist;
    prevTwistError = twistError;

    float maxSpeed = 30.0;
    float maxTwist = 1.5;

    if (speed > maxSpeed) {
        speed = maxSpeed;
    } else if (speed < -1.0 * maxSpeed) {
        speed = -1.0 * maxSpeed;
    }

    if (twist > maxTwist) {
        twist = maxTwist;
    } else if (twist < (-1.0 * maxTwist)) {
        twist = -1.0 * maxTwist;
    }

    //CHECK FOR LOST TAG
    lastTagCounter++;
    if (lastTagCounter > 50){
        Serial.println("TAG LOST_____________________");
        tagLost = true;
    }

    chassis.SetTwist(speed, twist);
    Serial.print("speed: ");
    Serial.print(speed);
    Serial.print("\ttwist: ");
    Serial.print(twist);
    Serial.print("\ttagH: ");
    Serial.println(tag.h);

}

bool Robot::CheckApproachComplete(void){
    if ((tag.h >= 55) || tagLost) {
        return true;
    }
    return false;
}

void Robot::RobotLoop(void) 
{
    //IR REMOTE KEY PRESSES
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);

    //TIMER RELATED
    if(chassis.CheckChassisTimer())
    {
        if(robotState == ROBOT_LINING)
        {
            LiningUpdate();

            //CHECK FOR INTERSECTIONS
            if(lineSensor.CheckIntersection()) HandleIntersection();
        }

        if (robotState == ROBOT_TURNING){
            if(CheckTurnComplete()) {
                HandleTurnComplete();
            }
        }

        if (robotState == ROBOT_DEAD_RECKONING){
            if(CheckDeadReckonComplete()){
                EnterIdleState();
            }
        }

        if (robotState == ROBOT_SEARCHING){
            if (CheckSearchComplete()){
                if (callFlag){
                    callFlag = false;
                } else {
                EnterApproach();
                }
            }
        }

        if (robotState == ROBOT_APPROACHING){
            ApproachUpdate();
            if(CheckApproachComplete()){
                if (tagLost){
                    EnterIdleState();
                } else {
                    EnterDeadReckon();
                }
            }
        }
        chassis.UpdateMotors();
    }

    //CHECK IMU
    if(imu.checkForNewData()) HandleOrientationUpdate();
}

