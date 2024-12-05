#include "robot.h"
#include <IRdecoder.h>
#include <openmv.h>
#include <HX711.h>


void Robot::InitializeRobot(void)
{
    Serial.begin(9600);
    chassis.InititalizeChassis();

    //Initialize the IR decoder
    decoder.init();

    //Initialize the IMU and set the rate and scale to reasonable values.
    imu.init();

    //The line sensor elements default to INPUTs, but we'll initialize anyways, for completeness
    lineSensor.Initialize();

    servo.attach();
    loadCell.Init();
    loadCell.SetGain(1);
}

void Robot::EnterIdleState(void)
{
    Serial.println("-> IDLE");
    chassis.Stop();
    keyString = "";
    robotSubState = ROBOT_IDLE;
}

void Robot::EnterNav(void){
    Serial.println("SUPER -> NAV");
    robotSuperState = ROBOT_NAVIGATING;
}

void Robot::EnterCollect(void){
    Serial.println("SUPER -> COLLECT");
    robotSuperState = ROBOT_COLLECTING;
}

void Robot::EnterDeliver(void){
    Serial.println("SUPER -> DELIVER");
    robotSuperState = ROBOT_DELIVERING;
}

int8_t currDirection = 0; //EAST
int8_t targetDirection;

void Robot::EnterTurn(int8_t target)
{
    Serial.println(" -> TURN");
    robotSubState = ROBOT_TURNING;
    targetDirection = target;

    if ((targetDirection - currDirection + 4) % 4 < 2){
        chassis.SetTwist(0, 1);
    } else {
        chassis.SetTwist(0,-1);
    }
}

bool Robot::CheckTurnComplete(void)
{
    Serial.print("targetDir: " + (String) targetDirection + ", ");
    Serial.println("euler: " + (String) eulerAngles.z);

    if (abs(targetDirection * 90 - fmod(eulerAngles.z + 3600, 360)) < 1){
        return true;
    }

    return false;
}

void Robot::HandleTurnComplete(void)
{
    EnterLining(10);
    currDirection = targetDirection;
    robotSubState = ROBOT_LINING;
}

float accelBiasX, accelBiasY, accelBiasZ, zetaAccel = 0.9;
float predictedPitchAngle, observedPitchAngle, correctionPitchAngle, kappa = 0.1;

void Robot::HandleOrientationUpdate(void)
{
    // Update previous Euler angles
    prevEulerAngles = eulerAngles;

    if (robotSubState == ROBOT_IDLE) {

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
    robotSubState = ROBOT_LINING;
}

float KpLine = 0.0008;
float KdLine = 0.00008;
float prevLineError, prevCorrected;
bool onAngle = false;
float angleThreshold = -7, hysteresisBand = 3;

void Robot::LiningUpdate(void)
{
    if(robotSubState == ROBOT_LINING) 
    {
        int16_t lineError = lineSensor.CalcError();
        float deltaLineError = lineError - prevLineError;
        float turnEffort = KpLine*lineError + KdLine*deltaLineError;
        prevLineError = lineError;

        chassis.SetTwist(baseSpeed, turnEffort);
    }
}

float deadReckonTime, prevTimestamp;

void Robot::EnterCentering(void){
    robotSubState = ROBOT_CENTERING;
    Serial.println(" -> CENTERING");
    chassis.SetTwist(10,0);
    deadReckonTime = 0;
    prevTimestamp = millis();
}

bool Robot::CheckCenteringComplete(void){
    deadReckonTime += millis() - prevTimestamp;
    prevTimestamp = millis();

    if (deadReckonTime > 1000){
        chassis.Stop();
        return true;
    }
    return false;
}

int currentI = 0, currentJ = 0;
int targetI = 2, targetJ = 2;
bool reverseMode = false;

void Robot::HandleCenteringComplete(void)
{
    if(robotSubState == ROBOT_CENTERING)
    {
        if (currentJ == targetJ){
            chassis.SetTwist(0, 0);
            if (currentI > targetI){
                EnterTurn(2);
            }
            else if (currentI < targetI){
                EnterTurn(0);
            }
            else if (currentI == targetI){
                if(currDirection == 2){
                    EnterTurn(0);
                }
                else if(currDirection == 0){
                    EnterTurn(2);
                }
                else{
                    EnterTurn((int)(currDirection + 2) % 4);
                }

                targetI = 0, targetJ = 0;
            }
        }
        else if (currentJ > targetJ){
            EnterTurn(3);
        }
        else if (currentJ < targetJ){
            EnterTurn(1);
        }
    }

    if((currentJ == 0 && currentI == 0) && (targetJ == 0 && targetI == 0)){
        EnterIdleState();
    }   
}

void Robot::EnterDeadReckon(void){
    robotSubState = ROBOT_DEAD_RECKONING;
    Serial.println(" -> DEAD RECKONING");
    chassis.SetTwist(-10,0);
    deadReckonTime = 0;
    prevTimestamp = millis();
}

bool Robot::CheckDeadReckonComplete(void){
    deadReckonTime += millis() - prevTimestamp;
    prevTimestamp = millis();

    if (deadReckonTime > 1500){
        chassis.Stop();
        return true;
    }
    return false;
}

void Robot::SetTargetI(int i){
    targetI = i;
}

void Robot::SetTargetJ(int j){
    targetJ = j;
}

void Robot::HandleIntersection(void)
{

    if(robotSubState == ROBOT_LINING) 
    {

        if(targetDirection == 1){
            currentJ++;
        }
        else if(targetDirection == 2){
            currentI--;
        }
        else if(targetDirection == 3){
            currentJ--;
        }
        else if(targetDirection == 0){
            currentI++;
        }
       
        Serial.print("X: ");
        Serial.print(currentI);
        Serial.print(", Y: ");
        Serial.println(currentJ);

        if(currentI == targetI && currentJ == 0){
            EnterCentering();
        }

        if(currentJ == targetJ){
            chassis.Stop();
            EnterLowering();
        }
    }

}

AprilTagDatum tag;
bool tagFound = false;
bool tagLost = false;
uint16_t lastTagCounter;
void Robot::FindAprilTags(void)
{
    if (camera.checkUART(tag)) {
        tagFound = true;
        lastTagCounter = 0;
    } else {
        tagFound = false;
    }
}

bool callFlag;

void Robot::EnterSearch(void){
    EnterCollect();
    robotSubState = ROBOT_SEARCHING;
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
    robotSubState = ROBOT_APPROACHING;
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

    float maxSpeed = 10.0;
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

void Robot::SetLifter(uint16_t position){
    servo.setTargetPos(position);
    Serial.println("NEW TARGET: " + (String) position);
}

void Robot::EnterLifting(void){
    servo.setTargetPos(800);
    robotSubState = ROBOT_LIFTING;
    Serial.println("-> LIFTING");
    deadReckonTime = 0;
    prevTimestamp = millis();
}

bool Robot::CheckLiftComplete(void){
    deadReckonTime += millis() - prevTimestamp;
    prevTimestamp = millis();

    if (deadReckonTime > 1500){
        return true;
    }
    return false;
}

void Robot::EnterLowering(void){
    servo.setTargetPos(3200);
    robotSubState = ROBOT_LOWERING;
    Serial.println("-> LOWERING");
    deadReckonTime = 0;
    prevTimestamp = millis();
}

bool Robot::CheckLowerComplete(void){
    deadReckonTime += millis() - prevTimestamp;
    prevTimestamp = millis();

    if (deadReckonTime > 1500){
        return true;
    }
    return false;
}

float weighingTime;
int weighCounter;
float liftedWeight;
float sum;

void Robot::EnterWeighing(void){
    robotSubState = ROBOT_WEIGHING;
    Serial.println("-> WEIGHING");
    weighCounter = 0;
    sum = 0;
    weighingTime = 0;
    prevTimestamp = millis();
}

int32_t reading, prevReading;

void Robot::WeighUpdate(void){
    weighingTime += millis() - prevTimestamp;
    prevTimestamp = millis();

    if (loadCell.GetReading(reading)  && reading != prevReading) {
        //float singleMeasurement = (0.001429 * (float) reading) + 268.6;
        float singleMeasurement = ((float) reading + 579834) / 659.134;
        Serial.print("Measuring... ");
        Serial.println(singleMeasurement);
        sum += singleMeasurement;
        weighCounter++;
    }
    prevReading = reading;
}

bool Robot::CheckWeighComplete(){
    if (weighCounter >= 25){
        liftedWeight = sum / weighCounter;
        Serial.println("DONE WEIGHING, Tag ID " + (String) tag.id  + " user charged for " + (String) liftedWeight + "grams of trash.");
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
        if(robotSubState == ROBOT_LINING)
        {
            LiningUpdate();
            //CHECK FOR INTERSECTIONS
            if(lineSensor.CheckIntersection()) HandleIntersection();
        }

        if (robotSubState == ROBOT_TURNING){
            if(CheckTurnComplete()) {
                HandleTurnComplete();
            }
        }

        if (robotSubState == ROBOT_CENTERING){
            if(CheckCenteringComplete()){
                HandleCenteringComplete();
            }
        }

        if (robotSubState == ROBOT_DEAD_RECKONING){
            if(CheckDeadReckonComplete()){
                if (robotSuperState == ROBOT_NAVIGATING) {
                    EnterCentering();
                }
                if (robotSuperState == ROBOT_COLLECTING) {
                    EnterLifting();
                }
                if (robotSuperState == ROBOT_DELIVERING) {
                    EnterLowering();
                }
            }
        }

        if (robotSubState == ROBOT_SEARCHING){
            if (CheckSearchComplete()){
                if (callFlag){
                    callFlag = false;
                } else {
                EnterApproach();
                }
            }
        }

        if (robotSubState == ROBOT_APPROACHING){
            ApproachUpdate();
            if(CheckApproachComplete()){
                if (tagLost){
                    EnterIdleState();
                } else {
                    EnterDeadReckon();
                }
            }
        }

        if (robotSubState == ROBOT_LIFTING){
            servo.update();
            if (CheckLiftComplete()){
                EnterWeighing();
            }
        }

        if (robotSubState == ROBOT_LOWERING){
            servo.update();
            if (CheckLowerComplete()){
                if (robotSuperState == ROBOT_NAVIGATING){
                    EnterSearch();
                } else if (robotSuperState == ROBOT_DELIVERING){

                }
            }
        }

        if (robotSubState == ROBOT_WEIGHING){
            WeighUpdate();
            if(CheckWeighComplete()){
                EnterIdleState();
            }
        }
        chassis.UpdateMotors();
    }
    //CHECK IMU
    if(imu.checkForNewData()) HandleOrientationUpdate();
}

