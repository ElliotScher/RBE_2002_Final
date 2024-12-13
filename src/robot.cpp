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
int8_t targetAngle; 

void Robot::EnterTurn(int8_t target)
{
    Serial.println(" -> TURN");
    robotSubState = ROBOT_TURNING;
    targetDirection = target;
    Serial.print("current direction: ");
    Serial.print(currDirection);
    Serial.print(", target direction: ");
    Serial.println(targetDirection);

    int dirOff = targetDirection - currDirection;
    // targetAngle = eulerAngles.z + (dirOff * 90.0);

    if(dirOff == 3 || dirOff == -3){
        dirOff /= -3;
    }

    if (dirOff > 0){
        chassis.SetTwist(0, 1);
        Serial.println("left turn");
    } else {
        chassis.SetTwist(0, -1);
        Serial.println("right turn");
    }
}

float KpTurn = 0.02;
float KiTurn = 0.05;
float prevTurnError = 0;
float maxEffort = 2.5;
float wrapped;
float turnError;

void Robot::TurnUpdate(void){
    wrapped = fmod(3600.0 + eulerAngles.z, 360.0);
    turnError = targetDirection * 90.0 - wrapped;
    float effort = KpTurn * turnError + KiTurn * (turnError - prevTurnError);

    if (effort > maxEffort){
        effort = maxEffort;
    } else if (effort < -1*maxEffort){
        effort = -1*maxEffort;
    }

    chassis.SetTwist(0, effort);
    prevTurnError = turnError;    
}

bool Robot::CheckTurnComplete(void)
{
    // Serial.print("targetDir: " + (String) targetDirection + ", ");
    // Serial.println("euler: " + (String) eulerAngles.z);

    if (abs(targetDirection * 90 - fmod(eulerAngles.z + 3600, 360)) < 0.6){
        return true;
    }

    return false;
}

void Robot::HandleTurnComplete(void){}

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
        imu.gyroBias.y -= imu.EPSILON / ((imu.mdpsPerLSB / 1000.0) * (1.0 / imu.gyroODR)) * (observedPitchAngle - predictedPitchAngle);
        
        observedPitchAngle = atan2((float) -imu.a.x, (float) imu.a.z - accelBiasX) * 360 / (2*3.14159);
        correctionPitchAngle = predictedPitchAngle + kappa*(observedPitchAngle-predictedPitchAngle);
    }

#ifdef __IMU_DEBUG__

#endif
}

void Robot::EnterLining(float speed) 
{
    Serial.println("-> LINING"); 
    baseSpeed = speed; 
    robotSubState = ROBOT_LINING;
}

float KpLine = 0.0013;
float KdLine = 0.00008;
float prevLineError, prevCorrected;
bool onAngle = false;
float angleThreshold = -6, hysteresisBand = 3;

void Robot::LiningUpdate(void)
{
    if(robotSubState == ROBOT_LINING) 
    {
        int16_t lineError = lineSensor.CalcError();
        float deltaLineError = lineError - prevLineError;
        float turnEffort = KpLine*lineError + KdLine*deltaLineError;
        prevLineError = lineError;

        chassis.SetTwist(baseSpeed, turnEffort);

        if (!onAngle){
            if(correctionPitchAngle < angleThreshold - hysteresisBand){
                onAngle = true;
                EnterDeliver();
                EnterClimbing();
            }
            else if (correctionPitchAngle > -angleThreshold + hysteresisBand) {
                onAngle = true;
                EnterDescending();
            }
        } else {
            if(correctionPitchAngle > angleThreshold + hysteresisBand){
                onAngle = false;
            }
        }
        // Serial.print("pitch: ");
        // Serial.println(correctionPitchAngle);
        prevCorrected = correctionPitchAngle;

        //eulerAngles.z = currDirection * 90.0;
    }
}

void Robot::EnterClimbing(){
    Serial.println("-> CLIMBING"); 
    robotSubState = ROBOT_CLIMBING;
}

void Robot::ClimbingUpdate(){
    int16_t lineError = lineSensor.CalcError();
        float deltaLineError = lineError - prevLineError;
        float turnEffort = KpLine*lineError + KdLine*deltaLineError;
        prevLineError = lineError;

        chassis.SetTwist(22, turnEffort * 2.2);

        if (onAngle){
            if(correctionPitchAngle > angleThreshold + hysteresisBand){
                onAngle = false;
            }
        }
        
        // Serial.print("pitch: ");
        // Serial.println(correctionPitchAngle);
        prevCorrected = correctionPitchAngle;
}

bool Robot::CheckClimbingComplete() {
    return !onAngle;
}

void Robot::HandleClimbingComplete() {
    EnterDeadReckon(1500);
}

void Robot::EnterDescending(){
    Serial.println("-> DESCENDING"); 
    robotSubState = ROBOT_DESCENDING;
}

void Robot::DescendUpdate(){
    int16_t lineError = lineSensor.CalcError();
        float deltaLineError = lineError - prevLineError;
        float turnEffort = KpLine*lineError + KdLine*deltaLineError;
        prevLineError = lineError;

        chassis.SetTwist(10, turnEffort);

        if (onAngle){
            if(correctionPitchAngle < -angleThreshold - hysteresisBand){
                onAngle = false;
            }
        }
        
        // Serial.print("pitch: ");
        // Serial.println(correctionPitchAngle);
        prevCorrected = correctionPitchAngle;
}

bool Robot::CheckDescendComplete() {
    return !onAngle;
}

void Robot::HandleDescendComplete() {
    EnterNav();
    EnterLining(10);
    currentI = 4;
    currentJ = 0;
    destination--;
    targetI = destination;
    targetJ = 0;
}

float deadReckonTime, prevTimestamp;

void Robot::EnterCentering(int milliseconds){
    robotSubState = ROBOT_CENTERING;
    Serial.println("-> CENTERING");
    chassis.SetTwist(10,0);
    deadReckonTime = 0;
    prevTimestamp = millis();
    centeringMillis = milliseconds;
}

float Kpcenter = 0.3;

void Robot::CenteringUpdate(){
    float centerError = -1.0 * fmod(eulerAngles.z, 360.0);
    Serial.println(centerError);
    chassis.SetTwist(10, Kpcenter * centerError);
}

bool Robot::CheckCenteringComplete(void){
    deadReckonTime += millis() - prevTimestamp;
    prevTimestamp = millis();

    if (deadReckonTime > centeringMillis){
        chassis.Stop();
        return true;
    }
    return false;
}

void Robot::HandleCenteringComplete(void)
{
    if(robotSuperState == ROBOT_NAVIGATING)
    {
        if (currentJ == targetJ){
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
            }
        }
        else if (currentJ > targetJ){
            EnterTurn(3);
        }
        else if (currentJ < targetJ){
            EnterTurn(1);
        }
    } else if (robotSuperState == ROBOT_COLLECTING){
        EnterNav();
        targetI = 20;
        targetJ = 0;
        EnterTurn(0);
    } else if (robotSuperState == ROBOT_DELIVERING) {
        EnterTurn(2);
    }

    if((currentJ == 0 && currentI == 0) && (targetJ == 0 && targetI == 0)){
        EnterIdleState();
    }   
}

void Robot::EnterDeadReckon(int milliseconds){
    robotSubState = ROBOT_DEAD_RECKONING;
    Serial.println("-> DEAD RECKONING");
    chassis.SetTwist(-10,0);
    deadReckonTime = 0;
    prevTimestamp = millis();
    deadReckoningMillis = milliseconds;
}

bool Robot::CheckDeadReckonComplete(void){
    deadReckonTime += millis() - prevTimestamp;
    prevTimestamp = millis();

    if (deadReckonTime > deadReckoningMillis){
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

void Robot::ResetCurrents(){
    currentI = 0;
    currentJ = 0;
    targetI = destination;
    targetJ = 0;
    currDirection = 0;
    hasBucket = false;
}

void Robot::HandleIntersection(void)
{

    if(robotSubState == ROBOT_LINING && robotSuperState == ROBOT_NAVIGATING) 
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
            EnterCentering(1000);
        }

        if(currentI == targetI && currentJ == targetJ){
            EnterLowering();
        }

        if(currentI == 0 && currentJ == 0){
            EnterIdleState();
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
    // Serial.print("speed: ");
    // Serial.print(speed);
    // Serial.print("\ttwist: ");
    // Serial.print(twist);
    // Serial.print("\ttagH: ");
    // Serial.println(tag.h);

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
    chassis.Stop();
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
        hasBucket = true;
        return true;
    }
    return false;
}

void Robot::EnterReturning(){
    robotSubState = ROBOT_RETURNING;
    Serial.println("-> RETURNING");
    chassis.SetTwist(10,0);
    deadReckonTime = 0;
    prevTimestamp = millis();
}

float Kpreturn = 0.3;

void Robot::ReturnUpdate(){
    float returnError = 180 - fmod(3600.0 + eulerAngles.z, 360.0);
    Serial.println(returnError);
    chassis.SetTwist(10, Kpreturn * returnError);
}

bool Robot::CheckReturningComplete(){
    deadReckonTime += millis() - prevTimestamp;
    prevTimestamp = millis();

    if (robotSuperState == ROBOT_COLLECTING) {
    return lineSensor.CheckIntersection();
    }
    else if (robotSuperState == ROBOT_DELIVERING) {
        return deadReckonTime > 1300;
    } else {
        return false;
    }
}

void Robot::HandleReturningComplete(){

}

unsigned long previousMicros = 0; 
unsigned long toneStartTime = 0; 
bool buzzerState = false; 
int halfPeriod = 0;
bool isToneActive = false;

int melody[] = {
    880, 1046, 1318, 784, 1174, 
    988, 1480, 660, 2092, 
    1760, 588, 1568 
};

int noteDurations[] = {
    200, 200, 200, 200, 200, 
    200, 200, 200, 200,     
    200, 200, 200           
};

const int melodySize = sizeof(melody) / sizeof(melody[0]);
int currentNoteIndex = 0; // Track the current note

void Robot::BuzzerUpdate() {
    if (isToneActive) {
        unsigned long currentMicros = micros();

        // Toggle the buzzer state at the end of each half period
        if (currentMicros - previousMicros >= halfPeriod) {
            previousMicros = currentMicros;
            buzzerState = !buzzerState;
            digitalWrite(6, buzzerState);
        }

        // Check if the tone duration has elapsed
        if (millis() - toneStartTime >= noteDurations[currentNoteIndex]) {
            isToneActive = false;
            digitalWrite(6, LOW); // Turn off buzzer
        }
    }
}

void Robot::PlayTone(int frequency) {
    halfPeriod = 500000 / frequency; // Calculate half the period in microseconds
    toneStartTime = millis(); // Record when the tone started
    isToneActive = true; // Mark the tone as active
    previousMicros = micros(); // Initialize the microsecond timer
}

void Robot::RobotLoop(void) 
{
    //IR REMOTE KEY PRESSES
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);

    //TIMER RELATED
    if(chassis.CheckChassisTimer())
    {
        // Serial.print("x: ");
        // Serial.print(eulerAngles.x);
        // Serial.print(", y: ");
        // Serial.print(correctionPitchAngle);
        // Serial.print(", z: ");
        // Serial.println(eulerAngles.z);

        if(robotSubState == ROBOT_LINING)
        {
            LiningUpdate();
            //CHECK FOR INTERSECTIONS
            if(lineSensor.CheckIntersection()) HandleIntersection();
        }

        if (robotSubState == ROBOT_TURNING){
            TurnUpdate();
            if(CheckTurnComplete()) {
                currDirection = targetDirection;
                if(robotSuperState == ROBOT_NAVIGATING){
                    EnterLining(10);
                } else if(robotSuperState == ROBOT_COLLECTING){
                    EnterReturning();                  
                } else if (robotSuperState == ROBOT_DELIVERING) {
                    EnterLowering();
                }
            }
        }

        if (robotSubState == ROBOT_CENTERING){
            if (robotSuperState == ROBOT_DELIVERING){
                //CenteringUpdate();
            }

            if(CheckCenteringComplete()){
                HandleCenteringComplete();
            }
        }

        if (robotSubState == ROBOT_DEAD_RECKONING){
            if(CheckDeadReckonComplete()){
                if (robotSuperState == ROBOT_NAVIGATING) {
                    EnterCentering(1000);
                }
                if (robotSuperState == ROBOT_COLLECTING) {
                    EnterLifting();
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
                    EnterDeadReckon(1400);
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
                    hasBucket = false;
                    EnterReturning();
                }
            }
        }

        if (robotSubState == ROBOT_WEIGHING){
            WeighUpdate();
            if(CheckWeighComplete()){
                if (fmod(3600.0 + eulerAngles.z, 360.0) > 180){
                    currDirection = 3;
                    EnterTurn(3);
                } else {
                    currDirection = 1;
                    EnterTurn(1);
                }
            }
        }

        if(robotSubState == ROBOT_RETURNING){

            if (robotSuperState == ROBOT_DELIVERING) {
                ReturnUpdate();
            }
            if(CheckReturningComplete()){
                if (robotSuperState == ROBOT_COLLECTING) {
                EnterCentering(1000);
                }
                else if (robotSuperState == ROBOT_DELIVERING) {
                    EnterLining(10);
                }
            }
        }

        if (robotSubState == ROBOT_CLIMBING) {
            ClimbingUpdate();
            if (CheckClimbingComplete()) {
                EnterCentering(3200);
            }
        }

        if (robotSubState == ROBOT_DESCENDING) {
            DescendUpdate();
            if (CheckDescendComplete()) {
                HandleDescendComplete();
            }
        }

        chassis.UpdateMotors();

    }

    //CHECK IMU
    if(imu.checkForNewData()) HandleOrientationUpdate();

    // If no tone is active, move to the next note in the melody
    if (hasBucket){
            BuzzerUpdate();

            if (!isToneActive) {
                PlayTone(melody[currentNoteIndex]); // Play the next note

                // Move to the next note
                currentNoteIndex++; 

                // Reset index if we've played all notes
                if (currentNoteIndex >= melodySize) {
                    currentNoteIndex = 0; // Reset to start the melody again
                }
            }
        }
}
