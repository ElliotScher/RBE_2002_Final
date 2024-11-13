#include "chassis.h"
#include "Romi32U4MotorTemplate.h"

Romi32U4EncodedMotor<LEFT_XOR, LEFT_B, PWM_L, DIR_L, OCR_L> leftMotor;
Romi32U4EncodedMotor<RIGHT_XOR, RIGHT_B, PWM_R, DIR_R, OCR_R> rightMotor;

void Chassis::SetMotorKp(float kp) {leftMotor.Kp = kp; rightMotor.Kp = kp;}
void Chassis::SetMotorKi(float ki) {leftMotor.Ki = ki; rightMotor.Ki = ki; leftMotor.sumError = 0; rightMotor.sumError = 0;}
void Chassis::SetMotorKd(float kd) {leftMotor.Kd = kd; rightMotor.Kd = kd;}

/**
 * Because it's declared static, we initialize Chassis::loopFlag here.
 */
uint8_t Chassis::loopFlag = 0;

void Chassis::Timer4OverflowISRHandler(void) 
{
    loopFlag++;

    leftMotor.speed = leftMotor.CalcEncoderDelta();
    rightMotor.speed = rightMotor.CalcEncoderDelta();
}

/**
 * Sets up a hardware timer on Timer4 to manage motor control on a precise schedule.
 * 
 * We set the timer to set an interrupt flag on overflow, which is handled
 * by ISR(TIMER4_OVF_vect) below.
 */
void Chassis::InitializeMotorControlTimer(void)
{
    Serial.println("InitTimer");
    // Disable interupts while we mess with the Timer4 registers
    cli(); 
  
    // Set up Timer4
    TCCR4A = 0x00; // Disable output to pins
    TCCR4B = 0x0A; // Sets the prescaler -- see pp. 167-8 in datasheet
    TCCR4C = 0x00; // Disables output to pins (but see below for buzzer)
    TCCR4D = 0x00; // Normal mode: count up and roll-over

    /**
     * Calculate TOP based on prescaler and loop duration. Note that loop is in integer ms --
     * there may be some rounding. Multiples of 4 ms will be exact.
     */
    uint16_t top = ((CONTROL_LOOP_PERIOD_MS * 16000ul) >> 9) - 1; // divides by 512

    /**
     * Here we do a little trick to allow full 10-bit register access. 
     * We have 2 _bits_ in TC4H that we can use to add capacity to TOP.
     * 
     * Note that the maximum period is limited by TOP = 0x3FF. If you want
     * a longer period, you'll need to adjust the pre-scaler.
     * 
     * There is no minumum period, but precision breaks down with low values, 
     * unless you adjust the pre-scaler, but the encoder resolution is limited,
     * so you only want to go so fast.
    */
    uint8_t highbits = top / 256;
    uint8_t lowbits = top - highbits;
    TC4H = highbits; OCR4C = lowbits;

    // Enable overflow interrupt
    TIMSK4 = 0x04; 

    /**
     * Uncommenting the following lines will pipe the timer signal to pin 6, 
     * which controls the buzzer. The pin will toggle at the loop rate, which
     * allows you to check that the timing is correct. It will also make a lot
     * of noise, so do so sparingly.
     */
    // TCCR4C = 0x04
    // pinMode(6, OUTPUT);

    // Re-enable interrupts
    sei(); 

    Serial.println("/InitTimer");
}

bool Chassis::CheckChassisTimer(void)
{
    bool retVal = false;
    if(loopFlag)
    {
        if(loopFlag > 1) Serial.println("Missed an update in Robot::RobotLoop()!");

#ifdef __LOOP_DEBUG__
        Serial.print(millis());
        Serial.print('\t');
#endif

        loopFlag = 0;
        retVal = true;

#ifdef __LOOP_DEBUG__
        Serial.println();
#endif
    }

    return retVal;
}

void Chassis::UpdateMotors(void)
{
#ifdef __MOTOR_DEBUG__
    leftMotor.ControlMotorSpeed(true);
#else
    leftMotor.ControlMotorSpeed();
#endif
    rightMotor.ControlMotorSpeed();
}

/**
 * SetWheelSpeeds converts the linear wheel speeds (axes relative to ground) to motor speeds.
 */
void Chassis::SetWheelSpeeds(float leftSpeedCMperSec, float rightSpeedCMperSec)
{
    /** 
     * TODO: Add code to convert cm/sec -> encoder ticks/control interval
     */
    
    float leftEncoderTicksPerInterval = leftSpeedCMperSec*LEFT_TICKS_PER_CM*CONTROL_LOOP_PERIOD_MS*0.001;
    float rightEncoderTicksPerInterval = rightSpeedCMperSec*RIGHT_TICKS_PER_CM*CONTROL_LOOP_PERIOD_MS*0.001;

    leftMotor.SetTargetSpeed(leftEncoderTicksPerInterval);
    rightMotor.SetTargetSpeed(rightEncoderTicksPerInterval);
    // Serial.print("left: ");
    // Serial.print(leftEncoderTicksPerInterval);
    // Serial.print("\tright: ");
    // Serial.println(rightEncoderTicksPerInterval);

}

/**
 * ISR for timing. On Timer4 overflow, we take a 'snapshot' of the encoder counts 
 * and raise a flag to let the program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
   Chassis::Timer4OverflowISRHandler();
}

void Chassis::InitializeMotors(void)
{
    Romi32U4MotorBase::InitializePWMTimerAndInterrupts();

    leftMotor.InitializeMotor();
    rightMotor.InitializeMotor();
}

void Chassis::SetMotorEfforts(int16_t left, int16_t right) 
{
    leftMotor.SetMotorEffortDirect(left); 
    rightMotor.SetMotorEffortDirect(right);
}

/**
 * Converts a forward speed and angular velocity to wheel speeds.
 * 
 * fwdSpeed in cm/s; rotSpeed in rad/s.
 */
void Chassis::SetTwist(float fwdSpeed, float rotSpeed)
{
    float leftSpeed = fwdSpeed - rotSpeed * ROBOT_RADIUS;
    float rightSpeed = fwdSpeed + rotSpeed * ROBOT_RADIUS;

    SetWheelSpeeds(leftSpeed, rightSpeed);
}