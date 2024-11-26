#pragma once

#include <Arduino.h>

/** \file Manages servos on up to three pins: 
 *  pin 5, 
 *  pin 13, and
 *  either pin 6 OR 12 (don't use 6 and 12 simultaneously!).
 * 
 * Pin 5 uses Timer3 and has the fewest restrictions as well as the highest precision (0.5 us).
 * 
 * The other three have potential conflicts and much lower resolution (64 us).
 * 
 * Pin 13 uses Timer4, but shares functionality with the bootloader, which flashes the LED connected
 * to that pin. So, your servo will go a bit nuts when you upload -- be careful!
 * 
 * Pins 6 and 12 use Timer4, but share the same output compare -- except that they are inverted.
 * Practically, that means you can use one or the other. Note that pin 6 is shared with the buzzer,
 * so you'll need to cut the buzzer trace if you want to use that pin and maintain your sanity.
 * 
 * If using the Chassis class, Timer4 is used for the control loop, so be sure you don't create any 
 * conflicts.
 * 
 * You can either call writeMicroseconds directly (which will make it move really fast), or
 * set the target and call update() regularly.
 * 
 * */

/** \class Servo32U4Base
 * \brief Base class class for servos.
 * 
 * Each derived class controls a specific pin (obvious from the name).
 * 
 * Defaults to a range of 1000 - 2000 us, but can be customized.
 */
class Servo32U4Base
{
protected:
    uint16_t usMin = 0;
    uint16_t usMax = 4000;

    uint8_t feedbackPin = -1;
    bool isAttached = false;

    uint16_t targetPos = 1500;
    uint16_t currentPos = 1500;

public:
    // Virtual functions defined for each specific class
    virtual void attach(void) = 0;
    virtual void detach(void) = 0;
    void setTargetPos(uint16_t target) {targetPos = target;}
    void update(void) 
    {
        if(targetPos == currentPos) return; // no need to update

        else if(abs(targetPos - currentPos) <= 40) currentPos = targetPos;
        else if(targetPos > currentPos) currentPos += 40;
        else if(targetPos < currentPos) currentPos -= 40;

        writeMicroseconds(currentPos);
    }
    uint16_t setMinMaxMicroseconds(uint16_t min, uint16_t max);

    virtual void writeMicroseconds(uint16_t microseconds) = 0;
};

/** \class Servo32U4Pin5
 * \brief A servo class to control a servo on pin 5.
 * 
 * Servo32U4 uses output compare on Timer3 to control the pulse to the servo. 
 * The 16-bit Timer3 is set up with a pre-scaler of 8, TOP of 39999 + 1 => 20 ms interval.
 * 
 * OCR3A controls the pulse on pin 5 -- this servo must be on pin 5! 
 * 
 * Defaults to a range of 1000 - 2000 us, but can be customized.
 */
class Servo32U4Pin5 : public Servo32U4Base
{
public:
    void attach(void);
    void detach(void);

protected:    
    void writeMicroseconds(uint16_t microseconds);
};

/** \class Servo32U4Pin6
 * \brief A servo class to control a servo on pin 6.
 * 
 * Servo32U4Pin6 uses output compare on Timer4 to control the pulse to the servo. 
 * _In Chassis::init(),
 * The 8-bit Timer4 is set up with a pre-scaler of 1024, TOP of 249 + 1 => 16 ms interval.
 * 
 * YOU MUST CALL Chasssis::init() IN setup() FOR THIS TO WORK, 
 * AND YOU MUST CALL Chassis::init() BEFORE YOU CALL attach()
 * 
 * OCR4D controls the pulse on pin 6 -- this servo must be on pin 6! 
 * 
 * Note that pin 6 controls the buzzer, so you'll go crazy if you don't cut the buzzer trace. 
 * See: https://www.pololu.com/docs/0J69/3.2 for how to cut the trace.
 * 
 * Defaults to a range of 1000 - 2000 us, but can be customized.
 * 
 * Note that because we're using an 8-bit timer, resolution is only 64 us.
 */
class Servo32U4Pin6 : public Servo32U4Base
{
public:
    void attach(void);
    void detach(void);

protected:
    void writeMicroseconds(uint16_t microseconds);
};

/** \class Servo32U4Pin13
 * \brief A servo class to control a servo on pin 13.
 * 
 * Servo32U4Pin6 uses output compare on Timer4 to control the pulse to the servo. 
 * _In Chassis::init(),
 * The 8-bit Timer4 is set up with a pre-scaler of 1024, TOP of 249 + 1 => 16 ms interval.
 * 
 * YOU MUST CALL Chasssis::init() IN setup() FOR THIS TO WORK, 
 * AND YOU MUST CALL Chassis::init() BEFORE YOU CALL attach()
 * 
 * OCR4A controls the pulse on pin 13 -- this servo must be on pin 13! 
 * 
 * Note that there is a useful LED on pin 13 -- you'll lose that functionality.
 * 
 * Pin 13 is also used during the upload process, so your servo will go crazy when uploading!
 * 
 * Defaults to a range of 1000 - 2000 us, but can be customized.
 * 
 * Note that because we're using an 8-bit timer, resolution is only 64 us.
 */
class Servo32U4Pin13 : public Servo32U4Base
{
public:
    void attach(void);
    void detach(void);

protected:
    void writeMicroseconds(uint16_t microseconds);
};

/** \class Servo32U4Pin12
 * \brief A servo class to control a servo on pin 12.
 * 
 * Servo32U4Pin12 uses output compare on Timer4 to control the pulse to the servo. 
 * _In Chassis::init(),
 * 
 * YOU MUST CALL Chasssis::init() IN setup() FOR THIS TO WORK, 
 * AND YOU MUST CALL Chassis::init() BEFORE YOU CALL attach()
 * 
 * ^OCR4D controls the pulse on pin 12 -- this servo _must_ be on pin 12!
 * 
 * DO NOT USE IN CONJUNCTION WITH Servo32U4Pin6
 * 
 * Defaults to a range of 1000 - 2000 us, but can be customized.
 * 
 * Note that because we're using an 8-bit timer, resolution is only 64 us.
 */
class Servo32U4Pin12 : public Servo32U4Base
{
public:
    void attach(void);
    void detach(void);

protected:
    void writeMicroseconds(uint16_t microseconds);
};
