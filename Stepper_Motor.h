#ifndef _Stepper_Motor_h
#define _Stepper_Motor_h

#include "Arduino.h"

class StepperMotor {
public:

    // DISCLAIMER: This library overrides the compare match B interrupt of Timer2.

    /** 
     * Constructor:
     * Creates a StepperMotor object that can be associated with a stepper motor (after specifying the control
     * pins by calling [StepperMotor].attach([StepsPerRevolution], [ControlPin1], ..., [ContolPin4]) ).
     *  
     * !! OTHER METHODS WON'T WORK IF THE STEPPER ISN'T ATTACHED !!
     */
    StepperMotor();
    ~StepperMotor();

    /**
     * attach():
     * ------------------------------------------------------------------------------------------------
     * Associates this StepperMotor object with a stepper connected to the specified pins.
     */
    void attach(float stepsPerRevolution, uint8_t pinIN1, uint8_t pinIN2, uint8_t pinIN3, uint8_t pinIN4);

    /**
     * powerSaverMode():
     * ------------------------------------------------------------------------------------------------
     * If set to true, the stepper motor will not consume power while idle, i.e. the coils in the motor will not be energised. 
     * Of course, this means that the motor will not hold it's position if force is applied to it.
     */
    void powerSaverMode(bool powerSaverMode);

    /**
     * interruptable():
     * ------------------------------------------------------------------------------------------------
     * If set to true, the stepper motor's operation will be interruptable. 
     * 
     * example: you can tell the motor to rotate to 90 degrees, and before it has finished the operation, you can tell 
     * it to rotate to 0. The motor will stop executing the previous command to start rotating towards 0. The downside
     * is that the motor is more prone to skipping steps this way and may become less accurate.
     */
    void interruptable(bool interruptMode);

    /**
     * setPosition():
     * ------------------------------------------------------------------------------------------------
     * Rotates the stepper to the specified degree (In regards to the initial position).
     */
    void setPosition(int degrees);

    /**
     * setStartPosition():
     * ------------------------------------------------------------------------------------------------
     * Sets the current position of the stepper motor as the initial position.
     */
    void setStartPosition();

    /**
     * setStartPosition():
     * ------------------------------------------------------------------------------------------------
     * Returns the current position of the stepper motor (In regards to the initial position).
     */
    float getPosition();

    /**
     * setSpeed():
     * ------------------------------------------------------------------------------------------------
     * Sets the speed of rotation of the stepper. The speed is set by specifying the delay between steps in 
     * the coil energizing sequence. In general the smaller the number the faster the motor, but less accurate.
     */
    void setSpeed(long delayUSeconds);

    /**
     * step():
     * ------------------------------------------------------------------------------------------------
     * Executes the specified amount of steps. Negative steps can be passed to rotate the stepper in reverse.
     */
    void step(long nSteps);
    void step(); // This method is for the timer interrupt, you do not need to use it. 

    /**
     * isRotating():
     * ------------------------------------------------------------------------------------------------
     * Returns true if the stepper is rotating in either direction.
     */
    bool isRotating();

	/**
     * stop():
     * ------------------------------------------------------------------------------------------------
     * Immediately stops the stepper motor's rotation, regardless of it being interruptable/non-interruptable.
     */
	void stop();

private:

    float stepsPerRevolution, delayUSeconds;
    volatile long nSteps;

    double degreePerStep;
    volatile double currPosition;
    uint8_t *stepperPins;

    bool powerSaver, interruptMode;

    volatile bool forwardSequence, backwardSequence;
    volatile long sequenceTimer;
    volatile int forwardSequenceIterator, backwardSequenceIterator;

    void addStepper();
    void removeStepper();
    void enableTimer();

    bool stepForward();
    bool stepBackward();
};

#endif