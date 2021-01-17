#include "Stepper_Motor.h"

#define N_STEPS 8
#define N_STEPPER_PINS 4

#define MAX_STEPPERS 3
#define TIMER_MATCH_VALUE 0B01111111

#define FULL_REVOLUTION_D 360
#define DEFAULT_DELAY 13e2

const uint8_t SEQUENCE[N_STEPS] =
{
    0B00001110,
    0B00001100,
    0B00001101,
    0B00001001,
    0B00001011,
    0B00000011,
    0B00000111,
    0B00000110
};

static StepperMotor* callback_steppers[MAX_STEPPERS];
static int last_stepper = 0;

StepperMotor::StepperMotor() {
    
	powerSaverMode(false);
	interruptable(true);
    setSpeed(DEFAULT_DELAY);
    setStartPosition();

    forwardSequence = false;
    backwardSequence = false;
    sequenceTimer = 0;
    forwardSequenceIterator = 0;
    backwardSequenceIterator = N_STEPS - 1;

    nSteps = 0;
}

StepperMotor::~StepperMotor() {

    removeStepper();
    delete[] stepperPins;
}

void StepperMotor::attach(float stepsPerRevolution, uint8_t pinIN1, uint8_t pinIN2, uint8_t pinIN3, uint8_t pinIN4) {

    this->stepsPerRevolution = stepsPerRevolution;
    
    stepperPins = new uint8_t[N_STEPPER_PINS];

    stepperPins[0] = pinIN1;
    stepperPins[1] = pinIN2;
    stepperPins[2] = pinIN3;
    stepperPins[3] = pinIN4;

    degreePerStep = FULL_REVOLUTION_D / stepsPerRevolution;

    for (int i = 0; i < N_STEPPER_PINS; i++) pinMode(stepperPins[i], OUTPUT);
    
    addStepper();
    enableTimer();
}

void StepperMotor::powerSaverMode(bool powerSaver) {
    this->powerSaver = powerSaver;
}

void StepperMotor::interruptable(bool interruptMode) {
    this->interruptMode = interruptMode;
}

void StepperMotor::setPosition(int degrees) {

    degrees = (FULL_REVOLUTION_D + degrees) % FULL_REVOLUTION_D;
    int stepsToTake = (degrees - currPosition) / FULL_REVOLUTION_D * stepsPerRevolution;
    
    step(stepsToTake);
}

void StepperMotor::setStartPosition() {
    currPosition = 0;
}

float StepperMotor::getPosition() {
    return currPosition;
}

void StepperMotor::setSpeed(long delayUSeconds) {
    this->delayUSeconds = delayUSeconds;
}

void StepperMotor::step(long nSteps) {
    if(interruptMode || this->nSteps == 0) this->nSteps = nSteps;
}

void StepperMotor::step() {

    if (powerSaver && nSteps == 0) for (int i = 0; i < N_STEPPER_PINS; i++) digitalWrite(stepperPins[i], 0);
    else if (forwardSequence || nSteps > 0) {

        forwardSequence = true;
        if(!stepForward()) return;        
        forwardSequence = false;        

    } else if (backwardSequence || nSteps < 0) {

        backwardSequence = true;
        if(!stepBackward()) return;
        backwardSequence = false;
    }

    if(currPosition > FULL_REVOLUTION_D) currPosition -= FULL_REVOLUTION_D;
    if(currPosition < 0) currPosition += FULL_REVOLUTION_D;
}

bool StepperMotor::isRotating() {
    return nSteps != 0;
}

void StepperMotor::stop() {
    nSteps = 0;
}

bool StepperMotor::stepForward() {
   
    while(forwardSequenceIterator < N_STEPS) {

        for (int j = 0; j < N_STEPPER_PINS; j++) digitalWrite(stepperPins[j], SEQUENCE[forwardSequenceIterator] & _BV(j));
        
        if(micros() - sequenceTimer < delayUSeconds) return false;
        sequenceTimer = micros();

        forwardSequenceIterator++;
    }
    forwardSequenceIterator = 0;

    nSteps--;
    currPosition += degreePerStep;
    return true;
}

bool StepperMotor::stepBackward() {

    while(backwardSequenceIterator >= 0) {

        for (int j = 0; j < N_STEPPER_PINS; j++) digitalWrite(stepperPins[j], SEQUENCE[backwardSequenceIterator] & _BV(j));
        
        if(micros() - sequenceTimer < delayUSeconds) return false;
        sequenceTimer = micros();

        backwardSequenceIterator--;
    }
    backwardSequenceIterator = N_STEPS - 1;
    
    nSteps++;
    currPosition -= degreePerStep;
    return true;
}

void StepperMotor::addStepper() {

    if(last_stepper == MAX_STEPPERS) return;
    for (int i = 0; i < last_stepper; i++) if(callback_steppers[i] == this) return;

    callback_steppers[last_stepper++] = this;
}

void StepperMotor::removeStepper() {

    int index = -1;
    for (int i = 0; i < last_stepper; i++) {
        
        if(callback_steppers[i] == this) {

            index = i;
            break;
        }
    }

    if(index != -1)  {

        for(int i = index + 1; i < last_stepper; i++) callback_steppers[i - 1] = callback_steppers[i]; 
        last_stepper--;
    }
}

void StepperMotor::enableTimer() {

	if(TCCR2B & 0B00000111 == 0 || TCCR2B & 0B00000111 > 0B00000011) {
		
		TCCR2B &= 0B11111000;
		TCCR2B |= _BV(CS21) | _BV(CS20);
	}

    OCR2B = TIMER_MATCH_VALUE;

    TIMSK2 |= _BV(OCIE2B);
}

ISR(TIMER2_COMPB_vect) {
    for(int i = 0; i < last_stepper; i++) callback_steppers[i]->step();
}