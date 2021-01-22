#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the top shield
Adafruit_StepperMotor *myStepper1 = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *myStepper2 = AFMS.getStepper(200, 1);

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
  myStepper1->onestep(FORWARD, DOUBLE);
}
void backwardstep1() {  
  myStepper1->onestep(BACKWARD, DOUBLE);
}
// wrappers for the second motor!
void forwardstep2() {  
  myStepper2->onestep(FORWARD, DOUBLE);
}
void backwardstep2() {  
  myStepper2->onestep(BACKWARD, DOUBLE);
}
// Now we'll wrap the 3 steppers in an AccelStepper object

AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);


void setup()
// put your setup code here, to run once:
{  
AFMS.begin();

  stepper1.setMaxSpeed(100.0);
  stepper1.setAcceleration(100.0);
  stepper1.moveTo(2400);
    
  stepper2.setMaxSpeed(100.0);
  stepper2.setAcceleration(100.0);
  stepper2.moveTo(2400);
}
void loop()
  // put your main code here, to run repeatedly:
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
  stepper1.moveTo(-stepper1.currentPosition());

    if (stepper2.distanceToGo() == 0)
  stepper2.moveTo(-stepper2.currentPosition());

    stepper1.run();
    stepper2.run();
}
