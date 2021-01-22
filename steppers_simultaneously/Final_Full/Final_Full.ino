#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include <Arduino.h> 

#define Q_SOFT    2   /**< Hall Switch Output Pin  */
#define Q_HARD    3   /**< Hall Switch Output Pin  */

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Connect two steppers with 200 steps per revolution (1.8 degree)
// to the top shield
Adafruit_StepperMotor *myStepper1 = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *myStepper2 = AFMS.getStepper(200, 1);

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
  myStepper1->onestep(FORWARD, INTERLEAVE);
}
void backwardstep1() {  
  myStepper1->onestep(BACKWARD, INTERLEAVE);
}
// wrappers for the second motor!
void forwardstep2() {  
  myStepper2->onestep(FORWARD, INTERLEAVE);
}
void backwardstep2() {  
  myStepper2->onestep(BACKWARD, INTERLEAVE);
}
// Now we'll wrap the 2 steppers in an AccelStepper object

AccelStepper stepper1(forwardstep1, backwardstep1);
AccelStepper stepper2(forwardstep2, backwardstep2);

float cf_soft = 72.15; // caliberation factor (19.5 was the initial)
float cf_hard = 19.5; // caliberation factor (19.5 was the initial)

int     hall_soft = HIGH;       /**< Output hall */
int     hall_hard = HIGH;       /**< Output hall */

int ffs1_soft = A0; // FlexiForce sensor is connected analog pin A0 of arduino or mega. 
int ffs1_hard = A1; // FlexiForce sensor is connected analog pin A1 of arduino or mega. 

int ffsdata_soft = 0; 
int ffsdata_hard = 0; 

float FSR_soft; 
float FSR_hard; 

float flag=0;

void int_handler()

{
  hall_soft = digitalRead(Q_SOFT);   
  hall_hard = digitalRead(Q_HARD);   

}

void setup()
{
  AFMS.begin();
  
  Serial.begin(115200);   
        
  pinMode(Q_SOFT, INPUT_PULLUP);
  pinMode(Q_HARD, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Q_SOFT), int_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Q_HARD), int_handler, CHANGE);

  
  pinMode(ffs1_soft, INPUT);
  pinMode(ffs1_hard, INPUT);


  stepper1.setMaxSpeed(150.0);
  stepper1.setAcceleration(150.0);
  stepper1.moveTo(10000);
    
  stepper2.setMaxSpeed(150.0);
  stepper2.setAcceleration(150.0);
  stepper2.moveTo(-1500);}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()

{
  stepper1.run();
  ffsdata_soft = analogRead(ffs1_soft);
  FSR_soft = (ffsdata_soft * 5.0) / 1023.0; 
  FSR_soft = FSR_soft * cf_soft ; 
  ffsdata_hard = analogRead(ffs1_hard);
  FSR_hard = (ffsdata_hard * 5.0) / 1023.0; 
  FSR_hard = FSR_hard * cf_hard ; 
      
 if ((stepper1.distanceToGo() < 2000)&& (flag==0)){
    stepper2.run();
    stepper1.run();
    }

 if (FSR_soft>1){
  stepper1.setSpeed(0);
  myStepper1->release();
  flag=1;
  stepper2.moveTo(-3500);
  stepper2.run();
    if (FSR_hard>4){
      stepper2.setSpeed(0);
      myStepper2->release();
      flag=2;}
 }

 /* while (flag==2){
    if (hall_hard==1){
      stepper2.moveTo(1320);
    stepper2.setSpeed(100);
    stepper2.run();}
    else{
    stepper2.setSpeed(0);
    myStepper2->release();
    }
      if (hall_soft==1){
    stepper1.moveTo(-4400);
    stepper1.setSpeed(-100);
    stepper1.run();}
    else{
     stepper1.setSpeed(0);
  myStepper1->release();  
  flag=3;
    }}
    */
        
Serial.print("FSR_soft: "); 
Serial.print(FSR_soft,3); 
Serial.println("");
Serial.print("FSR_hard: "); 
Serial.print(FSR_hard,3); 
Serial.println("");
/*Serial.print("hall_soft: "); 
Serial.println(hall_soft);
Serial.println("");
Serial.print("hall_hard: "); 
Serial.println(hall_hard);
Serial.println("");
*/
}
