/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include <Arduino.h> 

#define Q_SOFT    2   /**< Hall Switch Output Pin  */
#define Q_HARD    3   /**< Hall Switch Output Pin  */
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myStepper1 = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *myStepper2 = AFMS.getStepper(200, 1);

// wrappers for the first motor!
void forwardstep1() {  
   // Serial.println("Double coil steps"); //for a full step rotation to position where two coils are powered providing more torque
    // Serial.println("Interleave coil steps"); //for a half step rotation interleaving single and double coil positions and torque
      // Serial.println("Microstep steps"); //for a microstep rotation to a position where two coils are partially active.
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

void setup() {
  
  Serial.begin(115200);           // set up Serial library at 38400 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  pinMode(Q_SOFT, INPUT_PULLUP);
  pinMode(Q_HARD, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Q_SOFT), int_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Q_HARD), int_handler, CHANGE);

  
  pinMode(ffs1_soft, INPUT);
  pinMode(ffs1_hard, INPUT);


  stepper1.setMaxSpeed(200.0);
  stepper1.setAcceleration(200.0);
  stepper1.moveTo(-4800);
    
  stepper2.setMaxSpeed(200.0);
  stepper2.setAcceleration(200.0);
  stepper2.moveTo(1500);



  /////////////////////////////////////////TESTING OF MOVEMENTS////////////////////////////////////////////////////
 /* myStepper1->setSpeed(150);  // 150 rpm  
  myStepper1->step(300,FORWARD , DOUBLE);    //FORWARD BACKWARD
  //myStepper1->step(2000, FORWARD, DOUBLE);//CCW as you look at the face of shaft //it goes back to pillow or the finger comes to lar
  myStepper1->release();
  myStepper2->setSpeed(150);  // 150 rpm  
  myStepper2->step(300,BACKWARD , DOUBLE);    //FORWARD BACKWARD
  //myStepper1->step(2000, FORWARD, DOUBLE);//CCW as you look at the face of shaft //it goes back to pillow or the finger comes to lar
  myStepper2->release();
  */
  //////////////////////////////////////////////////END OF TESTING///////////////////////////////////////////////////////
 
 
}
void loop() {
     stepper2.run();
     stepper1.run();
     if (hall_soft==0){
      stepper1.setSpeed(0);
      myStepper1->release();
     }
     if (hall_hard==0){
      stepper2.setSpeed(0);
      myStepper2->release();
     }
     
Serial.print("hall_hard: "); 
Serial.println(hall_hard);
Serial.println("");    
Serial.print("hall_soft: "); 
Serial.println(hall_soft);
Serial.println("");
}
/*
void loop() {
stepper1.run();

  ffsdata_soft = analogRead(ffs1_soft);
  FSR_soft = (ffsdata_soft * 5.0) / 1023.0; 
  FSR_soft = FSR_soft * cf_soft ; 
  
  stepper1.run();

 /* ffsdata_hard = analogRead(ffs1_hard);
  FSR_hard = (ffsdata_hard * 5.0) / 1023.0; 
  FSR_hard = FSR_hard * cf_hard ; 
  */
  /*
  stepper1.run();
     
 if ((stepper1.distanceToGo() < 50)&& (flag==0)){
  stepper1.setSpeed(50);
  stepper1.run();
   }
   
  stepper1.run();

 if (FSR_soft>1){
  stepper1.setSpeed(0);
  myStepper1->release();
  flag=1;
  delay(6000); 
    }

    if (flag==1){
  myStepper1->setSpeed(100);
  myStepper1->step(500,BACKWARD,DOUBLE);
  flag=2;}

  while ((flag==2)&&(hall_soft==1)){
    stepper1.moveTo(1000);
    stepper1.setSpeed(150);
    stepper1.run();
  }
 
   stepper1.run();

  if (hall_soft==0){
  stepper1.setSpeed(0);
  myStepper1->release(); 
 }    
Serial.print("FSR_soft: "); 
Serial.print(FSR_soft,3); 
Serial.println("");
//Serial.print("FSR_hard: "); 
//Serial.print(FSR_hard,3); 
//Serial.println("");
Serial.print("hall_soft: "); 
Serial.println(hall_soft);
Serial.println("");
//Serial.print("hall_hard: "); 
//Serial.println(hall_hard);
//Serial.println("");

 
}
*/
