/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <AccelStepper.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *myMotor2 = AFMS.getStepper(200, 1);



void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  myMotor->setSpeed(10);  // 10 rpm   
  myMotor2->setSpeed(10);  // 10 rpm   

}

void loop() {
  Serial.println("Single coil steps"); //for a full step rotation to a position where one single coil is powered
  myMotor->step(100, FORWARD, SINGLE); //CCW as you look at the face of shaft
  myMotor2->step(100, FORWARD, SINGLE);  
//  myMotor->step(100, BACKWARD, SINGLE);
//  myMotor2->step(100, BACKWARD, SINGLE);  

  Serial.println("Double coil steps"); //for a full step rotation to position where two coils are powered providing more torque
  myMotor->step(100, FORWARD, DOUBLE); 
  myMotor2->step(100, FORWARD, DOUBLE); 
//  myMotor->step(100, BACKWARD, DOUBLE);
//  myMotor2->step(100, BACKWARD, DOUBLE);
  
  Serial.println("Interleave coil steps"); //for a half step rotation interleaving single and double coil positions and torque
  myMotor->step(100, FORWARD, INTERLEAVE);
  myMotor2->step(100, FORWARD, INTERLEAVE); 
//  myMotor2->step(100, BACKWARD, INTERLEAVE);  
//  myMotor->step(100, BACKWARD, INTERLEAVE); 
  
  Serial.println("Microstep steps"); //for a microstep rotation to a position where two coils are partially active.
  myMotor->step(50, FORWARD, MICROSTEP); 
  myMotor2->step(50, FORWARD, MICROSTEP); 
//  myMotor2->step(50, BACKWARD, MICROSTEP);
//  myMotor->step(50, BACKWARD, MICROSTEP);
}
