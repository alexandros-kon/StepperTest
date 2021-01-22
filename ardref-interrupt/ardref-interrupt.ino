/**
 * @file        ardref-interrupt.ino
 * @brief       GPIO Interrupt Example for TLE4964-3M Shield2Go
 *          
 *              Usage of the Arduino Reference Library Functions.
 *              - How does it work? 
 *              The transition from magnetic field (B) presence to absence will 
 *              generate a Q output signal level change: either high to low 
 *      	    (for no B field to B field), or low to high (for B field to
 *              no B field). The interrupt occurs for any of both events.
 *              The interrupt handler function determines which of the events
 *              have ocurred, and print the event through the serial.
 *              In the presence of magnetic field, the LED1 is turned ON.
 * @copyright   Copyright (c) 2019 Infineon Technologies AG         
 */

#include <Arduino.h> 

#define Q_OUTPUT_PIN    2   /**< Hall Switch Output Pin  */
int     value = HIGH;       /**< Output Value */

/**
 * @brief   hall-switch interrupt handler
 *          
 */
void int_handler()
{
	value = digitalRead(Q_OUTPUT_PIN);   
}

void setup()
{  
    Serial.begin(9600);        
    pinMode(Q_OUTPUT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Q_OUTPUT_PIN), int_handler, CHANGE);
}

void loop()
{
  if(value == HIGH)
    {
    Serial.println("B Field OFF");}
    else {
    Serial.println("B Field ON");
    }
    Serial.println(value);

delay(500);
}
