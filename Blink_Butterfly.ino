/* LED Blink, for Butterfly
 
   This example code is in the public domain.
*/
#include <Arduino.h>
#include <Wire.h>

// Butterfly
#define myLed1 38 // green led
#define myLed2 13 // red led
#define myLed3 26 // blue led  

float VDDA, Temperature;

void setup() 
{
  Serial.begin(38400);
 // while (!SerialUSB) { }
  delay(2000);
  Serial.println("Serial enabled!");
 
  pinMode(myLed1, OUTPUT);
  digitalWrite(myLed1, HIGH);  // start with leds off, since active LOW
  pinMode(myLed2, OUTPUT);
  digitalWrite(myLed2, HIGH);
  pinMode(myLed3, OUTPUT);
  digitalWrite(myLed3, HIGH);
}

void loop() 
{
  digitalWrite(myLed1, !digitalRead(myLed1)); // toggle red led on
  delay(100);                                 // wait 1 second
  digitalWrite(myLed1, !digitalRead(myLed1)); // toggle red led off
  delay(1000);
  digitalWrite(myLed2, !digitalRead(myLed2));
  delay(100);
  digitalWrite(myLed2, !digitalRead(myLed2));
  delay(1000);
  digitalWrite(myLed3, !digitalRead(myLed3));
  delay(100);
  digitalWrite(myLed3, !digitalRead(myLed3));
  delay(1000);

  VDDA = STM32.getVREF();
  Temperature = STM32.getTemperature();

  Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
}
