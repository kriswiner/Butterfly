/* Asset tracker example, for Butterfly
 *  Uses the GNSS add-on with CAM M8Q found here:
 *  
 * https://www.tindie.com/products/TleraCorp/gps-add-on-for-dragonfly-and-butterfly/
 * 
 * Idea is ultra-low power for longest LiPo battery life so I would run this with 
 * 1 - 4 MHz clock speed; this reduction plus use of STM32 stop mode means no serial 
 * through the USB. That's why there is a low power Sharp TFT display here.
 *
 *  This example code is in the public domain.
*/
#include <Arduino.h>
#include <Wire.h>
#include "GNSS.h"
#include <RTC.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

// define pins for Sharp LCD display, any pins can be used
uint8_t DSCK  = 12;
uint8_t DMOSI = 11;
uint8_t DSS   = 10;

Adafruit_SharpMem display(DSCK, DMOSI, DSS);

#define BLACK 0
#define WHITE 1

// Butterfly
#define myLed1 26 // blue led 
#define pps 8

uint16_t Hour = 0, Minute = 0, Second = 0, Millisec, Year = 0, Month = 0, Day = 0, Alt = 0;
uint16_t hour = 0, minute = 0, second = 0, year = 0, month = 0, day = 0, millisec;
bool ppsFlag = false, firstSync = false, alarmFlag = false;
uint8_t count = 0, fixType = 0, fixQuality;

float VDDA, Temperature, Long, Lat;

void setup() 
{
    Serial.begin(38400);
    delay(2000);
    Serial.println("Serial enabled!");
 
    pinMode(myLed1, OUTPUT);
    digitalWrite(myLed1, HIGH);  // start with blue led off, since active HIGH

    // Set up for data display
    display.begin(); // Initialize the display
    display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
    display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
    display.clearDisplay();   // clears the screen and buffer

    // Start device display with ID of sensor
    display.setCursor(0, 10); display.print("Butterfly");
    display.setCursor(0, 20); display.print("CAM M8Q");
    display.setCursor(0,40); display.print("Concurrent");
    display.setCursor(0,60); display.print("GNSS");
    display.refresh();
    delay(1000);

    pinMode(pps, INPUT);
    
 //   while (!Serial) { }
 
    GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ); // Start GNSS
    while (!GNSS.done()) { } // wait for begin to complete
 
    GNSS.setConstellation(GNSS.CONSTELLATION_GPS); // choose satellites
    while (!GNSS.done()) { } // wait for set to complete

    GNSS.setSBAS(true); // choose satellites
    while (!GNSS.done()) { } // wait for set to complete

    GNSS.setPeriodic(10, 600, true);  // set periodic wake and sleep mode
    while (!GNSS.done()) { } // wait for set to complete

    // Set the RTC time
    RTC.setHours(hour);
    RTC.setMinutes(minute);
    RTC.setSeconds(second);
    RTC.setMinutes(minute);

    // Set the RTC date
    RTC.setDay(day);
    RTC.setMonth(month);
    RTC.setYear(year);
    
    int16_t calib = -255;
    RTC.setCalibration(calib);  // clock slow, add pulses, clock fast, subtract pulses
 
   // Check calibration
    int16_t calreg = RTC.getCalibration();
    Serial.print("Calibration pulses = "); Serial.println(calreg);

    // set alarm to update the RTC every second
    RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
    RTC.attachInterrupt(alarmMatch);

    attachInterrupt(pps, myHandler, RISING);
}

void loop() 
{
  VDDA = STM32.getVREF();
  Temperature = STM32.getTemperature();

  Serial.print("VDDA = "); Serial.println(VDDA, 2); 
  Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
     
   /*GNSS*/
   //    if (GNSS.available()) // check if new GNSS data is available
    if(ppsFlag == true) 
    ppsFlag = false;
//    while(!GNSS.available()) { STM32.sleep();}
    delay(100); // delay a bit to allow GNSS data to become available

    {
   GNSSLocation myLocation = GNSS.read(); // read available GNSS data

   if (myLocation) // if there is a fix
   {
      Lat = myLocation.latitude();
      Long = myLocation.longitude();
      Alt = myLocation.altitude();
      Serial.print("latitude = ");
      Serial.print(Lat, 7);
      Serial.print(", longitude = ");
      Serial.print(Long, 7);
      Serial.print(", altitude = ");
      Serial.print(Alt, 1);
      Serial.print(", satellites = ");
      Serial.print(myLocation.satellites());
      Serial.print(", pdop = ");
      Serial.print(myLocation.pdop());
      Serial.print(", fixType = ");
      fixType = myLocation.fixType();
      if(fixType == 0) Serial.print("none");
      if(fixType == 1) Serial.print("time");
      if(fixType == 2) Serial.print("2D");
      if(fixType == 3) Serial.print("3D");
      Serial.print(", fixQuality = ");
      fixQuality = myLocation.fixQuality();
      if(fixQuality == 0) Serial.print("none");
      if(fixQuality == 1) Serial.print("auto");
      if(fixQuality == 2) Serial.print("diff");
      if(fixQuality == 3) Serial.print("prec");
      if(fixQuality == 4) Serial.print("rtk_fixed");
      if(fixQuality == 5) Serial.print("rtk_float");
      if(fixQuality == 6) Serial.print("est");
      if(fixQuality == 7) Serial.print("man");
      if(fixQuality == 8) Serial.print("sim");
      Serial.println();

      Hour   = myLocation.hour();
      Minute = myLocation.minute();
      Second = myLocation.second();
      Millisec = myLocation.millis();
      Serial.print("GNSS Time = ");
      if(Hour < 10)   {Serial.print("0"); Serial.print(Hour);} else Serial.print(Hour);
      Serial.print(":");
      if(Minute < 10) {Serial.print("0"); Serial.print(Minute);} else Serial.print(Minute);
      Serial.print(":");
      if(Second < 10) {Serial.print("0"); Serial.print(Second);} else Serial.print(Second);     
      Serial.print(":");
      if(Millisec < 10) {Serial.print("0"); Serial.println(Millisec);} else Serial.println(Millisec);

      Year = myLocation.year();
      Month = myLocation.month();
      Day = myLocation.day();
      Serial.print("GNSS Date = ");
      Serial.print(Year);Serial.print(":");Serial.print(Month);Serial.print(":"); Serial.println(Day);
      Serial.println();

      // Test if the RTC has been synced after GNSS time available
      if(firstSync == false) 
      {
        firstSync = true;
        syncRTC();  // just need to sync once
      }
      
  }
    }
      /*RTC*/
      if(alarmFlag) { // update RTC output whenever there is a GNSS pulse
         alarmFlag = false;
   
      hour   = RTC.getHours();
      minute = RTC.getMinutes();
      second = RTC.getSeconds();
      millisec = RTC.getTicks();
      Serial.print("RTC Time = ");
      if(hour < 10)   {Serial.print("0"); Serial.print(hour);} else Serial.print(hour);
      Serial.print(":");
      if(minute < 10) {Serial.print("0"); Serial.print(minute);} else Serial.print(minute);
      Serial.print(":");
      if(second < 10) {Serial.print("0"); Serial.print(second);} else Serial.print(second);
      Serial.print(":");
      if(millisec < 10) {Serial.print("0"); Serial.println(millisec);} else Serial.println(millisec);

      year = RTC.getYear();
      month = RTC.getMonth();
      day = RTC.getDay();
      Serial.print("RTC Date = ");
      Serial.print(year);Serial.print(":");Serial.print(month);Serial.print(":"); Serial.println(day);
      Serial.println();
      }

      // set up TFT display data
      display.clearDisplay();
      display.setCursor(0, 10); display.print("lat "); display.print(Lat, 5);
      display.setCursor(0, 20); display.print("lon "); display.print(Long, 5);
      display.setCursor(0, 30); display.print("alt "); display.print(Alt, 1); display.print(" m"); 
      display.print(" fix "); display.print(fixType);
      display.setCursor(0, 40); display.print("GPS ");
      if(Hour < 10)   {display.print("0"); display.print(Hour);} else display.print(Hour);
      display.print(":");
      if(Minute < 10) {display.print("0"); display.print(Minute);} else display.print(Minute);
      display.print(":");
      if(Second < 10) {display.print("0"); display.print(Second);} else display.print(Second);     
      display.setCursor(0, 50); 
      display.print(Year); display.print(":"); display.print(Month); display.print(":"); display.print(Day);
      display.print(" q "); display.print(fixQuality);
      display.setCursor(0, 60); display.print("RTC ");
      if(hour < 10)   {display.print("0"); display.print(hour);} else display.print(hour);
      display.print(":");
      if(minute < 10) {display.print("0"); display.print(minute);} else display.print(minute);
      display.print(":");
      if(second < 10) {display.print("0"); display.print(second);} else display.print(second);  
      display.setCursor(0, 70); display.print("VDDA = "); display.print(VDDA, 2); display.print(" V");
      display.setCursor(0, 80); display.print("T = "); display.print(Temperature, 2); display.print(" C");

      display.refresh();
      digitalWrite(myLed1, LOW); delay(1); digitalWrite(myLed1, HIGH); 
      STM32.stop();
}
 /* end of loop*/

/* Useful functions */
 void myHandler()
 {
  ppsFlag = true;
 }

 void alarmMatch()
{
  alarmFlag = true;
}
 
void syncRTC()
{
  // Set the time
  RTC.setSeconds(Second);
  RTC.setMinutes(Minute);
  if(Hour < 7) {RTC.setHours(Hour + 17);} else RTC.setHours(Hour - 7);
  RTC.setMinutes(Minute);
  
  // Set the date
  if(Hour < 7) {RTC.setDay(Day - 1);} else RTC.setDay(Day);
  RTC.setMonth(Month);
  RTC.setYear(Year - 2000);
}
