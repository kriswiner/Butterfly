/*
 This example reads audio data from an Invensense's ICS43434 I2S microphone
 breakout board, and prints out the samples to the Serial console. The
 Serial Plotter built into the Arduino IDE can be used to plot the audio
 data (Tools -> Serial Plotter)
 */

#include <I2S.h>

void setup()
{
  Serial.begin(230400);
//  Serial.blockOnOverrun(false);
  delay(5000);

  // Start I2S at 8 kHz with 32-bits per sample 
  if(!I2S.begin(I2S_PHILIPS_MODE, 16000, 32))
  {
    Serial.println("Failed to initialize I2S!");
    while(1);                                                                        // Do nothing
  }
/*  Serial.println("I2S initialized!");
  Serial.println("Send a '1' to start recording data");
  while(1)
  {
    if(Serial.read() == 49) {break;}
  }
  */
}

void loop()
{
  // Read the I2S data stream and write it to serial
  
  uint32_t size;  
  size = I2S.available();
  
  uint8_t data[size];
  I2S.read(data, size);
  Serial.write(data, size);
}
