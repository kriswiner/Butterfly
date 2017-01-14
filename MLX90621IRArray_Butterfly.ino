/* MLX90621IRArray_t3
 *  
 *  Basic program to get 4 x 16 temperature array data from the MLX9062X I2C thermal sensor
 *  
 *  Intended to run on the Butterfly as host, but any host with an I2C bus should work well.
 *  
 *  adapted from https://github.com/GarageProto/GP-004-01/blob/master/MLX90621.h
 *  
 *  
 */
// This Butterfly native optimized version requires specific pins
//
#define sclk 13  // SCLK can also use pin 14
#define mosi 11  // MOSI can also use pin 7
#define cs   10  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define dc   9   //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define rst  8   // RST can use any pin
#define sdcs 4   // CS for SD card, can use any pin

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <Wire.h>

Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, rst);
float p = 3.1415926f;

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

uint16_t setColor[8] = {BLACK, BLUE, RED, GREEN, CYAN, MAGENTA, YELLOW, WHITE};

// 146 x 16-bit words in RAM

// 64, 16-bit words for the 4 x 16 bit array
#define MLX90621_DATA00  0x00 
#define MLX90621_DATA10  0x01 
#define MLX90621_DATA20  0x02 
#define MLX90621_DATA30  0x03 
#define MLX90621_DATA01  0x04 // IRAddress(x,y) = x + 4y, where 0 < x < 3, 0 < y < 15
 
#define MLX90621_DATA64  0x3F

#define MLX90621_PTAT    0x40
#define MLX90621_COMP    0x41
#define MLX90621_CONF    0x92
#define MLX90621_TRIM    0x93

#define MLX90621_ADDRESS 0x60
#define EEPROM_ADDRESS   0x50

/* 
EEPROM
A 2kbit, organized as 256x8 EEPROM is built in the MLX90621. The EEPROM has a separate I2C address SA=0x50 and is
used to store the calibration constants and the configuration of the device.
See: http://ww1.microchip.com/downloads/en/DeviceDoc/21709J.pdf
*/
uint8_t eepromData[256];

// EEPROM data values
#define KS_SCALE          0xC0
#define KS4_EE            0xC4
#define CAL_ACOMMON_L     0xD0
#define CAL_ACOMMON_H     0xD1
#define KT_SCALE          0xD2
#define CAL_ACP_L         0xD3
#define CAL_ACP_H         0xD4
#define CAL_BCP           0xD5
#define CAL_alphaCP_L     0xD6
#define CAL_alphaCP_H     0xD7
#define CAL_TGC           0xD8
#define CAL_AI_SCALE      0xD9
#define CAL_BI_SCALE      0xD9
#define VTH_L             0xDA
#define VTH_H             0xDB
#define KT1_L             0xDC
#define KT1_H             0xDD
#define KT2_L             0xDE
#define KT2_H             0xDF
//Common sensitivity coefficients
#define CAL_A0_L          0xE0
#define CAL_A0_H          0xE1
#define CAL_A0_SCALE      0xE2
#define CAL_DELTA_A_SCALE 0xE3
#define CAL_EMIS_L        0xE4
#define CAL_EMIS_H        0xE5
#define CAL_KSTA_L        0xE6
#define CAL_KSTA_H        0xE7
#define OSC_TRIM          0xF7
#define CHIP_ID           0xF8

uint16_t configuration;
float v_ir_off_comp, ksta, v_ir_tgc_comp, v_ir_comp, alpha_comp;
float tak4, resolution_comp;
int16_t a_common; 
uint16_t k_t1_scale, k_t2_scale, a_i_scale, b_i_scale;
float k_t1, k_t2, emissivity, tgc, alpha_cp, a_cp, b_cp, v_th, tmpTemp, ks_scale, ks4, sx;
uint16_t ptat;
int16_t cpix;
float a_ij, b_ij, alpha_ij;
float minTemp, maxTemp, Tambient;
float temperatures[64]; //Contains the calculated temperatures of each pixel in the array
int16_t irData[64];     //Contains the raw IR data from the sensor
uint16_t color;
uint8_t rgb, red, green, blue;

uint8_t rgb_colors[600] =
{  0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   0,   0,   0,
   5,   0,   7,
   9,   0,  14,
  14,   0,  21,
  18,   0,  29,
  23,   0,  36,
  27,   0,  43,
  32,   0,  50,
  37,   0,  57,
  41,   0,  64,
  46,   0,  72,
  50,   0,  79,
  55,   0,  86,
  60,   0,  93,
  64,   0, 100,
  69,   0, 107,
  73,   0, 115,
  78,   0, 122,
  82,   0, 129,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  87,   0, 136,
  84,   6, 132,
  82,  12, 128,
  79,  17, 124,
  77,  23, 120,
  74,  29, 116,
  72,  35, 113,
  69,  40, 109,
  67,  46, 105,
  64,  52, 101,
  62,  58,  97,
  59,  64,  93,
  56,  69,  89,
  54,  75,  85,
  51,  81,  81,
  49,  87,  77,
  46,  92,  74,
  44,  98,  70,
  41, 104,  66,
  39, 110,  62,
  36, 115,  58,
  33, 121,  54,
  31, 127,  50,
  28, 133,  46,
  26, 139,  42,
  23, 144,  38,
  21, 150,  35,
  18, 156,  31,
  16, 162,  27,
  13, 167,  23,
  11, 173,  19,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
   8, 179,  15,
  20, 183,  14,
  32, 186,  14,
  43, 190,  13,
  55, 193,  12,
  67, 197,  11,
  79, 201,  11,
  90, 204,  10,
 102, 208,   9,
 114, 212,   9,
 126, 215,   8,
 137, 219,   7,
 149, 222,   6,
 161, 226,   6,
 173, 230,   5,
 184, 233,   4,
 196, 237,   4,
 208, 241,   3,
 220, 244,   2,
 231, 248,   1,
 243, 251,   1,
 255, 255,   0,
 255, 246,   0,
 255, 237,   0,
 255, 228,   0,
 255, 219,   0,
 255, 209,   0,
 255, 200,   0,
 255, 191,   0,
 255, 182,   0,
 255, 173,   0,
 255, 164,   0,
 255, 155,   0,
 255, 146,   0,
 255, 137,   0,
 255, 128,   0,
 255, 118,   0,
 255, 109,   0,
 255, 100,   0,
 255,  91,   0,
 255,  82,   0,
 255,  73,   0,
 255,  64,   0,
 255,  55,   0,
 255,  46,   0,
 255,  36,   0,
 255,  27,   0,
 255,  18,   0,
 255,   9,   0,
 255,   0,   0,
 255,   0,   0,
 255,   0,   0,
 255,   0,   0,
 255,   0,   0,
 255,   0,   0,
 253,  13,  40,
 250,  27,  80,
 248,  40, 120,
 245,  53, 160,
 243,  67, 200,
 240,  80, 240,
 242, 102, 242,
 244, 124, 244,
 246, 146, 246,
 248, 168, 248,
 249, 189, 249,
 251, 211, 251,
 253, 233, 253,
 255, 255, 255,
 255, 255, 255,
 255, 255, 255,
 255, 255, 255,
 255, 255, 255,
 255, 255, 255
};

enum refreshRate {  // define IR Array refresh rate
  IRRrate_512Hz = 0x05,
  IRRRate_256Hz,
  IRRRate_128Hz,
  IRRRate_64Hz,
  IRRRate_32Hz,
  IRRRate_16Hz,
  IRRRate_8Hz,
  IRRRate_4Hz,
  IRRRate_2Hz,
  IRRRate_1Hz,  // default
  IRRRate_0_5Hz // 0xFF
};

enum resolution {  // define IR Array resolution
  ADC_15bit = 0,
  ADC_16bit,
  ADC_17bit,
  ADC_18bit
};

// Define configuration
  uint8_t refreshRate = IRRRate_4Hz; // set IR Array refresh rate
  uint8_t resolution =  ADC_18bit;   // set temperature resolution
  
void setup() 
{
  pinMode(sdcs, INPUT_PULLUP);  // don't touch the SD card
  Serial.begin(38400);
  delay(4000);
  Serial.print("MLX90621 4 x 16 IR Array");
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  tft.setRotation(3); // 0, 2 are portrait mode, 1,3 are landscape mode

  Serial.println("initialize display");

  uint16_t time = millis();
  tft.fillScreen(ST7735_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  // Initialize i2C bus and check for devices on the bus
  Wire.begin(); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  

  I2Cscan();

   // Initialization of the MLX90621
    readEEPROM();// Read the entire EEPROM
    Serial.print("EEPROM Chip ID = "); Serial.print(eepromData[CHIP_ID], HEX); Serial.print("  Should be "); Serial.println(0xA6, HEX);

    writeOSCTrim();
    writeConfiguration();
    configuration = readConfiguration();
    Serial.print("Resolution = "); Serial.println((configuration & 0x0030) >> 4);
    Serial.print("Rate = "); Serial.println((configuration & 0x000F) );
    calculateConstants();
}
/* End of setup*/

void loop() 
{
    if(!(readConfiguration() & 0x0400)){ // if POR bit cleared, reinitialize
      readEEPROM();
      writeOSCTrim();
      writeConfiguration();
    }
    
    readPTAT(); 
    readIR(); // get the raw data
  
    calculateTA(); // get ambient temperature
    Serial.print("Ambient temperature = "); Serial.println(Tambient);
    readCPIX(); // get pixel corrections
    calculateTO(); // compensate raw data to get 4 x 16 temperatures

    // Get min and max temperatures
    minTemp = 1000.0f;
    maxTemp =    0.0f;
    for(int y=0; y <4; y++){ //go through all the rows
    for(int x=0; x<16; x++){ //go through all the columns
      if(temperatures[y+x*4] > maxTemp) maxTemp = temperatures[y+x*4];
      if(temperatures[y+x*4] < minTemp) minTemp = temperatures[y+x*4];
    }
    }

    tft.fillScreen(ST7735_BLACK); // clear the screen
    for(int y=0; y <4; y++){ //go through all the rows
    for(int x=0; x<16; x++){ //go through all the columns
 
      tmpTemp = temperatures[y+x*4];
//      Serial.print(tmpTemp, 1); Serial.print(","); // use the serial monitor to plot the data, TFT diplay would be better
//      if(x == 15) Serial.println(" "); 
//      if(y+x*4 == 63) Serial.println(" "); 

      rgb =(uint8_t) (((tmpTemp - minTemp)/(maxTemp - minTemp)) * 199); // 0 - 199 = 200 possible rgb color values

      red   = rgb_colors[rgb*3] >> 3; // keep 5 MS bits
      green = rgb_colors[rgb*3 + 1] >> 2; // keep 6 MS bits
      blue  = rgb_colors[rgb*3 + 2] >> 3; // keep 5 MS bits
      color = red << 11 | green << 5 | blue; // construct rgb565 color for tft display

      tft.fillRect(x*10, y*10, 10, 10, color);
//      tft.drawRect(x*10, y*10, 10, 10, BLACK);  

      tft.setTextSize(0);
      tft.setTextColor(WHITE);
      tft.setCursor(10, 50 );
      tft.print("min T = "); tft.print((uint8_t) minTemp);
      tft.setCursor(10, 66 );
      tft.print("max T = "); tft.print((uint8_t) maxTemp);
 
      tmpTemp = 0;
    }          
    }

    delay(100);  // give some time to see the screen values
}


/* End of Loop*/

/*
************************************************************************************************************
* Useful Functions
************************************************************************************************************
*/
  void readEEPROM()
  {// Read the entire EEPROM
  for(int j=0; j<256; j+=32) {
    Wire.beginTransmission(EEPROM_ADDRESS);
    Wire.write(j);
    byte rc = Wire.endTransmission(false);
    Wire.requestFrom(EEPROM_ADDRESS, 32);
    for (int i = 0; i < 32; i++) {
      eepromData[j+i] = (uint8_t) Wire.read();
    }
  }
  }
  
void writeConfiguration()
{
    // Write MLX90621 configuration register
    Wire.beginTransmission(MLX90621_ADDRESS);
    Wire.write(0x03);                                           // transmit Op Code for Configuration file write
    Wire.write( ((resolution << 4) | (refreshRate))  - 0x55);   // transmit LSB check
    Wire.write( ((resolution << 4) | (refreshRate)) );          // transmit LSB data
    Wire.write(0x46 - 0x55);                                    // transmit MSB check
    Wire.write(0x46);                                           // transmit MSB, 0x46 default
    Wire.endTransmission();
}


uint16_t readConfiguration()
{
    //Read MLX90621 configuration register
    Wire.beginTransmission(MLX90621_ADDRESS);
    Wire.write(0x02);           // transmit Op Code for register read
    Wire.write(MLX90621_CONF);  // transmit address for Configuration register
    Wire.write(0x00);           // transmit address step for Configuration register
    Wire.write(0x01);           // transmit number of words to be read
    Wire.endTransmission(false);
    Wire.requestFrom(MLX90621_ADDRESS, 2);
    uint8_t dataLSB = Wire.read();  // read LSB of Configuration register
    uint8_t dataMSB = Wire.read();  // read MSB of Configuration register
    uint16_t data = ( (uint16_t) dataMSB << 8) | dataLSB;
    return data;
}


void   writeOSCTrim() {
  Wire.beginTransmission(MLX90621_ADDRESS);
  Wire.write(0x04);
  Wire.write(eepromData[OSC_TRIM] - 0xAA);
  Wire.write(eepromData[OSC_TRIM]);
  Wire.write(0x56);
  Wire.write(0x00);
  Wire.endTransmission();
}


void calculateConstants() {
  resolution_comp = pow(2.0f, (3 - resolution));
  emissivity = (float)(((uint16_t)eepromData[CAL_EMIS_H] << 8) | eepromData[CAL_EMIS_L]) / 32768.0f;
  Serial.print("emmissivity = "); Serial.println(emissivity);
  a_common = ((int16_t)eepromData[CAL_ACOMMON_H] << 8) | eepromData[CAL_ACOMMON_L];
  Serial.print("a_common = "); Serial.println(a_common);
  a_i_scale = (uint16_t)(eepromData[CAL_AI_SCALE] & 0x00F0) >> 4;
  b_i_scale = (uint16_t) eepromData[CAL_BI_SCALE] & 0x000F;
  Serial.print("a_i_scale = "); Serial.println(a_i_scale);
  Serial.print("b_i_scale = "); Serial.println(b_i_scale);
  
  alpha_cp = (float)(((uint16_t)eepromData[CAL_alphaCP_H] << 8) | eepromData[CAL_alphaCP_L]) /
         (pow(2.0, eepromData[CAL_A0_SCALE]) * resolution_comp);
  Serial.print("alpha_cp = "); Serial.println(alpha_cp);
         
  a_cp = (float) ((int16_t)(((int16_t)eepromData[CAL_ACP_H] << 8) | eepromData[CAL_ACP_L])) / resolution_comp;
  Serial.print("a_cp = "); Serial.println(a_cp);
  b_cp = (float) (((int16_t)eepromData[CAL_BCP] << 8) >> 8) / (pow(2.0f, (float)b_i_scale) * resolution_comp);
  Serial.print("b_cp = "); Serial.println(b_cp);
  tgc  = (float) (((int16_t)eepromData[CAL_TGC] << 8) >> 8) / 32.0f;
  Serial.print("tgc = "); Serial.println(tgc);
  
  ks_scale = (uint8_t) eepromData[KS_SCALE];
  ks4  = (float) (((int16_t)eepromData[KS4_EE] << 8) >> 8) / pow(2, ks_scale + 8);
  Serial.print("ks4 = "); Serial.println(ks4);
  k_t1_scale = (uint16_t) (eepromData[KT_SCALE] & 0x00F0) >> 4;
  Serial.print("k_t1_scale = "); Serial.println(k_t1_scale);
  k_t2_scale = (uint16_t) (eepromData[KT_SCALE] & 0x000F) + 10;
  Serial.print("k_t2_scale = "); Serial.println(k_t2_scale);
  v_th = (float) (((int16_t)eepromData[VTH_H] << 8) | eepromData[VTH_L]);
  v_th = v_th / resolution_comp;
  Serial.print("v_th = "); Serial.println(v_th);
  k_t1 = (float) (((int16_t)eepromData[KT1_H] << 8) | eepromData[KT1_L]);
  k_t1 /= (pow(2.0f, k_t1_scale) * resolution_comp);
  k_t2 = (float) (((int16_t)eepromData[KT2_H]  << 8) | eepromData[KT2_L]);
  k_t2 /= (pow(2.0f, k_t2_scale) * resolution_comp);
  Serial.print("k_t1 = "); Serial.println(k_t1);
  Serial.print("k_t2 = "); Serial.println(k_t2);
}

void readPTAT() 
{
  Wire.beginTransmission(MLX90621_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x40);
  Wire.write(0x00);
  Wire.write(0x01);
  Wire.endTransmission(false);
  Wire.requestFrom(MLX90621_ADDRESS, 2);
  uint8_t ptatLSB  = Wire.read();
  uint8_t ptatMSB = Wire.read();
  ptat = ((uint16_t)ptatMSB << 8) | ptatLSB;
}

void readIR() {
  for (int j = 0; j < 64; j += 16) { // Read in blocks of 32 bytes to overcome Wire buffer limit
    Wire.beginTransmission(MLX90621_ADDRESS);
    Wire.write(0x02);
    Wire.write(j);
    Wire.write(0x01);
    Wire.write(0x20);
    Wire.endTransmission(false);
    Wire.requestFrom(MLX90621_ADDRESS, 32);
    for (int i = 0; i < 16; i++) {
      uint8_t pixelDataLSB = Wire.read();
      uint8_t pixelDataMSB = Wire.read();
      irData[j + i] = ((int16_t)pixelDataMSB << 8) | pixelDataLSB;
    }
  }
}

float getAmbient() {
  return Tambient;
}

void readCPIX() {
  Wire.beginTransmission(MLX90621_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x41);
  Wire.write(0x00);
  Wire.write(0x01);
  Wire.endTransmission(false);
  Wire.requestFrom(MLX90621_ADDRESS, 2);
  byte cpixLow = Wire.read();
  byte cpixHigh = Wire.read();
  cpix = ((int16_t)cpixHigh << 8) | cpixLow;
}

void calculateTA(void) {
  Tambient = ((-k_t1 + sqrt(sq(k_t1) - (4 * k_t2 * (v_th - (float) ptat))))
      / (2 * k_t2)) + 25.0;
}

void calculateTO() {
  float v_cp_off_comp = (float) cpix - (a_cp + b_cp * (Tambient - 25.0f));
  tak4 = pow((float) Tambient + 273.15f, 4.0f);
  minTemp = NULL, maxTemp = NULL;
  for (int i = 0; i < 64; i++) {
    a_ij = ((float) a_common + eepromData[i] * pow(2.0, a_i_scale)) / resolution_comp;
    b_ij = (float) (((int16_t)eepromData[0x40 + i] << 8) >> 8) / (pow(2.0, b_i_scale) * resolution_comp);
    v_ir_off_comp = (float) irData[i] - (a_ij + b_ij * (Tambient - 25.0));
    v_ir_tgc_comp = (float) v_ir_off_comp - tgc * v_cp_off_comp;
    float alpha_ij = ((float) (((uint16_t)eepromData[CAL_A0_H] << 8) | eepromData[CAL_A0_L]) / pow(2.0f, (float) eepromData[CAL_A0_SCALE]));
    alpha_ij += ((float) eepromData[0x80 + i] / pow(2.0f, (float) eepromData[CAL_DELTA_A_SCALE]));
    alpha_ij = alpha_ij / resolution_comp;
    ksta = (float) (((int16_t)eepromData[CAL_KSTA_H] <<8) | eepromData[CAL_KSTA_L]) / pow(2.0, 20.0);
    alpha_comp = (1.0f + ksta * (Tambient - 25.0f)) * (alpha_ij - tgc * alpha_cp);
    v_ir_comp = v_ir_tgc_comp / emissivity;

    sx = ks4*pow(pow(alpha_comp, 3)*v_ir_comp+pow(alpha_comp, 4)*tak4, 0.25f);    
    float temperature = pow((v_ir_comp / (alpha_comp*(1 - ks4*273.15f) + sx)) + tak4, 0.25f) - 273.15f;

    temperatures[i] = temperature;
    if (minTemp == NULL || temperature < minTemp) {
      minTemp = temperature;
    }
    if (maxTemp == NULL || temperature > maxTemp) {
      maxTemp = temperature;
    }
  }
}


// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    error = Wire.transfer(address, NULL, 0, NULL, 0);

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


// I2C read/write functions for the BMP280 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
        uint8_t temp[2];
        temp[0] = subAddress;
        temp[1] = data;
        Wire.transfer(address, &temp[0], 2, NULL, 0); 
        }

        uint8_t readByte(uint8_t address, uint8_t subAddress) {
        uint8_t temp[1];
        Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        return temp[0];
        }

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
        Wire.transfer(address, &subAddress, 1, dest, count); 
        }

// I2C read/write functions for the MLX90621 and EEPROM

  void writeWord(uint8_t address, uint8_t subAddress, uint16_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write((data & 0xFF00) << 8); // Put data MSB in Tx buffer
  Wire.write(data & 0x00FF);        // Put data LSB in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

  uint16_t readWord(uint8_t address, uint8_t subAddress)
{
  uint16_t data; // `data` will store the register data   
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (size_t) 1);   // Read one word from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data; 
}
