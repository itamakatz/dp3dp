////////////////////////////////////////////////////////////
// Arduino firmware for use with FreeSixCube processing example
////////////////////////////////////////////////////////////

#include <FreeSixIMU_Tenssy3.h>
#include <FIMU_ADXL345_Tenssy3.h>
#include <FIMU_ITG3200_Tenssy3.h>

#define DEBUG
#ifdef DEBUG
#include "DebugUtils_Tenssy3.h"
#endif

// #include "CommunicationUtils_Tenssy3.h"
#include "FreeSixIMU_Tenssy3.h"
#include <i2c_t3.h>


float q[4]; //hold q values

// Set the FreeIMU object
FreeSixIMU_Tenssy3 my3IMU = FreeSixIMU_Tenssy3();

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  // Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);

  delay(5);
  my3IMU.init();
  delay(5);
}


void loop() { 
  my3IMU.getQ(q);
  serialPrintFloatArr(q, 4);
  Serial.println(""); //line break
 
  delay(60);
}

// ====================================== CommunicationUtils_Tenssy3 ====================================== //

void serialPrintFloatArr(float * arr, int length) {
  for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    Serial.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}