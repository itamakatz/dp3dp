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

#include "CommunicationUtils_Tenssy3.h"
#include "FreeSixIMU_Tenssy3.h"
#include <i2c_t3.h>


float q[4]; //hold q values

// Set the FreeIMU object
FreeSixIMU_Tenssy3 my3IMU = FreeSixIMU_Tenssy3();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

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
