#include <FreeSixIMU_Tenssy3.h>
#include <FIMU_ADXL345_Tenssy3.h>
#include <FIMU_ITG3200_Tenssy3.h>

#include <i2c_t3.h>

float angles[3]; // yaw pitch roll

// Set the FreeSixIMU_Tenssy3 object
FreeSixIMU_Tenssy3 sixDOF = FreeSixIMU_Tenssy3();

void setup() { 
  Serial.begin(250000);
  Wire.begin();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}

void loop() { 
  
  sixDOF.getEuler(angles);
  
  Serial.print(angles[0]);
  Serial.print(" | ");  
  Serial.print(angles[1]);
  Serial.print(" | ");
  Serial.println(angles[2]);
  
  delay(100); 
}

