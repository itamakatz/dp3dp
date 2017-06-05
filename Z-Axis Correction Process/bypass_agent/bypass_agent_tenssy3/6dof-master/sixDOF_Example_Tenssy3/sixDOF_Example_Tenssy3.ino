#include <FreeSixIMU_Tenssy3.h>
#include <FIMU_ADXL345_Tenssy3.h>
#include <FIMU_ITG3200_Tenssy3.h>

#include <i2c_t3.h>

float angles[3]; // yaw pitch roll

// Set the FreeSixIMU_Tenssy3 object
FreeSixIMU_Tenssy3 sixDOF = FreeSixIMU_Tenssy3();

void setup() { 
	Serial.begin(115200);
	// Wire.begin();
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
	Wire.setDefaultTimeout(200000); // 200ms
	delay(5);
	sixDOF.init(); //begin the IMU
	delay(5);
}

void loop() { 
	
	// sixDOF.getEuler(angles);
	sixDOF.getAngles(angles);
	// sixDOF.getQ(q);


	Serial.print(angles[0]);
	Serial.print(" | ");	
	Serial.print(angles[1]);
	Serial.print(" | ");
	Serial.println(angles[2]);
	

	delay(100); 
}

