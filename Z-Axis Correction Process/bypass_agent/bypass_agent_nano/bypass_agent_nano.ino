#include "pins.h"
// #include "use_MCP3202.h"
#include "sixDOF.h"
#include "use_VL6180.h"
#include "intermediator.h"

#include <Wire.h>

#define LONG_DELAY 1500
#define CRITICAL_DELAY 60


int distance;
float* angles_Euler;
sixDOF sixDOF_object = sixDOF();
VL6180 VL6180_object = VL6180();

void setup(){
	// Serial.begin(9600);
	Serial.begin(115200);
	Wire.begin();
	
	// intermediator_setup();
	sixDOF_object.sixDOF_setup(300, (float)1.1);
	VL6180_object.VL6180_setup();
}

void loop(){
	// intermediator_loop();
	distance = VL6180_object.read_distance();
	sixDOF_object.sixDOF_loop();
	angles_Euler = sixDOF_object.get_average();
	Serial.print("CDF is: ");
	Serial.println(sixDOF_object.get_CDF());
	print_results();

	check_key();

	delay(CRITICAL_DELAY);
	// delay(LONG_DELAY);
}

void print_results(){
	Serial.println(distance);

	for (int i = 0; i < 3; ++i){
		Serial.print(angles_Euler[i]);	
		Serial.print(", ");
	}

	Serial.println();
}

void check_key(){
	if (Serial.available() > 0) {
		// read the incoming byte:
		int incomingByte = Serial.read();
		if (incomingByte == 1)
		{
			sixDOF_object.calibrate();
		}
		// say what you got:
		Serial.print("Calibrated");
	}
}