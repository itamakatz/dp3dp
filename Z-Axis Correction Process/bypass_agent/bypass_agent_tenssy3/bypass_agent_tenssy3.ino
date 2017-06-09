// #include "use_MCP3202.h"
#include "sixDOF_Tenssy3.h"
#include "use_VL6180.h"
#include "intermediator.h"

#include <i2c_t3.h>
#include "general_defs.h"


int distance = 0;
float angles_Euler[3] = {0};
float angles_Euler_average[3] = {0};
sixDOF_Tenssy3 sixDOF_object = sixDOF_Tenssy3();
VL6180 VL6180_object = VL6180();

void setup(){

	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		delay(DEBUG_DELAY);
	#endif

	Serial.begin(115200);
	Wire.begin();
	
	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("setup: after Wire.begin()");
	#endif

	// intermediator_setup();
	sixDOF_object.sixDOF_setup((float)0.1);
	VL6180_object.VL6180_setup();

	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("setup: after objects init");
	#endif
}

void loop(){

	// intermediator_loop();
	distance = VL6180_object.read_distance();
	sixDOF_object.sixDOF_loop();
	sixDOF_object.get_angles(&angles_Euler[0]);
	sixDOF_object.get_average(&angles_Euler_average[0]);
	
	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("loop: after updating objects");
	#endif

	#ifndef DISABLE_NORMAL_PRINTS
		print_results();
	#endif

	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("setup: after print_results()");
	#endif

	check_key();

	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("setup: after check_key()");
	#endif

	delay(MAIN_LOOP_CRITICAL_DELAY);
}

void print_results(){

	Serial.print("Distance: ");
	Serial.println(distance);

	Serial.print("angles_Euler: ");
	for (int i = 0; i < 3; ++i){
		Serial.print(angles_Euler[i]);	
		Serial.print(", ");
	}

	Serial.println();

	Serial.print("angles_Euler_average: ");
	for (int i = 0; i < 3; ++i){
		Serial.print(angles_Euler_average[i]);	
		Serial.print(", ");
	}

	Serial.println();
}

void check_key(){

	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("check_key()");
	#endif

	if (Serial.available() > 0) {

		#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
			Serial.println("check_key() - after if (Serial.available() > 0)");
		#endif
		
		// int incomingByte = Serial.read();
		int incomingByte = Serial.parseInt();
	
		if (incomingByte == 1){
			#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
				Serial.println("incomingByte == 1");
			#endif
			sixDOF_object.calibrate();
			Serial.println("Gyro calibrated");
		} else if (incomingByte == 2){
			#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
				Serial.println("incomingByte == 2");
			#endif
			sixDOF_object.set_zero();
			Serial.println("Calibrated values to zero");
		}
	
	}
}