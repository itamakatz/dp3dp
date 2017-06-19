#include "VL6180.h"
// #include "MCP3202.h"
#include "sixDOF_Tenssy3.h"
#include "intermediator.h"

#include <i2c_t3.h>
#include "general_defs.h"
#include <Metro.h>

Metro led_metro = Metro(METRO_HIGH_INTERVAL);
bool led_state = false;

float distance = 0;
float angles_Euler[3] = {0};
float angles_Euler_average[3] = {0};
sixDOF_Tenssy3 sixDOF_object = sixDOF_Tenssy3(sixDOF_ALPHA);
VL6180 VL6180_object = VL6180(VL6180_ALPHA);

void setup(){

	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		delay(DEBUG_DELAY);
	#endif

	Serial.begin(115200);
	Wire.begin();
	
	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("setup: after Wire.begin()");
	#endif

	intermediator_setup();
	sixDOF_object.sixDOF_setup();
	VL6180_object.VL6180_setup();
	pinMode(LED_PIN, OUTPUT);  

	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("setup: after objects init");
	#endif

	#ifdef DEBUG_MILIS__BYPASS_AGENT__
		Serial.print(millis());
		Serial.println(" - end of setup");
	#endif
}

void loop(){

	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("loop: beginning");
	#endif

	check_metro();

	VL6180_object.VL6180_loop();
	distance = VL6180_object.get_average();

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
		Serial.println("loop: after print_results()");
	#endif

	check_key();

	#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		Serial.println("loop: after check_key()");
	#endif

	#ifdef DEBUG_MILIS__BYPASS_AGENT__
		Serial.print(millis());
		Serial.println(" - end of loop");
	#endif

	delay(MAIN_LOOP_CRITICAL_DELAY);
}

void check_metro(){
	if (led_metro.check()) { // check if the metro has passed its interval .
		if (led_state == HIGH)  { 
			led_state = LOW;
			led_metro.interval(METRO_LOW_INTERVAL); // if the pin is LOW, set the interval to 0.25 seconds.
		} else {
			led_state = HIGH;
			led_metro.interval(METRO_HIGH_INTERVAL); // if the pin is HIGH, set the interval to 1 second.
		}
		digitalWrite(LED_PIN, led_state);
	}
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
		
		// read ascii char and convert to int representation
		int incomingByte = Serial.read() - '0';
		
		#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
			Serial.print("check_key() - incomingByte is ");
			Serial.println(incomingByte);
		#endif

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
		} else if (incomingByte == 3){
			#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
				Serial.println("incomingByte == 3");
			#endif
			sixDOF_object.sixDOF_setup();
			Serial.println("Run sixDOF_setup again");
		}	
	}
}