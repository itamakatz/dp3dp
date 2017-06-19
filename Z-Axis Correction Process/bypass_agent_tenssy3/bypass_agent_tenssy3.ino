#include "VL6180.h"
// #include "MCP3202.h"
#include "sixDOF_Tenssy3.h"
#include "intermediator.h"

#include <i2c_t3.h>
#include "general_defs.h"
#include <Metro.h>

#ifndef DISABLE_NORMAL_PRINTS
	Metro print_metro = Metro(METRO_PRINT_INTERVAL);
#endif

Metro led_metro = Metro(METRO_HIGH_INTERVAL);
bool led_state = false;

float distance = 0;
float angles_Euler[3] = {0};
float angles_Euler_average[3] = {0};
sixDOF_Tenssy3 sixDOF_object = sixDOF_Tenssy3();
VL6180 VL6180_object = VL6180();

void setup(){

	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		delay(DEBUG_DELAY);
	#endif

	Serial.begin(115200);
	Wire.begin();
	
	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		Serial.println(F("setup: after Wire.begin()"));
	#endif

	intermediator_setup();
	sixDOF_object.sixDOF_setup(sixDOF_ALPHA);
	VL6180_object.VL6180_setup(VL6180_ALPHA);
	pinMode(LED_PIN, OUTPUT);  

	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		Serial.println(F("setup: after objects init"));
	#endif

	#ifdef DEBUG_MILIS_BYPASS_AGENT_
		Serial.print(millis());
		Serial.println(F(" - end of setup"));
	#endif
}

void loop(){

	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		Serial.println(F("loop: beginning"));
	#endif

	check_metro();

	VL6180_object.VL6180_loop();
	distance = VL6180_object.get_average();

	sixDOF_object.sixDOF_loop();
	sixDOF_object.get_angles(&angles_Euler[0]);
	sixDOF_object.get_average(&angles_Euler_average[0]);
	
	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		Serial.println(F("loop: after updating objects"));
	#endif

	check_key();

	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		Serial.println(F("loop: after check_key()"));
	#endif

	#ifdef DEBUG_MILIS_BYPASS_AGENT_
		Serial.print(millis());
		Serial.println(F(" - end of loop"));
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

	#ifndef DISABLE_NORMAL_PRINTS
		if (print_metro.check()) { // check if the metro has passed its interval .
			print_metro.interval(METRO_PRINT_INTERVAL); // if the pin is LOW, set the interval to 0.25 seconds.
			print_results();
			#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
				Serial.println(F("loop: after print_results()"));
			#endif
		}
	#endif
}

void print_results(){

	Serial.print(F("Distance: "));
	Serial.println(distance);

	Serial.print(F("angles_Euler: "));
	for (int i = 0; i < 3; ++i){
		Serial.print(angles_Euler[i]);	
		Serial.print(F(", "));
	}

	Serial.println();

	Serial.print(F("angles_Euler_average: "));
	for (int i = 0; i < 3; ++i){
		Serial.print(angles_Euler_average[i]);	
		Serial.print(F(", "));
	}

	Serial.println();
}

void check_key(){

	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		Serial.println(F("check_key()"));
	#endif

	if (Serial.available() > 0) {

		#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
			Serial.println(F("check_key() - after if (Serial.available() > 0)"));
		#endif
		
		// read ascii char and convert to int representation
		int incomingByte = Serial.read() - '0';
		
		#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
			Serial.print(F("check_key() - incomingByte is "));
			Serial.println(incomingByte);
		#endif

		if (incomingByte == 1){
			#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
				Serial.println(F("incomingByte == 1"));
			#endif
			sixDOF_object.calibrate();
			Serial.println(F("Gyro calibrated"));
		} else if (incomingByte == 2){
			#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
				Serial.println(F("incomingByte == 2"));
			#endif
			sixDOF_object.set_zero();
			Serial.println(F("Calibrated values to zero"));
		} else if (incomingByte == 3){
			#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
				Serial.println(F("incomingByte == 3"));
			#endif
			sixDOF_object.sixDOF_setup((float)0.1);
			Serial.println(F("Run sixDOF_setup again"));
		}	
	}
}