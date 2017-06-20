#include "VL6180.h"
// #include "MCP3202.h"
#include "s6DoF_Tenssy3.h"
#include "z_axis_interrupt.h"
#include "r_axis_interrupt.h"
#include "stepper.h"

#include <i2c_t3.h>
#include "general_defs.h"
#include <Metro.h>

extern volatile bool enable_state;

#ifndef DISABLE_NORMAL_PRINTS
	Metro print_metro = Metro(METRO_BIG_NUM);
#endif

Metro stepper_metro_z = Metro(METRO_BIG_NUM);
Metro led_metro = Metro(METRO_BIG_NUM);
bool led_state = HIGH;

float distance = 0;
float r_distance = 0;

float angles_Euler[3] = {0};
float angles_Euler_average[3] = {0};
s6DoF_Tenssy3 s6DoF_object = s6DoF_Tenssy3();
VL6180 VL6180_object = VL6180();

float z_correction;
float current_position = 0;

void setup(){

	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		delay(DEBUG_DELAY);
	#endif

	Serial.begin(115200);
	Wire.begin();
	
	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		Serial.println(F("setup: after Wire.begin()"));
	#endif

	intermediator_setup_z();
	intermediator_setup_r();
	s6DoF_object.sixDOF_setup(sixDOF_ALPHA);
	VL6180_object.VL6180_setup(VL6180_ALPHA);
	pinMode(LED_PIN, OUTPUT);
	digitalWriteFast(LED_PIN, HIGH);
	stepper_setup();

	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		Serial.println(F("setup: after objects init"));
	#endif

	#ifndef DISABLE_NORMAL_PRINTS
		print_metro = Metro(METRO_PRINT_INTERVAL);
	#endif

	stepper_metro_z = Metro(METRO_STEPPER_INTERVAL);
	led_metro = Metro(METRO_HIGH_INTERVAL);
	
	delay(DISTANCE_CONVERSION_TIME);
	distance = VL6180_object.get_average();
	// current_position = distance;

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
	// distance = VL6180_object.get_average();

	s6DoF_object.sixDOF_loop();
	s6DoF_object.get_angles(&angles_Euler[0]);
	s6DoF_object.get_average(&angles_Euler_average[0]);
	
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

	feedback_z();

	delay(MAIN_LOOP_CRITICAL_DELAY);
}

void feedback_z(){
	r_distance = -1 * (float)r_num_of_steps / R_AXIS_STEPS_PER_MM;
	// r_distance = (float)r_num_of_steps;
	// z_correction = r_distance;
	z_correction = (r_distance + DISTANCE_CONVERSION_TIME)/tan(angles_Euler_average[1]);
	// z_correction = distance/tan(angles_Euler_average[1]);
	stepper_add(current_position - z_correction);
	current_position = z_correction;
}

void check_metro(){

	#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		Serial.println(F("check_metro()"));
	#endif

	if (led_metro.check()) {
		
		#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
			Serial.println(F("check_metro() - led_metro"));
		#endif
		
		if (led_state == HIGH)  { 
			led_state = LOW;
			led_metro.interval(METRO_LOW_INTERVAL); // if the pin is LOW, set the interval to 0.25 seconds.
		} else {
			led_state = HIGH;
			led_metro.interval(METRO_HIGH_INTERVAL); // if the pin is HIGH, set the interval to 1 second.
		}
		digitalWriteFast(LED_PIN, led_state);
	}

	#ifndef DISABLE_NORMAL_PRINTS
		if (print_metro.check()) {
		
			#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
				Serial.println(F("check_metro() - print_metro"));
			#endif
			
			print_metro.interval(METRO_PRINT_INTERVAL);
			print_results();
		
			#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
				Serial.println(F("loop: after print_results()"));
			#endif
		}
	#endif

	if (stepper_metro_z.check()) {
		#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
			Serial.println(F("check_metro() - print_metro"));
		#endif

		if (enable_state == DISABLED) {
			stepper_loop();
		}

		stepper_metro_z.interval(METRO_STEPPER_INTERVAL);
	}
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
			s6DoF_object.calibrate();
			Serial.println(F("Gyro calibrated"));
		} else if (incomingByte == 2){
			#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
				Serial.println(F("incomingByte == 2"));
			#endif
			s6DoF_object.set_zero();
			Serial.println(F("Calibrated values to zero"));
		} else if (incomingByte == 3){
			#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
				Serial.println(F("incomingByte == 3"));
			#endif
			s6DoF_object.sixDOF_setup((float)0.1);
			Serial.println(F("Run sixDOF_setup again"));
		}
	}
}