#include "pins.h"
#include "MCP3202.h"
#include "intermediator.h"
#include "InterleavedContinuous.h"

#define LONG_DELAY 700

void setup(){
	Serial.begin(9600);
	// Serial.begin(19200);
	
	MCP3202_setup();
	// intermediator_setup();
	// interleavedContinuous_setup();

}

void loop(){
	MCP3202_loop();
	// intermediator_loop();
	// interleavedContinuous_loop();
	delay(LONG_DELAY);
}