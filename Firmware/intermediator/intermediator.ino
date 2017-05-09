
#define ENABLED 0     // for stepper driver A4988 - enable with low signal
#define DISABLES 1

const int RECIEVE_ENABLE = A0; // connected to Z_ENABLE_PIN 62 of the mega
const int RECIEVE_STEP = 2;    // connected to Z_STEP_PIN 46 of the mega
const int RECIEVE_DIR = 4;     // connected to Z_DIR_PIN 48 of the mega

const int PASS_ENABLE = A1;  // connected to ENABLE pin 2 of the stepper driver 
const int PASS_STEP = 13;    // connected to STEP pin 16 of the stepper driver 
const int PASS_DIR = 12;     // connected to DIR pin 19 of the stepper driver 


void setup(){

	Serial.begin(250000);

	pinMode(RECIEVE_ENABLE, INPUT);
	pinMode(RECIEVE_STEP, INPUT);
	pinMode(RECIEVE_DIR, INPUT);

	pinMode(PASS_ENABLE, OUTPUT);
	pinMode(PASS_STEP, OUTPUT);
	pinMode(PASS_DIR, OUTPUT);
}


void loop(){

	int state = digitalRead(RECIEVE_ENABLE);
	if (state == ENABLED){
		digitalWrite(PASS_ENABLE, ENABLED);
		digitalWrite(PASS_STEP, digitalRead(RECIEVE_STEP));
		digitalWrite(PASS_DIR, digitalRead(RECIEVE_DIR));
	} else {
		// do our stuff
	}

}

