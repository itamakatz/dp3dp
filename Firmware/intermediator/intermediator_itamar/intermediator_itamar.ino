
const int RECIEVE_ENABLE = 2; // connected to Z_ENABLE_PIN 62 of the mega
// const int RECIEVE_STEP = 3;    // connected to Z_STEP_PIN 46 of the mega
// const int RECIEVE_DIR = 4;     // connected to Z_DIR_PIN 48 of the mega

// const int PASS_ENABLE = 8;    // connected to ENABLE pin 2 of the stepper driver 
// const int PASS_STEP = 13;      // connected to STEP pin 16 of the stepper driver 
// const int PASS_DIR = 12;       // connected to DIR pin 19 of the stepper driver 

const int PASS_ENABLE = 0;    // connected to ENABLE pin 2 of the stepper driver 


volatile bool state = false;


void toggle_enabled_pin(){
	cli();
	state = !state;
	digitalWrite(PASS_ENABLE, state);
	sei();
}


// void step_received(){
// 	cli();
// 	if (state){
// 		digitalWrite(PASS_DIR, digitalRead(RECIEVE_DIR));
// 		digitalWrite(PASS_STEP, digitalRead(RECIEVE_STEP));
// 	}
// 	sei();
// }



void setup(){

	pinMode(RECIEVE_ENABLE, INPUT);
	// pinMode(RECIEVE_STEP, INPUT);
	// pinMode(RECIEVE_DIR, INPUT);

	pinMode(PASS_ENABLE, OUTPUT);
	// pinMode(PASS_STEP, OUTPUT);
	// pinMode(PASS_DIR, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE), toggle_enabled_pin, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(RECIEVE_STEP), step_received, CHANGE);
}


void loop(){

	// if (state){
	// 	digitalWrite(PASS_DIR, digitalRead(RECIEVE_DIR));
	// 	digitalWrite(PASS_STEP, digitalRead(RECIEVE_STEP));
	// }
	// digitalWrite(PASS_DIR, digitalRead(RECIEVE_DIR));
	// digitalWrite(PASS_STEP, digitalRead(RECIEVE_STEP));

}

