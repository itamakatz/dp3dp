
const int RECIEVE_ENABLE = 2; // connected to Z_ENABLE_PIN 62 of the mega
const int PASS_ENABLE = 8;    // connected to ENABLE pin 2 of the stepper driver 

#define LONG_DELAY 700

volatile bool state = false;

void toggle_enabled_pin(){
	cli();
	state = !state;
	// digitalWrite(PASS_ENABLE, digitalRead(RECIEVE_ENABLE));
	digitalWrite(PASS_ENABLE, state);
	Serial.println("enable_FALLING");
	sei();
}

void setup(){
	Serial.begin(9600);

	state = false;

	// pinMode(RECIEVE_ENABLE, INPUT_PULLUP);
	pinMode(RECIEVE_ENABLE, INPUT);

	pinMode(PASS_ENABLE, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(RECIEVE_ENABLE), toggle_enabled_pin, CHANGE);
}


void loop(){

	Serial.print("RECIEVE_ENABLE is:");
	Serial.println(digitalRead(RECIEVE_ENABLE));

	Serial.print("state is:");
	Serial.println(state);

	delay(LONG_DELAY);
}