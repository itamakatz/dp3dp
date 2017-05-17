
#define ANALOG_PIN A0

void setup(){
	Serial.begin(9600);
	pinMode(ANALOG_PIN, OUTPUT);
}

void loop(){
	uint16_t value = analogRead(ANALOG_INPUT);
	Serial.println(value, DEC);
	delay(30);
}

