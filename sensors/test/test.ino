
#define ANALOG_PIN A0


void setup(){
	Serial.begin(9600);
	pinMode(ANALOG_PIN, OUTPUT);
}




void loop(){
	Serial.println(analogRead(ANALOG_PIN));
	delay(30);
}