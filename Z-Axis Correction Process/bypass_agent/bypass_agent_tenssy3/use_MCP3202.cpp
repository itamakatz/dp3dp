#include "use_MCP3202.h"

// #include <StandardCplusplus.h>
// #include <iterator>
// #include <map>

#define BIG_LOOP_ITER 10
#define ITER_NUM 10
#define PACKET_SIZE 200
#define MICROSECOND_DELAY 10

MCP320X adc(MCP_SELECT_CH);
// std::map<word, int> hist;
word current_word;
// bool end_sampling;

void MCP3202_setup() {
	Serial.println("MCP3202_setup");
	SPI.begin();
	adc.begin();
	adc.setupSPI();
	// end_sampling = false;
}

void MCP3202_loop() {

	// double average = 0;

	// for(double i = 0.0; i < ITER_NUM; ++i) {

	// 	adc.select();
	// 	current_word = adc.read12(0);
	// 	adc.deselect();

	// 	average = i / (i + 1) * average + current_word / (i + 1);
	// 	delayMicroseconds(MICROSECOND_DELAY);
	// }

	// Serial.println(average);

	// if (millis() < 5000){
		adc.select();
		current_word = adc.read12(0);
		adc.deselect();
		// current_word = analogRead(A5);
		Serial.print(current_word);
		Serial.print(",");
		Serial.println(micros());

		// delayMicroseconds(MICROSECOND_DELAY);

		// if ( hist.find(current_word) == hist.end() ) {
			// hist.insert(std::make_pair(current_word, 1));
		// } else {
			// hist[current_word]++;
		// }
	// } else if(!end_sampling and millis() >= 5000) {

	// 	Serial.println("\n========== hist data: ==========\n");

	// 	for (std::map<word,int>::iterator it = hist.begin(); it != hist.end(); ++it){
	// 		Serial.print(it->first);
	// 		Serial.print(",");
	// 		Serial.println(it->second);
	// 	}

	// 	end_sampling = true;
	// }

	// delay(LONG_DELAY);
}