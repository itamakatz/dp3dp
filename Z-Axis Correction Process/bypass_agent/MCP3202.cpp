#include "MCP3202.h"

#define BIG_LOOP_ITER 10
#define ITER_NUM 10
#define PACKET_SIZE 200
#define LONG_DELAY 100
#define MICROSECOND_DELAY 10

MCP320X adc(10);


void MCP3202_setup() {
	// Serial.println("MCP3202_setup");
	Serial.println("000000000");
	SPI.begin();
	adc.begin();
	adc.setupSPI();
}

void MCP3202_loop() {

	// double average = 0;
	word current_word = 0;

	// for(double i = 0.0; i < ITER_NUM; ++i) {

	// 	adc.select();
	// 	current_word = adc.read12(0);
	// 	adc.deselect();

	// 	average = i / (i + 1) * average + current_word / (i + 1);
	// 	delayMicroseconds(MICROSECOND_DELAY);
	// }

	// // Serial.print("Average is:");
	// Serial.println(average);
	// // Serial.println("==================");
	// // Serial.println();

	adc.select();
	current_word = adc.read12(0);
	adc.deselect();
	Serial.println(current_word);
	// 	delayMicroseconds(MICROSECOND_DELAY);

	delay(LONG_DELAY);
}

void MCP3202_loop2() {

	word w[PACKET_SIZE];
	double average_array[BIG_LOOP_ITER];

	double loop_average = 0;
	double average = 0;

	for (int iter = 0; iter < BIG_LOOP_ITER; ++iter)
	{
		for(int i = 0; i < PACKET_SIZE; i++) {
			adc.select();
			w[i] = adc.read12(0);
			adc.deselect();
			delayMicroseconds(MICROSECOND_DELAY);
		}

		for(double i = 0.0; i < PACKET_SIZE; i++) {
			loop_average = i / (i + 1) * loop_average + w[(int)i] / (i + 1);
		}

		average_array[iter] = loop_average;
	}

	for(double i = 0.0; i < BIG_LOOP_ITER; i++) {
		average = i / (i + 1) * average + average_array[(int)i] / (i + 1);
	}

	Serial.print("Average is:");
	Serial.println(average);
	Serial.println("==================");
	Serial.println();

	delay(LONG_DELAY);
}