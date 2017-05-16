#include "MCP3202.h"

#define BIG_LOOP_ITER 10
#define PACKET_SIZE 200
#define BIG_NUMBER 100000
#define LONG_DELAY 100
#define MICROSECOND_DELAY 10

MCP320X adc(10);

word w[PACKET_SIZE];

double average_array[BIG_LOOP_ITER];

void MCP3202_setup() {
	Serial.println("MCP3202_setup");
	SPI.begin();
	adc.begin();
	adc.setupSPI();
}

void MCP3202_loop() {

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