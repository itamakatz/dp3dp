#ifndef __MCP3202__
#define __MCP3202__

#include <SPI.h>
#include "pins.h"
#include "MCP320X.h"
#include "Arduino.h"


extern MCP320X adc;

void MCP3202_setup();

void MCP3202_loop();

#endif