#ifndef __USE_MCP3202__
#define __USE_MCP3202__

#include <SPI.h>
#include "MCP320X.h"
#include "Arduino.h"
#include "general_defs.h"

extern MCP320X adc;

void MCP3202_setup();

void MCP3202_loop();

#endif