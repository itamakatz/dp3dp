/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef MARLIN_CONFIG_H
#define MARLIN_CONFIG_H

#include "fastio.h"
#include "macros.h"
#include "boards.h"
#include "Version.h"
#include "Configuration.h"
//#include "Conditionals_LCD.h"
#include "Configuration_adv.h"
#include "pins.h"
#ifndef USBCON
    #define HardwareSerial_h // trick to disable the standard HWserial
#endif
#include "Arduino.h"
#include "Conditionals_post.h"
#include "SanityCheck.h"



#if ENABLED(DISTINCT_E_FACTORS) && E_STEPPERS > 1
#define XYZE_N (XYZ + E_STEPPERS)
    #define E_AXIS_N (E_AXIS + extruder)
#else
#undef DISTINCT_E_FACTORS
#define XYZE_N XYZE
#define E_AXIS_N E_AXIS
#endif


/**
   * Extruders have some combination of stepper motors and hotends
   * so we separate these concepts into the defines:
   *
   *  EXTRUDERS    - Number of Selectable Tools
   *  HOTENDS      - Number of hotends, whether connected or separate
   *  E_STEPPERS   - Number of actual E stepper motors
   *  TOOL_E_INDEX - Index to use when getting/setting the tool state
   *
   */
#if ENABLED(SINGLENOZZLE)             // One hotend, multi-extruder
#define HOTENDS      1
    #define E_STEPPERS   EXTRUDERS
    #define E_MANUAL     EXTRUDERS
    #define TOOL_E_INDEX current_block->active_extruder
    #undef TEMP_SENSOR_1_AS_REDUNDANT
    #undef HOTEND_OFFSET_X
    #undef HOTEND_OFFSET_Y
#elif ENABLED(SWITCHING_EXTRUDER)     // One E stepper, unified E axis, two hotends
#define HOTENDS      EXTRUDERS
    #define E_STEPPERS   1
    #define E_MANUAL     1
    #define TOOL_E_INDEX 0
    #ifndef HOTEND_OFFSET_Z
      #define HOTEND_OFFSET_Z { 0 }
    #endif
#elif ENABLED(MIXING_EXTRUDER)        // Multi-stepper, unified E axis, one hotend
#define HOTENDS      1
    #define E_STEPPERS   MIXING_STEPPERS
    #define E_MANUAL     1
    #define TOOL_E_INDEX 0
#else                                 // One stepper, E axis, and hotend per tool
#define HOTENDS      EXTRUDERS
#define E_STEPPERS   EXTRUDERS
#define E_MANUAL     EXTRUDERS
#define TOOL_E_INDEX current_block->active_extruder
#endif


/**
   * The BLTouch Probe emulates a servo probe
   * and uses "special" angles for its state.
   */
#if ENABLED(BLTOUCH)
#ifndef Z_ENDSTOP_SERVO_NR
      #define Z_ENDSTOP_SERVO_NR 0
    #endif
    #ifndef NUM_SERVOS
      #define NUM_SERVOS (Z_ENDSTOP_SERVO_NR + 1)
    #endif
    #undef DEACTIVATE_SERVOS_AFTER_MOVE
    #undef SERVO_DELAY
    #define SERVO_DELAY 50
    #undef Z_SERVO_ANGLES
    #define Z_SERVO_ANGLES { BLTOUCH_DEPLOY, BLTOUCH_STOW }

    #define BLTOUCH_DEPLOY    10
    #define BLTOUCH_STOW      90
    #define BLTOUCH_SELFTEST 120
    #define BLTOUCH_RESET    160
    #define _TEST_BLTOUCH(P) (READ(P##_PIN) != P##_ENDSTOP_INVERTING)

    #if ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)
      #undef Z_MIN_ENDSTOP_INVERTING
      #define Z_MIN_ENDSTOP_INVERTING false
      #define TEST_BLTOUCH() _TEST_BLTOUCH(Z_MIN)
    #else
      #define TEST_BLTOUCH() _TEST_BLTOUCH(Z_MIN_PROBE)
    #endif
#endif

/**
 * Set a flag for a servo probe
 */
#define HAS_Z_SERVO_ENDSTOP (defined(Z_ENDSTOP_SERVO_NR) && Z_ENDSTOP_SERVO_NR >= 0)

/**
 * Set a flag for any enabled probe
 */
#define PROBE_SELECTED (ENABLED(FIX_MOUNTED_PROBE) || ENABLED(Z_PROBE_ALLEN_KEY) || HAS_Z_SERVO_ENDSTOP || ENABLED(Z_PROBE_SLED))

/**
 * Clear probe pin settings when no probe is selected
 */
#if !PROBE_SELECTED
#undef Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
#undef Z_MIN_PROBE_ENDSTOP
#endif



#endif // MARLIN_CONFIG_H
