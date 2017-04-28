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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.	If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *	- https://github.com/kliment/Sprinter
 *	- https://github.com/simen/grbl/tree
 *
 * It has preliminary support for Matthew Roberts advance algorithm
 *	- http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

/**
 * -----------------
 * G-Codes in Marlin
 * -----------------
 *
 * Helpful G-code references:
 *	- http://linuxcnc.org/handbook/gcode/g-code.html
 *	- http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
 *
 * Help to document Marlin's G-codes online:
 *	- http://reprap.org/wiki/G-code
 *	- https://github.com/MarlinFirmware/MarlinDocumentation
 *
 * -----------------
 *
 * "G" Codes
 *
 * G0	-> G1
 * G1	- Coordinated Movement X Y Z E
 * G2	- CW ARC
 * G3	- CCW ARC
 * G4	- Dwell S<seconds> or P<milliseconds>
 * G5	- Cubic B-spline with XYZE destination and IJPQ offsets
 * G10 - Retract filament according to settings of M207
 * G11 - Retract recover filament according to settings of M208
 * G12 - Clean tool
 * G20 - Set input units to inches
 * G21 - Set input units to millimeters
 * G28 - Home one or more axes
 * G29 - Detailed Z probe, probes the bed at 3 or more points.	Will fail if you haven't homed yet.
 * G30 - Single Z probe, probes bed at X Y location (defaults to current XY location)
 * G31 - Dock sled (Z_PROBE_SLED only)
 * G32 - Undock sled (Z_PROBE_SLED only)
 * G38 - Probe target - similar to G28 except it uses the Z_MIN endstop for all three axes
 * G90 - Use Absolute Coordinates
 * G91 - Use Relative Coordinates
 * G92 - Set current position to coordinates given
 *
 * "M" Codes
 *
 * M0	 - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
 * M1	 - Same as M0
 * M17	- Enable/Power all stepper motors
 * M18	- Disable all stepper motors; same as M84
 * M30	- Delete file from SD: "M30 /path/file.gco"
 * M31	- Report time since last M109 or SD card start to serial.
 * M33	- Get the longname version of a path. (Requires LONG_FILENAME_HOST_SUPPORT)
 * M42	- Change pin status via gcode: M42 P<pin> S<value>. LED pin assumed if P is omitted.
 * M43	- Monitor pins & report changes - report active pins
 * M48	- Measure Z Probe repeatability: M48 P<points> X<pos> Y<pos> V<level> E<engage> L<legs>. (Requires Z_MIN_PROBE_REPEATABILITY_TEST)
 * M75	- Start the print job timer.
 * M76	- Pause the print job timer.
 * M77	- Stop the print job timer.
 * M78	- Show statistical information about the print jobs. (Requires PRINTCOUNTER)
 * M80	- Turn on Power Supply. (Requires POWER_SUPPLY)
 * M81	- Turn off Power Supply. (Requires POWER_SUPPLY)
 * M82	- Set E codes absolute (default).
 * M83	- Set E codes relative while in Absolute (G90) mode.
 * M84	- Disable steppers until next move, or use S<seconds> to specify an idle
 *				duration after which steppers should turn off. S0 disables the timeout.
 * M85	- Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 * M92	- Set planner.axis_steps_per_mm for one or more axes.
 * M104 - Set extruder target temp.
 * M105 - Report current temperatures.
 * M106 - Fan on.
 * M107 - Fan off.
 * M108 - Break out of heating loops (M109, M190, M303). With no controller, breaks out of M0/M1. (Requires EMERGENCY_PARSER)
 * M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
 *				Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
 *				If AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
 * M110 - Set the current line number. (Used by host printing)
 * M111 - Set debug flags: "M111 S<flagbits>". See flag bits defined in enum.h.
 * M112 - Emergency stop.
 * M113 - Get or set the timeout interval for Host Keepalive "busy" messages. (Requires HOST_KEEPALIVE_FEATURE)
 * M114 - Report current position.
 * M115 - Report capabilities. (Extended capabilities requires EXTENDED_CAPABILITIES_REPORT)
 * M117 - Display a message on the controller screen. (Requires an LCD)
 * M119 - Report endstops status.
 * M120 - Enable endstops detection.
 * M121 - Disable endstops detection.
 * M126 - Solenoid Air Valve Open. (Requires BARICUDA)
 * M127 - Solenoid Air Valve Closed. (Requires BARICUDA)
 * M128 - EtoP Open. (Requires BARICUDA)
 * M129 - EtoP Closed. (Requires BARICUDA)
 * M140 - Set bed target temp. S<temp>
 * M145 - Set heatup values for materials on the LCD. H<hotend> B<bed> F<fan speed> for S<material> (0=PLA, 1=ABS)
 * M149 - Set temperature units. (Requires TEMPERATURE_UNITS_SUPPORT)
 * M150 - Set Status LED Color as R<red> U<green> B<blue>. Values 0-255. (Requires BLINKM or RGB_LED)
 * M155 - Auto-report temperatures with interval of S<seconds>. (Requires AUTO_REPORT_TEMPERATURES)
 * M190 - Sxxx Wait for bed current temp to reach target temp. ** Waits only when heating! **
 *				Rxxx Wait for bed current temp to reach target temp. ** Waits for heating or cooling. **
 * M200 - Set filament diameter, D<diameter>, setting E axis units to cubic. (Use S0 to revert to linear units.)
 * M201 - Set max acceleration in units/s^2 for print moves: "M201 X<accel> Y<accel> Z<accel> E<accel>"
 * M202 - Set max acceleration in units/s^2 for travel moves: "M202 X<accel> Y<accel> Z<accel> E<accel>" ** UNUSED IN MARLIN! **
 * M203 - Set maximum feedrate: "M203 X<fr> Y<fr> Z<fr> E<fr>" in units/sec.
 * M204 - Set default acceleration in units/sec^2: P<printing> R<extruder_only> T<travel>
 * M205 - Set advanced settings. Current units apply:
						S<print> T<travel> minimum speeds
						B<minimum segment time>
						X<max X jerk>, Y<max Y jerk>, Z<max Z jerk>, E<max E jerk>
 * M206 - Set additional homing offset.
 * M207 - Set Retract Length: S<length>, Feedrate: F<units/min>, and Z lift: Z<distance>. (Requires FWRETRACT)
 * M208 - Set Recover (unretract) Additional (!) Length: S<length> and Feedrate: F<units/min>. (Requires FWRETRACT)
 * M209 - Turn Automatic Retract Detection on/off: S<0|1> (For slicers that don't support G10/11). (Requires FWRETRACT)
					Every normal extrude-only move will be classified as retract depending on the direction.
 * M211 - Enable, Disable, and/or Report software endstops: S<0|1>
 * M218 - Set a tool offset: "M218 T<index> X<offset> Y<offset>". (Requires 2 or more extruders)
 * M220 - Set Feedrate Percentage: "M220 S<percent>" (i.e., "FR" on the LCD)
 * M221 - Set Flow Percentage: "M221 S<percent>"
 * M226 - Wait until a pin is in a given state: "M226 P<pin> S<state>"
 * M240 - Trigger a camera to take a photograph. (Requires CHDK or PHOTOGRAPH_PIN)
 * M250 - Set LCD contrast: "M250 C<contrast>" (0-63). (Requires LCD support)
 * M260 - i2c Send Data (Requires EXPERIMENTAL_I2CBUS)
 * M261 - i2c Request Data (Requires EXPERIMENTAL_I2CBUS)
 * M280 - Set servo position absolute: "M280 P<index> S<angle|µs>". (Requires servos)
 * M300 - Play beep sound S<frequency Hz> P<duration ms>
 * M301 - Set PID parameters P I and D. (Requires PIDTEMP)
 * M302 - Allow cold extrudes, or set the minimum extrude S<temperature>. (Requires PREVENT_COLD_EXTRUSION)
 * M303 - PID relay autotune S<temperature> sets the target temperature. Default 150C. (Requires PIDTEMP)
 * M304 - Set bed PID parameters P I and D. (Requires PIDTEMPBED)
 * M355 - Turn the Case Light on/off and set its brightness. (Requires CASE_LIGHT_PIN)
 * M380 - Activate solenoid on active extruder. (Requires EXT_SOLENOID)
 * M381 - Disable all solenoids. (Requires EXT_SOLENOID)
 * M400 - Finish all moves.
 * M401 - Lower Z probe. (Requires a probe)
 * M402 - Raise Z probe. (Requires a probe)
 * M404 - Display or set the Nominal Filament Width: "W<diameter>". (Requires FILAMENT_WIDTH_SENSOR)
 * M405 - Enable Filament Sensor flow control. "M405 D<delay_cm>". (Requires FILAMENT_WIDTH_SENSOR)
 * M406 - Disable Filament Sensor flow control. (Requires FILAMENT_WIDTH_SENSOR)
 * M407 - Display measured filament diameter in millimeters. (Requires FILAMENT_WIDTH_SENSOR)
 * M410 - Quickstop. Abort all planned moves.
 * M428 - Set the home_offset based on the current_position. Nearest edge applies.
 * M500 - Store parameters in EEPROM. (Requires EEPROM_SETTINGS)
 * M501 - Restore parameters from EEPROM. (Requires EEPROM_SETTINGS)
 * M502 - Revert to the default "factory settings". ** Does not write them to EEPROM! **
 * M503 - Print the current settings (in memory): "M503 S<verbose>". S0 specifies compact output.
 * M540 - Enable/disable SD card abort on endstop hit: "M540 S<state>". (Requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
 * M600 - Pause for filament change: "M600 X<pos> Y<pos> Z<raise> E<first_retract> L<later_retract>". (Requires FILAMENT_CHANGE_FEATURE)
 * M665 - Set delta configurations: "M665 L<diagonal rod> R<delta radius> S<segments/s>" (Requires DELTA)
 * M666 - Set delta endstop adjustment. (Requires DELTA)
 * M605 - Set dual x-carriage movement mode: "M605 S<mode> [X<x_offset>] [R<temp_offset>]". (Requires DUAL_X_CARRIAGE)
 * M851 - Set Z probe's Z offset in current units. (Negative = below the nozzle.)
 * M907 - Set digital trimpot motor current using axis codes. (Requires a board with digital trimpots)
 * M908 - Control digital trimpot directly. (Requires DAC_STEPPER_CURRENT or DIGIPOTSS_PIN)
 * M909 - Print digipot/DAC current value. (Requires DAC_STEPPER_CURRENT)
 * M910 - Commit digipot/DAC value to external EEPROM via I2C. (Requires DAC_STEPPER_CURRENT)
 * M350 - Set microstepping mode. (Requires digital microstepping pins.)
 * M351 - Toggle MS1 MS2 pins directly. (Requires digital microstepping pins.)
 *
 * ************ SCARA Specific - This can change to suit future G-code regulations
 * M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 * M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 * M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 * M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 * M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
 * ************* SCARA End ***************
 *
 * ************ Custom codes - This can change to suit future G-code regulations
 * M100 - Watch Free Memory (For Debugging). (Requires M100_FREE_MEMORY_WATCHER)
 * M928 - Start SD logging: "M928 filename.gco". Stop with M29. (Requires SDSUPPORT)
 * M999 - Restart after being stopped by error
 *
 * "T" Codes
 *
 * T0-T3 - Select an extruder (tool) by index: "T<n> F<units/min>"
 *
 */

#include "Marlin.h"

#include "planner.h"
#include "stepper.h"
#include "endstops.h"
#include "temperature.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "nozzle.h"
#include "duration_t.h"
#include "types.h"


#if ENABLED(USE_WATCHDOG)
	#include "watchdog.h"
#endif

#if ENABLED(BLINKM)
	#include "blinkm.h"
	#include "Wire.h"
#endif


#if ENABLED(M100_FREE_MEMORY_WATCHER)
	void gcode_M100();
#endif


#if ENABLED(G38_PROBE_TARGET)
	bool G38_move = false,
			 G38_endstop_hit = false;
#endif

bool Running = true;

uint8_t marlin_debug_flags = DEBUG_NONE;

/**
 * Cartesian Current Position
 *	 Used to track the logical position as moves are queued.
 *	 Used by 'line_to_current_position' to do a move after changing it.
 *	 Used by 'SYNC_PLAN_POSITION_KINEMATIC' to update 'planner.position'.
 */
float current_position[XYZE] = { 0.0 };

/**
 * Cartesian Destination
 *	 A temporary position, usually applied to 'current_position'.
 *	 Set with 'gcode_get_destination' or 'set_destination_to_current'.
 *	 'line_to_destination' sets 'current_position' to 'destination'.
 */
static float destination[XYZE] = { 0.0 };

/**
 * axis_homed
 *	 Flags that each linear axis was homed.
 *	 XYZ on cartesian, ABC on delta, ABZ on SCARA.
 *
 * axis_known_position
 *	 Flags that the position is known in each linear axis. Set when homed.
 *	 Cleared whenever a stepper powers off, potentially losing its position.
 */
bool axis_homed[XYZ] = { false }, axis_known_position[XYZ] = { false };

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to Marlin, and lines will be checked for sequentiality.
 * M110 S<int> sets the current line number.
 */
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

/**
 * GCode Command Queue
 * A simple ring buffer of BUFSIZE command strings.
 *
 * Commands are copied into this buffer by the command injectors
 * (immediate, serial, sd card) and they are processed sequentially by
 * the main loop. The process_next_command function parses the next
 * command and hands off execution to individual handler functions.
 */
static char command_queue[BUFSIZE][MAX_CMD_SIZE];
static uint8_t cmd_queue_index_r = 0, // Ring buffer read position
							 cmd_queue_index_w = 0, // Ring buffer write position
							 commands_in_queue = 0; // Count of commands in the queue

/**
 * Current GCode Command
 * When a GCode handler is running, these will be set
 */
static char *current_command,			// The command currently being executed
						*current_command_args, // The address where arguments begin
						*seen_pointer;				 // Set by code_seen(), used by the code_value functions

/**
 * Next Injected Command pointer. NULL if no commands are being injected.
 * Used by Marlin internally to ensure that commands initiated from within
 * are enqueued ahead of any pending serial or sd card commands.
 */
static const char *injected_commands_P = NULL;

#if ENABLED(INCH_MODE_SUPPORT)
	float linear_unit_factor = 1.0, volumetric_unit_factor = 1.0;
#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
	TempUnit input_temp_units = TEMPUNIT_C;
#endif

/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
float constexpr homing_feedrate_mm_s[] = {
	MMM_TO_MMS(HOMING_FEEDRATE_XY), MMM_TO_MMS(HOMING_FEEDRATE_XY),
	MMM_TO_MMS(HOMING_FEEDRATE_Z), 0
};
static float feedrate_mm_s = MMM_TO_MMS(1500.0), saved_feedrate_mm_s;
int feedrate_percentage = 100, saved_feedrate_percentage,
		flow_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(100);

bool axis_relative_modes[] = AXIS_RELATIVE_MODES,
		 volumetric_enabled = false;
float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(DEFAULT_NOMINAL_FILAMENT_DIA),
			volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(1.0);

// The distance that XYZ has been offset by G92. Reset by G28.
float position_shift[XYZ] = { 0 };

// This offset is added to the configured home position.
// Set by M206, M428, or menu item. Saved to EEPROM.
float home_offset[XYZ] = { 0 };

// Software Endstops are based on the configured limits.
#if ENABLED(min_software_endstops) || ENABLED(max_software_endstops)
	bool soft_endstops_enabled = true;
#endif
float soft_endstop_min[XYZ] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
			soft_endstop_max[XYZ] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

#if FAN_COUNT > 0
	int fanSpeeds[FAN_COUNT] = { 0 };
#endif

// The active extruder (tool). Set with T<extruder> command.
uint8_t active_extruder = 0;

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode = false;

// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
volatile bool wait_for_heatup = true;

// For M0/M1, this flag may be cleared (by M108) to exit the wait-for-user loop
#if ENABLED(EMERGENCY_PARSER)
	volatile bool wait_for_user = false;
#endif

const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

// Number of characters read in the current line of serial input
static int serial_count = 0;

// Inactivity shutdown
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
static millis_t stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;

// Print Job Timer
#if ENABLED(PRINTCOUNTER)
	PrintCounter print_job_timer = PrintCounter();
#else
	Stopwatch print_job_timer = Stopwatch();
#endif

// Buzzer - I2C on the LCD or a BEEPER_PIN
#if ENABLED(LCD_USE_I2C_BUZZER)
	#define BUZZ(d,f) lcd_buzz(d, f)
#elif PIN_EXISTS(BEEPER)
	Buzzer buzzer;
	#define BUZZ(d,f) buzzer.tone(d, f)
#else
	#define BUZZ(d,f) NOOP
#endif

static uint8_t target_extruder;

#if HAS_BED_PROBE
	float zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
#endif

#define PLANNER_XY_FEEDRATE() (min(planner.max_feedrate_mm_s[X_AXIS], planner.max_feedrate_mm_s[Y_AXIS]))


#if defined(XY_PROBE_SPEED)
	#define XY_PROBE_FEEDRATE_MM_S MMM_TO_MMS(XY_PROBE_SPEED)
#else
	#define XY_PROBE_FEEDRATE_MM_S PLANNER_XY_FEEDRATE()
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
	#define ADJUST_DELTA(V) if (planner.abl_enabled) { delta[Z_AXIS] += bilinear_z_offset(V); }
#endif

#if ENABLED(Z_DUAL_ENDSTOPS)
	float z_endstop_adj = 0;
#endif

#if HAS_Z_SERVO_ENDSTOP
	const int z_servo_angle[2] = Z_SERVO_ANGLES;
#endif

#if ENABLED(BARICUDA)
	int baricuda_valve_pressure = 0;
	int baricuda_e_to_p_pressure = 0;
#endif

#if ENABLED(FWRETRACT)

	bool autoretract_enabled = false;
	bool retracted[EXTRUDERS] = { false };
	bool retracted_swap[EXTRUDERS] = { false };

	float retract_length = RETRACT_LENGTH;
	float retract_length_swap = RETRACT_LENGTH_SWAP;
	float retract_feedrate_mm_s = RETRACT_FEEDRATE;
	float retract_zlift = RETRACT_ZLIFT;
	float retract_recover_length = RETRACT_RECOVER_LENGTH;
	float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
	float retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE;

#endif // FWRETRACT

static bool home_all_axis = true;


#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
	int bilinear_grid_spacing[2] = { 0 }, bilinear_start[2] = { 0 };
	float bed_level_grid[ABL_GRID_POINTS_X][ABL_GRID_POINTS_Y];
#endif

float cartes[XYZ] = { 0 };

#if ENABLED(FILAMENT_WIDTH_SENSOR)
	bool filament_sensor = false;	//M405 turns on filament_sensor control, M406 turns it off
	float filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA,	// Nominal filament width. Change with M404
				filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA;		// Measured filament diameter
	int8_t measurement_delay[MAX_MEASUREMENT_DELAY + 1]; // Ring buffer to delayed measurement. Store extruder factor after subtracting 100
	int filwidth_delay_index[2] = { 0, -1 };	// Indexes into ring buffer
	int meas_delay_cm = MEASUREMENT_DELAY_CM;	//distance delay setting
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
	static bool filament_ran_out = false;
#endif

#if ENABLED(FILAMENT_CHANGE_FEATURE)
	FilamentChangeMenuResponse filament_change_menu_response;
#endif


static bool send_ok[BUFSIZE];

#ifdef CHDK
	millis_t chdkHigh = 0;
	boolean chdkActive = false;
#endif

#if ENABLED(PID_EXTRUSION_SCALING)
	int lpq_len = 20;
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
	static MarlinBusyState busy_state = NOT_BUSY;
	static millis_t next_busy_signal_ms = 0;
	uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
	#define KEEPALIVE_STATE(n) do{ busy_state = n; }while(0)
#else
	#define host_keepalive() ;
	#define KEEPALIVE_STATE(n) ;
#endif // HOST_KEEPALIVE_FEATURE

#define DEFINE_PGM_READ_ANY(type, reader)			 \
	static inline type pgm_read_any(const type *p)	\
	{ return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,			 float)
DEFINE_PGM_READ_ANY(signed char, byte)

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
	static const PROGMEM type array##_P[XYZ] =				\
			{ X_##CONFIG, Y_##CONFIG, Z_##CONFIG };		 \
	static inline type array(int axis)					\
	{ return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,	 MIN_POS)
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,	 MAX_POS)
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,	HOME_POS)
XYZ_CONSTS_FROM_CONFIG(float, max_length,		 MAX_LENGTH)
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,	 HOME_BUMP_MM)
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR)

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

void get_available_commands();
void process_next_command();
void prepare_move_to_destination();

void get_cartesian_from_steppers();
void set_current_from_steppers_for_axis(const AxisEnum axis);

#if ENABLED(ARC_SUPPORT)
	void plan_arc(float target[NUM_AXIS], float* offset, uint8_t clockwise);
#endif


void serial_echopair_P(const char* s_P, const char *v)	 { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, char v)					{ serialprintPGM(s_P); SERIAL_CHAR(v); }
void serial_echopair_P(const char* s_P, int v)					 { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, long v)					{ serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, float v)				 { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, double v)				{ serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, unsigned long v) { serialprintPGM(s_P); SERIAL_ECHO(v); }

void tool_change(const uint8_t tmp_extruder, const float fr_mm_s=0.0, bool no_move=false);
static void report_current_position();

#if ENABLED(DEBUG_LEVELING_FEATURE)
	void print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z) {
		serialprintPGM(prefix);
		SERIAL_ECHOPAIR("(", x);
		SERIAL_ECHOPAIR(", ", y);
		SERIAL_ECHOPAIR(", ", z);
		SERIAL_ECHOPGM(")");

		if (suffix) serialprintPGM(suffix);
		else SERIAL_EOL;
	}

	void print_xyz(const char* prefix, const char* suffix, const float xyz[]) {
		print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
	}


	#define DEBUG_POS(SUFFIX,VAR) do { \
		print_xyz(PSTR("	" STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); } while(0)
#endif

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
inline void sync_plan_position() {
	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position", current_position);
	#endif
	planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
inline void sync_plan_position_e() { planner.set_e_position_mm(current_position[E_AXIS]); }

#define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()

#if ENABLED(SDSUPPORT)
	#include "SdFatUtil.h"
	int freeMemory() { return SdFatUtil::FreeRam(); }
#else
extern "C" {
	extern unsigned int __bss_end;
	extern unsigned int __heap_start;
	extern void* __brkval;

	int freeMemory() {
		int free_memory;
		if ((int)__brkval == 0)
			free_memory = ((int)&free_memory) - ((int)&__bss_end);
		else
			free_memory = ((int)&free_memory) - ((int)__brkval);
		return free_memory;
	}
}
#endif //!SDSUPPORT

#if ENABLED(DIGIPOT_I2C)
	extern void digipot_i2c_set_current(int channel, float current);
	extern void digipot_i2c_init();
#endif

/**
 * Inject the next "immediate" command, when possible.
 * Return true if any immediate commands remain to inject.
 */
static bool drain_injected_commands_P() {
	if (injected_commands_P != NULL) {
		size_t i = 0;
		char c, cmd[30];
		strncpy_P(cmd, injected_commands_P, sizeof(cmd) - 1);
		cmd[sizeof(cmd) - 1] = '\0';
		while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
		cmd[i] = '\0';
		if (enqueue_and_echo_command(cmd)) {	 // success?
			if (c)															 // newline char?
				injected_commands_P += i + 1;				// advance to the next command
			else
				injected_commands_P = NULL;					// nul char? no more commands
		}
	}
	return (injected_commands_P != NULL);			// return whether any more remain
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_injected_commands_P() must be called repeatedly to drain the commands afterwards
 */
void enqueue_and_echo_commands_P(const char* pgcode) {
	injected_commands_P = pgcode;
	drain_injected_commands_P(); // first command executed asap (when possible)
}

void clear_command_queue() {
	cmd_queue_index_r = cmd_queue_index_w;
	commands_in_queue = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
inline void _commit_command(bool say_ok) {
	send_ok[cmd_queue_index_w] = say_ok;
	cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
	commands_in_queue++;
}

/**
 * Copy a command directly into the main command buffer, from RAM.
 * Returns true if successfully adds the command
 */
inline bool _enqueuecommand(const char* cmd, bool say_ok=false) {
	if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
	strcpy(command_queue[cmd_queue_index_w], cmd);
	_commit_command(say_ok);
	return true;
}

void enqueue_and_echo_command_now(const char* cmd) {
	while (!enqueue_and_echo_command(cmd)) idle();
}

/**
 * Enqueue with Serial Echo
 */
bool enqueue_and_echo_command(const char* cmd, bool say_ok/*=false*/) {
	if (_enqueuecommand(cmd, say_ok)) {
		SERIAL_ECHO_START;
		SERIAL_ECHOPAIR(MSG_Enqueueing, cmd);
		SERIAL_CHAR('"');
		SERIAL_EOL;
		return true;
	}
	return false;
}

void setup_killpin() {
	#if HAS_KILL
		SET_INPUT(KILL_PIN);
		WRITE(KILL_PIN, HIGH);
	#endif
}

#if ENABLED(FILAMENT_RUNOUT_SENSOR)

	void setup_filrunoutpin() {
		SET_INPUT(FIL_RUNOUT_PIN);
		#if ENABLED(ENDSTOPPULLUP_FIL_RUNOUT)
			WRITE(FIL_RUNOUT_PIN, HIGH);
		#endif
	}

#endif

// Set home pin
void setup_homepin(void) {
	#if HAS_HOME
		SET_INPUT(HOME_PIN);
		WRITE(HOME_PIN, HIGH);
	#endif
}

void setup_powerhold() {
	#if HAS_SUICIDE
		OUT_WRITE(SUICIDE_PIN, HIGH);
	#endif
	#if HAS_POWER_SWITCH
		#if ENABLED(PS_DEFAULT_OFF)
			OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
		#else
			OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
		#endif
	#endif
}

void suicide() {
	#if HAS_SUICIDE
		OUT_WRITE(SUICIDE_PIN, LOW);
	#endif
}

void servo_init() {
	#if NUM_SERVOS >= 1 && HAS_SERVO_0
		servo[0].attach(SERVO0_PIN);
		servo[0].detach(); // Just set up the pin. We don't have a position yet. Don't move to a random position.
	#endif
	#if NUM_SERVOS >= 2 && HAS_SERVO_1
		servo[1].attach(SERVO1_PIN);
		servo[1].detach();
	#endif
	#if NUM_SERVOS >= 3 && HAS_SERVO_2
		servo[2].attach(SERVO2_PIN);
		servo[2].detach();
	#endif
	#if NUM_SERVOS >= 4 && HAS_SERVO_3
		servo[3].attach(SERVO3_PIN);
		servo[3].detach();
	#endif

	#if HAS_Z_SERVO_ENDSTOP
		/**
		 * Set position of Z Servo Endstop
		 *
		 * The servo might be deployed and positioned too low to stow
		 * when starting up the machine or rebooting the board.
		 * There's no way to know where the nozzle is positioned until
		 * homing has been done - no homing with z-probe without init!
		 *
		 */
		STOW_Z_SERVO();
	#endif
}

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
	void disableStepperDrivers() {
		OUT_WRITE(STEPPER_RESET_PIN, LOW);	// drive it down to hold in reset motor driver chips
	}
	void enableStepperDrivers() { SET_INPUT(STEPPER_RESET_PIN); }	// set to input, which allows it to be pulled high by pullups
#endif


void gcode_line_error(const char* err, bool doFlush = true) {
	SERIAL_ERROR_START;
	serialprintPGM(err);
	SERIAL_ERRORLN(gcode_LastN);
	//Serial.println(gcode_N);
	if (doFlush) FlushSerialRequestResend();
	serial_count = 0;
}

inline void get_serial_commands() {
	static char serial_line_buffer[MAX_CMD_SIZE];
	static boolean serial_comment_mode = false;

	// If the command buffer is empty for too long,
	// send "wait" to indicate Marlin is still waiting.
	#if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
		static millis_t last_command_time = 0;
		millis_t ms = millis();
		if (commands_in_queue == 0 && !MYSERIAL.available() && ELAPSED(ms, last_command_time + NO_TIMEOUTS)) {
			SERIAL_ECHOLNPGM(MSG_WAIT);
			last_command_time = ms;
		}
	#endif

	/**
	 * Loop while serial characters are incoming and the queue is not full
	 */
	while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {

		char serial_char = MYSERIAL.read();

		/**
		 * If the character ends the line
		 */
		if (serial_char == '\n' || serial_char == '\r') {

			serial_comment_mode = false; // end of line == end of comment

			if (!serial_count) continue; // skip empty lines

			serial_line_buffer[serial_count] = 0; // terminate string
			serial_count = 0; //reset buffer

			char* command = serial_line_buffer;

			while (*command == ' ') command++; // skip any leading spaces
			char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line
			char* apos = strchr(command, '*');

			if (npos) {

				boolean M110 = strstr_P(command, PSTR("M110")) != NULL;

				if (M110) {
					char* n2pos = strchr(command + 4, 'N');
					if (n2pos) npos = n2pos;
				}

				gcode_N = strtol(npos + 1, NULL, 10);

				if (gcode_N != gcode_LastN + 1 && !M110) {
					gcode_line_error(PSTR(MSG_ERR_LINE_NO));
					return;
				}

				if (apos) {
					byte checksum = 0, count = 0;
					while (command[count] != '*') checksum ^= command[count++];

					if (strtol(apos + 1, NULL, 10) != checksum) {
						gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
						return;
					}
					// if no errors, continue parsing
				}
				else {
					gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
					return;
				}

				gcode_LastN = gcode_N;
				// if no errors, continue parsing
			}
			else if (apos) { // No '*' without 'N'
				gcode_line_error(PSTR(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
				return;
			}

			// Movement commands alert when stopped
			if (IsStopped()) {
				char* gpos = strchr(command, 'G');
				if (gpos) {
					int codenum = strtol(gpos + 1, NULL, 10);
					switch (codenum) {
						case 0:
						case 1:
						case 2:
						case 3:
							SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
                            NOOP;
							break;
					}
				}
			}

			#if DISABLED(EMERGENCY_PARSER)
				// If command was e-stop process now
				if (strcmp(command, "M108") == 0) {
					wait_for_heatup = false;
				}
				if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));
				if (strcmp(command, "M410") == 0) { quickstop_stepper(); }
			#endif

			#if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
				last_command_time = ms;
			#endif

			// Add the command to the queue
			_enqueuecommand(serial_line_buffer, true);
		}
		else if (serial_count >= MAX_CMD_SIZE - 1) {
			// Keep fetching, but ignore normal characters beyond the max length
			// The command will be injected when EOL is reached
		}
		else if (serial_char == '\\') {	// Handle escapes
			if (MYSERIAL.available() > 0) {
				// if we have one more character, copy it over
				serial_char = MYSERIAL.read();
				if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
			}
			// otherwise do nothing
		}
		else { // it's not a newline, carriage return or escape char
			if (serial_char == ';') serial_comment_mode = true;
			if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
		}

	} // queue has space, serial has data
}


/**
 * Add to the circular command queue the next command from:
 *	- The command-injection queue (injected_commands_P)
 *	- The active serial input (usually USB)
 *	- The SD card file being actively printed
 */
void get_available_commands() {

	// if any immediate commands remain, don't get other commands yet
	if (drain_injected_commands_P()) return;

	get_serial_commands();

}

inline bool code_has_value() {
	int i = 1;
	char c = seen_pointer[i];
	while (c == ' ') c = seen_pointer[++i];
	if (c == '-' || c == '+') c = seen_pointer[++i];
	if (c == '.') c = seen_pointer[++i];
	return NUMERIC(c);
}

inline float code_value_float() {
	float ret;
	char* e = strchr(seen_pointer, 'E');
	if (e) {
		*e = 0;
		ret = strtod(seen_pointer + 1, NULL);
		*e = 'E';
	}
	else
		ret = strtod(seen_pointer + 1, NULL);
	return ret;
}

inline unsigned long code_value_ulong() { return strtoul(seen_pointer + 1, NULL, 10); }

inline long code_value_long() { return strtol(seen_pointer + 1, NULL, 10); }

inline int code_value_int() { return (int)strtol(seen_pointer + 1, NULL, 10); }

inline uint16_t code_value_ushort() { return (uint16_t)strtoul(seen_pointer + 1, NULL, 10); }

inline uint8_t code_value_byte() { return (uint8_t)(constrain(strtol(seen_pointer + 1, NULL, 10), 0, 255)); }

inline bool code_value_bool() { return !code_has_value() || code_value_byte() > 0; }

#if ENABLED(INCH_MODE_SUPPORT)
	inline void set_input_linear_units(LinearUnit units) {
		switch (units) {
			case LINEARUNIT_INCH:
				linear_unit_factor = 25.4;
				break;
			case LINEARUNIT_MM:
			default:
				linear_unit_factor = 1.0;
				break;
		}
		volumetric_unit_factor = pow(linear_unit_factor, 3.0);
	}

	inline float axis_unit_factor(int axis) {
		return (axis >= E_AXIS && volumetric_enabled ? volumetric_unit_factor : linear_unit_factor);
	}

	inline float code_value_linear_units() { return code_value_float() * linear_unit_factor; }
	inline float code_value_axis_units(int axis) { return code_value_float() * axis_unit_factor(axis); }
	inline float code_value_per_axis_unit(int axis) { return code_value_float() / axis_unit_factor(axis); }

#else

	inline float code_value_linear_units() { return code_value_float(); }
	inline float code_value_axis_units(int axis) { UNUSED(axis); return code_value_float(); }
	inline float code_value_per_axis_unit(int axis) { UNUSED(axis); return code_value_float(); }

#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
	inline void set_input_temp_units(TempUnit units) { input_temp_units = units; }

	float code_value_temp_abs() {
		switch (input_temp_units) {
			case TEMPUNIT_C:
				return code_value_float();
			case TEMPUNIT_F:
				return (code_value_float() - 32) * 0.5555555556;
			case TEMPUNIT_K:
				return code_value_float() - 272.15;
			default:
				return code_value_float();
		}
	}

	float code_value_temp_diff() {
		switch (input_temp_units) {
			case TEMPUNIT_C:
			case TEMPUNIT_K:
				return code_value_float();
			case TEMPUNIT_F:
				return code_value_float() * 0.5555555556;
			default:
				return code_value_float();
		}
	}
#else
	float code_value_temp_abs() { return code_value_float(); }
	float code_value_temp_diff() { return code_value_float(); }
#endif

FORCE_INLINE millis_t code_value_millis() { return code_value_ulong(); }
inline millis_t code_value_millis_from_seconds() { return code_value_float() * 1000; }

bool code_seen(char code) {
	seen_pointer = strchr(current_command_args, code);
	return (seen_pointer != NULL); // Return TRUE if the code-letter was found
}

/**
 * Set target_extruder from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool get_target_extruder_from_command(int code) {
	if (code_seen('T')) {
		if (code_value_byte() >= EXTRUDERS) {
			SERIAL_ECHO_START;
			SERIAL_CHAR('M');
			SERIAL_ECHO(code);
			SERIAL_ECHOLNPAIR(" " MSG_INVALID_EXTRUDER " ", code_value_byte());
			return true;
		}
		target_extruder = code_value_byte();
	}
	else
		target_extruder = active_extruder;

	return false;
}

#if ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
	bool extruder_duplication_enabled = false; // Used in Dual X mode 2
#endif


/**
 * Software endstops can be used to monitor the open end of
 * an axis that has a hardware endstop on the other end. Or
 * they can prevent axes from moving past endstops and grinding.
 *
 * To keep doing their job as the coordinate system changes,
 * the software endstop positions must be refreshed to remain
 * at the same positions relative to the machine.
 */
void update_software_endstops(AxisEnum axis) {
	float offs = LOGICAL_POSITION(0, axis);

    soft_endstop_min[axis] = base_min_pos(axis) + offs;
    soft_endstop_max[axis] = base_max_pos(axis) + offs;

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			SERIAL_ECHOPAIR("For ", axis_codes[axis]);
			SERIAL_ECHOPAIR(" axis:\n home_offset = ", home_offset[axis]);
			SERIAL_ECHOPAIR("\n position_shift = ", position_shift[axis]);
			SERIAL_ECHOPAIR("\n soft_endstop_min = ", soft_endstop_min[axis]);
			SERIAL_ECHOLNPAIR("\n soft_endstop_max = ", soft_endstop_max[axis]);
		}
	#endif

}

/**
 * Change the home offset for an axis, update the current
 * position and the software endstops to retain the same
 * relative distance to the new home.
 *
 * Since this changes the current_position, code should
 * call sync_plan_position soon after this.
 */
static void set_home_offset(AxisEnum axis, float v) {
	current_position[axis] += v - home_offset[axis];
	home_offset[axis] = v;
	update_software_endstops(axis);
}

/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are already known and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */
static void set_axis_is_at_home(AxisEnum axis) {
	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			SERIAL_ECHOPAIR(">>> set_axis_is_at_home(", axis_codes[axis]);
			SERIAL_CHAR(')');
			SERIAL_EOL;
		}
	#endif

	axis_known_position[axis] = axis_homed[axis] = true;

	position_shift[axis] = 0;
	update_software_endstops(axis);

	current_position[axis] = LOGICAL_POSITION(base_home_pos(axis), axis);

	/**
	 * Z Probe Z Homing? Account for the probe's Z offset.
	 */
	#if HAS_BED_PROBE && Z_HOME_DIR < 0
		if (axis == Z_AXIS) {
			#if HOMING_Z_WITH_PROBE

				current_position[Z_AXIS] -= zprobe_zoffset;

				#if ENABLED(DEBUG_LEVELING_FEATURE)
					if (DEBUGGING(LEVELING)) {
						SERIAL_ECHOLNPGM("*** Z HOMED WITH PROBE (Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN) ***");
						SERIAL_ECHOLNPAIR("> zprobe_zoffset = ", zprobe_zoffset);
					}
				#endif

			#elif ENABLED(DEBUG_LEVELING_FEATURE)

				if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("*** Z HOMED TO ENDSTOP (Z_MIN_PROBE_ENDSTOP) ***");

			#endif
		}
	#endif

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			SERIAL_ECHOPAIR("> home_offset[", axis_codes[axis]);
			SERIAL_ECHOLNPAIR("] = ", home_offset[axis]);
			DEBUG_POS("", current_position);
			SERIAL_ECHOPAIR("<<< set_axis_is_at_home(", axis_codes[axis]);
			SERIAL_CHAR(')');
			SERIAL_EOL;
		}
	#endif
}

/**
 * Some planner shorthand inline functions
 */
inline float get_homing_bump_feedrate(AxisEnum axis) {
	int constexpr homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
	int hbd = homing_bump_divisor[axis];
	if (hbd < 1) {
		hbd = 10;
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");
	}
	return homing_feedrate_mm_s[axis] / hbd;
}

//
// line_to_current_position
// Move the planner to the current position from wherever it last moved
// (or from wherever it has been told it is located).
//
inline void line_to_current_position() {
	planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder);
}

//
// line_to_destination
// Move the planner, not necessarily synced with current_position
//
inline void line_to_destination(float fr_mm_s) {
	planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder);
}
inline void line_to_destination() { line_to_destination(feedrate_mm_s); }

inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }



/**
 *	Plan a move to (X, Y, Z) and set the current_position
 *	The final current_position may not be the one that was requested
 */
void do_blocking_move_to(const float &x, const float &y, const float &z, const float &fr_mm_s /*=0.0*/) {
	float old_feedrate_mm_s = feedrate_mm_s;

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, x, y, z);
	#endif

    // If Z needs to raise, do it before moving XY
    if (current_position[Z_AXIS] < z) {
        feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];
        current_position[Z_AXIS] = z;
        line_to_current_position();
    }

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
    current_position[X_AXIS] = x;
    current_position[Y_AXIS] = y;
    line_to_current_position();

    // If Z needs to lower, do it after moving XY
    if (current_position[Z_AXIS] > z) {
        feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];
        current_position[Z_AXIS] = z;
        line_to_current_position();
    }


	stepper.synchronize();

	feedrate_mm_s = old_feedrate_mm_s;

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< do_blocking_move_to");
	#endif
}
void do_blocking_move_to_x(const float &x, const float &fr_mm_s/*=0.0*/) {
	do_blocking_move_to(x, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
}
void do_blocking_move_to_z(const float &z, const float &fr_mm_s/*=0.0*/) {
	do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z, fr_mm_s);
}
void do_blocking_move_to_xy(const float &x, const float &y, const float &fr_mm_s/*=0.0*/) {
	do_blocking_move_to(x, y, current_position[Z_AXIS], fr_mm_s);
}

//
// Prepare to do endstop or probe moves
// with custom feedrates.
//
//	- Save current feedrates
//	- Reset the rate multiplier
//	- Reset the command timeout
//	- Enable the endstops (for endstop moves)
//
static void setup_for_endstop_or_probe_move() {
	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) DEBUG_POS("setup_for_endstop_or_probe_move", current_position);
	#endif
	saved_feedrate_mm_s = feedrate_mm_s;
	saved_feedrate_percentage = feedrate_percentage;
	feedrate_percentage = 100;
	refresh_cmd_timeout();
}

static void clean_up_after_endstop_or_probe_move() {
	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) DEBUG_POS("clean_up_after_endstop_or_probe_move", current_position);
	#endif
	feedrate_mm_s = saved_feedrate_mm_s;
	feedrate_percentage = saved_feedrate_percentage;
	refresh_cmd_timeout();
}

#if HAS_BED_PROBE
	/**
	 * Raise Z to a minimum height to make room for a probe to move
	 */
	inline void do_probe_raise(float z_raise) {
		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) {
				SERIAL_ECHOPAIR("do_probe_raise(", z_raise);
				SERIAL_CHAR(')');
				SERIAL_EOL;
			}
		#endif

		float z_dest = LOGICAL_Z_POSITION(z_raise);
		if (zprobe_zoffset < 0) z_dest -= zprobe_zoffset;

		if (z_dest > current_position[Z_AXIS])
			do_blocking_move_to_z(z_dest);
	}

#endif //HAS_BED_PROBE

#if ENABLED(Z_PROBE_ALLEN_KEY) || ENABLED(Z_PROBE_SLED) || HAS_PROBING_PROCEDURE || ENABLED(NOZZLE_CLEAN_FEATURE) || ENABLED(NOZZLE_PARK_FEATURE)
	static bool axis_unhomed_error(const bool x, const bool y, const bool z) {
		const bool xx = x && !axis_homed[X_AXIS],
							 yy = y && !axis_homed[Y_AXIS],
							 zz = z && !axis_homed[Z_AXIS];
		if (xx || yy || zz) {
			SERIAL_ECHO_START;
			SERIAL_ECHOPGM(MSG_HOME " ");
			if (xx) SERIAL_ECHOPGM(MSG_X);
			if (yy) SERIAL_ECHOPGM(MSG_Y);
			if (zz) SERIAL_ECHOPGM(MSG_Z);
			SERIAL_ECHOLNPGM(" " MSG_FIRST);

			return true;
		}
		return false;
	}
#endif

#if ENABLED(Z_PROBE_SLED)

	#ifndef SLED_DOCKING_OFFSET
		#define SLED_DOCKING_OFFSET 0
	#endif

	/**
	 * Method to dock/undock a sled designed by Charles Bell.
	 *
	 * stow[in]		 If false, move to MAX_X and engage the solenoid
	 *							If true, move to MAX_X and release the solenoid
	 */
	static void dock_sled(bool stow) {
		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) {
				SERIAL_ECHOPAIR("dock_sled(", stow);
				SERIAL_CHAR(')');
				SERIAL_EOL;
			}
		#endif

		// Dock sled a bit closer to ensure proper capturing
		do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));

		#if PIN_EXISTS(SLED)
			digitalWrite(SLED_PIN, !stow); // switch solenoid
		#endif
	}

#elif ENABLED(Z_PROBE_ALLEN_KEY)

	void run_deploy_moves_script() {
		#if defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_1_Z)
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_X
				#define Z_PROBE_ALLEN_KEY_DEPLOY_1_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_Y
				#define Z_PROBE_ALLEN_KEY_DEPLOY_1_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_Z
				#define Z_PROBE_ALLEN_KEY_DEPLOY_1_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_1_X, Z_PROBE_ALLEN_KEY_DEPLOY_1_Y, Z_PROBE_ALLEN_KEY_DEPLOY_1_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE));
		#endif
		#if defined(Z_PROBE_ALLEN_KEY_DEPLOY_2_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_2_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_2_Z)
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_X
				#define Z_PROBE_ALLEN_KEY_DEPLOY_2_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_Y
				#define Z_PROBE_ALLEN_KEY_DEPLOY_2_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_Z
				#define Z_PROBE_ALLEN_KEY_DEPLOY_2_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_2_X, Z_PROBE_ALLEN_KEY_DEPLOY_2_Y, Z_PROBE_ALLEN_KEY_DEPLOY_2_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE));
		#endif
		#if defined(Z_PROBE_ALLEN_KEY_DEPLOY_3_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_3_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_3_Z)
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_X
				#define Z_PROBE_ALLEN_KEY_DEPLOY_3_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_Y
				#define Z_PROBE_ALLEN_KEY_DEPLOY_3_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_Z
				#define Z_PROBE_ALLEN_KEY_DEPLOY_3_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_3_X, Z_PROBE_ALLEN_KEY_DEPLOY_3_Y, Z_PROBE_ALLEN_KEY_DEPLOY_3_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE));
		#endif
		#if defined(Z_PROBE_ALLEN_KEY_DEPLOY_4_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_4_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_4_Z)
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_X
				#define Z_PROBE_ALLEN_KEY_DEPLOY_4_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_Y
				#define Z_PROBE_ALLEN_KEY_DEPLOY_4_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_Z
				#define Z_PROBE_ALLEN_KEY_DEPLOY_4_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_4_X, Z_PROBE_ALLEN_KEY_DEPLOY_4_Y, Z_PROBE_ALLEN_KEY_DEPLOY_4_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE));
		#endif
		#if defined(Z_PROBE_ALLEN_KEY_DEPLOY_5_X) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_5_Y) || defined(Z_PROBE_ALLEN_KEY_DEPLOY_5_Z)
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_X
				#define Z_PROBE_ALLEN_KEY_DEPLOY_5_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_Y
				#define Z_PROBE_ALLEN_KEY_DEPLOY_5_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_Z
				#define Z_PROBE_ALLEN_KEY_DEPLOY_5_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_DEPLOY_5_X, Z_PROBE_ALLEN_KEY_DEPLOY_5_Y, Z_PROBE_ALLEN_KEY_DEPLOY_5_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE));
		#endif
	}

	void run_stow_moves_script() {
		#if defined(Z_PROBE_ALLEN_KEY_STOW_1_X) || defined(Z_PROBE_ALLEN_KEY_STOW_1_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_1_Z)
			#ifndef Z_PROBE_ALLEN_KEY_STOW_1_X
				#define Z_PROBE_ALLEN_KEY_STOW_1_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_1_Y
				#define Z_PROBE_ALLEN_KEY_STOW_1_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_1_Z
				#define Z_PROBE_ALLEN_KEY_STOW_1_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_1_X, Z_PROBE_ALLEN_KEY_STOW_1_Y, Z_PROBE_ALLEN_KEY_STOW_1_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE));
		#endif
		#if defined(Z_PROBE_ALLEN_KEY_STOW_2_X) || defined(Z_PROBE_ALLEN_KEY_STOW_2_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_2_Z)
			#ifndef Z_PROBE_ALLEN_KEY_STOW_2_X
				#define Z_PROBE_ALLEN_KEY_STOW_2_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_2_Y
				#define Z_PROBE_ALLEN_KEY_STOW_2_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_2_Z
				#define Z_PROBE_ALLEN_KEY_STOW_2_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_2_X, Z_PROBE_ALLEN_KEY_STOW_2_Y, Z_PROBE_ALLEN_KEY_STOW_2_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE));
		#endif
		#if defined(Z_PROBE_ALLEN_KEY_STOW_3_X) || defined(Z_PROBE_ALLEN_KEY_STOW_3_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_3_Z)
			#ifndef Z_PROBE_ALLEN_KEY_STOW_3_X
				#define Z_PROBE_ALLEN_KEY_STOW_3_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_3_Y
				#define Z_PROBE_ALLEN_KEY_STOW_3_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_3_Z
				#define Z_PROBE_ALLEN_KEY_STOW_3_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_3_X, Z_PROBE_ALLEN_KEY_STOW_3_Y, Z_PROBE_ALLEN_KEY_STOW_3_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE));
		#endif
		#if defined(Z_PROBE_ALLEN_KEY_STOW_4_X) || defined(Z_PROBE_ALLEN_KEY_STOW_4_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_4_Z)
			#ifndef Z_PROBE_ALLEN_KEY_STOW_4_X
				#define Z_PROBE_ALLEN_KEY_STOW_4_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_4_Y
				#define Z_PROBE_ALLEN_KEY_STOW_4_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_4_Z
				#define Z_PROBE_ALLEN_KEY_STOW_4_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_4_X, Z_PROBE_ALLEN_KEY_STOW_4_Y, Z_PROBE_ALLEN_KEY_STOW_4_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE));
		#endif
		#if defined(Z_PROBE_ALLEN_KEY_STOW_5_X) || defined(Z_PROBE_ALLEN_KEY_STOW_5_Y) || defined(Z_PROBE_ALLEN_KEY_STOW_5_Z)
			#ifndef Z_PROBE_ALLEN_KEY_STOW_5_X
				#define Z_PROBE_ALLEN_KEY_STOW_5_X current_position[X_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_5_Y
				#define Z_PROBE_ALLEN_KEY_STOW_5_Y current_position[Y_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_5_Z
				#define Z_PROBE_ALLEN_KEY_STOW_5_Z current_position[Z_AXIS]
			#endif
			#ifndef Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE
				#define Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE 0.0
			#endif
			do_blocking_move_to(Z_PROBE_ALLEN_KEY_STOW_5_X, Z_PROBE_ALLEN_KEY_STOW_5_Y, Z_PROBE_ALLEN_KEY_STOW_5_Z, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE));
		#endif
	}

#endif

#if HAS_BED_PROBE

 // TRIGGERED_WHEN_STOWED_TEST can easily be extended to servo probes, ... if needed.
	#if ENABLED(PROBE_IS_TRIGGERED_WHEN_STOWED_TEST)
		#if ENABLED(Z_MIN_PROBE_ENDSTOP)
			#define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING)
		#else
			#define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING)
		#endif
	#endif

	#define DEPLOY_PROBE() set_probe_deployed(true)
	#define STOW_PROBE() set_probe_deployed(false)

	#if ENABLED(BLTOUCH)
		FORCE_INLINE void set_bltouch_deployed(const bool &deploy) {
			servo[Z_ENDSTOP_SERVO_NR].move(deploy ? BLTOUCH_DEPLOY : BLTOUCH_STOW);
			#if ENABLED(DEBUG_LEVELING_FEATURE)
				if (DEBUGGING(LEVELING)) {
					SERIAL_ECHOPAIR("set_bltouch_deployed(", deploy);
					SERIAL_CHAR(')');
					SERIAL_EOL;
				}
			#endif
		}
	#endif

	// returns false for ok and true for failure
	static bool set_probe_deployed(bool deploy) {

		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) {
				DEBUG_POS("set_probe_deployed", current_position);
				SERIAL_ECHOLNPAIR("deploy: ", deploy);
			}
		#endif

		if (endstops.z_probe_enabled == deploy) return false;

		// Make room for probe
		do_probe_raise(_Z_CLEARANCE_DEPLOY_PROBE);

		// When deploying make sure BLTOUCH is not already triggered
		#if ENABLED(BLTOUCH)
			if (deploy && TEST_BLTOUCH()) { stop(); return true; }
		#elif ENABLED(Z_PROBE_SLED)
			if (axis_unhomed_error(true, false, false)) { stop(); return true; }
		#elif ENABLED(Z_PROBE_ALLEN_KEY)
			if (axis_unhomed_error(true, true,	true )) { stop(); return true; }
		#endif

		float oldXpos = current_position[X_AXIS],
					oldYpos = current_position[Y_AXIS];

		#ifdef _TRIGGERED_WHEN_STOWED_TEST

			// If endstop is already false, the Z probe is deployed
			if (_TRIGGERED_WHEN_STOWED_TEST == deploy) {		 // closed after the probe specific actions.
																											 // Would a goto be less ugly?
				//while (!_TRIGGERED_WHEN_STOWED_TEST) idle(); // would offer the opportunity
																											 // for a triggered when stowed manual probe.

				if (!deploy) endstops.enable_z_probe(false); // Switch off triggered when stowed probes early
																										 // otherwise an Allen-Key probe can't be stowed.
		#endif

				#if ENABLED(Z_PROBE_SLED)

					dock_sled(!deploy);

				#elif HAS_Z_SERVO_ENDSTOP && DISABLED(BLTOUCH)

					servo[Z_ENDSTOP_SERVO_NR].move(z_servo_angle[deploy ? 0 : 1]);

				#elif ENABLED(Z_PROBE_ALLEN_KEY)

					deploy ? run_deploy_moves_script() : run_stow_moves_script();

				#endif

		#ifdef _TRIGGERED_WHEN_STOWED_TEST
			} // _TRIGGERED_WHEN_STOWED_TEST == deploy

			if (_TRIGGERED_WHEN_STOWED_TEST == deploy) { // State hasn't changed?

				if (IsRunning()) {
					SERIAL_ERROR_START;
					SERIAL_ERRORLNPGM("Z-Probe failed");
					NOOP;
				}
				stop();
				return true;

			} // _TRIGGERED_WHEN_STOWED_TEST == deploy

		#endif

		do_blocking_move_to(oldXpos, oldYpos, current_position[Z_AXIS]); // return to position before deploy
		endstops.enable_z_probe(deploy);
		return false;
	}

	static void do_probe_move(float z, float fr_mm_m) {
		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) DEBUG_POS(">>> do_probe_move", current_position);
		#endif

		// Deploy BLTouch at the start of any probe
		#if ENABLED(BLTOUCH)
			set_bltouch_deployed(true);
		#endif

		// Move down until probe triggered
		do_blocking_move_to_z(LOGICAL_Z_POSITION(z), MMM_TO_MMS(fr_mm_m));

		// Retract BLTouch immediately after a probe
		#if ENABLED(BLTOUCH)
			set_bltouch_deployed(false);
		#endif

		// Clear endstop flags
		endstops.hit_on_purpose();

		// Get Z where the steppers were interrupted
		set_current_from_steppers_for_axis(Z_AXIS);

		// Tell the planner where we actually are
		SYNC_PLAN_POSITION_KINEMATIC();

		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) DEBUG_POS("<<< do_probe_move", current_position);
		#endif
	}

	// Do a single Z probe and return with current_position[Z_AXIS]
	// at the height where the probe triggered.
	static float run_z_probe() {

		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) DEBUG_POS(">>> run_z_probe", current_position);
		#endif

		// Prevent stepper_inactive_time from running out and EXTRUDER_RUNOUT_PREVENT from extruding
		refresh_cmd_timeout();

		#if ENABLED(PROBE_DOUBLE_TOUCH)

			// Do a first probe at the fast speed
			do_probe_move(-(Z_MAX_LENGTH) - 10, Z_PROBE_SPEED_FAST);

			#if ENABLED(DEBUG_LEVELING_FEATURE)
				float first_probe_z = current_position[Z_AXIS];
				if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("1st Probe Z:", first_probe_z);
			#endif

			// move up by the bump distance
			do_blocking_move_to_z(current_position[Z_AXIS] + home_bump_mm(Z_AXIS), MMM_TO_MMS(Z_PROBE_SPEED_FAST));

		#else

			// If the nozzle is above the travel height then
			// move down quickly before doing the slow probe
			float z = LOGICAL_Z_POSITION(Z_CLEARANCE_BETWEEN_PROBES);
			if (zprobe_zoffset < 0) z -= zprobe_zoffset;
			if (z < current_position[Z_AXIS])
				do_blocking_move_to_z(z, MMM_TO_MMS(Z_PROBE_SPEED_FAST));

		#endif

		// move down slowly to find bed
		do_probe_move(-(Z_MAX_LENGTH) - 10, Z_PROBE_SPEED_SLOW);

		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) DEBUG_POS("<<< run_z_probe", current_position);
		#endif

		// Debug: compare probe heights
		#if ENABLED(PROBE_DOUBLE_TOUCH) && ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) {
				SERIAL_ECHOPAIR("2nd Probe Z:", current_position[Z_AXIS]);
				SERIAL_ECHOLNPAIR(" Discrepancy:", first_probe_z - current_position[Z_AXIS]);
			}
		#endif
		return current_position[Z_AXIS];
	}

	//
	// - Move to the given XY
	// - Deploy the probe, if not already deployed
	// - Probe the bed, get the Z position
	// - Depending on the 'stow' flag
	//	 - Stow the probe, or
	//	 - Raise to the BETWEEN height
	// - Return the probed Z position
	//
	static float probe_pt(const float &x, const float &y, bool stow = true, int verbose_level = 1) {
		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) {
				SERIAL_ECHOPAIR(">>> probe_pt(", x);
				SERIAL_ECHOPAIR(", ", y);
				SERIAL_ECHOPAIR(", ", stow ? "" : "no ");
				SERIAL_ECHOLNPGM("stow)");
				DEBUG_POS("", current_position);
			}
		#endif

		float old_feedrate_mm_s = feedrate_mm_s;

		// Ensure a minimum height before moving the probe
		do_probe_raise(Z_CLEARANCE_BETWEEN_PROBES);

		feedrate_mm_s = XY_PROBE_FEEDRATE_MM_S;

		// Move the probe to the given XY
		do_blocking_move_to_xy(x - (X_PROBE_OFFSET_FROM_EXTRUDER), y - (Y_PROBE_OFFSET_FROM_EXTRUDER));

		if (DEPLOY_PROBE()) return NAN;

		float measured_z = run_z_probe();

		if (!stow)
			do_probe_raise(Z_CLEARANCE_BETWEEN_PROBES);
		else
			if (STOW_PROBE()) return NAN;

		if (verbose_level > 2) {
			SERIAL_PROTOCOLPGM("Bed X: ");
			SERIAL_PROTOCOL_F(x, 3);
			SERIAL_PROTOCOLPGM(" Y: ");
			SERIAL_PROTOCOL_F(y, 3);
			SERIAL_PROTOCOLPGM(" Z: ");
			SERIAL_PROTOCOL_F(measured_z, 3);
			SERIAL_EOL;
		}

		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< probe_pt");
		#endif

		feedrate_mm_s = old_feedrate_mm_s;

		return measured_z;
	}

#endif // HAS_BED_PROBE

#if PLANNER_LEVELING
	/**
	 * Turn bed leveling on or off, fixing the current
	 * position as-needed.
	 *
	 * Disable: Current position = physical position
	 *	Enable: Current position = "unleveled" physical position
	 */
	void set_bed_leveling_enabled(bool enable=true) {

	}

	#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)

		void set_z_fade_height(const float zfh) {
			planner.z_fade_height = zfh;
			planner.inverse_z_fade_height = RECIPROCAL(zfh);

			if (planner.abl_enabled) {
				set_current_from_steppers_for_axis(
					#if ABL_PLANAR
						ALL_AXES
					#else
						Z_AXIS
					#endif
				);
			}
		}

	#endif // LEVELING_FADE_HEIGHT

	/**
	 * Reset calibration results to zero.
	 */
	void reset_bed_level() {
		planner.abl_enabled = false;
		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("reset_bed_level");
		#endif
		#if ABL_PLANAR
			planner.bed_level_matrix.set_to_identity();
		#elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
			for (uint8_t x = 0; x < ABL_GRID_POINTS_X; x++)
				for (uint8_t y = 0; y < ABL_GRID_POINTS_Y; y++)
					bed_level_grid[x][y] = 1000.0;
		#endif
	}

#endif // PLANNER_LEVELING

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

	/**
	 * Extrapolate a single point from its neighbors
	 */
	static void extrapolate_one_point(uint8_t x, uint8_t y, int8_t xdir, int8_t ydir) {
		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) {
				SERIAL_ECHOPGM("Extrapolate [");
				if (x < 10) SERIAL_CHAR(' ');
				SERIAL_ECHO((int)x);
				SERIAL_CHAR(xdir ? (xdir > 0 ? '+' : '-') : ' ');
				SERIAL_CHAR(' ');
				if (y < 10) SERIAL_CHAR(' ');
				SERIAL_ECHO((int)y);
				SERIAL_CHAR(ydir ? (ydir > 0 ? '+' : '-') : ' ');
				SERIAL_CHAR(']');
			}
		#endif
		if (bed_level_grid[x][y] < 999.0) {
			#if ENABLED(DEBUG_LEVELING_FEATURE)
				if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM(" (done)");
			#endif
			return;	// Don't overwrite good values.
		}
		SERIAL_EOL;

		// Get X neighbors, Y neighbors, and XY neighbors
		float a1 = bed_level_grid[x + xdir][y], a2 = bed_level_grid[x + xdir * 2][y],
					b1 = bed_level_grid[x][y + ydir], b2 = bed_level_grid[x][y + ydir * 2],
					c1 = bed_level_grid[x + xdir][y + ydir], c2 = bed_level_grid[x + xdir * 2][y + ydir * 2];

		// Treat far unprobed points as zero, near as equal to far
		if (a2 > 999.0) a2 = 0.0; if (a1 > 999.0) a1 = a2;
		if (b2 > 999.0) b2 = 0.0; if (b1 > 999.0) b1 = b2;
		if (c2 > 999.0) c2 = 0.0; if (c1 > 999.0) c1 = c2;

		float a = 2 * a1 - a2, b = 2 * b1 - b2, c = 2 * c1 - c2;

		// Take the average intstead of the median
		bed_level_grid[x][y] = (a + b + c) / 3.0;

		// Median is robust (ignores outliers).
		// bed_level_grid[x][y] = (a < b) ? ((b < c) ? b : (c < a) ? a : c)
		//																: ((c < b) ? b : (a < c) ? a : c);
	}

	//Enable this if your SCARA uses 180° of total area
	//#define EXTRAPOLATE_FROM_EDGE

	#if ENABLED(EXTRAPOLATE_FROM_EDGE)
		#if ABL_GRID_POINTS_X < ABL_GRID_POINTS_Y
			#define HALF_IN_X
		#elif ABL_GRID_POINTS_Y < ABL_GRID_POINTS_X
			#define HALF_IN_Y
		#endif
	#endif

	/**
	 * Fill in the unprobed points (corners of circular print surface)
	 * using linear extrapolation, away from the center.
	 */
	static void extrapolate_unprobed_bed_level() {
		#ifdef HALF_IN_X
			const uint8_t ctrx2 = 0, xlen = ABL_GRID_POINTS_X - 1;
		#else
			const uint8_t ctrx1 = (ABL_GRID_POINTS_X - 1) / 2, // left-of-center
										ctrx2 = ABL_GRID_POINTS_X / 2,			 // right-of-center
										xlen = ctrx1;
		#endif

		#ifdef HALF_IN_Y
			const uint8_t ctry2 = 0, ylen = ABL_GRID_POINTS_Y - 1;
		#else
			const uint8_t ctry1 = (ABL_GRID_POINTS_Y - 1) / 2, // top-of-center
										ctry2 = ABL_GRID_POINTS_Y / 2,			 // bottom-of-center
										ylen = ctry1;
		#endif

		for (uint8_t xo = 0; xo <= xlen; xo++)
			for (uint8_t yo = 0; yo <= ylen; yo++) {
				uint8_t x2 = ctrx2 + xo, y2 = ctry2 + yo;
				#ifndef HALF_IN_X
					uint8_t x1 = ctrx1 - xo;
				#endif
				#ifndef HALF_IN_Y
					uint8_t y1 = ctry1 - yo;
					#ifndef HALF_IN_X
						extrapolate_one_point(x1, y1, +1, +1);	 //	left-below + +
					#endif
					extrapolate_one_point(x2, y1, -1, +1);		 // right-below - +
				#endif
				#ifndef HALF_IN_X
					extrapolate_one_point(x1, y2, +1, -1);		 //	left-above + -
				#endif
				extrapolate_one_point(x2, y2, -1, -1);			 // right-above - -
			}

	}

	/**
	 * Print calibration results for plotting or manual frame adjustment.
	 */
	static void print_bed_level() {
		SERIAL_ECHOPGM("Bilinear Leveling Grid:\n ");
		for (uint8_t x = 0; x < ABL_GRID_POINTS_X; x++) {
			SERIAL_PROTOCOLPGM("		");
			if (x < 10) SERIAL_PROTOCOLCHAR(' ');
			SERIAL_PROTOCOL((int)x);
		}
		SERIAL_EOL;
		for (uint8_t y = 0; y < ABL_GRID_POINTS_Y; y++) {
			if (y < 10) SERIAL_PROTOCOLCHAR(' ');
			SERIAL_PROTOCOL((int)y);
			for (uint8_t x = 0; x < ABL_GRID_POINTS_X; x++) {
				SERIAL_PROTOCOLCHAR(' ');
				float offset = bed_level_grid[x][y];
				if (offset < 999.0) {
					if (offset > 0) SERIAL_CHAR('+');
					SERIAL_PROTOCOL_F(offset, 2);
				}
				else
					SERIAL_PROTOCOLPGM(" ====");
			}
			SERIAL_EOL;
		}
		SERIAL_EOL;
	}

	#if ENABLED(ABL_BILINEAR_SUBDIVISION)
		#define ABL_GRID_POINTS_VIRT_X (ABL_GRID_POINTS_X - 1) * (BILINEAR_SUBDIVISIONS) + 1
		#define ABL_GRID_POINTS_VIRT_Y (ABL_GRID_POINTS_Y - 1) * (BILINEAR_SUBDIVISIONS) + 1
		float bed_level_grid_virt[ABL_GRID_POINTS_VIRT_X][ABL_GRID_POINTS_VIRT_Y];
		float bed_level_grid_virt_temp[ABL_GRID_POINTS_X + 2][ABL_GRID_POINTS_Y + 2]; //temporary for calculation (maybe dynamical?)
		int bilinear_grid_spacing_virt[2] = { 0 };

		static void bed_level_virt_print() {
			SERIAL_ECHOLNPGM("Subdivided with CATMULL ROM Leveling Grid:");
			for (uint8_t x = 0; x < ABL_GRID_POINTS_VIRT_X; x++) {
				SERIAL_PROTOCOLPGM("			 ");
				if (x < 10) SERIAL_PROTOCOLCHAR(' ');
				SERIAL_PROTOCOL((int)x);
			}
			SERIAL_EOL;
			for (uint8_t y = 0; y < ABL_GRID_POINTS_VIRT_Y; y++) {
				if (y < 10) SERIAL_PROTOCOLCHAR(' ');
				SERIAL_PROTOCOL((int)y);
				for (uint8_t x = 0; x < ABL_GRID_POINTS_VIRT_X; x++) {
					SERIAL_PROTOCOLCHAR(' ');
					float offset = bed_level_grid_virt[x][y];
					if (offset < 999.0) {
						if (offset > 0) SERIAL_CHAR('+');
						SERIAL_PROTOCOL_F(offset, 5);
					}
					else
						SERIAL_PROTOCOLPGM(" ====");
				}
				SERIAL_EOL;
			}
			SERIAL_EOL;
		}
		#define LINEAR_EXTRAPOLATION(E, I) (E * 2 - I)
		static void bed_level_virt_prepare() {
			for (uint8_t y = 1; y <= ABL_GRID_POINTS_Y; y++) {

				for (uint8_t x = 1; x <= ABL_GRID_POINTS_X; x++)
					bed_level_grid_virt_temp[x][y] = bed_level_grid[x - 1][y - 1];

				bed_level_grid_virt_temp[0][y] = LINEAR_EXTRAPOLATION(
					bed_level_grid_virt_temp[1][y],
					bed_level_grid_virt_temp[2][y]
				);

				bed_level_grid_virt_temp[(ABL_GRID_POINTS_X + 2) - 1][y] =
					LINEAR_EXTRAPOLATION(
						bed_level_grid_virt_temp[(ABL_GRID_POINTS_X + 2) - 2][y],
						bed_level_grid_virt_temp[(ABL_GRID_POINTS_X + 2) - 3][y]
					);
			}
			for (uint8_t x = 0; x < ABL_GRID_POINTS_X + 2; x++) {
				bed_level_grid_virt_temp[x][0] = LINEAR_EXTRAPOLATION(
					bed_level_grid_virt_temp[x][1],
					bed_level_grid_virt_temp[x][2]
				);
				bed_level_grid_virt_temp[x][(ABL_GRID_POINTS_Y + 2) - 1] =
					LINEAR_EXTRAPOLATION(
						bed_level_grid_virt_temp[x][(ABL_GRID_POINTS_Y + 2) - 2],
						bed_level_grid_virt_temp[x][(ABL_GRID_POINTS_Y + 2) - 3]
					);
			}
		}
		static float bed_level_virt_cmr(const float p[4], const uint8_t i, const float t) {
			return (
					p[i-1] * -t * sq(1 - t)
				+ p[i]	 * (2 - 5 * sq(t) + 3 * t * sq(t))
				+ p[i+1] * t * (1 + 4 * t - 3 * sq(t))
				- p[i+2] * sq(t) * (1 - t)
			) * 0.5;
		}
		static float bed_level_virt_2cmr(const uint8_t x, const uint8_t y, const float &tx, const float &ty) {
			float row[4], column[4];
			for (uint8_t i = 0; i < 4; i++) {
				for (uint8_t j = 0; j < 4; j++) // can be memcopy or through memory access
					column[j] = bed_level_grid_virt_temp[i + x - 1][j + y - 1];
				row[i] = bed_level_virt_cmr(column, 1, ty);
			}
			return bed_level_virt_cmr(row, 1, tx);
		}
		static void bed_level_virt_interpolate() {
			for (uint8_t y = 0; y < ABL_GRID_POINTS_Y; y++)
				for (uint8_t x = 0; x < ABL_GRID_POINTS_X; x++)
					for (uint8_t ty = 0; ty < BILINEAR_SUBDIVISIONS; ty++)
						for (uint8_t tx = 0; tx < BILINEAR_SUBDIVISIONS; tx++) {
							if ((ty && y == ABL_GRID_POINTS_Y - 1) || (tx && x == ABL_GRID_POINTS_X - 1))
								continue;
							bed_level_grid_virt[x * (BILINEAR_SUBDIVISIONS) + tx][y * (BILINEAR_SUBDIVISIONS) + ty] =
								bed_level_virt_2cmr(
									x + 1,
									y + 1,
									(float)tx / (BILINEAR_SUBDIVISIONS),
									(float)ty / (BILINEAR_SUBDIVISIONS)
								);
						}
		}
	#endif // ABL_BILINEAR_SUBDIVISION
#endif // AUTO_BED_LEVELING_BILINEAR


/**
 * Home an individual linear axis
 */
static void do_homing_move(const AxisEnum axis, float distance, float fr_mm_s=0.0) {

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			SERIAL_ECHOPAIR(">>> do_homing_move(", axis_codes[axis]);
			SERIAL_ECHOPAIR(", ", distance);
			SERIAL_ECHOPAIR(", ", fr_mm_s);
			SERIAL_CHAR(')');
			SERIAL_EOL;
		}
	#endif

	#if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
		bool deploy_bltouch = (axis == Z_AXIS && distance < 0);
		if (deploy_bltouch) set_bltouch_deployed(true);
	#endif

	// Tell the planner we're at Z=0
	current_position[axis] = 0;

	sync_plan_position();
	current_position[axis] = distance;
	planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], active_extruder);


	stepper.synchronize();

	#if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
		if (deploy_bltouch) set_bltouch_deployed(false);
	#endif

	endstops.hit_on_purpose();

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			SERIAL_ECHOPAIR("<<< do_homing_move(", axis_codes[axis]);
			SERIAL_CHAR(')');
			SERIAL_EOL;
		}
	#endif
}

/**
 * Home an individual "raw axis" to its endstop.
 * This applies to XYZ on Cartesian and Core robots, and
 * to the individual ABC steppers on DELTA and SCARA.
 *
 * At the end of the procedure the axis is marked as
 * homed and the current position of that axis is updated.
 * Kinematic robots should wait till all axes are homed
 * before updating the current position.
 */

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

static void homeaxis(AxisEnum axis) {

	#define CAN_HOME(A) \
		(axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
	if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;


	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			SERIAL_ECHOPAIR(">>> homeaxis(", axis_codes[axis]);
			SERIAL_CHAR(')');
			SERIAL_EOL;
		}
	#endif

	int axis_home_dir = home_dir(axis);

	// Homing Z towards the bed? Deploy the Z probe or endstop.
	#if HOMING_Z_WITH_PROBE
		if (axis == Z_AXIS && DEPLOY_PROBE()) return;
	#endif

	// Set a flag for Z motor locking
	#if ENABLED(Z_DUAL_ENDSTOPS)
		if (axis == Z_AXIS) stepper.set_homing_flag(true);
	#endif

	// Fast move towards endstop until triggered
	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Home 1 Fast:");
	#endif
	do_homing_move(axis, 1.5 * max_length(axis) * axis_home_dir);

	// When homing Z with probe respect probe clearance
	const float bump = axis_home_dir * (
		#if HOMING_Z_WITH_PROBE
			(axis == Z_AXIS) ? max(Z_CLEARANCE_BETWEEN_PROBES, home_bump_mm(Z_AXIS)) :
		#endif
		home_bump_mm(axis)
	);

	// If a second homing move is configured...
	if (bump) {
		// Move away from the endstop by the axis HOME_BUMP_MM
		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Move Away:");
		#endif
		do_homing_move(axis, -bump);

		// Slow move towards endstop until triggered
		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Home 2 Slow:");
		#endif
		do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
	}

	#if ENABLED(Z_DUAL_ENDSTOPS)
		if (axis == Z_AXIS) {
			float adj = fabs(z_endstop_adj);
			bool lockZ1;
			if (axis_home_dir > 0) {
				adj = -adj;
				lockZ1 = (z_endstop_adj > 0);
			}
			else
				lockZ1 = (z_endstop_adj < 0);

			if (lockZ1) stepper.set_z_lock(true); else stepper.set_z2_lock(true);

			// Move to the adjusted endstop height
			do_homing_move(axis, adj);

			if (lockZ1) stepper.set_z_lock(false); else stepper.set_z2_lock(false);
			stepper.set_homing_flag(false);
		} // Z_AXIS
	#endif

    // For cartesian/core machines,
    // set the axis to its home position
    set_axis_is_at_home(axis);
    sync_plan_position();

    destination[axis] = current_position[axis];

    #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> AFTER set_axis_is_at_home", current_position);
    #endif


	// Put away the Z probe
	#if HOMING_Z_WITH_PROBE
		if (axis == Z_AXIS && STOW_PROBE()) return;
	#endif

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			SERIAL_ECHOPAIR("<<< homeaxis(", axis_codes[axis]);
			SERIAL_CHAR(')');
			SERIAL_EOL;
		}
	#endif
} // homeaxis()

#if ENABLED(FWRETRACT)

	void retract(bool retracting, bool swapping = false) {

		if (retracting == retracted[active_extruder]) return;

		float old_feedrate_mm_s = feedrate_mm_s;

		set_destination_to_current();

		if (retracting) {

			feedrate_mm_s = retract_feedrate_mm_s;
			current_position[E_AXIS] += (swapping ? retract_length_swap : retract_length) / volumetric_multiplier[active_extruder];
			sync_plan_position_e();
			prepare_move_to_destination();

			if (retract_zlift > 0.01) {
				current_position[Z_AXIS] -= retract_zlift;
				SYNC_PLAN_POSITION_KINEMATIC();
				prepare_move_to_destination();
			}
		}
		else {

			if (retract_zlift > 0.01) {
				current_position[Z_AXIS] += retract_zlift;
				SYNC_PLAN_POSITION_KINEMATIC();
			}

			feedrate_mm_s = retract_recover_feedrate_mm_s;
			float move_e = swapping ? retract_length_swap + retract_recover_length_swap : retract_length + retract_recover_length;
			current_position[E_AXIS] -= move_e / volumetric_multiplier[active_extruder];
			sync_plan_position_e();
			prepare_move_to_destination();
		}

		feedrate_mm_s = old_feedrate_mm_s;
		retracted[active_extruder] = retracting;

	} // retract()

#endif // FWRETRACT


/**
 * ***************************************************************************
 * ***************************** G-CODE HANDLING *****************************
 * ***************************************************************************
 */

/**
 * Set XYZE destination and feedrate from the current GCode command
 *
 *	- Set destination from included axis codes
 *	- Set to current for missing axis codes
 *	- Set the feedrate, if included
 */
void gcode_get_destination() {
	LOOP_XYZE(i) {
		if (code_seen(axis_codes[i]))
			destination[i] = code_value_axis_units(i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
		else
			destination[i] = current_position[i];
	}

	if (code_seen('F') && code_value_linear_units() > 0.0)
		feedrate_mm_s = MMM_TO_MMS(code_value_linear_units());

	#if ENABLED(PRINTCOUNTER)
		if (!DEBUGGING(DRYRUN))
			print_job_timer.incFilamentUsed(destination[E_AXIS] - current_position[E_AXIS]);
	#endif

}

void unknown_command_error() {
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR(MSG_UNKNOWN_COMMAND, current_command);
	SERIAL_CHAR('"');
	SERIAL_EOL;
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

	/**
	 * Output a "busy" message at regular intervals
	 * while the machine is not accepting commands.
	 */
	void host_keepalive() {
		millis_t ms = millis();
		if (host_keepalive_interval && busy_state != NOT_BUSY) {
			if (PENDING(ms, next_busy_signal_ms)) return;
			switch (busy_state) {
				case IN_HANDLER:
				case IN_PROCESS:
					SERIAL_ECHO_START;
					SERIAL_ECHOLNPGM(MSG_BUSY_PROCESSING);
					break;
				case PAUSED_FOR_USER:
					SERIAL_ECHO_START;
					SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_USER);
					break;
				case PAUSED_FOR_INPUT:
					SERIAL_ECHO_START;
					SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_INPUT);
					break;
				default:
					break;
			}
		}
		next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
	}

#endif //HOST_KEEPALIVE_FEATURE

bool position_is_reachable(float target[XYZ]
	#if HAS_BED_PROBE
		, bool by_probe=false
	#endif
) {
	float dx = RAW_X_POSITION(target[X_AXIS]),
				dy = RAW_Y_POSITION(target[Y_AXIS]);

	#if HAS_BED_PROBE
		if (by_probe) {
			dx -= X_PROBE_OFFSET_FROM_EXTRUDER;
			dy -= Y_PROBE_OFFSET_FROM_EXTRUDER;
		}
	#endif

    const float dz = RAW_Z_POSITION(target[Z_AXIS]);
    return dx >= X_MIN_POS - 0.0001 && dx <= X_MAX_POS + 0.0001
            && dy >= Y_MIN_POS - 0.0001 && dy <= Y_MAX_POS + 0.0001
            && dz >= Z_MIN_POS - 0.0001 && dz <= Z_MAX_POS + 0.0001;
}

/**************************************************
 ***************** GCode Handlers *****************
 **************************************************/

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1() {
	if (IsRunning()) {
		gcode_get_destination(); // For X Y Z E F

		#if ENABLED(FWRETRACT)

			if (autoretract_enabled && !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
				float echange = destination[E_AXIS] - current_position[E_AXIS];
				// Is this move an attempt to retract or recover?
				if ((echange < -MIN_RETRACT && !retracted[active_extruder]) || (echange > MIN_RETRACT && retracted[active_extruder])) {
					current_position[E_AXIS] = destination[E_AXIS]; // hide the slicer-generated retract/recover from calculations
					sync_plan_position_e();	// AND from the planner
					retract(!retracted[active_extruder]);
					return;
				}
			}

		#endif //FWRETRACT

		prepare_move_to_destination();

	}
}

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 *
 * This command has two forms: IJ-form and R-form.
 *
 *	- I specifies an X offset. J specifies a Y offset.
 *		At least one of the IJ parameters is required.
 *		X and Y can be omitted to do a complete circle.
 *		The given XY is not error-checked. The arc ends
 *		 based on the angle of the destination.
 *		Mixing I or J with R will throw an error.
 *
 *	- R specifies the radius. X or Y is required.
 *		Omitting both X and Y will throw an error.
 *		X or Y must differ from the current XY.
 *		Mixing R with I or J will throw an error.
 *
 *	Examples:
 *
 *		G2 I10					 ; CW circle centered at X+10
 *		G3 X20 Y12 R14	 ; CCW circle with r=14 ending at X20 Y12
 */
#if ENABLED(ARC_SUPPORT)
	inline void gcode_G2_G3(bool clockwise) {
		if (IsRunning()) {

			#if ENABLED(SF_ARC_FIX)
				bool relative_mode_backup = relative_mode;
				relative_mode = true;
			#endif

			gcode_get_destination();

			#if ENABLED(SF_ARC_FIX)
				relative_mode = relative_mode_backup;
			#endif

			float arc_offset[2] = { 0.0, 0.0 };
			if (code_seen('R')) {
				const float r = code_value_axis_units(X_AXIS),
										x1 = current_position[X_AXIS], y1 = current_position[Y_AXIS],
										x2 = destination[X_AXIS], y2 = destination[Y_AXIS];
				if (r && (x2 != x1 || y2 != y1)) {
					const float e = clockwise ^ (r < 0) ? -1 : 1,					 // clockwise -1/1, counterclockwise 1/-1
											dx = x2 - x1, dy = y2 - y1,								 // X and Y differences
											d = HYPOT(dx, dy),													// Linear distance between the points
											h = sqrt(sq(r) - sq(d * 0.5)),							// Distance to the arc pivot-point
											mx = (x1 + x2) * 0.5, my = (y1 + y2) * 0.5, // Point between the two points
											sx = -dy / d, sy = dx / d,									// Slope of the perpendicular bisector
											cx = mx + e * h * sx, cy = my + e * h * sy; // Pivot-point of the arc
					arc_offset[X_AXIS] = cx - x1;
					arc_offset[Y_AXIS] = cy - y1;
				}
			}
			else {
				if (code_seen('I')) arc_offset[X_AXIS] = code_value_axis_units(X_AXIS);
				if (code_seen('J')) arc_offset[Y_AXIS] = code_value_axis_units(Y_AXIS);
			}

			if (arc_offset[0] || arc_offset[1]) {
				// Send an arc to the planner
				plan_arc(destination, arc_offset, clockwise);
				refresh_cmd_timeout();
			}
			else {
				// Bad arguments
				SERIAL_ERROR_START;
				SERIAL_ERRORLNPGM(MSG_ERR_ARC_ARGS);
			}
		}
	}
#endif

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
	millis_t dwell_ms = 0;

	if (code_seen('P')) dwell_ms = code_value_millis(); // milliseconds to wait
	if (code_seen('S')) dwell_ms = code_value_millis_from_seconds(); // seconds to wait

	stepper.synchronize();
	refresh_cmd_timeout();
	dwell_ms += previous_cmd_ms;	// keep track of when we started waiting

	while (PENDING(millis(), dwell_ms)) idle();
}


#if ENABLED(FWRETRACT)

	/**
	 * G10 - Retract filament according to settings of M207
	 * G11 - Recover filament according to settings of M208
	 */
	inline void gcode_G10_G11(bool doRetract=false) {
		#if EXTRUDERS > 1
			if (doRetract) {
				retracted_swap[active_extruder] = (code_seen('S') && code_value_bool()); // checks for swap retract argument
			}
		#endif
		retract(doRetract
		 #if EXTRUDERS > 1
			, retracted_swap[active_extruder]
		 #endif
		);
	}

#endif //FWRETRACT

#if ENABLED(NOZZLE_CLEAN_FEATURE)
	/**
	 * G12: Clean the nozzle
	 */
	inline void gcode_G12() {
		// Don't allow nozzle cleaning without homing first
		if (axis_unhomed_error(true, true, true)) { return; }

		uint8_t const pattern = code_seen('P') ? code_value_ushort() : 0;
		uint8_t const strokes = code_seen('S') ? code_value_ushort() : NOZZLE_CLEAN_STROKES;
		uint8_t const objects = code_seen('T') ? code_value_ushort() : 3;

		Nozzle::clean(pattern, strokes, objects);
	}
#endif

#if ENABLED(INCH_MODE_SUPPORT)
	/**
	 * G20: Set input mode to inches
	 */
	inline void gcode_G20() { set_input_linear_units(LINEARUNIT_INCH); }

	/**
	 * G21: Set input mode to millimeters
	 */
	inline void gcode_G21() { set_input_linear_units(LINEARUNIT_MM); }
#endif

#if ENABLED(NOZZLE_PARK_FEATURE)
	/**
	 * G27: Park the nozzle
	 */
	inline void gcode_G27() {
		// Don't allow nozzle parking without homing first
		if (axis_unhomed_error(true, true, true)) { return; }
		uint8_t const z_action = code_seen('P') ? code_value_ushort() : 0;
		Nozzle::park(z_action);
	}
#endif // NOZZLE_PARK_FEATURE

#if ENABLED(QUICK_HOME)

	static void quick_home_xy() {

		// Pretend the current position is 0,0
		current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
		sync_plan_position();

		int x_axis_home_dir = home_dir(X_AXIS);

		float mlx = max_length(X_AXIS),
					mly = max_length(Y_AXIS),
					mlratio = mlx > mly ? mly / mlx : mlx / mly,
					fr_mm_s = min(homing_feedrate_mm_s[X_AXIS], homing_feedrate_mm_s[Y_AXIS]) * sqrt(sq(mlratio) + 1.0);

		do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir(Y_AXIS), fr_mm_s);
		endstops.hit_on_purpose(); // clear endstop hit flags
		current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;

	}

#endif // QUICK_HOME

#if ENABLED(DEBUG_LEVELING_FEATURE)

	void log_machine_info() {
		SERIAL_ECHOPGM("Machine Type: ");
		#if IS_CORE
			SERIAL_ECHOLNPGM("Core");
		#else
			SERIAL_ECHOLNPGM("Cartesian");
		#endif

		SERIAL_ECHOPGM("Probe: ");
		#if ENABLED(FIX_MOUNTED_PROBE)
			SERIAL_ECHOLNPGM("FIX_MOUNTED_PROBE");
		#elif ENABLED(BLTOUCH)
			SERIAL_ECHOLNPGM("BLTOUCH");
		#elif HAS_Z_SERVO_ENDSTOP
			SERIAL_ECHOLNPGM("SERVO PROBE");
		#elif ENABLED(Z_PROBE_SLED)
			SERIAL_ECHOLNPGM("Z_PROBE_SLED");
		#elif ENABLED(Z_PROBE_ALLEN_KEY)
			SERIAL_ECHOLNPGM("Z_PROBE_ALLEN_KEY");
		#else
			SERIAL_ECHOLNPGM("NONE");
		#endif

		#if HAS_BED_PROBE
			SERIAL_ECHOPAIR("Probe Offset X:", X_PROBE_OFFSET_FROM_EXTRUDER);
			SERIAL_ECHOPAIR(" Y:", Y_PROBE_OFFSET_FROM_EXTRUDER);
			SERIAL_ECHOPAIR(" Z:", zprobe_zoffset);
			#if (X_PROBE_OFFSET_FROM_EXTRUDER > 0)
				SERIAL_ECHOPGM(" (Right");
			#elif (X_PROBE_OFFSET_FROM_EXTRUDER < 0)
				SERIAL_ECHOPGM(" (Left");
			#elif (Y_PROBE_OFFSET_FROM_EXTRUDER != 0)
				SERIAL_ECHOPGM(" (Middle");
			#else
				SERIAL_ECHOPGM(" (Aligned With");
			#endif
			#if (Y_PROBE_OFFSET_FROM_EXTRUDER > 0)
				SERIAL_ECHOPGM("-Back");
			#elif (Y_PROBE_OFFSET_FROM_EXTRUDER < 0)
				SERIAL_ECHOPGM("-Front");
			#elif (X_PROBE_OFFSET_FROM_EXTRUDER != 0)
				SERIAL_ECHOPGM("-Center");
			#endif
			if (zprobe_zoffset < 0)
				SERIAL_ECHOPGM(" & Below");
			else if (zprobe_zoffset > 0)
				SERIAL_ECHOPGM(" & Above");
			else
				SERIAL_ECHOPGM(" & Same Z as");
			SERIAL_ECHOLNPGM(" Nozzle)");
		#endif


	}

#endif // DEBUG_LEVELING_FEATURE


#if ENABLED(Z_SAFE_HOMING)

	inline void home_z_safely() {

		// Disallow Z homing if X or Y are unknown
		if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
			NOOP;
			SERIAL_ECHO_START;
			SERIAL_ECHOLNPGM(MSG_ERR_Z_HOMING);
			return;
		}

		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Z_SAFE_HOMING >>>");
		#endif

		SYNC_PLAN_POSITION_KINEMATIC();

		/**
		 * Move the Z probe (or just the nozzle) to the safe homing point
		 */
		destination[X_AXIS] = LOGICAL_X_POSITION(Z_SAFE_HOMING_X_POINT);
		destination[Y_AXIS] = LOGICAL_Y_POSITION(Z_SAFE_HOMING_Y_POINT);
		destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

		if (position_is_reachable(
					destination
					#if HOMING_Z_WITH_PROBE
						, true
					#endif
				)
		) {

			#if HOMING_Z_WITH_PROBE
				destination[X_AXIS] -= X_PROBE_OFFSET_FROM_EXTRUDER;
				destination[Y_AXIS] -= Y_PROBE_OFFSET_FROM_EXTRUDER;
			#endif

			#if ENABLED(DEBUG_LEVELING_FEATURE)
				if (DEBUGGING(LEVELING)) DEBUG_POS("Z_SAFE_HOMING", destination);
			#endif

			do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);
			HOMEAXIS(Z);
		}
		else {
			NOOP;
			SERIAL_ECHO_START;
			SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
		}

		#if ENABLED(DEBUG_LEVELING_FEATURE)
			if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< Z_SAFE_HOMING");
		#endif
	}

#endif // Z_SAFE_HOMING

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *	None	Home to all axes with no parameters.
 *				With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *	X	 Home to the X endstop
 *	Y	 Home to the Y endstop
 *	Z	 Home to the Z endstop
 *
 */
inline void gcode_G28() {

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			SERIAL_ECHOLNPGM(">>> gcode_G28");
			log_machine_info();
		}
	#endif

	// Wait for planner moves to finish!
	stepper.synchronize();

	#if ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
		extruder_duplication_enabled = false;
	#endif

	setup_for_endstop_or_probe_move();
	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> endstops.enable(true)");
	#endif
	endstops.enable(true); // Enable endstops for next homing move

    bool homeX = code_seen('X'), homeY = code_seen('Y'), homeZ = code_seen('Z');

    home_all_axis = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    set_destination_to_current();

    #if Z_HOME_DIR > 0	// If homing away from BED do Z first

        if (home_all_axis || homeZ) {
            HOMEAXIS(Z);
            #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) DEBUG_POS("> HOMEAXIS(Z)", current_position);
            #endif
        }

    #else

        if (home_all_axis || homeX || homeY) {
            // Raise Z before homing any other axes and z is not already high enough (never lower z)
            destination[Z_AXIS] = LOGICAL_Z_POSITION(Z_HOMING_HEIGHT);
            if (destination[Z_AXIS] > current_position[Z_AXIS]) {

                #if ENABLED(DEBUG_LEVELING_FEATURE)
                    if (DEBUGGING(LEVELING))
                        SERIAL_ECHOLNPAIR("Raise Z (before homing) to ", destination[Z_AXIS]);
                #endif

                do_blocking_move_to_z(destination[Z_AXIS]);
            }
        }

    #endif

    #if ENABLED(QUICK_HOME)

        if (home_all_axis || (homeX && homeY)) quick_home_xy();

    #endif

    #if ENABLED(HOME_Y_BEFORE_X)

        // Home Y
        if (home_all_axis || homeY) {
            HOMEAXIS(Y);
            #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
            #endif
        }

    #endif

    // Home X
    if (home_all_axis || homeX) {

        HOMEAXIS(X);

        #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("> homeX", current_position);
        #endif
    }

    #if DISABLED(HOME_Y_BEFORE_X)
        // Home Y
        if (home_all_axis || homeY) {
            HOMEAXIS(Y);
            #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
            #endif
        }
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
        if (home_all_axis || homeZ) {
            #if ENABLED(Z_SAFE_HOMING)
                home_z_safely();
            #else
                HOMEAXIS(Z);
            #endif
            #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) DEBUG_POS("> (home_all_axis || homeZ) > final", current_position);
            #endif
        } // home_all_axis || homeZ
    #endif // Z_HOME_DIR < 0

    SYNC_PLAN_POSITION_KINEMATIC();

	endstops.not_homing();

	clean_up_after_endstop_or_probe_move();

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< gcode_G28");
	#endif

	report_current_position();
}

#if HAS_PROBING_PROCEDURE

	void out_of_range_error(const char* p_edge) {
		SERIAL_PROTOCOLPGM("?Probe ");
		serialprintPGM(p_edge);
		SERIAL_PROTOCOLLNPGM(" position out of range.");
	}

#endif

#if HAS_BED_PROBE

	/**
	 * G30: Do a single Z probe at the current XY
	 * Usage:
	 *	 G30 <X#> <Y#> <S#>
	 *		 X = Probe X position (default=current probe position)
	 *		 Y = Probe Y position (default=current probe position)
	 *		 S = Stows the probe if 1 (default=1)
	 */
	inline void gcode_G30() {
		float X_probe_location = code_seen('X') ? code_value_axis_units(X_AXIS) : current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER,
					Y_probe_location = code_seen('Y') ? code_value_axis_units(Y_AXIS) : current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;

		float pos[XYZ] = { X_probe_location, Y_probe_location, LOGICAL_Z_POSITION(0) };
		if (!position_is_reachable(pos, true)) return;

		bool stow = code_seen('S') ? code_value_bool() : true;

		// Disable leveling so the planner won't mess with us
		#if PLANNER_LEVELING
			set_bed_leveling_enabled(false);
		#endif

		setup_for_endstop_or_probe_move();

		float measured_z = probe_pt(X_probe_location, Y_probe_location, stow, 1);

		SERIAL_PROTOCOLPGM("Bed X: ");
		SERIAL_PROTOCOL(X_probe_location + 0.0001);
		SERIAL_PROTOCOLPGM(" Y: ");
		SERIAL_PROTOCOL(Y_probe_location + 0.0001);
		SERIAL_PROTOCOLPGM(" Z: ");
		SERIAL_PROTOCOLLN(measured_z + 0.0001);

		clean_up_after_endstop_or_probe_move();

		report_current_position();
	}

	#if ENABLED(Z_PROBE_SLED)

		/**
		 * G31: Deploy the Z probe
		 */
		inline void gcode_G31() { DEPLOY_PROBE(); }

		/**
		 * G32: Stow the Z probe
		 */
		inline void gcode_G32() { STOW_PROBE(); }

	#endif // Z_PROBE_SLED

#endif // HAS_BED_PROBE

#if ENABLED(G38_PROBE_TARGET)

	static bool G38_run_probe() {

		bool G38_pass_fail = false;

		// Get direction of move and retract
		float retract_mm[XYZ];
		LOOP_XYZ(i) {
			float dist = destination[i] - current_position[i];
			retract_mm[i] = fabs(dist) < G38_MINIMUM_MOVE ? 0 : home_bump_mm((AxisEnum)i) * (dist > 0 ? -1 : 1);
		}

		stepper.synchronize();	// wait until the machine is idle

		// Move until destination reached or target hit
		endstops.enable(true);
		G38_move = true;
		G38_endstop_hit = false;
		prepare_move_to_destination();
		stepper.synchronize();
		G38_move = false;

		endstops.hit_on_purpose();
		set_current_from_steppers_for_axis(ALL_AXES);
		SYNC_PLAN_POSITION_KINEMATIC();

		// Only do remaining moves if target was hit
		if (G38_endstop_hit) {

			G38_pass_fail = true;

			// Move away by the retract distance
			set_destination_to_current();
			LOOP_XYZ(i) destination[i] += retract_mm[i];
			endstops.enable(false);
			prepare_move_to_destination();
			stepper.synchronize();

			feedrate_mm_s /= 4;

			// Bump the target more slowly
			LOOP_XYZ(i) destination[i] -= retract_mm[i] * 2;

			endstops.enable(true);
			G38_move = true;
			prepare_move_to_destination();
			stepper.synchronize();
			G38_move = false;

			set_current_from_steppers_for_axis(ALL_AXES);
			SYNC_PLAN_POSITION_KINEMATIC();
		}

		endstops.hit_on_purpose();
		endstops.not_homing();
		return G38_pass_fail;
	}

	/**
	 * G38.2 - probe toward workpiece, stop on contact, signal error if failure
	 * G38.3 - probe toward workpiece, stop on contact
	 *
	 * Like G28 except uses Z min endstop for all axes
	 */
	inline void gcode_G38(bool is_38_2) {
		// Get X Y Z E F
		gcode_get_destination();

		setup_for_endstop_or_probe_move();

		// If any axis has enough movement, do the move
		LOOP_XYZ(i)
			if (fabs(destination[i] - current_position[i]) >= G38_MINIMUM_MOVE) {
				if (!code_seen('F')) feedrate_mm_s = homing_feedrate_mm_s[i];
				// If G38.2 fails throw an error
				if (!G38_run_probe() && is_38_2) {
					SERIAL_ERROR_START;
					SERIAL_ERRORLNPGM("Failed to reach target");
				}
				break;
			}

		clean_up_after_endstop_or_probe_move();
	}

#endif // G38_PROBE_TARGET

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
	bool didXYZ = false,
			 didE = code_seen('E');

	if (!didE) stepper.synchronize();

	LOOP_XYZE(i) {
		if (code_seen(axis_codes[i])) {
			float p = current_position[i],
						v = code_value_axis_units(i);

			current_position[i] = v;

			if (i != E_AXIS) {
				didXYZ = true;
				position_shift[i] += v - p; // Offset the coordinate space
				update_software_endstops((AxisEnum)i);
			}
		}
	}
	if (didXYZ)
		SYNC_PLAN_POSITION_KINEMATIC();
	else if (didE)
		sync_plan_position_e();

	report_current_position();
}

#if ENABLED(EMERGENCY_PARSER)

	/**
	 * M0: Unconditional stop - Wait for user button press on LCD
	 * M1: Conditional stop	 - Wait for user button press on LCD
	 */
	inline void gcode_M0_M1() {
		char* args = current_command_args;

		millis_t codenum = 0;
		bool hasP = false, hasS = false;
		if (code_seen('P')) {
			codenum = code_value_millis(); // milliseconds to wait
			hasP = codenum > 0;
		}
		if (code_seen('S')) {
			codenum = code_value_millis_from_seconds(); // seconds to wait
			hasS = codenum > 0;
		}

        if (!hasP && !hasS && *args != '\0') {
            SERIAL_ECHO_START;
            SERIAL_ECHOLN(args);
        }

		wait_for_user = true;
		KEEPALIVE_STATE(PAUSED_FOR_USER);

		stepper.synchronize();
		refresh_cmd_timeout();

		if (codenum > 0) {
			codenum += previous_cmd_ms;	// wait until this time for a click
			while (PENDING(millis(), codenum) && wait_for_user) idle();
		}
		else {
			while (wait_for_user) idle();
		}

		wait_for_user = false;
		KEEPALIVE_STATE(IN_HANDLER);
	}

#endif // EMERGENCY_PARSER

/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
	NOOP;
	enable_all_steppers();
}

/**
 * M31: Get the time since the start of SD Print (or last M109)
 */
inline void gcode_M31() {
	char buffer[21];
	duration_t elapsed = print_job_timer.duration();
	elapsed.toString(buffer);

	SERIAL_ECHO_START;
	SERIAL_ECHOLNPAIR("Print time: ", buffer);

	#if ENABLED(AUTOTEMP)
		thermalManager.autotempShutdown();
	#endif
}


/**
 * Sensitive pin test for M42, M226
 */
static bool pin_is_protected(uint8_t pin) {
	static const int sensitive_pins[] = SENSITIVE_PINS;
	for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
		if (sensitive_pins[i] == pin) return true;
	return false;
}

/**
 * M42: Change pin status via GCode
 *
 *	P<pin>	Pin number (LED if omitted)
 *	S<byte> Pin status from 0 - 255
 */
inline void gcode_M42() {
	if (!code_seen('S')) return;

	int pin_status = code_value_int();
	if (pin_status < 0 || pin_status > 255) return;

	int pin_number = code_seen('P') ? code_value_int() : LED_PIN;
	if (pin_number < 0) return;

	if (pin_is_protected(pin_number)) {
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM(MSG_ERR_PROTECTED_PIN);
		return;
	}

	pinMode(pin_number, OUTPUT);
	digitalWrite(pin_number, pin_status);
	analogWrite(pin_number, pin_status);

	#if FAN_COUNT > 0
		switch (pin_number) {
			#if HAS_FAN0
				case FAN_PIN: fanSpeeds[0] = pin_status; break;
			#endif
			#if HAS_FAN1
				case FAN1_PIN: fanSpeeds[1] = pin_status; break;
			#endif
			#if HAS_FAN2
				case FAN2_PIN: fanSpeeds[2] = pin_status; break;
			#endif
		}
	#endif
}

#if ENABLED(PINS_DEBUGGING)

	#include "pinsDebug.h"

	/**
	 * M43: Pin report and debug
	 *
	 *			E<bool> Enable / disable background endstop monitoring
	 *							 - Machine continues to operate
	 *							 - Reports changes to endstops
	 *							 - Toggles LED when an endstop changes
	 *
	 *	 or
	 *
	 *			P<pin>	Pin to read or watch. If omitted, read/watch all pins.
	 *			W<bool> Watch pins -reporting changes- until reset, click, or M108.
	 *			I<bool> Flag to ignore Marlin's pin protection.
	 *
	 */
	inline void gcode_M43() {

		// Enable or disable endstop monitoring
		if (code_seen('E')) {
			endstop_monitor_flag = code_value_bool();
			SERIAL_PROTOCOLPGM("endstop monitor ");
			SERIAL_PROTOCOL(endstop_monitor_flag ? "en" : "dis");
			SERIAL_PROTOCOLLNPGM("abled");
			return;
		}

		// Get the range of pins to test or watch
		int first_pin = 0, last_pin = NUM_DIGITAL_PINS - 1;
		if (code_seen('P')) {
			first_pin = last_pin = code_value_byte();
			if (first_pin > NUM_DIGITAL_PINS - 1) return;
		}

		bool ignore_protection = code_seen('I') ? code_value_bool() : false;

		// Watch until click, M108, or reset
		if (code_seen('W') && code_value_bool()) { // watch digital pins
			byte pin_state[last_pin - first_pin + 1];
			for (int8_t pin = first_pin; pin <= last_pin; pin++) {
				if (pin_is_protected(pin) && !ignore_protection) continue;
				pinMode(pin, INPUT_PULLUP);
				// if (IS_ANALOG(pin))
				//	 pin_state[pin - first_pin] = analogRead(pin - analogInputToDigitalPin(0)); // int16_t pin_state[...]
				// else
					pin_state[pin - first_pin] = digitalRead(pin);
			}

			#if ENABLED(EMERGENCY_PARSER)
				wait_for_user = true;
			#endif

			for(;;) {
				for (int8_t pin = first_pin; pin <= last_pin; pin++) {
					if (pin_is_protected(pin)) continue;
					byte val;
					// if (IS_ANALOG(pin))
					//	 val = analogRead(pin - analogInputToDigitalPin(0)); // int16_t val
					// else
						val = digitalRead(pin);
					if (val != pin_state[pin - first_pin]) {
						report_pin_state(pin);
						pin_state[pin - first_pin] = val;
					}
				}

				#if ENABLED(EMERGENCY_PARSER)
					if (!wait_for_user) break;
				#endif

				safe_delay(500);
			}
			return;
		}

		// Report current state of selected pin(s)
		for (uint8_t pin = first_pin; pin <= last_pin; pin++)
			report_pin_state_extended(pin, ignore_protection);
	}

#endif // PINS_DEBUGGING

#if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)

	/**
	 * M48: Z probe repeatability measurement function.
	 *
	 * Usage:
	 *	 M48 <P#> <X#> <Y#> <V#> <E> <L#>
	 *		 P = Number of sampled points (4-50, default 10)
	 *		 X = Sample X position
	 *		 Y = Sample Y position
	 *		 V = Verbose level (0-4, default=1)
	 *		 E = Engage Z probe for each reading
	 *		 L = Number of legs of movement before probe
	 *		 S = Schizoid (Or Star if you prefer)
	 *
	 * This function assumes the bed has been homed.	Specifically, that a G28 command
	 * as been issued prior to invoking the M48 Z probe repeatability measurement function.
	 * Any information generated by a prior G29 Bed leveling command will be lost and need to be
	 * regenerated.
	 */
	inline void gcode_M48() {

		if (axis_unhomed_error(true, true, true)) return;

		int8_t verbose_level = code_seen('V') ? code_value_byte() : 1;
		if (verbose_level < 0 || verbose_level > 4) {
			SERIAL_PROTOCOLLNPGM("?Verbose Level not plausible (0-4).");
			return;
		}

		if (verbose_level > 0)
			SERIAL_PROTOCOLLNPGM("M48 Z-Probe Repeatability Test");

		int8_t n_samples = code_seen('P') ? code_value_byte() : 10;
		if (n_samples < 4 || n_samples > 50) {
			SERIAL_PROTOCOLLNPGM("?Sample size not plausible (4-50).");
			return;
		}

		float	X_current = current_position[X_AXIS],
					 Y_current = current_position[Y_AXIS];

		bool stow_probe_after_each = code_seen('E');

		float X_probe_location = code_seen('X') ? code_value_axis_units(X_AXIS) : X_current + X_PROBE_OFFSET_FROM_EXTRUDER;
        if (X_probe_location < LOGICAL_X_POSITION(MIN_PROBE_X) || X_probe_location > LOGICAL_X_POSITION(MAX_PROBE_X)) {
            out_of_range_error(PSTR("X"));
            return;
        }

		float Y_probe_location = code_seen('Y') ? code_value_axis_units(Y_AXIS) : Y_current + Y_PROBE_OFFSET_FROM_EXTRUDER;
        if (Y_probe_location < LOGICAL_Y_POSITION(MIN_PROBE_Y) || Y_probe_location > LOGICAL_Y_POSITION(MAX_PROBE_Y)) {
            out_of_range_error(PSTR("Y"));
            return;
        }


		bool seen_L = code_seen('L');
		uint8_t n_legs = seen_L ? code_value_byte() : 0;
		if (n_legs > 15) {
			SERIAL_PROTOCOLLNPGM("?Number of legs in movement not plausible (0-15).");
			return;
		}
		if (n_legs == 1) n_legs = 2;

		bool schizoid_flag = code_seen('S');
		if (schizoid_flag && !seen_L) n_legs = 7;

		/**
		 * Now get everything to the specified probe point So we can safely do a
		 * probe to get us close to the bed.	If the Z-Axis is far from the bed,
		 * we don't want to use that as a starting point for each probe.
		 */
		if (verbose_level > 2)
			SERIAL_PROTOCOLLNPGM("Positioning the probe...");



		setup_for_endstop_or_probe_move();

		// Move to the first point, deploy, and probe
		probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, verbose_level);

		randomSeed(millis());

		double mean = 0.0, sigma = 0.0, min = 99999.9, max = -99999.9, sample_set[n_samples];

		for (uint8_t n = 0; n < n_samples; n++) {
			if (n_legs) {
				int dir = (random(0, 10) > 5.0) ? -1 : 1;	// clockwise or counter clockwise
				float angle = random(0.0, 360.0),
							radius = random(5, X_MAX_LENGTH / 8);
				if (verbose_level > 3) {
					SERIAL_ECHOPAIR("Starting radius: ", radius);
					SERIAL_ECHOPAIR("	 angle: ", angle);
					SERIAL_ECHOPGM(" Direction: ");
					if (dir > 0) SERIAL_ECHOPGM("Counter-");
					SERIAL_ECHOLNPGM("Clockwise");
				}

				for (uint8_t l = 0; l < n_legs - 1; l++) {
					double delta_angle;

					if (schizoid_flag)
						// The points of a 5 point star are 72 degrees apart.	We need to
						// skip a point and go to the next one on the star.
						delta_angle = dir * 2.0 * 72.0;

					else
						// If we do this line, we are just trying to move further
						// around the circle.
						delta_angle = dir * (float) random(25, 45);

					angle += delta_angle;

					while (angle > 360.0)	 // We probably do not need to keep the angle between 0 and 2*PI, but the
						angle -= 360.0;			 // Arduino documentation says the trig functions should not be given values
					while (angle < 0.0)		 // outside of this range.	 It looks like they behave correctly with
						angle += 360.0;			 // numbers outside of the range, but just to be safe we clamp them.

					X_current = X_probe_location - (X_PROBE_OFFSET_FROM_EXTRUDER) + cos(RADIANS(angle)) * radius;
					Y_current = Y_probe_location - (Y_PROBE_OFFSET_FROM_EXTRUDER) + sin(RADIANS(angle)) * radius;

                    X_current = constrain(X_current, X_MIN_POS, X_MAX_POS);
                    Y_current = constrain(Y_current, Y_MIN_POS, Y_MAX_POS);
					if (verbose_level > 3) {
						SERIAL_PROTOCOLPGM("Going to:");
						SERIAL_ECHOPAIR(" X", X_current);
						SERIAL_ECHOPAIR(" Y", Y_current);
						SERIAL_ECHOLNPAIR(" Z", current_position[Z_AXIS]);
					}
					do_blocking_move_to_xy(X_current, Y_current);
				} // n_legs loop
			} // n_legs

			// Probe a single point
			sample_set[n] = probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, 0);

			/**
			 * Get the current mean for the data points we have so far
			 */
			double sum = 0.0;
			for (uint8_t j = 0; j <= n; j++) sum += sample_set[j];
			mean = sum / (n + 1);

			NOMORE(min, sample_set[n]);
			NOLESS(max, sample_set[n]);

			/**
			 * Now, use that mean to calculate the standard deviation for the
			 * data points we have so far
			 */
			sum = 0.0;
			for (uint8_t j = 0; j <= n; j++)
				sum += sq(sample_set[j] - mean);

			sigma = sqrt(sum / (n + 1));
			if (verbose_level > 0) {
				if (verbose_level > 1) {
					SERIAL_PROTOCOL(n + 1);
					SERIAL_PROTOCOLPGM(" of ");
					SERIAL_PROTOCOL((int)n_samples);
					SERIAL_PROTOCOLPGM(": z: ");
					SERIAL_PROTOCOL_F(sample_set[n], 3);
					if (verbose_level > 2) {
						SERIAL_PROTOCOLPGM(" mean: ");
						SERIAL_PROTOCOL_F(mean, 4);
						SERIAL_PROTOCOLPGM(" sigma: ");
						SERIAL_PROTOCOL_F(sigma, 6);
						SERIAL_PROTOCOLPGM(" min: ");
						SERIAL_PROTOCOL_F(min, 3);
						SERIAL_PROTOCOLPGM(" max: ");
						SERIAL_PROTOCOL_F(max, 3);
						SERIAL_PROTOCOLPGM(" range: ");
						SERIAL_PROTOCOL_F(max-min, 3);
					}
				}
				SERIAL_EOL;
			}

		} // End of probe loop

		if (STOW_PROBE()) return;

		SERIAL_PROTOCOLPGM("Finished!");
		SERIAL_EOL;

		if (verbose_level > 0) {
			SERIAL_PROTOCOLPGM("Mean: ");
			SERIAL_PROTOCOL_F(mean, 6);
			SERIAL_PROTOCOLPGM(" Min: ");
			SERIAL_PROTOCOL_F(min, 3);
			SERIAL_PROTOCOLPGM(" Max: ");
			SERIAL_PROTOCOL_F(max, 3);
			SERIAL_PROTOCOLPGM(" Range: ");
			SERIAL_PROTOCOL_F(max-min, 3);
			SERIAL_EOL;
		}

		SERIAL_PROTOCOLPGM("Standard Deviation: ");
		SERIAL_PROTOCOL_F(sigma, 6);
		SERIAL_EOL;
		SERIAL_EOL;

		clean_up_after_endstop_or_probe_move();

		report_current_position();
	}

#endif // Z_MIN_PROBE_REPEATABILITY_TEST

/**
 * M75: Start print timer
 */
inline void gcode_M75() { print_job_timer.start(); }

/**
 * M76: Pause print timer
 */
inline void gcode_M76() { print_job_timer.pause(); }

/**
 * M77: Stop print timer
 */
inline void gcode_M77() { print_job_timer.stop(); }

#if ENABLED(PRINTCOUNTER)
	/**
	 * M78: Show print statistics
	 */
	inline void gcode_M78() {
		// "M78 S78" will reset the statistics
		if (code_seen('S') && code_value_int() == 78)
			print_job_timer.initStats();
		else
			print_job_timer.showStats();
	}
#endif

/**
 * M104: Set hot end temperature
 */
inline void gcode_M104() {
	if (get_target_extruder_from_command(104)) return;
	if (DEBUGGING(DRYRUN)) return;

	#if ENABLED(SINGLENOZZLE)
		if (target_extruder != active_extruder) return;
	#endif

	if (code_seen('S')) {
		thermalManager.setTargetHotend(code_value_temp_abs(), target_extruder);

		#if ENABLED(PRINTJOB_TIMER_AUTOSTART)
			/**
			 * Stop the timer at the end of print, starting is managed by
			 * 'heat and wait' M109.
			 * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
			 * stand by mode, for instance in a dual extruder setup, without affecting
			 * the running print timer.
			 */
			if (code_value_temp_abs() <= (EXTRUDE_MINTEMP)/2) {
				print_job_timer.stop();
                NOOP;
			}
		#endif

		if (code_value_temp_abs() > thermalManager.degHotend(target_extruder)) NOOP;
	}
	
	#if ENABLED(AUTOTEMP)
		planner.autotemp_M104_M109();
	#endif
}

#if HAS_TEMP_HOTEND || HAS_TEMP_BED

	void print_heaterstates() {
		#if HAS_TEMP_HOTEND
			SERIAL_PROTOCOLPGM(" T:");
			SERIAL_PROTOCOL_F(thermalManager.degHotend(target_extruder), 1);
			SERIAL_PROTOCOLPGM(" /");
			SERIAL_PROTOCOL_F(thermalManager.degTargetHotend(target_extruder), 1);
			#if ENABLED(SHOW_TEMP_ADC_VALUES)
				SERIAL_PROTOCOLPAIR(" (", thermalManager.current_temperature_raw[target_extruder] / OVERSAMPLENR);
				SERIAL_CHAR(')');
			#endif
		#endif
		#if HAS_TEMP_BED
			SERIAL_PROTOCOLPGM(" B:");
			SERIAL_PROTOCOL_F(thermalManager.degBed(), 1);
			SERIAL_PROTOCOLPGM(" /");
			SERIAL_PROTOCOL_F(thermalManager.degTargetBed(), 1);
			#if ENABLED(SHOW_TEMP_ADC_VALUES)
				SERIAL_PROTOCOLPAIR(" (", thermalManager.current_temperature_bed_raw / OVERSAMPLENR);
				SERIAL_CHAR(')');
			#endif
		#endif

		SERIAL_PROTOCOLPGM(" @:");
		SERIAL_PROTOCOL(thermalManager.getHeaterPower(target_extruder));
		#if HAS_TEMP_BED
			SERIAL_PROTOCOLPGM(" B@:");
			SERIAL_PROTOCOL(thermalManager.getHeaterPower(-1));
		#endif

	}
#endif

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {
	if (get_target_extruder_from_command(105)) return;

	#if HAS_TEMP_HOTEND || HAS_TEMP_BED
		SERIAL_PROTOCOLPGM(MSG_OK);
		print_heaterstates();
	#else // !HAS_TEMP_HOTEND && !HAS_TEMP_BED
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
	#endif

	SERIAL_EOL;
}

#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)

	static uint8_t auto_report_temp_interval;
	static millis_t next_temp_report_ms;

	/**
	 * M155: Set temperature auto-report interval. M155 S<seconds>
	 */
	inline void gcode_M155() {
		if (code_seen('S')) {
			auto_report_temp_interval = code_value_byte();
			NOMORE(auto_report_temp_interval, 60);
			next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
		}
	}

	inline void auto_report_temperatures() {
		if (auto_report_temp_interval && ELAPSED(millis(), next_temp_report_ms)) {
			next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
			print_heaterstates();
			SERIAL_EOL;
		}
	}

#endif // AUTO_REPORT_TEMPERATURES

#if FAN_COUNT > 0

	/**
	 * M106: Set Fan Speed
	 *
	 *	S<int>	 Speed between 0-255
	 *	P<index> Fan index, if more than one fan
	 */
	inline void gcode_M106() {
		uint16_t s = code_seen('S') ? code_value_ushort() : 255,
						 p = code_seen('P') ? code_value_ushort() : 0;
		NOMORE(s, 255);
		if (p < FAN_COUNT) fanSpeeds[p] = s;
	}

	/**
	 * M107: Fan Off
	 */
	inline void gcode_M107() {
		uint16_t p = code_seen('P') ? code_value_ushort() : 0;
		if (p < FAN_COUNT) fanSpeeds[p] = 0;
	}

#endif // FAN_COUNT > 0

#if DISABLED(EMERGENCY_PARSER)

	/**
	 * M108: Stop the waiting for heaters in M109, M190, M303. Does not affect the target temperature.
	 */
	inline void gcode_M108() { wait_for_heatup = false; }


	/**
	 * M112: Emergency Stop
	 */
	inline void gcode_M112() { kill(PSTR(MSG_KILLED)); }


	/**
	 * M410: Quickstop - Abort all planned moves
	 *
	 * This will stop the carriages mid-move, so most likely they
	 * will be out of sync with the stepper position after this.
	 */
	inline void gcode_M410() { quickstop_stepper(); }

#endif

	#ifndef MIN_COOLING_SLOPE_DEG
		#define MIN_COOLING_SLOPE_DEG 1.50
	#endif
	#ifndef MIN_COOLING_SLOPE_TIME
		#define MIN_COOLING_SLOPE_TIME 60
	#endif

/**
 * M109: Sxxx Wait for extruder(s) to reach temperature. Waits only when heating.
 *			 Rxxx Wait for extruder(s) to reach temperature. Waits when heating and cooling.
 */
inline void gcode_M109() {

	if (get_target_extruder_from_command(109)) return;
	if (DEBUGGING(DRYRUN)) return;

	#if ENABLED(SINGLENOZZLE)
		if (target_extruder != active_extruder) return;
	#endif

	bool no_wait_for_cooling = code_seen('S');
	if (no_wait_for_cooling || code_seen('R')) {
		thermalManager.setTargetHotend(code_value_temp_abs(), target_extruder);

		#if ENABLED(PRINTJOB_TIMER_AUTOSTART)
			/**
			 * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
			 * stand by mode, for instance in a dual extruder setup, without affecting
			 * the running print timer.
			 */
			if (code_value_temp_abs() <= (EXTRUDE_MINTEMP)/2) {
				print_job_timer.stop();
                NOOP;
			}
			/**
			 * We do not check if the timer is already running because this check will
			 * be done for us inside the Stopwatch::start() method thus a running timer
			 * will not restart.
			 */
			else print_job_timer.start();
		#endif

		if (thermalManager.isHeatingHotend(target_extruder)) NOOP;
	}

	#if ENABLED(AUTOTEMP)
		planner.autotemp_M104_M109();
	#endif

	#if TEMP_RESIDENCY_TIME > 0
		millis_t residency_start_ms = 0;
		// Loop until the temperature has stabilized
		#define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
	#else
		// Loop until the temperature is very close target
		#define TEMP_CONDITIONS (wants_to_cool ? thermalManager.isCoolingHotend(target_extruder) : thermalManager.isHeatingHotend(target_extruder))
	#endif //TEMP_RESIDENCY_TIME > 0

	float theTarget = -1.0, old_temp = 9999.0;
	bool wants_to_cool = false;
	wait_for_heatup = true;
	millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

	KEEPALIVE_STATE(NOT_BUSY);

	do {
		// Target temperature might be changed during the loop
		if (theTarget != thermalManager.degTargetHotend(target_extruder)) {
			wants_to_cool = thermalManager.isCoolingHotend(target_extruder);
			theTarget = thermalManager.degTargetHotend(target_extruder);

			// Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
			if (no_wait_for_cooling && wants_to_cool) break;
		}

		now = millis();
		if (ELAPSED(now, next_temp_ms)) { //Print temp & remaining time every 1s while waiting
			next_temp_ms = now + 1000UL;
			print_heaterstates();
			#if TEMP_RESIDENCY_TIME > 0
				SERIAL_PROTOCOLPGM(" W:");
				if (residency_start_ms) {
					long rem = (((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
					SERIAL_PROTOCOLLN(rem);
				}
				else {
					SERIAL_PROTOCOLLNPGM("?");
				}
			#else
				SERIAL_EOL;
			#endif
		}

		idle();
		refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

		float temp = thermalManager.degHotend(target_extruder);

		#if TEMP_RESIDENCY_TIME > 0

			float temp_diff = fabs(theTarget - temp);

			if (!residency_start_ms) {
				// Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
				if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
			}
			else if (temp_diff > TEMP_HYSTERESIS) {
				// Restart the timer whenever the temperature falls outside the hysteresis.
				residency_start_ms = now;
			}

		#endif //TEMP_RESIDENCY_TIME > 0

		// Prevent a wait-forever situation if R is misused i.e. M109 R0
		if (wants_to_cool) {
			// break after MIN_COOLING_SLOPE_TIME seconds
			// if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
			if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
				if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
				next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
				old_temp = temp;
			}
		}

	} while (wait_for_heatup && TEMP_CONDITIONS);

	if (wait_for_heatup) NOOP;

	KEEPALIVE_STATE(IN_HANDLER);
}

#if HAS_TEMP_BED

	#ifndef MIN_COOLING_SLOPE_DEG_BED
		#define MIN_COOLING_SLOPE_DEG_BED 1.50
	#endif
	#ifndef MIN_COOLING_SLOPE_TIME_BED
		#define MIN_COOLING_SLOPE_TIME_BED 60
	#endif

	/**
	 * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
	 *			 Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
	 */
	inline void gcode_M190() {
		if (DEBUGGING(DRYRUN)) return;

        NOOP;
		bool no_wait_for_cooling = code_seen('S');
		if (no_wait_for_cooling || code_seen('R')) {
			thermalManager.setTargetBed(code_value_temp_abs());
			#if ENABLED(PRINTJOB_TIMER_AUTOSTART)
				if (code_value_temp_abs() > BED_MINTEMP) {
					/**
					* We start the timer when 'heating and waiting' command arrives, LCD
					* functions never wait. Cooling down managed by extruders.
					*
					* We do not check if the timer is already running because this check will
					* be done for us inside the Stopwatch::start() method thus a running timer
					* will not restart.
					*/
					print_job_timer.start();
				}
			#endif
		}

		#if TEMP_BED_RESIDENCY_TIME > 0
			millis_t residency_start_ms = 0;
			// Loop until the temperature has stabilized
			#define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))
		#else
			// Loop until the temperature is very close target
			#define TEMP_BED_CONDITIONS (wants_to_cool ? thermalManager.isCoolingBed() : thermalManager.isHeatingBed())
		#endif //TEMP_BED_RESIDENCY_TIME > 0

		float theTarget = -1.0, old_temp = 9999.0;
		bool wants_to_cool = false;
		wait_for_heatup = true;
		millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

		KEEPALIVE_STATE(NOT_BUSY);

		target_extruder = active_extruder; // for print_heaterstates

		do {
			// Target temperature might be changed during the loop
			if (theTarget != thermalManager.degTargetBed()) {
				wants_to_cool = thermalManager.isCoolingBed();
				theTarget = thermalManager.degTargetBed();

				// Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
				if (no_wait_for_cooling && wants_to_cool) break;
			}

			now = millis();
			if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
				next_temp_ms = now + 1000UL;
				print_heaterstates();
				#if TEMP_BED_RESIDENCY_TIME > 0
					SERIAL_PROTOCOLPGM(" W:");
					if (residency_start_ms) {
						long rem = (((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
						SERIAL_PROTOCOLLN(rem);
					}
					else {
						SERIAL_PROTOCOLLNPGM("?");
					}
				#else
					SERIAL_EOL;
				#endif
			}

			idle();
			refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

			float temp = thermalManager.degBed();

			#if TEMP_BED_RESIDENCY_TIME > 0

				float temp_diff = fabs(theTarget - temp);

				if (!residency_start_ms) {
					// Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
					if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
				}
				else if (temp_diff > TEMP_BED_HYSTERESIS) {
					// Restart the timer whenever the temperature falls outside the hysteresis.
					residency_start_ms = now;
				}

			#endif //TEMP_BED_RESIDENCY_TIME > 0

			// Prevent a wait-forever situation if R is misused i.e. M190 R0
			if (wants_to_cool) {
				// break after MIN_COOLING_SLOPE_TIME_BED seconds
				// if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_BED
				if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
					if (old_temp - temp < MIN_COOLING_SLOPE_DEG_BED) break;
					next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME_BED;
					old_temp = temp;
				}
			}

		} while (wait_for_heatup && TEMP_BED_CONDITIONS);

		if (wait_for_heatup) NOOP;
		KEEPALIVE_STATE(IN_HANDLER);
	}

#endif // HAS_TEMP_BED

/**
 * M110: Set Current Line Number
 */
inline void gcode_M110() {
	if (code_seen('N')) gcode_N = code_value_long();
}

/**
 * M111: Set the debug level
 */
inline void gcode_M111() {
	marlin_debug_flags = code_seen('S') ? code_value_byte() : (uint8_t) DEBUG_NONE;

	const static char str_debug_1[] PROGMEM = MSG_DEBUG_ECHO;
	const static char str_debug_2[] PROGMEM = MSG_DEBUG_INFO;
	const static char str_debug_4[] PROGMEM = MSG_DEBUG_ERRORS;
	const static char str_debug_8[] PROGMEM = MSG_DEBUG_DRYRUN;
	const static char str_debug_16[] PROGMEM = MSG_DEBUG_COMMUNICATION;
	#if ENABLED(DEBUG_LEVELING_FEATURE)
		const static char str_debug_32[] PROGMEM = MSG_DEBUG_LEVELING;
	#endif

	const static char* const debug_strings[] PROGMEM = {
		str_debug_1, str_debug_2, str_debug_4, str_debug_8, str_debug_16,
		#if ENABLED(DEBUG_LEVELING_FEATURE)
			str_debug_32
		#endif
	};

	SERIAL_ECHO_START;
	SERIAL_ECHOPGM(MSG_DEBUG_PREFIX);
	if (marlin_debug_flags) {
		uint8_t comma = 0;
		for (uint8_t i = 0; i < COUNT(debug_strings); i++) {
			if (TEST(marlin_debug_flags, i)) {
				if (comma++) SERIAL_CHAR(',');
				serialprintPGM((char*)pgm_read_word(&(debug_strings[i])));
			}
		}
	}
	else {
		SERIAL_ECHOPGM(MSG_DEBUG_OFF);
	}
	SERIAL_EOL;
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

	/**
	 * M113: Get or set Host Keepalive interval (0 to disable)
	 *
	 *	 S<seconds> Optional. Set the keepalive interval.
	 */
	inline void gcode_M113() {
		if (code_seen('S')) {
			host_keepalive_interval = code_value_byte();
			NOMORE(host_keepalive_interval, 60);
		}
		else {
			SERIAL_ECHO_START;
			SERIAL_ECHOLNPAIR("M113 S", (unsigned long)host_keepalive_interval);
		}
	}

#endif

#if ENABLED(BARICUDA)

	#if HAS_HEATER_1
		/**
		 * M126: Heater 1 valve open
		 */
		inline void gcode_M126() { baricuda_valve_pressure = code_seen('S') ? code_value_byte() : 255; }
		/**
		 * M127: Heater 1 valve close
		 */
		inline void gcode_M127() { baricuda_valve_pressure = 0; }
	#endif

	#if HAS_HEATER_2
		/**
		 * M128: Heater 2 valve open
		 */
		inline void gcode_M128() { baricuda_e_to_p_pressure = code_seen('S') ? code_value_byte() : 255; }
		/**
		 * M129: Heater 2 valve close
		 */
		inline void gcode_M129() { baricuda_e_to_p_pressure = 0; }
	#endif

#endif //BARICUDA

/**
 * M140: Set bed temperature
 */
inline void gcode_M140() {
	if (DEBUGGING(DRYRUN)) return;
	if (code_seen('S')) thermalManager.setTargetBed(code_value_temp_abs());
}

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
	/**
	 * M149: Set temperature units
	 */
	inline void gcode_M149() {
				 if (code_seen('C')) set_input_temp_units(TEMPUNIT_C);
		else if (code_seen('K')) set_input_temp_units(TEMPUNIT_K);
		else if (code_seen('F')) set_input_temp_units(TEMPUNIT_F);
	}
#endif

#if HAS_POWER_SWITCH

	/**
	 * M80: Turn on Power Supply
	 */
	inline void gcode_M80() {
		OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); //GND

		/**
		 * If you have a switch on suicide pin, this is useful
		 * if you want to start another print with suicide feature after
		 * a print without suicide...
		 */
		#if HAS_SUICIDE
			OUT_WRITE(SUICIDE_PIN, HIGH);
		#endif

	}

#endif // HAS_POWER_SWITCH

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *			This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
	thermalManager.disable_all_heaters();
	stepper.finish_and_disable();
	#if FAN_COUNT > 0
		#if FAN_COUNT > 1
			for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
		#else
			fanSpeeds[0] = 0;
		#endif
	#endif
	delay(1000); // Wait 1 second before switching off
	#if HAS_SUICIDE
		stepper.synchronize();
		suicide();
	#elif HAS_POWER_SWITCH
		OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
	#endif

}

/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode
 */
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }

/**
 * M18, M84: Disable all stepper motors
 */
inline void gcode_M18_M84() {
	if (code_seen('S')) {
		stepper_inactive_time = code_value_millis_from_seconds();
	}
	else {
		bool all_axis = !((code_seen('X')) || (code_seen('Y')) || (code_seen('Z')) || (code_seen('E')));
		if (all_axis) {
			stepper.finish_and_disable();
		}
		else {
			stepper.synchronize();
			if (code_seen('X')) disable_x();
			if (code_seen('Y')) disable_y();
			if (code_seen('Z')) disable_z();
			#if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
				if (code_seen('E')) {
					disable_e0();
					disable_e1();
					disable_e2();
					disable_e3();
				}
			#endif
		}
	}
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
	if (code_seen('S')) max_inactive_time = code_value_millis_from_seconds();
}

/**
 * Multi-stepper support for M92, M201, M203
 */
#if ENABLED(DISTINCT_E_FACTORS)
	#define GET_TARGET_EXTRUDER(CMD) if (get_target_extruder_from_command(CMD)) return
	#define TARGET_EXTRUDER target_extruder
#else
	#define GET_TARGET_EXTRUDER(CMD) NOOP
	#define TARGET_EXTRUDER 0
#endif

/**
 * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
 *			(Follows the same syntax as G92)
 *
 *			With multiple extruders use T to specify which one.
 */
inline void gcode_M92() {

	GET_TARGET_EXTRUDER(92);

	LOOP_XYZE(i) {
		if (code_seen(axis_codes[i])) {
			if (i == E_AXIS) {
				float value = code_value_per_axis_unit(E_AXIS + TARGET_EXTRUDER);
				if (value < 20.0) {
					float factor = planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] / value; // increase e constants if M92 E14 is given for netfab.
					planner.max_jerk[E_AXIS] *= factor;
					planner.max_feedrate_mm_s[E_AXIS + TARGET_EXTRUDER] *= factor;
					planner.max_acceleration_steps_per_s2[E_AXIS + TARGET_EXTRUDER] *= factor;
				}
				planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] = value;
			}
			else {
				planner.axis_steps_per_mm[i] = code_value_per_axis_unit(i);
			}
		}
	}
	planner.refresh_positioning();
}

/**
 * Output the current position to serial
 */
static void report_current_position() {
	SERIAL_PROTOCOLPGM("X:");
	SERIAL_PROTOCOL(current_position[X_AXIS]);
	SERIAL_PROTOCOLPGM(" Y:");
	SERIAL_PROTOCOL(current_position[Y_AXIS]);
	SERIAL_PROTOCOLPGM(" Z:");
	SERIAL_PROTOCOL(current_position[Z_AXIS]);
	SERIAL_PROTOCOLPGM(" E:");
	SERIAL_PROTOCOL(current_position[E_AXIS]);

	stepper.report_positions();
}

/**
 * M114: Output current position to serial port
 */
inline void gcode_M114() { report_current_position(); }

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
	SERIAL_PROTOCOLLNPGM(MSG_M115_REPORT);

	#if ENABLED(EXTENDED_CAPABILITIES_REPORT)

		// EEPROM (M500, M501)
		#if ENABLED(EEPROM_SETTINGS)
			SERIAL_PROTOCOLLNPGM("Cap:EEPROM:1");
		#else
			SERIAL_PROTOCOLLNPGM("Cap:EEPROM:0");
		#endif

		// AUTOREPORT_TEMP (M155)
		#if ENABLED(AUTO_REPORT_TEMPERATURES)
			SERIAL_PROTOCOLLNPGM("Cap:AUTOREPORT_TEMP:1");
		#else
			SERIAL_PROTOCOLLNPGM("Cap:AUTOREPORT_TEMP:0");
		#endif

		// PROGRESS (M530 S L, M531 <file>, M532 X L)
		SERIAL_PROTOCOLLNPGM("Cap:PROGRESS:0");

		SERIAL_PROTOCOLLNPGM("Cap:AUTOLEVEL:0");

		// Z_PROBE (G30)
		#if HAS_BED_PROBE
			SERIAL_PROTOCOLLNPGM("Cap:Z_PROBE:1");
		#else
			SERIAL_PROTOCOLLNPGM("Cap:Z_PROBE:0");
		#endif

		// SOFTWARE_POWER (G30)
		#if HAS_POWER_SWITCH
			SERIAL_PROTOCOLLNPGM("Cap:SOFTWARE_POWER:1");
		#else
			SERIAL_PROTOCOLLNPGM("Cap:SOFTWARE_POWER:0");
		#endif

		// TOGGLE_LIGHTS (M355)
		#if HAS_CASE_LIGHT
			SERIAL_PROTOCOLLNPGM("Cap:TOGGLE_LIGHTS:1");
		#else
			SERIAL_PROTOCOLLNPGM("Cap:TOGGLE_LIGHTS:0");
		#endif

		// EMERGENCY_PARSER (M108, M112, M410)
		#if ENABLED(EMERGENCY_PARSER)
			SERIAL_PROTOCOLLNPGM("Cap:EMERGENCY_PARSER:1");
		#else
			SERIAL_PROTOCOLLNPGM("Cap:EMERGENCY_PARSER:0");
		#endif

	#endif // EXTENDED_CAPABILITIES_REPORT
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() {

}

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() { endstops.M119(); }

/**
 * M120: Enable endstops and set non-homing endstop state to "enabled"
 */
inline void gcode_M120() { endstops.enable_globally(true); }

/**
 * M121: Disable endstops and set non-homing endstop state to "disabled"
 */
inline void gcode_M121() { endstops.enable_globally(false); }

#if ENABLED(HAVE_TMC2130DRIVER)

	/**
	 * M122: Output Trinamic TMC2130 status to serial output. Very bad formatting.
	 */

	static void tmc2130_report(Trinamic_TMC2130 &stepr, const char *name) {
		stepr.read_STAT();
		SERIAL_PROTOCOL(name);
		SERIAL_PROTOCOL(": ");
		stepr.isReset() ? SERIAL_PROTOCOLPGM("RESET ") : SERIAL_PROTOCOLPGM("----- ");
		stepr.isError() ? SERIAL_PROTOCOLPGM("ERROR ") : SERIAL_PROTOCOLPGM("----- ");
		stepr.isStallguard() ? SERIAL_PROTOCOLPGM("SLGRD ") : SERIAL_PROTOCOLPGM("----- ");
		stepr.isStandstill() ? SERIAL_PROTOCOLPGM("STILL ") : SERIAL_PROTOCOLPGM("----- ");
		SERIAL_PROTOCOLLN(stepr.debug());
	}

	inline void gcode_M122() {
		SERIAL_PROTOCOLLNPGM("Reporting TMC2130 status");
		#if ENABLED(X_IS_TMC2130)
			tmc2130_report(stepperX, "X");
		#endif
		#if ENABLED(X2_IS_TMC2130)
			tmc2130_report(stepperX2, "X2");
		#endif
		#if ENABLED(Y_IS_TMC2130)
			tmc2130_report(stepperY, "Y");
		#endif
		#if ENABLED(Y2_IS_TMC2130)
			tmc2130_report(stepperY2, "Y2");
		#endif
		#if ENABLED(Z_IS_TMC2130)
			tmc2130_report(stepperZ, "Z");
		#endif
		#if ENABLED(Z2_IS_TMC2130)
			tmc2130_report(stepperZ2, "Z2");
		#endif
		#if ENABLED(E0_IS_TMC2130)
			tmc2130_report(stepperE0, "E0");
		#endif
		#if ENABLED(E1_IS_TMC2130)
			tmc2130_report(stepperE1, "E1");
		#endif
		#if ENABLED(E2_IS_TMC2130)
			tmc2130_report(stepperE2, "E2");
		#endif
		#if ENABLED(E3_IS_TMC2130)
			tmc2130_report(stepperE3, "E3");
		#endif
	}
#endif // HAVE_TMC2130DRIVER

#if ENABLED(BLINKM) || ENABLED(RGB_LED)

	void set_led_color(const uint8_t r, const uint8_t g, const uint8_t b) {

		#if ENABLED(BLINKM)

			// This variant uses i2c to send the RGB components to the device.
			SendColors(r, g, b);

		#else

			// This variant uses 3 separate pins for the RGB components.
			// If the pins can do PWM then their intensity will be set.
			digitalWrite(RGB_LED_R_PIN, r ? HIGH : LOW);
			digitalWrite(RGB_LED_G_PIN, g ? HIGH : LOW);
			digitalWrite(RGB_LED_B_PIN, b ? HIGH : LOW);
			analogWrite(RGB_LED_R_PIN, r);
			analogWrite(RGB_LED_G_PIN, g);
			analogWrite(RGB_LED_B_PIN, b);

		#endif
	}

	/**
	 * M150: Set Status LED Color - Use R-U-B for R-G-B
	 *
	 * Always sets all 3 components. If a component is left out, set to 0.
	 *
	 * Examples:
	 *
	 *	 M150 R255			 ; Turn LED red
	 *	 M150 R255 U127	; Turn LED orange (PWM only)
	 *	 M150						; Turn LED off
	 *	 M150 R U B			; Turn LED white
	 *
	 */
	inline void gcode_M150() {
		set_led_color(
			code_seen('R') ? (code_has_value() ? code_value_byte() : 255) : 0,
			code_seen('U') ? (code_has_value() ? code_value_byte() : 255) : 0,
			code_seen('B') ? (code_has_value() ? code_value_byte() : 255) : 0
		);
	}

#endif // BLINKM || RGB_LED

/**
 * M200: Set filament diameter and set E axis units to cubic units
 *
 *		T<extruder> - Optional extruder number. Current extruder if omitted.
 *		D<linear> - Diameter of the filament. Use "D0" to switch back to linear units on the E axis.
 */
inline void gcode_M200() {

	if (get_target_extruder_from_command(200)) return;

	if (code_seen('D')) {
		// setting any extruder filament size disables volumetric on the assumption that
		// slicers either generate in extruder values as cubic mm or as as filament feeds
		// for all extruders
		volumetric_enabled = (code_value_linear_units() != 0.0);
		if (volumetric_enabled) {
			filament_size[target_extruder] = code_value_linear_units();
			// make sure all extruders have some sane value for the filament size
			for (uint8_t i = 0; i < COUNT(filament_size); i++)
				if (! filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
		}
	}
	else {
		//reserved for setting filament diameter via UFID or filament measuring device
		return;
	}
	calculate_volumetric_multipliers();
}

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 *
 *			 With multiple extruders use T to specify which one.
 */
inline void gcode_M201() {

	GET_TARGET_EXTRUDER(201);

	LOOP_XYZE(i) {
		if (code_seen(axis_codes[i])) {
			const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
			planner.max_acceleration_mm_per_s2[a] = code_value_axis_units(a);
		}
	}
	// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
	planner.reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
	inline void gcode_M202() {
		LOOP_XYZE(i) {
			if (code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value_axis_units(i) * planner.axis_steps_per_mm[i];
		}
	}
#endif


/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 *
 *			 With multiple extruders use T to specify which one.
 */
inline void gcode_M203() {

	GET_TARGET_EXTRUDER(203);

	LOOP_XYZE(i)
		if (code_seen(axis_codes[i])) {
			const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
			planner.max_feedrate_mm_s[a] = code_value_axis_units(a);
		}
}

/**
 * M204: Set Accelerations in units/sec^2 (M204 P1200 R3000 T3000)
 *
 *		P = Printing moves
 *		R = Retract only (no X, Y, Z) moves
 *		T = Travel (non printing) moves
 *
 *	Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 */
inline void gcode_M204() {
	if (code_seen('S')) {	// Kept for legacy compatibility. Should NOT BE USED for new developments.
		planner.travel_acceleration = planner.acceleration = code_value_linear_units();
		SERIAL_ECHOLNPAIR("Setting Print and Travel Acceleration: ", planner.acceleration);
	}
	if (code_seen('P')) {
		planner.acceleration = code_value_linear_units();
		SERIAL_ECHOLNPAIR("Setting Print Acceleration: ", planner.acceleration);
	}
	if (code_seen('R')) {
		planner.retract_acceleration = code_value_linear_units();
		SERIAL_ECHOLNPAIR("Setting Retract Acceleration: ", planner.retract_acceleration);
	}
	if (code_seen('T')) {
		planner.travel_acceleration = code_value_linear_units();
		SERIAL_ECHOLNPAIR("Setting Travel Acceleration: ", planner.travel_acceleration);
	}
}

/**
 * M205: Set Advanced Settings
 *
 *		S = Min Feed Rate (units/s)
 *		T = Min Travel Feed Rate (units/s)
 *		B = Min Segment Time (µs)
 *		X = Max X Jerk (units/sec^2)
 *		Y = Max Y Jerk (units/sec^2)
 *		Z = Max Z Jerk (units/sec^2)
 *		E = Max E Jerk (units/sec^2)
 */
inline void gcode_M205() {
	if (code_seen('S')) planner.min_feedrate_mm_s = code_value_linear_units();
	if (code_seen('T')) planner.min_travel_feedrate_mm_s = code_value_linear_units();
	if (code_seen('B')) planner.min_segment_time = code_value_millis();
	if (code_seen('X')) planner.max_jerk[X_AXIS] = code_value_axis_units(X_AXIS);
	if (code_seen('Y')) planner.max_jerk[Y_AXIS] = code_value_axis_units(Y_AXIS);
	if (code_seen('Z')) planner.max_jerk[Z_AXIS] = code_value_axis_units(Z_AXIS);
	if (code_seen('E')) planner.max_jerk[E_AXIS] = code_value_axis_units(E_AXIS);
}

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {
	LOOP_XYZ(i)
		if (code_seen(axis_codes[i]))
			set_home_offset((AxisEnum)i, code_value_axis_units(i));

	SYNC_PLAN_POSITION_KINEMATIC();
	report_current_position();
}


#if ENABLED(Z_DUAL_ENDSTOPS) // !DELTA && ENABLED(Z_DUAL_ENDSTOPS)

	/**
	 * M666: For Z Dual Endstop setup, set z axis offset to the z2 axis.
	 */
	inline void gcode_M666() {
		if (code_seen('Z')) z_endstop_adj = code_value_axis_units(Z_AXIS);
		SERIAL_ECHOLNPAIR("Z Endstop Adjustment set to (mm):", z_endstop_adj);
	}

#endif // !DELTA && Z_DUAL_ENDSTOPS

#if ENABLED(FWRETRACT)

	/**
	 * M207: Set firmware retraction values
	 *
	 *	 S[+units]		retract_length
	 *	 W[+units]		retract_length_swap (multi-extruder)
	 *	 F[units/min] retract_feedrate_mm_s
	 *	 Z[units]		 retract_zlift
	 */
	inline void gcode_M207() {
		if (code_seen('S')) retract_length = code_value_axis_units(E_AXIS);
		if (code_seen('F')) retract_feedrate_mm_s = MMM_TO_MMS(code_value_axis_units(E_AXIS));
		if (code_seen('Z')) retract_zlift = code_value_axis_units(Z_AXIS);
		#if EXTRUDERS > 1
			if (code_seen('W')) retract_length_swap = code_value_axis_units(E_AXIS);
		#endif
	}

	/**
	 * M208: Set firmware un-retraction values
	 *
	 *	 S[+units]		retract_recover_length (in addition to M207 S*)
	 *	 W[+units]		retract_recover_length_swap (multi-extruder)
	 *	 F[units/min] retract_recover_feedrate_mm_s
	 */
	inline void gcode_M208() {
		if (code_seen('S')) retract_recover_length = code_value_axis_units(E_AXIS);
		if (code_seen('F')) retract_recover_feedrate_mm_s = MMM_TO_MMS(code_value_axis_units(E_AXIS));
		#if EXTRUDERS > 1
			if (code_seen('W')) retract_recover_length_swap = code_value_axis_units(E_AXIS);
		#endif
	}

	/**
	 * M209: Enable automatic retract (M209 S1)
	 *	 For slicers that don't support G10/11, reversed extrude-only
	 *	 moves will be classified as retraction.
	 */
	inline void gcode_M209() {
		if (code_seen('S')) {
			autoretract_enabled = code_value_bool();
			for (int i = 0; i < EXTRUDERS; i++) retracted[i] = false;
		}
	}

#endif // FWRETRACT

/**
 * M211: Enable, Disable, and/or Report software endstops
 *
 * Usage: M211 S1 to enable, M211 S0 to disable, M211 alone for report
 */
inline void gcode_M211() {
	SERIAL_ECHO_START;
	#if ENABLED(min_software_endstops) || ENABLED(max_software_endstops)
		if (code_seen('S')) soft_endstops_enabled = code_value_bool();
	#endif
	#if ENABLED(min_software_endstops) || ENABLED(max_software_endstops)
		SERIAL_ECHOPGM(MSG_SOFT_ENDSTOPS);
		serialprintPGM(soft_endstops_enabled ? PSTR(MSG_ON) : PSTR(MSG_OFF));
	#else
		SERIAL_ECHOPGM(MSG_SOFT_ENDSTOPS);
		SERIAL_ECHOPGM(MSG_OFF);
	#endif
	SERIAL_ECHOPGM(MSG_SOFT_MIN);
	SERIAL_ECHOPAIR(		MSG_X, soft_endstop_min[X_AXIS]);
	SERIAL_ECHOPAIR(" " MSG_Y, soft_endstop_min[Y_AXIS]);
	SERIAL_ECHOPAIR(" " MSG_Z, soft_endstop_min[Z_AXIS]);
	SERIAL_ECHOPGM(MSG_SOFT_MAX);
	SERIAL_ECHOPAIR(		MSG_X, soft_endstop_max[X_AXIS]);
	SERIAL_ECHOPAIR(" " MSG_Y, soft_endstop_max[Y_AXIS]);
	SERIAL_ECHOLNPAIR(" " MSG_Z, soft_endstop_max[Z_AXIS]);
}

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
	if (code_seen('S')) feedrate_percentage = code_value_int();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {
	if (get_target_extruder_from_command(221)) return;
	if (code_seen('S'))
		flow_percentage[target_extruder] = code_value_int();
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
	if (code_seen('P')) {
		int pin_number = code_value_int(),
				pin_state = code_seen('S') ? code_value_int() : -1; // required pin state - default is inverted

		if (pin_state >= -1 && pin_state <= 1 && pin_number > -1 && !pin_is_protected(pin_number)) {

			int target = LOW;

			stepper.synchronize();

			pinMode(pin_number, INPUT);
			switch (pin_state) {
				case 1:
					target = HIGH;
					break;
				case 0:
					target = LOW;
					break;
				case -1:
					target = !digitalRead(pin_number);
					break;
			}

			while (digitalRead(pin_number) != target) idle();

		} // pin_state -1 0 1 && pin_number > -1
	} // code_seen('P')
}

#if ENABLED(PIDTEMP)

	/**
	 * M301: Set PID parameters P I D (and optionally C, L)
	 *
	 *	 P[float] Kp term
	 *	 I[float] Ki term (unscaled)
	 *	 D[float] Kd term (unscaled)
	 *
	 * With PID_EXTRUSION_SCALING:
	 *
	 *	 C[float] Kc term
	 *	 L[float] LPQ length
	 */
	inline void gcode_M301() {

		// multi-extruder PID patch: M301 updates or prints a single extruder's PID values
		// default behaviour (omitting E parameter) is to update for extruder 0 only
		int e = code_seen('E') ? code_value_int() : 0; // extruder being updated

		if (e < HOTENDS) { // catch bad input value
			if (code_seen('P')) PID_PARAM(Kp, e) = code_value_float();
			if (code_seen('I')) PID_PARAM(Ki, e) = scalePID_i(code_value_float());
			if (code_seen('D')) PID_PARAM(Kd, e) = scalePID_d(code_value_float());
			#if ENABLED(PID_EXTRUSION_SCALING)
				if (code_seen('C')) PID_PARAM(Kc, e) = code_value_float();
				if (code_seen('L')) lpq_len = code_value_float();
				NOMORE(lpq_len, LPQ_MAX_LEN);
			#endif

			thermalManager.updatePID();
			SERIAL_ECHO_START;
			#if ENABLED(PID_PARAMS_PER_HOTEND)
				SERIAL_ECHOPAIR(" e:", e); // specify extruder in serial output
			#endif // PID_PARAMS_PER_HOTEND
			SERIAL_ECHOPAIR(" p:", PID_PARAM(Kp, e));
			SERIAL_ECHOPAIR(" i:", unscalePID_i(PID_PARAM(Ki, e)));
			SERIAL_ECHOPAIR(" d:", unscalePID_d(PID_PARAM(Kd, e)));
			#if ENABLED(PID_EXTRUSION_SCALING)
				//Kc does not have scaling applied above, or in resetting defaults
				SERIAL_ECHOPAIR(" c:", PID_PARAM(Kc, e));
			#endif
			SERIAL_EOL;
		}
		else {
			SERIAL_ERROR_START;
			SERIAL_ERRORLN(MSG_INVALID_EXTRUDER);
		}
	}

#endif // PIDTEMP

#if ENABLED(PIDTEMPBED)

	inline void gcode_M304() {
		if (code_seen('P')) thermalManager.bedKp = code_value_float();
		if (code_seen('I')) thermalManager.bedKi = scalePID_i(code_value_float());
		if (code_seen('D')) thermalManager.bedKd = scalePID_d(code_value_float());

		thermalManager.updatePID();

		SERIAL_ECHO_START;
		SERIAL_ECHOPAIR(" p:", thermalManager.bedKp);
		SERIAL_ECHOPAIR(" i:", unscalePID_i(thermalManager.bedKi));
		SERIAL_ECHOLNPAIR(" d:", unscalePID_d(thermalManager.bedKd));
	}

#endif // PIDTEMPBED

#if defined(CHDK) || HAS_PHOTOGRAPH

	/**
	 * M240: Trigger a camera by emulating a Canon RC-1
	 *			 See http://www.doc-diy.net/photo/rc-1_hacked/
	 */
	inline void gcode_M240() {
		#ifdef CHDK

			OUT_WRITE(CHDK, HIGH);
			chdkHigh = millis();
			chdkActive = true;

		#elif HAS_PHOTOGRAPH

			const uint8_t NUM_PULSES = 16;
			const float PULSE_LENGTH = 0.01524;
			for (int i = 0; i < NUM_PULSES; i++) {
				WRITE(PHOTOGRAPH_PIN, HIGH);
				_delay_ms(PULSE_LENGTH);
				WRITE(PHOTOGRAPH_PIN, LOW);
				_delay_ms(PULSE_LENGTH);
			}
			delay(7.33);
			for (int i = 0; i < NUM_PULSES; i++) {
				WRITE(PHOTOGRAPH_PIN, HIGH);
				_delay_ms(PULSE_LENGTH);
				WRITE(PHOTOGRAPH_PIN, LOW);
				_delay_ms(PULSE_LENGTH);
			}

		#endif // !CHDK && HAS_PHOTOGRAPH
	}

#endif // CHDK || PHOTOGRAPH_PIN

#if HAS_LCD_CONTRAST

	/**
	 * M250: Read and optionally set the LCD contrast
	 */
	inline void gcode_M250() {
		if (code_seen('C')) set_lcd_contrast(code_value_int());
		SERIAL_PROTOCOLPGM("lcd contrast value: ");
		SERIAL_PROTOCOL(lcd_contrast);
		SERIAL_EOL;
	}

#endif // HAS_LCD_CONTRAST

#if ENABLED(PREVENT_COLD_EXTRUSION)

	/**
	 * M302: Allow cold extrudes, or set the minimum extrude temperature
	 *
	 *			 S<temperature> sets the minimum extrude temperature
	 *			 P<bool> enables (1) or disables (0) cold extrusion
	 *
	 *	Examples:
	 *
	 *			 M302				 ; report current cold extrusion state
	 *			 M302 P0			; enable cold extrusion checking
	 *			 M302 P1			; disables cold extrusion checking
	 *			 M302 S0			; always allow extrusion (disables checking)
	 *			 M302 S170		; only allow extrusion above 170
	 *			 M302 S170 P1 ; set min extrude temp to 170 but leave disabled
	 */
	inline void gcode_M302() {
		bool seen_S = code_seen('S');
		if (seen_S) {
			thermalManager.extrude_min_temp = code_value_temp_abs();
			thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0);
		}

		if (code_seen('P'))
			thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0) || code_value_bool();
		else if (!seen_S) {
			// Report current state
			SERIAL_ECHO_START;
			SERIAL_ECHOPAIR("Cold extrudes are ", (thermalManager.allow_cold_extrude ? "en" : "dis"));
			SERIAL_ECHOPAIR("abled (min temp ", int(thermalManager.extrude_min_temp + 0.5));
			SERIAL_ECHOLNPGM("C)");
		}
	}

#endif // PREVENT_COLD_EXTRUSION

/**
 * M303: PID relay autotune
 *
 *			 S<temperature> sets the target temperature. (default 150C)
 *			 E<extruder> (-1 for the bed) (default 0)
 *			 C<cycles>
 *			 U<bool> with a non-zero value will apply the result to current settings
 */
inline void gcode_M303() {
	#if HAS_PID_HEATING
		int e = code_seen('E') ? code_value_int() : 0;
		int c = code_seen('C') ? code_value_int() : 5;
		bool u = code_seen('U') && code_value_bool();

		float temp = code_seen('S') ? code_value_temp_abs() : (e < 0 ? 70.0 : 150.0);

		if (e >= 0 && e < HOTENDS)
			target_extruder = e;

		KEEPALIVE_STATE(NOT_BUSY); // don't send "busy: processing" messages during autotune output

		thermalManager.PID_autotune(temp, e, c, u);

		KEEPALIVE_STATE(IN_HANDLER);
	#else
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM(MSG_ERR_M303_DISABLED);
	#endif
}


#if ENABLED(EXT_SOLENOID)

	void enable_solenoid(uint8_t num) {
		switch (num) {
			case 0:
				OUT_WRITE(SOL0_PIN, HIGH);
				break;
				#if HAS_SOLENOID_1
					case 1:
						OUT_WRITE(SOL1_PIN, HIGH);
						break;
				#endif
				#if HAS_SOLENOID_2
					case 2:
						OUT_WRITE(SOL2_PIN, HIGH);
						break;
				#endif
				#if HAS_SOLENOID_3
					case 3:
						OUT_WRITE(SOL3_PIN, HIGH);
						break;
				#endif
			default:
				SERIAL_ECHO_START;
				SERIAL_ECHOLNPGM(MSG_INVALID_SOLENOID);
				break;
		}
	}

	void enable_solenoid_on_active_extruder() { enable_solenoid(active_extruder); }

	void disable_all_solenoids() {
		OUT_WRITE(SOL0_PIN, LOW);
		OUT_WRITE(SOL1_PIN, LOW);
		OUT_WRITE(SOL2_PIN, LOW);
		OUT_WRITE(SOL3_PIN, LOW);
	}

	/**
	 * M380: Enable solenoid on the active extruder
	 */
	inline void gcode_M380() { enable_solenoid_on_active_extruder(); }

	/**
	 * M381: Disable all solenoids
	 */
	inline void gcode_M381() { disable_all_solenoids(); }

#endif // EXT_SOLENOID

/**
 * M400: Finish all moves
 */
inline void gcode_M400() { stepper.synchronize(); }

#if HAS_BED_PROBE

	/**
	 * M401: Engage Z Servo endstop if available
	 */
	inline void gcode_M401() { DEPLOY_PROBE(); }

	/**
	 * M402: Retract Z Servo endstop if enabled
	 */
	inline void gcode_M402() { STOW_PROBE(); }

#endif // HAS_BED_PROBE

#if ENABLED(FILAMENT_WIDTH_SENSOR)

	/**
	 * M404: Display or set (in current units) the nominal filament width (3mm, 1.75mm ) W<3.0>
	 */
	inline void gcode_M404() {
		if (code_seen('W')) {
			filament_width_nominal = code_value_linear_units();
		}
		else {
			SERIAL_PROTOCOLPGM("Filament dia (nominal mm):");
			SERIAL_PROTOCOLLN(filament_width_nominal);
		}
	}

	/**
	 * M405: Turn on filament sensor for control
	 */
	inline void gcode_M405() {
		// This is technically a linear measurement, but since it's quantized to centimeters and is a different unit than
		// everything else, it uses code_value_int() instead of code_value_linear_units().
		if (code_seen('D')) meas_delay_cm = code_value_int();
		NOMORE(meas_delay_cm, MAX_MEASUREMENT_DELAY);

		if (filwidth_delay_index[1] == -1) { // Initialize the ring buffer if not done since startup
			int temp_ratio = thermalManager.widthFil_to_size_ratio();

			for (uint8_t i = 0; i < COUNT(measurement_delay); ++i)
				measurement_delay[i] = temp_ratio - 100;	// Subtract 100 to scale within a signed byte

			filwidth_delay_index[0] = filwidth_delay_index[1] = 0;
		}

		filament_sensor = true;

		//SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
		//SERIAL_PROTOCOL(filament_width_meas);
		//SERIAL_PROTOCOLPGM("Extrusion ratio(%):");
		//SERIAL_PROTOCOL(flow_percentage[active_extruder]);
	}

	/**
	 * M406: Turn off filament sensor for control
	 */
	inline void gcode_M406() { filament_sensor = false; }

	/**
	 * M407: Get measured filament diameter on serial output
	 */
	inline void gcode_M407() {
		SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
		SERIAL_PROTOCOLLN(filament_width_meas);
	}

#endif // FILAMENT_WIDTH_SENSOR

void quickstop_stepper() {
	stepper.quick_stop();
	stepper.synchronize();
	set_current_from_steppers_for_axis(ALL_AXES);
	SYNC_PLAN_POSITION_KINEMATIC();
}

#if PLANNER_LEVELING
	/**
	 * M420: Enable/Disable Bed Leveling and/or set the Z fade height.
	 *
	 *			 S[bool]	 Turns leveling on or off
	 *			 Z[height] Sets the Z fade height (0 or none to disable)
	 */
	inline void gcode_M420() {
		if (code_seen('S')) set_bed_leveling_enabled(code_value_bool());
		#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
			if (code_seen('Z')) set_z_fade_height(code_value_linear_units());
		#endif
	}
#endif


/**
 * M428: Set home_offset based on the distance between the
 *			 current_position and the nearest "reference point."
 *			 If an axis is past center its endstop position
 *			 is the reference-point. Otherwise it uses 0. This allows
 *			 the Z offset to be set near the bed when using a max endstop.
 *
 *			 M428 can't be used more than 2cm away from 0 or an endstop.
 *
 *			 Use M206 to set these values directly.
 */
inline void gcode_M428() {
	bool err = false;
	LOOP_XYZ(i) {
		if (axis_homed[i]) {
			float base = (current_position[i] > (soft_endstop_min[i] + soft_endstop_max[i]) * 0.5) ? base_home_pos((AxisEnum)i) : 0,
						diff = current_position[i] - LOGICAL_POSITION(base, i);
			if (diff > -20 && diff < 20) {
				set_home_offset((AxisEnum)i, home_offset[i] - diff);
			}
			else {
				SERIAL_ERROR_START;
				SERIAL_ERRORLNPGM(MSG_ERR_M428_TOO_FAR);
				NOOP;
				BUZZ(200, 40);
				err = true;
				break;
			}
		}
	}

	if (!err) {
		SYNC_PLAN_POSITION_KINEMATIC();
		report_current_position();
        NOOP;
		BUZZ(200, 659);
		BUZZ(200, 698);
	}
}

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
	Config_StoreSettings();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
	Config_RetrieveSettings();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
	Config_ResetDefault();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
	Config_PrintSettings(code_seen('S') && !code_value_bool());
}

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)

	/**
	 * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)
	 */
	inline void gcode_M540() {
		if (code_seen('S')) stepper.abort_on_endstop_hit = code_value_bool();
	}

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#if HAS_BED_PROBE

	inline void gcode_M851() {

		SERIAL_ECHO_START;
		SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET);
		SERIAL_CHAR(' ');

		if (code_seen('Z')) {
			float value = code_value_axis_units(Z_AXIS);
			if (Z_PROBE_OFFSET_RANGE_MIN <= value && value <= Z_PROBE_OFFSET_RANGE_MAX) {
				zprobe_zoffset = value;
				SERIAL_ECHO(zprobe_zoffset);
			}
			else {
				SERIAL_ECHOPAIR(MSG_Z_MIN, Z_PROBE_OFFSET_RANGE_MIN);
				SERIAL_CHAR(' ');
				SERIAL_ECHOPAIR(MSG_Z_MAX, Z_PROBE_OFFSET_RANGE_MAX);
			}
		}
		else {
			SERIAL_ECHOPAIR(": ", zprobe_zoffset);
		}

		SERIAL_EOL;
	}

#endif // HAS_BED_PROBE

#if ENABLED(FILAMENT_CHANGE_FEATURE)

	/**
	 * M600: Pause for filament change
	 *
	 *	E[distance] - Retract the filament this far (negative value)
	 *	Z[distance] - Move the Z axis by this distance
	 *	X[position] - Move to this X position, with Y
	 *	Y[position] - Move to this Y position, with X
	 *	L[distance] - Retract distance for removal (manual reload)
	 *
	 *	Default values are used for omitted arguments.
	 *
	 */
	inline void gcode_M600() {

		if (thermalManager.tooColdToExtrude(active_extruder)) {
			SERIAL_ERROR_START;
			SERIAL_ERRORLNPGM(MSG_TOO_COLD_FOR_M600);
			return;
		}

		// Show initial message and wait for synchronize steppers
		lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_INIT);
		stepper.synchronize();

		float lastpos[NUM_AXIS];

		// Save current position of all axes
		LOOP_XYZE(i)
			lastpos[i] = destination[i] = current_position[i];

		#define RUNPLAN(RATE_MM_S) line_to_destination(RATE_MM_S);

		KEEPALIVE_STATE(IN_HANDLER);

		// Initial retract before move to filament change position
		if (code_seen('E')) destination[E_AXIS] += code_value_axis_units(E_AXIS);
		#if defined(FILAMENT_CHANGE_RETRACT_LENGTH) && FILAMENT_CHANGE_RETRACT_LENGTH > 0
			else destination[E_AXIS] -= FILAMENT_CHANGE_RETRACT_LENGTH;
		#endif

		RUNPLAN(FILAMENT_CHANGE_RETRACT_FEEDRATE);

		// Lift Z axis
		float z_lift = code_seen('Z') ? code_value_axis_units(Z_AXIS) :
			#if defined(FILAMENT_CHANGE_Z_ADD) && FILAMENT_CHANGE_Z_ADD > 0
				FILAMENT_CHANGE_Z_ADD
			#else
				0
			#endif
		;

		if (z_lift > 0) {
			destination[Z_AXIS] += z_lift;
			NOMORE(destination[Z_AXIS], Z_MAX_POS);
			RUNPLAN(FILAMENT_CHANGE_Z_FEEDRATE);
		}

		// Move XY axes to filament exchange position
		if (code_seen('X')) destination[X_AXIS] = code_value_axis_units(X_AXIS);
		#ifdef FILAMENT_CHANGE_X_POS
			else destination[X_AXIS] = FILAMENT_CHANGE_X_POS;
		#endif

		if (code_seen('Y')) destination[Y_AXIS] = code_value_axis_units(Y_AXIS);
		#ifdef FILAMENT_CHANGE_Y_POS
			else destination[Y_AXIS] = FILAMENT_CHANGE_Y_POS;
		#endif

		RUNPLAN(FILAMENT_CHANGE_XY_FEEDRATE);

		stepper.synchronize();
		lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_UNLOAD);

		// Unload filament
		if (code_seen('L')) destination[E_AXIS] += code_value_axis_units(E_AXIS);
		#if defined(FILAMENT_CHANGE_UNLOAD_LENGTH) && FILAMENT_CHANGE_UNLOAD_LENGTH > 0
			else destination[E_AXIS] -= FILAMENT_CHANGE_UNLOAD_LENGTH;
		#endif

		RUNPLAN(FILAMENT_CHANGE_UNLOAD_FEEDRATE);

		// Synchronize steppers and then disable extruders steppers for manual filament changing
		stepper.synchronize();
		disable_e0();
		disable_e1();
		disable_e2();
		disable_e3();
		delay(100);

		// Wait for filament insert by user and press button
		lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_INSERT);

		// LCD click or M108 will clear this
		wait_for_user = true;

		while (wait_for_user) {
			idle(true);
		}

		// Show load message
		lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_LOAD);

		// Load filament
		if (code_seen('L')) destination[E_AXIS] -= code_value_axis_units(E_AXIS);
		#if defined(FILAMENT_CHANGE_LOAD_LENGTH) && FILAMENT_CHANGE_LOAD_LENGTH > 0
			else destination[E_AXIS] += FILAMENT_CHANGE_LOAD_LENGTH;
		#endif

		RUNPLAN(FILAMENT_CHANGE_LOAD_FEEDRATE);
		stepper.synchronize();

		#if defined(FILAMENT_CHANGE_EXTRUDE_LENGTH) && FILAMENT_CHANGE_EXTRUDE_LENGTH > 0
			do {
				// Extrude filament to get into hotend
				lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_EXTRUDE);
				destination[E_AXIS] += FILAMENT_CHANGE_EXTRUDE_LENGTH;
				RUNPLAN(FILAMENT_CHANGE_EXTRUDE_FEEDRATE);
				stepper.synchronize();
				// Ask user if more filament should be extruded
				KEEPALIVE_STATE(PAUSED_FOR_USER);
				lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_OPTION);
				while (filament_change_menu_response == FILAMENT_CHANGE_RESPONSE_WAIT_FOR) idle(true);
				KEEPALIVE_STATE(IN_HANDLER);
			} while (filament_change_menu_response != FILAMENT_CHANGE_RESPONSE_RESUME_PRINT);
		#endif

		lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_RESUME);

		KEEPALIVE_STATE(IN_HANDLER);

		// Set extruder to saved position
		destination[E_AXIS] = current_position[E_AXIS] = lastpos[E_AXIS];
		planner.set_e_position_mm(current_position[E_AXIS]);

		// Move XY to starting position, then Z
        destination[X_AXIS] = lastpos[X_AXIS];
        destination[Y_AXIS] = lastpos[Y_AXIS];
        RUNPLAN(FILAMENT_CHANGE_XY_FEEDRATE);
        destination[Z_AXIS] = lastpos[Z_AXIS];
        RUNPLAN(FILAMENT_CHANGE_Z_FEEDRATE);

		stepper.synchronize();

		#if ENABLED(FILAMENT_RUNOUT_SENSOR)
			filament_ran_out = false;
		#endif

		// Show status screen
		lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_STATUS);
	}

#endif // FILAMENT_CHANGE_FEATURE


#if ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)

	inline void gcode_M605() {
		stepper.synchronize();
		extruder_duplication_enabled = code_seen('S') && code_value_int() == 2;
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPAIR(MSG_DUPLICATION_MODE, extruder_duplication_enabled ? MSG_ON : MSG_OFF);
	}

#endif // M605

#if ENABLED(LIN_ADVANCE)
	/**
	 * M905: Set advance factor
	 */
	inline void gcode_M905() {
		stepper.synchronize();
		planner.advance_M905(code_seen('K') ? code_value_float() : -1.0);
	}
#endif

/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
	#if HAS_MOTOR_CURRENT_PWM
		#if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
			if (code_seen('X')) stepper.digipot_current(0, code_value_int());
		#endif
		#if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
			if (code_seen('Z')) stepper.digipot_current(1, code_value_int());
		#endif
		#if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
			if (code_seen('E')) stepper.digipot_current(2, code_value_int());
		#endif
	#endif
	#if ENABLED(DIGIPOT_I2C)
		// this one uses actual amps in floating point
		LOOP_XYZE(i) if (code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value_float());
		// for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
		for (int i = NUM_AXIS; i < DIGIPOT_I2C_NUM_CHANNELS; i++) if (code_seen('B' + i - (NUM_AXIS))) digipot_i2c_set_current(i, code_value_float());
	#endif

}

#if HAS_MICROSTEPS

	// M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
	inline void gcode_M350() {
		if (code_seen('S')) for (int i = 0; i <= 4; i++) stepper.microstep_mode(i, code_value_byte());
		LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_mode(i, code_value_byte());
		if (code_seen('B')) stepper.microstep_mode(4, code_value_byte());
		stepper.microstep_readings();
	}

	/**
	 * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
	 *			 S# determines MS1 or MS2, X# sets the pin high/low.
	 */
	inline void gcode_M351() {
		if (code_seen('S')) switch (code_value_byte()) {
			case 1:
				LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_ms(i, code_value_byte(), -1);
				if (code_seen('B')) stepper.microstep_ms(4, code_value_byte(), -1);
				break;
			case 2:
				LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_ms(i, -1, code_value_byte());
				if (code_seen('B')) stepper.microstep_ms(4, -1, code_value_byte());
				break;
		}
		stepper.microstep_readings();
	}

#endif // HAS_MICROSTEPS

#if HAS_CASE_LIGHT

	uint8_t case_light_brightness = 255;

	void update_case_light() {
		digitalWrite(CASE_LIGHT_PIN, case_light_on != INVERT_CASE_LIGHT ? HIGH : LOW);
		analogWrite(CASE_LIGHT_PIN, case_light_on != INVERT_CASE_LIGHT ? case_light_brightness : 0);
	}

	/**
	 * M355: Turn case lights on/off and set brightness
	 *
	 *	 S<bool>	Turn case light on or off
	 *	 P<byte>	Set case light brightness (PWM pin required)
	 */
	inline void gcode_M355() {
		if (code_seen('P')) case_light_brightness = code_value_byte();
		if (code_seen('S')) case_light_on = code_value_bool();
		update_case_light();
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM("Case lights ");
		case_light_on ? SERIAL_ECHOLNPGM("on") : SERIAL_ECHOLNPGM("off");
	}

#endif // HAS_CASE_LIGHT


/**
 * M999: Restart after being stopped
 *
 * Default behaviour is to flush the serial buffer and request
 * a resend to the host starting on the last N line received.
 *
 * Sending "M999 S1" will resume printing without flushing the
 * existing command buffer.
 *
 */
inline void gcode_M999() {
	Running = true;

	if (code_seen('S') && code_value_bool()) return;

	// gcode_LastN = Stopped_gcode_LastN;
	FlushSerialRequestResend();
}


inline void invalid_extruder_error(const uint8_t &e) {
	SERIAL_ECHO_START;
	SERIAL_CHAR('T');
	SERIAL_PROTOCOL_F(e, DEC);
	SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
}

/**
 * Perform a tool-change, which may result in moving the
 * previous tool out of the way and the new tool into place.
 */
void tool_change(const uint8_t tmp_extruder, const float fr_mm_s/*=0.0*/, bool no_move/*=false*/) {
    // Set the new active extruder
    active_extruder = tmp_extruder;

    UNUSED(fr_mm_s);
    UNUSED(no_move);

    SERIAL_ECHO_START;
    SERIAL_ECHOLNPAIR(MSG_ACTIVE_EXTRUDER, (int)active_extruder);

}

/**
 * T0-T3: Switch tool, usually switching extruders
 *
 *	 F[units/min] Set the movement feedrate
 *	 S1					 Don't move the tool in XY after change
 */
inline void gcode_T(uint8_t tmp_extruder) {

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			SERIAL_ECHOPAIR(">>> gcode_T(", tmp_extruder);
			SERIAL_CHAR(')');
			SERIAL_EOL;
			DEBUG_POS("BEFORE", current_position);
		}
	#endif

    tool_change(tmp_extruder);

	#if ENABLED(DEBUG_LEVELING_FEATURE)
		if (DEBUGGING(LEVELING)) {
			DEBUG_POS("AFTER", current_position);
			SERIAL_ECHOLNPGM("<<< gcode_T");
		}
	#endif
}

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_next_command() {
	current_command = command_queue[cmd_queue_index_r];

	if (DEBUGGING(ECHO)) {
		SERIAL_ECHO_START;
		SERIAL_ECHOLN(current_command);
	}

	// Sanitize the current command:
	//	- Skip leading spaces
	//	- Bypass N[-0-9][0-9]*[ ]*
	//	- Overwrite * with nul to mark the end
	while (*current_command == ' ') ++current_command;
	if (*current_command == 'N' && NUMERIC_SIGNED(current_command[1])) {
		current_command += 2; // skip N[-0-9]
		while (NUMERIC(*current_command)) ++current_command; // skip [0-9]*
		while (*current_command == ' ') ++current_command; // skip [ ]*
	}
	char* starpos = strchr(current_command, '*');	// * should always be the last parameter
	if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

	char *cmd_ptr = current_command;

	// Get the command code, which must be G, M, or T
	char command_code = *cmd_ptr++;

	// Skip spaces to get the numeric part
	while (*cmd_ptr == ' ') cmd_ptr++;

	// Allow for decimal point in command
	#if ENABLED(G38_PROBE_TARGET)
		uint8_t subcode = 0;
	#endif

	uint16_t codenum = 0; // define ahead of goto

	// Bail early if there's no code
	bool code_is_good = NUMERIC(*cmd_ptr);
	if (!code_is_good) goto ExitUnknownCommand;

	// Get and skip the code number
	do {
		codenum = (codenum * 10) + (*cmd_ptr - '0');
		cmd_ptr++;
	} while (NUMERIC(*cmd_ptr));

	// Allow for decimal point in command
	#if ENABLED(G38_PROBE_TARGET)
		if (*cmd_ptr == '.') {
			cmd_ptr++;
			while (NUMERIC(*cmd_ptr))
				subcode = (subcode * 10) + (*cmd_ptr++ - '0');
		}
	#endif

	// Skip all spaces to get to the first argument, or nul
	while (*cmd_ptr == ' ') cmd_ptr++;

	// The command's arguments (if any) start here, for sure!
	current_command_args = cmd_ptr;

	KEEPALIVE_STATE(IN_HANDLER);

	// Handle a known G, M, or T
	switch (command_code) {
		case 'G': switch (codenum) {

			// G0, G1
			case 0:
			case 1:
				gcode_G0_G1();
				break;

			// G2, G3
			#if ENABLED(ARC_SUPPORT) && DISABLED(SCARA)
				case 2: // G2	- CW ARC
				case 3: // G3	- CCW ARC
					gcode_G2_G3(codenum == 2);
					break;
			#endif

			// G4 Dwell
			case 4:
				gcode_G4();
				break;

			#if ENABLED(FWRETRACT)
				case 10: // G10: retract
				case 11: // G11: retract_recover
					gcode_G10_G11(codenum == 10);
					break;
			#endif // FWRETRACT

			#if ENABLED(NOZZLE_CLEAN_FEATURE)
				case 12:
					gcode_G12(); // G12: Nozzle Clean
					break;
			#endif // NOZZLE_CLEAN_FEATURE

			#if ENABLED(INCH_MODE_SUPPORT)
				case 20: //G20: Inch Mode
					gcode_G20();
					break;

				case 21: //G21: MM Mode
					gcode_G21();
					break;
			#endif // INCH_MODE_SUPPORT

			#if ENABLED(NOZZLE_PARK_FEATURE)
				case 27: // G27: Nozzle Park
					gcode_G27();
					break;
			#endif // NOZZLE_PARK_FEATURE

			case 28: // G28: Home all axes, one at a time
				gcode_G28();
				break;

			#if PLANNER_LEVELING
				case 29: // G29 Detailed Z probe, probes the bed at 3 or more points.
					gcode_G29();
					break;
			#endif // PLANNER_LEVELING

			#if HAS_BED_PROBE

				case 30: // G30 Single Z probe
					gcode_G30();
					break;

				#if ENABLED(Z_PROBE_SLED)

						case 31: // G31: dock the sled
							gcode_G31();
							break;

						case 32: // G32: undock the sled
							gcode_G32();
							break;

				#endif // Z_PROBE_SLED
			#endif // HAS_BED_PROBE

			#if ENABLED(G38_PROBE_TARGET)
				case 38: // G38.2 & G38.3
					if (subcode == 2 || subcode == 3)
						gcode_G38(subcode == 2);
					break;
			#endif

			case 90: // G90
				relative_mode = false;
				break;
			case 91: // G91
				relative_mode = true;
				break;

			case 92: // G92
				gcode_G92();
				break;
		}
		break;

		case 'M': switch (codenum) {
			#if ENABLED(EMERGENCY_PARSER)
				case 0: // M0: Unconditional stop - Wait for user button press on LCD
				case 1: // M1: Conditional stop - Wait for user button press on LCD
					gcode_M0_M1();
					break;
			#endif

			case 17: // M17: Enable all stepper motors
				gcode_M17();
				break;



			case 31: // M31: Report time since the start of SD print or last M109
				gcode_M31(); break;

			case 42: // M42: Change pin state
				gcode_M42(); break;

			#if ENABLED(PINS_DEBUGGING)
				case 43: // M43: Read pin state
					gcode_M43(); break;
			#endif

			#if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
				case 48: // M48: Z probe repeatability test
					gcode_M48();
					break;
			#endif // Z_MIN_PROBE_REPEATABILITY_TEST

			case 75: // M75: Start print timer
				gcode_M75(); break;
			case 76: // M76: Pause print timer
				gcode_M76(); break;
			case 77: // M77: Stop print timer
				gcode_M77(); break;

			#if ENABLED(PRINTCOUNTER)
				case 78: // M78: Show print statistics
					gcode_M78(); break;
			#endif

			#if ENABLED(M100_FREE_MEMORY_WATCHER)
				case 100: // M100: Free Memory Report
					gcode_M100();
					break;
			#endif

			case 104: // M104: Set hot end temperature
				gcode_M104();
				break;

			case 110: // M110: Set Current Line Number
				gcode_M110();
				break;

			case 111: // M111: Set debug level
				gcode_M111();
				break;

			#if DISABLED(EMERGENCY_PARSER)

				case 108: // M108: Cancel Waiting
					gcode_M108();
					break;

				case 112: // M112: Emergency Stop
					gcode_M112();
					break;

				case 410: // M410 quickstop - Abort all the planned moves.
					gcode_M410();
					break;

			#endif

			#if ENABLED(HOST_KEEPALIVE_FEATURE)
				case 113: // M113: Set Host Keepalive interval
					gcode_M113();
					break;
			#endif

			case 140: // M140: Set bed temperature
				gcode_M140();
				break;

			case 105: // M105: Report current temperature
				gcode_M105();
				KEEPALIVE_STATE(NOT_BUSY);
				return; // "ok" already printed

			#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
				case 155: // M155: Set temperature auto-report interval
					gcode_M155();
					break;
			#endif

			case 109: // M109: Wait for hotend temperature to reach target
				gcode_M109();
				break;

			#if HAS_TEMP_BED
				case 190: // M190: Wait for bed temperature to reach target
					gcode_M190();
					break;
			#endif // HAS_TEMP_BED

			#if FAN_COUNT > 0
				case 106: // M106: Fan On
					gcode_M106();
					break;
				case 107: // M107: Fan Off
					gcode_M107();
					break;
			#endif // FAN_COUNT > 0

			#if ENABLED(BARICUDA)
				// PWM for HEATER_1_PIN
				#if HAS_HEATER_1
					case 126: // M126: valve open
						gcode_M126();
						break;
					case 127: // M127: valve closed
						gcode_M127();
						break;
				#endif // HAS_HEATER_1

				// PWM for HEATER_2_PIN
				#if HAS_HEATER_2
					case 128: // M128: valve open
						gcode_M128();
						break;
					case 129: // M129: valve closed
						gcode_M129();
						break;
				#endif // HAS_HEATER_2
			#endif // BARICUDA

			#if HAS_POWER_SWITCH

				case 80: // M80: Turn on Power Supply
					gcode_M80();
					break;

			#endif // HAS_POWER_SWITCH

			case 81: // M81: Turn off Power, including Power Supply, if possible
				gcode_M81();
				break;

			case 82: // M83: Set E axis normal mode (same as other axes)
				gcode_M82();
				break;
			case 83: // M83: Set E axis relative mode
				gcode_M83();
				break;
			case 18: // M18 => M84
			case 84: // M84: Disable all steppers or set timeout
				gcode_M18_M84();
				break;
			case 85: // M85: Set inactivity stepper shutdown timeout
				gcode_M85();
				break;
			case 92: // M92: Set the steps-per-unit for one or more axes
				gcode_M92();
				break;
			case 114: // M114: Report current position
				gcode_M114();
				break;
			case 115: // M115: Report capabilities
				gcode_M115();
				break;
			case 117: // M117: Set LCD message text, if possible
				gcode_M117();
				break;
			case 119: // M119: Report endstop states
				gcode_M119();
				break;
			case 120: // M120: Enable endstops
				gcode_M120();
				break;
			case 121: // M121: Disable endstops
				gcode_M121();
				break;

			#if ENABLED(HAVE_TMC2130DRIVER)
				case 122: // M122: Diagnose, used to debug TMC2130
					gcode_M122();
					break;
			#endif

			#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
				case 149: // M149: Set temperature units
					gcode_M149();
					break;
			#endif

			#if ENABLED(BLINKM) || ENABLED(RGB_LED)

				case 150: // M150: Set Status LED Color
					gcode_M150();
					break;

			#endif // BLINKM

			case 200: // M200: Set filament diameter, E to cubic units
				gcode_M200();
				break;
			case 201: // M201: Set max acceleration for print moves (units/s^2)
				gcode_M201();
				break;
			#if 0 // Not used for Sprinter/grbl gen6
				case 202: // M202
					gcode_M202();
					break;
			#endif
			case 203: // M203: Set max feedrate (units/sec)
				gcode_M203();
				break;
			case 204: // M204: Set acceleration
				gcode_M204();
				break;
			case 205: //M205: Set advanced settings
				gcode_M205();
				break;
			case 206: // M206: Set home offsets
				gcode_M206();
				break;

			#if ENABLED(Z_DUAL_ENDSTOPS)
				case 666: // M666: Set delta or dual endstop adjustment
					gcode_M666();
					break;
			#endif

			#if ENABLED(FWRETRACT)
				case 207: // M207: Set Retract Length, Feedrate, and Z lift
					gcode_M207();
					break;
				case 208: // M208: Set Recover (unretract) Additional Length and Feedrate
					gcode_M208();
					break;
				case 209: // M209: Turn Automatic Retract Detection on/off
					gcode_M209();
					break;
			#endif // FWRETRACT

			case 211: // M211: Enable, Disable, and/or Report software endstops
				gcode_M211();
				break;

			case 220: // M220: Set Feedrate Percentage: S<percent> ("FR" on your LCD)
				gcode_M220();
				break;

			case 221: // M221: Set Flow Percentage
				gcode_M221();
				break;

			case 226: // M226: Wait until a pin reaches a state
				gcode_M226();
				break;

			#if ENABLED(PIDTEMP)
				case 301: // M301: Set hotend PID parameters
					gcode_M301();
					break;
			#endif // PIDTEMP

			#if ENABLED(PIDTEMPBED)
				case 304: // M304: Set bed PID parameters
					gcode_M304();
					break;
			#endif // PIDTEMPBED

			#if defined(CHDK) || HAS_PHOTOGRAPH
				case 240: // M240: Trigger a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
					gcode_M240();
					break;
			#endif // CHDK || PHOTOGRAPH_PIN

			#if HAS_LCD_CONTRAST
				case 250: // M250: Set LCD contrast
					gcode_M250();
					break;
			#endif // HAS_LCD_CONTRAST

			#if ENABLED(PREVENT_COLD_EXTRUSION)
				case 302: // M302: Allow cold extrudes (set the minimum extrude temperature)
					gcode_M302();
					break;
			#endif // PREVENT_COLD_EXTRUSION

			case 303: // M303: PID autotune
				gcode_M303();
				break;

			case 400: // M400: Finish all moves
				gcode_M400();
				break;

			#if HAS_BED_PROBE
				case 401: // M401: Deploy probe
					gcode_M401();
					break;
				case 402: // M402: Stow probe
					gcode_M402();
					break;
			#endif // HAS_BED_PROBE

			#if ENABLED(FILAMENT_WIDTH_SENSOR)
				case 404:	// M404: Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
					gcode_M404();
					break;
				case 405:	// M405: Turn on filament sensor for control
					gcode_M405();
					break;
				case 406:	// M406: Turn off filament sensor for control
					gcode_M406();
					break;
				case 407:	 // M407: Display measured filament diameter
					gcode_M407();
					break;
			#endif // ENABLED(FILAMENT_WIDTH_SENSOR)

			#if PLANNER_LEVELING
				case 420: // M420: Enable/Disable Bed Leveling
					gcode_M420();
					break;
            #endif

			case 428: // M428: Apply current_position to home_offset
				gcode_M428();
				break;

			case 500: // M500: Store settings in EEPROM
				gcode_M500();
				break;
			case 501: // M501: Read settings from EEPROM
				gcode_M501();
				break;
			case 502: // M502: Revert to default settings
				gcode_M502();
				break;
			case 503: // M503: print settings currently in memory
				gcode_M503();
				break;

			#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
				case 540: // M540: Set abort on endstop hit for SD printing
					gcode_M540();
					break;
			#endif

			#if HAS_BED_PROBE
				case 851: // M851: Set Z Probe Z Offset
					gcode_M851();
					break;
			#endif // HAS_BED_PROBE

			#if ENABLED(FILAMENT_CHANGE_FEATURE)
				case 600: // M600: Pause for filament change
					gcode_M600();
					break;
			#endif // FILAMENT_CHANGE_FEATURE

			#if ENABLED(LIN_ADVANCE)
				case 905: // M905: Set advance K factor.
					gcode_M905();
					break;
			#endif

			case 907: // M907: Set digital trimpot motor current using axis codes.
				gcode_M907();
				break;

			#if HAS_MICROSTEPS

				case 350: // M350: Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
					gcode_M350();
					break;

				case 351: // M351: Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
					gcode_M351();
					break;

			#endif // HAS_MICROSTEPS

			#if HAS_CASE_LIGHT

				case 355: // M355 Turn case lights on/off
					gcode_M355();
					break;

			#endif // HAS_CASE_LIGHT

			case 999: // M999: Restart after being Stopped
				gcode_M999();
				break;
		}
		break;

		case 'T':
			gcode_T(codenum);
			break;

		default: code_is_good = false;
	}

	KEEPALIVE_STATE(NOT_BUSY);

ExitUnknownCommand:

	// Still unknown command? Throw an error
	if (!code_is_good) unknown_command_error();

	ok_to_send();
}

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void FlushSerialRequestResend() {
	//char command_queue[cmd_queue_index_r][100]="Resend:";
	MYSERIAL.flush();
	SERIAL_PROTOCOLPGM(MSG_RESEND);
	SERIAL_PROTOCOLLN(gcode_LastN + 1);
	ok_to_send();
}

/**
 * Send an "ok" message to the host, indicating
 * that a command was successfully processed.
 *
 * If ADVANCED_OK is enabled also include:
 *	 N<int>	Line number of the command, if any
 *	 P<int>	Planner space remaining
 *	 B<int>	Block queue space remaining
 */
void ok_to_send() {
	refresh_cmd_timeout();
	if (!send_ok[cmd_queue_index_r]) return;
	SERIAL_PROTOCOLPGM(MSG_OK);
	#if ENABLED(ADVANCED_OK)
		char* p = command_queue[cmd_queue_index_r];
		if (*p == 'N') {
			SERIAL_PROTOCOL(' ');
			SERIAL_ECHO(*p++);
			while (NUMERIC_SIGNED(*p))
				SERIAL_ECHO(*p++);
		}
		SERIAL_PROTOCOLPGM(" P"); SERIAL_PROTOCOL(int(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
		SERIAL_PROTOCOLPGM(" B"); SERIAL_PROTOCOL(BUFSIZE - commands_in_queue);
	#endif
	SERIAL_EOL;
}

#if ENABLED(min_software_endstops) || ENABLED(max_software_endstops)

	/**
	 * Constrain the given coordinates to the software endstops.
	 */
	void clamp_to_software_endstops(float target[XYZ]) {
		#if ENABLED(min_software_endstops)
			NOLESS(target[X_AXIS], soft_endstop_min[X_AXIS]);
			NOLESS(target[Y_AXIS], soft_endstop_min[Y_AXIS]);
			NOLESS(target[Z_AXIS], soft_endstop_min[Z_AXIS]);
		#endif
		#if ENABLED(max_software_endstops)
			NOMORE(target[X_AXIS], soft_endstop_max[X_AXIS]);
			NOMORE(target[Y_AXIS], soft_endstop_max[Y_AXIS]);
			NOMORE(target[Z_AXIS], soft_endstop_max[Z_AXIS]);
		#endif
	}

#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

	#if ENABLED(ABL_BILINEAR_SUBDIVISION)
		#define ABL_BG_SPACING(A) bilinear_grid_spacing_virt[A]
		#define ABL_BG_POINTS_X	 ABL_GRID_POINTS_VIRT_X
		#define ABL_BG_POINTS_Y	 ABL_GRID_POINTS_VIRT_Y
		#define ABL_BG_GRID(X,Y)	bed_level_grid_virt[X][Y]
	#else
		#define ABL_BG_SPACING(A) bilinear_grid_spacing[A]
		#define ABL_BG_POINTS_X	 ABL_GRID_POINTS_X
		#define ABL_BG_POINTS_Y	 ABL_GRID_POINTS_Y
		#define ABL_BG_GRID(X,Y)	bed_level_grid[X][Y]
	#endif

	// Get the Z adjustment for non-linear bed leveling
	float bilinear_z_offset(float cartesian[XYZ]) {

		// XY relative to the probed area
		const float x = RAW_X_POSITION(cartesian[X_AXIS]) - bilinear_start[X_AXIS],
								y = RAW_Y_POSITION(cartesian[Y_AXIS]) - bilinear_start[Y_AXIS];

		// Convert to grid box units
		float ratio_x = x / ABL_BG_SPACING(X_AXIS),
					ratio_y = y / ABL_BG_SPACING(Y_AXIS);

		// Whole units for the grid line indices. Constrained within bounds.
		const int gridx = constrain(floor(ratio_x), 0, ABL_BG_POINTS_X - 1),
							gridy = constrain(floor(ratio_y), 0, ABL_BG_POINTS_Y - 1),
							nextx = min(gridx + 1, ABL_BG_POINTS_X - 1),
							nexty = min(gridy + 1, ABL_BG_POINTS_Y - 1);

		// Subtract whole to get the ratio within the grid box
		ratio_x -= gridx; ratio_y -= gridy;

		// Never less than 0.0. (Over 1.0 is fine due to previous contraints.)
		NOLESS(ratio_x, 0); NOLESS(ratio_y, 0);

		// Z at the box corners
		const float z1 = ABL_BG_GRID(gridx, gridy),	// left-front
								z2 = ABL_BG_GRID(gridx, nexty),	// left-back
								z3 = ABL_BG_GRID(nextx, gridy),	// right-front
								z4 = ABL_BG_GRID(nextx, nexty),	// right-back

								// Bilinear interpolate
								L = z1 + (z2 - z1) * ratio_y,	 // Linear interp. LF -> LB
								R = z3 + (z4 - z3) * ratio_y,	 // Linear interp. RF -> RB
								offset = L + ratio_x * (R - L);

		/*
		static float last_offset = 0;
		if (fabs(last_offset - offset) > 0.2) {
			SERIAL_ECHOPGM("Sudden Shift at ");
			SERIAL_ECHOPAIR("x=", x);
			SERIAL_ECHOPAIR(" / ", bilinear_grid_spacing[X_AXIS]);
			SERIAL_ECHOLNPAIR(" -> gridx=", gridx);
			SERIAL_ECHOPAIR(" y=", y);
			SERIAL_ECHOPAIR(" / ", bilinear_grid_spacing[Y_AXIS]);
			SERIAL_ECHOLNPAIR(" -> gridy=", gridy);
			SERIAL_ECHOPAIR(" ratio_x=", ratio_x);
			SERIAL_ECHOLNPAIR(" ratio_y=", ratio_y);
			SERIAL_ECHOPAIR(" z1=", z1);
			SERIAL_ECHOPAIR(" z2=", z2);
			SERIAL_ECHOPAIR(" z3=", z3);
			SERIAL_ECHOLNPAIR(" z4=", z4);
			SERIAL_ECHOPAIR(" L=", L);
			SERIAL_ECHOPAIR(" R=", R);
			SERIAL_ECHOLNPAIR(" offset=", offset);
		}
		last_offset = offset;
		//*/

		return offset;
	}

#endif // AUTO_BED_LEVELING_BILINEAR

/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for current_position, etc.
 */
void get_cartesian_from_steppers() {
    cartes[X_AXIS] = stepper.get_axis_position_mm(X_AXIS);
    cartes[Y_AXIS] = stepper.get_axis_position_mm(Y_AXIS);
    cartes[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
	get_cartesian_from_steppers();
	#if PLANNER_LEVELING
		planner.unapply_leveling(cartes);
	#endif
	if (axis == ALL_AXES)
		memcpy(current_position, cartes, sizeof(cartes));
	else
		current_position[axis] = cartes[axis];
}


#if ENABLED(AUTO_BED_LEVELING_BILINEAR) && !IS_KINEMATIC

	#define CELL_INDEX(A,V) ((RAW_##A##_POSITION(V) - bilinear_start[A##_AXIS]) / ABL_BG_SPACING(A##_AXIS))

	/**
	 * Prepare a bilinear-leveled linear move on Cartesian,
	 * splitting the move where it crosses grid borders.
	 */
	void bilinear_line_to_destination(float fr_mm_s, uint16_t x_splits = 0xFFFF, uint16_t y_splits = 0xFFFF) {
		int cx1 = CELL_INDEX(X, current_position[X_AXIS]),
				cy1 = CELL_INDEX(Y, current_position[Y_AXIS]),
				cx2 = CELL_INDEX(X, destination[X_AXIS]),
				cy2 = CELL_INDEX(Y, destination[Y_AXIS]);
		cx1 = constrain(cx1, 0, ABL_BG_POINTS_X - 2);
		cy1 = constrain(cy1, 0, ABL_BG_POINTS_Y - 2);
		cx2 = constrain(cx2, 0, ABL_BG_POINTS_X - 2);
		cy2 = constrain(cy2, 0, ABL_BG_POINTS_Y - 2);

		if (cx1 == cx2 && cy1 == cy2) {
			// Start and end on same mesh square
			line_to_destination(fr_mm_s);
			set_current_to_destination();
			return;
		}

		#define LINE_SEGMENT_END(A) (current_position[A ##_AXIS] + (destination[A ##_AXIS] - current_position[A ##_AXIS]) * normalized_dist)

		float normalized_dist, end[XYZE];

		// Split at the left/front border of the right/top square
		int8_t gcx = max(cx1, cx2), gcy = max(cy1, cy2);
		if (cx2 != cx1 && TEST(x_splits, gcx)) {
			memcpy(end, destination, sizeof(end));
			destination[X_AXIS] = LOGICAL_X_POSITION(bilinear_start[X_AXIS] + ABL_BG_SPACING(X_AXIS) * gcx);
			normalized_dist = (destination[X_AXIS] - current_position[X_AXIS]) / (end[X_AXIS] - current_position[X_AXIS]);
			destination[Y_AXIS] = LINE_SEGMENT_END(Y);
			CBI(x_splits, gcx);
		}
		else if (cy2 != cy1 && TEST(y_splits, gcy)) {
			memcpy(end, destination, sizeof(end));
			destination[Y_AXIS] = LOGICAL_Y_POSITION(bilinear_start[Y_AXIS] + ABL_BG_SPACING(Y_AXIS) * gcy);
			normalized_dist = (destination[Y_AXIS] - current_position[Y_AXIS]) / (end[Y_AXIS] - current_position[Y_AXIS]);
			destination[X_AXIS] = LINE_SEGMENT_END(X);
			CBI(y_splits, gcy);
		}
		else {
			// Already split on a border
			line_to_destination(fr_mm_s);
			set_current_to_destination();
			return;
		}

		destination[Z_AXIS] = LINE_SEGMENT_END(Z);
		destination[E_AXIS] = LINE_SEGMENT_END(E);

		// Do the split and look for more borders
		bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);

		// Restore destination from stack
		memcpy(destination, end, sizeof(end));
		bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);
	}

#endif // AUTO_BED_LEVELING_BILINEAR


/**
 * Prepare a linear move in a Cartesian setup.
 * If Mesh Bed Leveling is enabled, perform a mesh move.
 */
inline bool prepare_move_to_destination_cartesian() {
    // Do not use feedrate_percentage for E or Z only moves
    if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS]) {
        line_to_destination();
    }
    else {
        #if ENABLED(MESH_BED_LEVELING)
            if (mbl.active()) {
                mesh_line_to_destination(MMS_SCALED(feedrate_mm_s));
                return false;
            }
            else
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
            if (planner.abl_enabled) {
                bilinear_line_to_destination(MMS_SCALED(feedrate_mm_s));
                return false;
            }
            else
        #endif
                line_to_destination(MMS_SCALED(feedrate_mm_s));
    }
    return true;
}


/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 */
void prepare_move_to_destination() {
	clamp_to_software_endstops(destination);
	refresh_cmd_timeout();

	#if ENABLED(PREVENT_COLD_EXTRUSION)

		if (!DEBUGGING(DRYRUN)) {
			if (destination[E_AXIS] != current_position[E_AXIS]) {
				if (thermalManager.tooColdToExtrude(active_extruder)) {
					current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
					SERIAL_ECHO_START;
					SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
				}
				#if ENABLED(PREVENT_LENGTHY_EXTRUDE)
					if (labs(destination[E_AXIS] - current_position[E_AXIS]) > EXTRUDE_MAXLENGTH) {
						current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
						SERIAL_ECHO_START;
						SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
					}
				#endif
			}
		}

	#endif

    if (!prepare_move_to_destination_cartesian()) return;

	set_current_to_destination();
}

#if ENABLED(ARC_SUPPORT)
	/**
	 * Plan an arc in 2 dimensions
	 *
	 * The arc is approximated by generating many small linear segments.
	 * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
	 * Arcs should only be made relatively large (over 5mm), as larger arcs with
	 * larger segments will tend to be more efficient. Your slicer should have
	 * options for G2/G3 arc generation. In future these options may be GCode tunable.
	 */
	void plan_arc(
		float logical[NUM_AXIS], // Destination position
		float* offset,					 // Center of rotation relative to current_position
		uint8_t clockwise				// Clockwise?
	) {

		float radius = HYPOT(offset[X_AXIS], offset[Y_AXIS]),
					center_X = current_position[X_AXIS] + offset[X_AXIS],
					center_Y = current_position[Y_AXIS] + offset[Y_AXIS],
					linear_travel = logical[Z_AXIS] - current_position[Z_AXIS],
					extruder_travel = logical[E_AXIS] - current_position[E_AXIS],
					r_X = -offset[X_AXIS],	// Radius vector from center to current location
					r_Y = -offset[Y_AXIS],
					rt_X = logical[X_AXIS] - center_X,
					rt_Y = logical[Y_AXIS] - center_Y;

		// CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
		float angular_travel = atan2(r_X * rt_Y - r_Y * rt_X, r_X * rt_X + r_Y * rt_Y);
		if (angular_travel < 0) angular_travel += RADIANS(360);
		if (clockwise) angular_travel -= RADIANS(360);

		// Make a circle if the angular rotation is 0
		if (angular_travel == 0 && current_position[X_AXIS] == logical[X_AXIS] && current_position[Y_AXIS] == logical[Y_AXIS])
			angular_travel += RADIANS(360);

		float mm_of_travel = HYPOT(angular_travel * radius, fabs(linear_travel));
		if (mm_of_travel < 0.001) return;

		uint16_t segments = floor(mm_of_travel / (MM_PER_ARC_SEGMENT));
		if (segments == 0) segments = 1;

		/**
		 * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
		 * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
		 *		 r_T = [cos(phi) -sin(phi);
		 *						sin(phi)	cos(phi)] * r ;
		 *
		 * For arc generation, the center of the circle is the axis of rotation and the radius vector is
		 * defined from the circle center to the initial position. Each line segment is formed by successive
		 * vector rotations. This requires only two cos() and sin() computations to form the rotation
		 * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
		 * all double numbers are single precision on the Arduino. (True double precision will not have
		 * round off issues for CNC applications.) Single precision error can accumulate to be greater than
		 * tool precision in some cases. Therefore, arc path correction is implemented.
		 *
		 * Small angle approximation may be used to reduce computation overhead further. This approximation
		 * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
		 * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
		 * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
		 * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
		 * issue for CNC machines with the single precision Arduino calculations.
		 *
		 * This approximation also allows plan_arc to immediately insert a line segment into the planner
		 * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
		 * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
		 * This is important when there are successive arc motions.
		 */
		// Vector rotation matrix values
		float arc_target[XYZE],
					theta_per_segment = angular_travel / segments,
					linear_per_segment = linear_travel / segments,
					extruder_per_segment = extruder_travel / segments,
					sin_T = theta_per_segment,
					cos_T = 1 - 0.5 * sq(theta_per_segment); // Small angle approximation

		// Initialize the linear axis
		arc_target[Z_AXIS] = current_position[Z_AXIS];

		// Initialize the extruder axis
		arc_target[E_AXIS] = current_position[E_AXIS];

		float fr_mm_s = MMS_SCALED(feedrate_mm_s);

		millis_t next_idle_ms = millis() + 200UL;

		int8_t count = 0;
		for (uint16_t i = 1; i < segments; i++) { // Iterate (segments-1) times

			thermalManager.manage_heater();
			if (ELAPSED(millis(), next_idle_ms)) {
				next_idle_ms = millis() + 200UL;
				idle();
			}

			if (++count < N_ARC_CORRECTION) {
				// Apply vector rotation matrix to previous r_X / 1
				float r_new_Y = r_X * sin_T + r_Y * cos_T;
				r_X = r_X * cos_T - r_Y * sin_T;
				r_Y = r_new_Y;
			}
			else {
				// Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
				// Compute exact location by applying transformation matrix from initial radius vector(=-offset).
				// To reduce stuttering, the sin and cos could be computed at different times.
				// For now, compute both at the same time.
				float cos_Ti = cos(i * theta_per_segment),
							sin_Ti = sin(i * theta_per_segment);
				r_X = -offset[X_AXIS] * cos_Ti + offset[Y_AXIS] * sin_Ti;
				r_Y = -offset[X_AXIS] * sin_Ti - offset[Y_AXIS] * cos_Ti;
				count = 0;
			}

			// Update arc_target location
			arc_target[X_AXIS] = center_X + r_X;
			arc_target[Y_AXIS] = center_Y + r_Y;
			arc_target[Z_AXIS] += linear_per_segment;
			arc_target[E_AXIS] += extruder_per_segment;

			clamp_to_software_endstops(arc_target);

			planner.buffer_line_kinematic(arc_target, fr_mm_s, active_extruder);
		}

		// Ensure last segment arrives at target location.
		planner.buffer_line_kinematic(logical, fr_mm_s, active_extruder);

		// As far as the parser is concerned, the position is now == target. In reality the
		// motion control system might still be processing the action and the real tool position
		// in any intermediate location.
		set_current_to_destination();
	}
#endif


#if HAS_CONTROLLERFAN

	void controllerFan() {
		static millis_t lastMotorOn = 0; // Last time a motor was turned on
		static millis_t nextMotorCheck = 0; // Last time the state was checked
		millis_t ms = millis();
		if (ELAPSED(ms, nextMotorCheck)) {
			nextMotorCheck = ms + 2500UL; // Not a time critical function, so only check every 2.5s
			if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || thermalManager.soft_pwm_bed > 0
					|| E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
					#if E_STEPPERS > 1
						|| E1_ENABLE_READ == E_ENABLE_ON
						#if HAS_X2_ENABLE
							|| X2_ENABLE_READ == X_ENABLE_ON
						#endif
						#if E_STEPPERS > 2
							|| E2_ENABLE_READ == E_ENABLE_ON
							#if E_STEPPERS > 3
								|| E3_ENABLE_READ == E_ENABLE_ON
							#endif
						#endif
					#endif
			) {
				lastMotorOn = ms; //... set time to NOW so the fan will turn on
			}

			// Fan off if no steppers have been enabled for CONTROLLERFAN_SECS seconds
			uint8_t speed = (!lastMotorOn || ELAPSED(ms, lastMotorOn + (CONTROLLERFAN_SECS) * 1000UL)) ? 0 : CONTROLLERFAN_SPEED;

			// allows digital or PWM fan output to be used (see M42 handling)
			digitalWrite(CONTROLLERFAN_PIN, speed);
			analogWrite(CONTROLLERFAN_PIN, speed);
		}
	}

#endif // HAS_CONTROLLERFAN

#if ENABLED(TEMP_STAT_LEDS)

	static bool red_led = false;
	static millis_t next_status_led_update_ms = 0;

	void handle_status_leds(void) {
		if (ELAPSED(millis(), next_status_led_update_ms)) {
			next_status_led_update_ms += 500; // Update every 0.5s
			float max_temp = 0.0;
			#if HAS_TEMP_BED
				max_temp = MAX3(max_temp, thermalManager.degTargetBed(), thermalManager.degBed());
			#endif
			HOTEND_LOOP() {
				max_temp = MAX3(max_temp, thermalManager.degHotend(e), thermalManager.degTargetHotend(e));
			}
			bool new_led = (max_temp > 55.0) ? true : (max_temp < 54.0) ? false : red_led;
			if (new_led != red_led) {
				red_led = new_led;
				#if PIN_EXISTS(STAT_LED_RED)
					WRITE(STAT_LED_RED_PIN, new_led ? HIGH : LOW);
					#if PIN_EXISTS(STAT_LED_BLUE)
						WRITE(STAT_LED_BLUE_PIN, new_led ? LOW : HIGH);
					#endif
				#else
					WRITE(STAT_LED_BLUE_PIN, new_led ? HIGH : LOW);
				#endif
			}
		}
	}

#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)

	void handle_filament_runout() {
		if (!filament_ran_out) {
			filament_ran_out = true;
			enqueue_and_echo_commands_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
			stepper.synchronize();
		}
	}

#endif // FILAMENT_RUNOUT_SENSOR

#if ENABLED(FAST_PWM_FAN)

	void setPwmFrequency(uint8_t pin, int val) {
		val &= 0x07;
		switch (digitalPinToTimer(pin)) {
			#if defined(TCCR0A)
				case TIMER0A:
				case TIMER0B:
					// TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
					// TCCR0B |= val;
					break;
			#endif
			#if defined(TCCR1A)
				case TIMER1A:
				case TIMER1B:
					// TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
					// TCCR1B |= val;
					break;
			#endif
			#if defined(TCCR2)
				case TIMER2:
				case TIMER2:
					TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
					TCCR2 |= val;
					break;
			#endif
			#if defined(TCCR2A)
				case TIMER2A:
				case TIMER2B:
					TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
					TCCR2B |= val;
					break;
			#endif
			#if defined(TCCR3A)
				case TIMER3A:
				case TIMER3B:
				case TIMER3C:
					TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
					TCCR3B |= val;
					break;
			#endif
			#if defined(TCCR4A)
				case TIMER4A:
				case TIMER4B:
				case TIMER4C:
					TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
					TCCR4B |= val;
					break;
			#endif
			#if defined(TCCR5A)
				case TIMER5A:
				case TIMER5B:
				case TIMER5C:
					TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
					TCCR5B |= val;
					break;
			#endif
		}
	}

#endif // FAST_PWM_FAN

float calculate_volumetric_multiplier(float diameter) {
	if (!volumetric_enabled || diameter == 0) return 1.0;
	return 1.0 / (M_PI * diameter * 0.5 * diameter * 0.5);
}

void calculate_volumetric_multipliers() {
	for (uint8_t i = 0; i < COUNT(filament_size); i++)
		volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}

void enable_all_steppers() {
	enable_x();
	enable_y();
	enable_z();
	enable_e0();
	enable_e1();
	enable_e2();
	enable_e3();
}

void disable_all_steppers() {
	disable_x();
	disable_y();
	disable_z();
	disable_e0();
	disable_e1();
	disable_e2();
	disable_e3();
}

/**
 * Manage several activities:
 *	- Check for Filament Runout
 *	- Keep the command buffer full
 *	- Check for maximum inactive time between commands
 *	- Check for maximum inactive time between stepper commands
 *	- Check if pin CHDK needs to go LOW
 *	- Check for KILL button held down
 *	- Check for HOME button held down
 *	- Check if cooling fan needs to be switched on
 *	- Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {

	#if ENABLED(FILAMENT_RUNOUT_SENSOR)
		if ((IS_SD_PRINTING || print_job_timer.isRunning()) && (READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_INVERTING))
			handle_filament_runout();
	#endif

	if (commands_in_queue < BUFSIZE) get_available_commands();

	millis_t ms = millis();

	if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) kill(PSTR(MSG_KILLED));

	if (stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
			&& !ignore_stepper_queue && !planner.blocks_queued()) {
		#if ENABLED(DISABLE_INACTIVE_X)
			disable_x();
		#endif
		#if ENABLED(DISABLE_INACTIVE_Y)
			disable_y();
		#endif
		#if ENABLED(DISABLE_INACTIVE_Z)
			disable_z();
		#endif
		#if ENABLED(DISABLE_INACTIVE_E)
			disable_e0();
			disable_e1();
			disable_e2();
			disable_e3();
		#endif
	}

	#ifdef CHDK // Check if pin should be set to LOW after M240 set it to HIGH
		if (chdkActive && PENDING(ms, chdkHigh + CHDK_DELAY)) {
			chdkActive = false;
			WRITE(CHDK, LOW);
		}
	#endif

	#if HAS_KILL

		// Check if the kill button was pressed and wait just in case it was an accidental
		// key kill key press
		// -------------------------------------------------------------------------------
		static int killCount = 0;	 // make the inactivity button a bit less responsive
		const int KILL_DELAY = 750;
		if (!READ(KILL_PIN))
			killCount++;
		else if (killCount > 0)
			killCount--;

		// Exceeded threshold and we can confirm that it was not accidental
		// KILL the machine
		// ----------------------------------------------------------------
		if (killCount >= KILL_DELAY) kill(PSTR(MSG_KILLED));
	#endif

	#if HAS_HOME
		// Check to see if we have to home, use poor man's debouncer
		// ---------------------------------------------------------
		static int homeDebounceCount = 0;	 // poor man's debouncing count
		const int HOME_DEBOUNCE_DELAY = 2500;
		if (!IS_SD_PRINTING && !READ(HOME_PIN)) {
			if (!homeDebounceCount) {
				enqueue_and_echo_commands_P(PSTR("G28"));
				NOOP;
			}
			if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
				homeDebounceCount++;
			else
				homeDebounceCount = 0;
		}
	#endif

	#if HAS_CONTROLLERFAN
		controllerFan(); // Check if fan should be turned on to cool stepper drivers down
	#endif

	#if ENABLED(EXTRUDER_RUNOUT_PREVENT)
		if (ELAPSED(ms, previous_cmd_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL)
			&& thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
			bool oldstatus;

            switch (active_extruder) {
                case 0:
                    oldstatus = E0_ENABLE_READ;
                    enable_e0();
                    break;
                #if E_STEPPERS > 1
                    case 1:
                        oldstatus = E1_ENABLE_READ;
                        enable_e1();
                        break;
                    #if E_STEPPERS > 2
                        case 2:
                            oldstatus = E2_ENABLE_READ;
                            enable_e2();
                            break;
                        #if E_STEPPERS > 3
                            case 3:
                                oldstatus = E3_ENABLE_READ;
                                enable_e3();
                                break;
                        #endif
                    #endif
                #endif
            }

			previous_cmd_ms = ms; // refresh_cmd_timeout()

            planner.buffer_line(
                current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
                current_position[E_AXIS] + EXTRUDER_RUNOUT_EXTRUDE,
                MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), active_extruder
            );

			stepper.synchronize();
			planner.set_e_position_mm(current_position[E_AXIS]);

            switch (active_extruder) {
                case 0:
                    E0_ENABLE_WRITE(oldstatus);
                    break;
                #if E_STEPPERS > 1
                    case 1:
                        E1_ENABLE_WRITE(oldstatus);
                        break;
                    #if E_STEPPERS > 2
                        case 2:
                            E2_ENABLE_WRITE(oldstatus);
                            break;
                        #if E_STEPPERS > 3
                            case 3:
                                E3_ENABLE_WRITE(oldstatus);
                                break;
                        #endif
                    #endif
                #endif
            }

		}
	#endif // EXTRUDER_RUNOUT_PREVENT

	#if ENABLED(TEMP_STAT_LEDS)
		handle_status_leds();
	#endif

	planner.check_axes_activity();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle(
	#if ENABLED(FILAMENT_CHANGE_FEATURE)
		bool no_stepper_sleep/*=false*/
	#endif
) {

	host_keepalive();

	#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
		auto_report_temperatures();
	#endif

	manage_inactivity(
		#if ENABLED(FILAMENT_CHANGE_FEATURE)
			no_stepper_sleep
		#endif
	);

	thermalManager.manage_heater();

	#if ENABLED(PRINTCOUNTER)
		print_job_timer.tick();
	#endif
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(const char* lcd_msg) {
	SERIAL_ERROR_START;
	SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

    UNUSED(lcd_msg);

	delay(500); // Wait a short time

	cli(); // Stop interrupts
	thermalManager.disable_all_heaters();
	disable_all_steppers();

	#if HAS_POWER_SWITCH
		SET_INPUT(PS_ON_PIN);
	#endif

	suicide();
	while (1) {
		#if ENABLED(USE_WATCHDOG)
			watchdog_reset();
		#endif
	} // Wait for reset
}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void stop() {
	thermalManager.disable_all_heaters();
	if (IsRunning()) {
		Running = false;
		Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
        NOOP;
	}
}

/**
 * Marlin entry-point: Set up before the program loop
 *	- Set up the kill pin, filament runout, power hold
 *	- Start the serial port
 *	- Print startup messages and diagnostics
 *	- Get EEPROM or default settings
 *	- Initialize managers for:
 *		• temperature
 *		• planner
 *		• watchdog
 *		• stepper
 *		• photo pin
 *		• servos
 *		• LCD controller
 *		• Digipot I2C
 *		• Z probe sled
 *		• status LEDs
 */
void setup() {

	#ifdef DISABLE_JTAG
		// Disable JTAG on AT90USB chips to free up pins for IO
		MCUCR = 0x80;
		MCUCR = 0x80;
	#endif

	#if ENABLED(FILAMENT_RUNOUT_SENSOR)
		setup_filrunoutpin();
	#endif

	setup_killpin();

	setup_powerhold();

	#if HAS_STEPPER_RESET
		disableStepperDrivers();
	#endif

	MYSERIAL.begin(BAUDRATE);
	SERIAL_PROTOCOLLNPGM("start");
	SERIAL_ECHO_START;

	// Check startup - does nothing if bootloader sets MCUSR to 0
	byte mcu = MCUSR;
	if (mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
	if (mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
	if (mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
	if (mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
	if (mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
	MCUSR = 0;

	SERIAL_ECHOPGM(MSG_MARLIN);
	SERIAL_CHAR(' ');
	SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
	SERIAL_EOL;

	#if defined(STRING_DISTRIBUTION_DATE) && defined(STRING_CONFIG_H_AUTHOR)
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
		SERIAL_ECHOPGM(STRING_DISTRIBUTION_DATE);
		SERIAL_ECHOLNPGM(MSG_AUTHOR STRING_CONFIG_H_AUTHOR);
		SERIAL_ECHOLNPGM("Compiled: " __DATE__);
	#endif

	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR(MSG_FREE_MEMORY, freeMemory());
	SERIAL_ECHOLNPAIR(MSG_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

	// Send "ok" after commands by default
	for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;

	// Load data from EEPROM if available (or use defaults)
	// This also updates variables in the planner, elsewhere
	Config_RetrieveSettings();

	// Initialize current position based on home_offset
	memcpy(current_position, home_offset, sizeof(home_offset));

	// Vital to init stepper/planner equivalent for current_position
	SYNC_PLAN_POSITION_KINEMATIC();

	thermalManager.init();		// Initialize temperature loop

	#if ENABLED(USE_WATCHDOG)
		watchdog_init();
	#endif

	stepper.init();		// Initialize stepper, this enables interrupts!
	servo_init();

	#if HAS_PHOTOGRAPH
		OUT_WRITE(PHOTOGRAPH_PIN, LOW);
	#endif

	#if HAS_CASE_LIGHT
		update_case_light();
	#endif

	#if HAS_BED_PROBE
		endstops.enable_z_probe(false);
	#endif

	#if HAS_CONTROLLERFAN
		SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
	#endif

	#if HAS_STEPPER_RESET
		enableStepperDrivers();
	#endif

	#if ENABLED(DIGIPOT_I2C)
		digipot_i2c_init();
	#endif


	#if ENABLED(Z_PROBE_SLED) && PIN_EXISTS(SLED)
		OUT_WRITE(SLED_PIN, LOW); // turn it off
	#endif // Z_PROBE_SLED

	setup_homepin();

	#if PIN_EXISTS(STAT_LED_RED)
		OUT_WRITE(STAT_LED_RED_PIN, LOW); // turn it off
	#endif

	#if PIN_EXISTS(STAT_LED_BLUE)
		OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // turn it off
	#endif

	#if ENABLED(RGB_LED)
		pinMode(RGB_LED_R_PIN, OUTPUT);
		pinMode(RGB_LED_G_PIN, OUTPUT);
		pinMode(RGB_LED_B_PIN, OUTPUT);
	#endif

	#if ENABLED(SHOW_BOOTSCREEN)
		#if ENABLED(DOGLCD)
			safe_delay(BOOTSCREEN_TIMEOUT);
		#endif
	#endif

}

/**
 * The main Marlin program loop
 *
 *	- Save or log commands to SD
 *	- Process available commands (if not saving)
 *	- Call heater manager
 *	- Call inactivity manager
 *	- Call endstop manager
 *	- Call LCD update
 */
void loop() {
	if (commands_in_queue < BUFSIZE) get_available_commands();

	if (commands_in_queue) {

		// The queue may be reset by a command handler or by code invoked by idle() within a handler
		if (commands_in_queue) {
			--commands_in_queue;
			cmd_queue_index_r = (cmd_queue_index_r + 1) % BUFSIZE;
		}
	}
	endstops.report_state();
	idle();
}
