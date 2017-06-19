#ifndef __DEBUG_DEF__
#define __DEBUG_DEF__

#define DEBUG_DELAY 1000

// #define DEBUG_FUNC_FLOW_ALL

#ifdef DEBUG_FUNC_FLOW_ALL

	#ifndef DEBUG_FUNC_FLOW_BYPASS_AGENT_
		#define DEBUG_FUNC_FLOW_BYPASS_AGENT_
	#endif

	#ifndef DEBUG_FUNC_FLOW_SIX_DFO_
		#define DEBUG_FUNC_FLOW_SIX_DFO_
	#endif

	#ifndef DEBUG_FUNC_FLOW_VL6180__
		#define DEBUG_FUNC_FLOW_VL6180__
	#endif

	#ifndef DEBUG_FUNC_FLOW_Cyc_array_6DoF_
		#define DEBUG_FUNC_FLOW_Cyc_array_6DoF_
	#endif

	#ifndef DEBUG_FUNC_FLOW_Cyc_array_VL6180_
		#define DEBUG_FUNC_FLOW_Cyc_array_VL6180_
	#endif

	#ifndef DEBUG_FUNC_FLOW_VL6180
		#define DEBUG_FUNC_FLOW_VL6180
	#endif

	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// =========================== Bypass Agent =========================== //

// #define DEBUG_FUNC_FLOW_BYPASS_AGENT_
#ifdef DEBUG_FUNC_FLOW_BYPASS_AGENT_
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// #define DEBUG_MILIS_BYPASS_AGENT_
#ifdef DEBUG_MILIS_BYPASS_AGENT_
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// =========================== VL6180 =========================== //

// #define DEBUG_FUNC_FLOW_VL6180
#ifdef DEBUG_FUNC_FLOW_VL6180
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// #define DEBUG_PRINT_VL6180
#ifdef DEBUG_PRINT_VL6180
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// =========================== 6DoF =========================== //

// #define DEBUG_PRINT_sixDOF_Tenssy3
#ifdef DEBUG_PRINT_sixDOF_Tenssy3
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// =========================== Interupts =========================== //

// #define DEBUG_INTERRUPTS
#ifdef DEBUG_INTERRUPTS
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// =========================== Cyclic array 6DoF =========================== //

// #define DEBUG_PRINTS_Cyc_array_6DoF
#ifdef DEBUG_PRINTS_Cyc_array_6DoF
	
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// #define DEBUG_FUNC_FLOW_Cyc_array_6DoF_
#ifdef DEBUG_FUNC_FLOW_Cyc_array_6DoF_
	
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// =========================== Cyclic array VL6180 =========================== //

// #define DEBUG_PRINTS_Cyc_array_VL6180
#ifdef DEBUG_PRINTS_Cyc_array_VL6180
	
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// #define DEBUG_FUNC_FLOW_Cyc_array_VL6180_
#ifdef DEBUG_FUNC_FLOW_Cyc_array_VL6180_
	
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// =========================== stepper =========================== //

// #define DEBUG_FUNC_FLOW_STEPPER
#ifdef DEBUG_FUNC_FLOW_STEPPER
	
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// =========================== Done =========================== //

#endif