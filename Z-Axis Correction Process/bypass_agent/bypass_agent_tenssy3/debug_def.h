#ifndef __DEBUG_DEF__
#define __DEBUG_DEF__

#define DEBUG_DELAY 1000

// #define DEBUG_FUNC_FLOW_ALL

#ifdef DEBUG_FUNC_FLOW_ALL

	#ifndef DEBUG_FUNC_FLOW__BYPASS_AGENT__
		#define DEBUG_FUNC_FLOW__BYPASS_AGENT__
	#endif

	#ifndef DEBUG_FUNC_FLOW__SIX_DFO__
		#define DEBUG_FUNC_FLOW__SIX_DFO__
	#endif

	#ifndef DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
		#define DEBUG_FUNC_FLOW__CYCLIC_ARRAY__
	#endif

	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

#define DEBUG_FUNC_FLOW__BYPASS_AGENT__
#ifdef DEBUG_FUNC_FLOW__BYPASS_AGENT__
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

// #define DEBUG_PRINT_sixDOF_Tenssy3
#ifdef DEBUG_PRINT_sixDOF_Tenssy3
	#ifndef DISABLE_NORMAL_PRINTS
		#define DISABLE_NORMAL_PRINTS
	#endif
#endif

#endif