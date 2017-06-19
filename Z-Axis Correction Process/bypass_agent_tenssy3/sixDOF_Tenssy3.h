#ifndef __SIX_DFO__
#define __SIX_DFO__

#include <FreeSixIMU_Tenssy3.h>
#include <FIMU_ADXL345_Tenssy3.h>
#include <FIMU_ITG3200_Tenssy3.h>

#include <i2c_t3.h>
#include "general_defs.h"
#include "Cyclic_array.h"

class sixDOF_Tenssy3 {
private:
	FreeSixIMU_Tenssy3 _FsixDOF_Tenssy3; // FreeSixIMU object
	Cyclic_array _c_array[3];

	float _average[3];
	float _average_zero_offset[3];

	float _angles_Euler[3];
	float _angles_Euler_zero_offset[3];
	
	float _weights;
	float _alpha;

	void _init_samples();
	void _update_average(); 

public:
	sixDOF_Tenssy3(){}
	void sixDOF_setup(float alpha);
	void sixDOF_loop();
	void set_zero();
	void get_angles(float* angles_average);
	void get_average(float* angles_Euler);
	void calibrate();
};

#endif