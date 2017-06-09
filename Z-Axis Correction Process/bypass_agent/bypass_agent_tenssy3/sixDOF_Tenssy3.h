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
	FreeSixIMU_Tenssy3 FsixDOF_Tenssy3; // FreeSixIMU object
	Cyclic_array c_array;

	float _average[3];
	float _angles_Euler[3];
	float _c_array_elements[3];
	float _weights;
	float _alpha;
	void _init_samples();
	void update_average(); 

public:
	sixDOF_Tenssy3(){}
	void sixDOF_setup(float alpha);
	void sixDOF_loop(); 
	void get_angles(float* angles_average);
	void get_average(float* angles_Euler);
	void calibrate();
};

#endif