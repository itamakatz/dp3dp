#ifndef __SIX_DFO__
#define __SIX_DFO__

#include <FreeSixIMU_Tenssy3.h>
#include <FIMU_ADXL345_Tenssy3.h>
#include <FIMU_ITG3200_Tenssy3.h>

#include <i2c_t3.h>

class sixDOF_Tenssy3 {
private:
	FreeSixIMU_Tenssy3 FsixDOF_Tenssy3; // FreeSixIMU object
	float _angles_Euler[3] = {0};
	float _average[3] = {0};
	int _sample_count;
	float _weights;
	float _CDF_weights;
	void _init_samples();

public:
	sixDOF_Tenssy3(){}
	void sixDOF_setup(int sample_count, float _weights);
	void sixDOF_loop(); 
	void sixDOF_loop(float* angles_Euler);
	float * get_average();
	void compute_CDF();
	float get_CDF();
	void calibrate();
};

#endif