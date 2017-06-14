#ifndef __SIX_DFO__
#define __SIX_DFO__

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>

class sixDOF {
private:
	FreeSixIMU FsixDOF; // FreeSixIMU object
	float _angles_Euler[3] = {0};
	float _average[3] = {0};
	int _sample_count;
	float _weights;
	float _CDF_weights;
	void _init_samples();

public:
	sixDOF(){}
	void sixDOF_setup(int sample_count, float _weights);
	void sixDOF_loop(); 
	void sixDOF_loop(float* angles_Euler);
	float * get_average();
	void compute_CDF();
	float get_CDF();
	void calibrate();
};

#endif