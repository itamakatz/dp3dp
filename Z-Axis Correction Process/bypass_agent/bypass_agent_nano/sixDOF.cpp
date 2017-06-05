#include "sixDOF.h"
#include "CommunicationUtils.h"

// float q[4];

void sixDOF::sixDOF_setup(int sample_count, float weights) {
	// init FreeSixIMU object
	_sample_count = sample_count;
	_weights = weights;
	FsixDOF = FreeSixIMU();
	FsixDOF.init(); //begin the IMU
	compute_CDF();
	_init_samples();
}

void sixDOF::_init_samples(){
	for (double j = 0.0; j < _sample_count; ++j)
	{
		FsixDOF.getEuler(_angles_Euler);
		for (int i = 0; i < 3; ++i)
		{
			_average[i] = j / (j + 1) * _average[i] + _angles_Euler[i] * pow(_weights, j) / (j + 1);
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		_average[i] /= _CDF_weights;
	}
}

void sixDOF::sixDOF_loop() { 
	// FsixDOF.getQ(q);
	// serialPrintFloatArr(q, 4);
	FsixDOF.getEuler(_angles_Euler);
	for (int i = 0; i < 3; ++i)
	{
		_average[i] /= _weights;
		_average[i] = _sample_count / (_sample_count + 1) * _average[i] + _angles_Euler[i] * pow(_weights, _sample_count) / (_sample_count + 1);
	}
}

void sixDOF::sixDOF_loop(float* angles_Euler) { 
	// FsixDOF.getQ(q);
	// serialPrintFloatArr(q, 4);
	sixDOF_loop();
	angles_Euler = _angles_Euler;
}

float * sixDOF::get_average(){
	return _average;
}

void sixDOF::compute_CDF(){
	_CDF_weights = 0.0;
	for (int i = 0; i < _sample_count; ++i)
	{
		_CDF_weights += pow(_weights, i);
	}
}

float sixDOF::get_CDF(){
	return _CDF_weights;
}

void sixDOF::calibrate(){
	FsixDOF.gyro.zeroCalibrate(128,5);
	for (int i = 0; i < 3; ++i)
	{
		_average[i] = 0.0;
	}
	
	_init_samples();
}