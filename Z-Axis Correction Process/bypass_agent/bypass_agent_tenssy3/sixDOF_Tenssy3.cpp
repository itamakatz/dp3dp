#include "sixDOF_Tenssy3.h"

// float q[4];

void sixDOF_Tenssy3::sixDOF_setup(int sample_count, float weights) {
	// init FreeSixIMU_Tenssy3 object
	_sample_count = sample_count;
	_weights = weights;
	FsixDOF_Tenssy3 = FreeSixIMU_Tenssy3();
	FsixDOF_Tenssy3.init(); //begin the IMU
	// compute_CDF();
	// _init_samples();
}

void sixDOF_Tenssy3::_init_samples(){
	for (double j = 0.0; j < _sample_count; ++j)
	{
		FsixDOF_Tenssy3.getEuler(_angles_Euler);
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

void sixDOF_Tenssy3::sixDOF_loop() { 
	// FsixDOF_Tenssy3.getQ(q);
	// serialPrintFloatArr(q, 4);
	FsixDOF_Tenssy3.getEuler(_angles_Euler);
	// for (int i = 0; i < 3; ++i)
	// {
	// 	_average[i] /= _weights;
	// 	_average[i] = _sample_count / (_sample_count + 1) * _average[i] + _angles_Euler[i] * pow(_weights, _sample_count) / (_sample_count + 1);
	// }
}

void sixDOF_Tenssy3::sixDOF_loop(float* angles_Euler) { 
	// FsixDOF_Tenssy3.getQ(q);
	// serialPrintFloatArr(q, 4);
	sixDOF_loop();
	angles_Euler = _angles_Euler;
}

float * sixDOF_Tenssy3::get_average(){
	return _average;
}

void sixDOF_Tenssy3::compute_CDF(){
	_CDF_weights = 0.0;
	for (int i = 0; i < _sample_count; ++i)
	{
		_CDF_weights += pow(_weights, i);
	}
}

float sixDOF_Tenssy3::get_CDF(){
	return _CDF_weights;
}

void sixDOF_Tenssy3::calibrate(){
	FsixDOF_Tenssy3.gyro.zeroCalibrate(128,5);
	for (int i = 0; i < 3; ++i)
	{
		_average[i] = 0.0;
	}
	
	_init_samples();
}