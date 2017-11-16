#include "PID.hpp"
#include <algorithm>
#include <cmath>

void PID::initialize(PID_Config_t _config)
{
	Kp = _config.Kp;
	Ki = _config.Ki;
	Kd = _config.Kd;

	dt = _config.dt;
	threshold = _config.threshold;
	max_output = _config.max_output;

	reset();
}

void PID::reset(void) {
	accumulator = 0.0;
	last_error = 0.0;
	filtered_differentive = 0.0;
}

void PID::update_Kp(double _Kp)
{
	Kp = _Kp;
}
void PID::update_Ki(double _Ki)
{
	Ki = _Ki;
}
void PID::update_Kd(double _Kd)
{
	Kd = _Kd;
}

double PID::get_output(double _error) {
	double a = 0.6; // filter coefficient

	double dot = _error*dt;
	accumulator += fabs(dot)>threshold?dot:0.0;

	double new_differentive = (_error - last_error)/dt;
	new_differentive = fabs(new_differentive)>threshold?new_differentive:0.0;
	filtered_differentive = (1-a)*filtered_differentive + a*new_differentive;

	double _output = (Kp*_error) + (Ki*accumulator) + (Kd*filtered_differentive);

	last_error = _error;
	return std::min(std::max(_output, -max_output), max_output);
}
