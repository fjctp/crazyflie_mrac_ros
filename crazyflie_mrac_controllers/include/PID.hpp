#ifndef __PID_HPP__
#define __PID_HPP__

typedef struct {
	double Kp, Ki, Kd;
	double dt; 			// in sec
	double threshold;
	double max_output;
} PID_Config_t;

class PID {
protected:
	double Kp, Ki, Kd;
	double dt; 			// in sec
	double threshold;
	double max_output;

	double filtered_differentive;
	double last_error;
	double accumulator;

public:
	void initialize(PID_Config_t _config);
	void reset(void);

	void update_Kp(double _Kp);
	void update_Ki(double _Ki);
	void update_Kd(double _Kd);

	double get_output(double _error);
};

#endif
