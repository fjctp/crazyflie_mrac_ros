#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "PID.hpp"

typedef enum  {
	STANDBY = 0,
	TRACKING,
	EMERGENCY
} states;

class Controller {
protected:
	PID pid_x, pid_y, pid_z;
    void rotate(const double _yaw, const double* _from, double* _to);
    double limit(double value,double vmin,double vmax) {
        return fminf(fmaxf(value, vmin), vmax);
    }
    double mapping(double value, double oldmin, double oldmax, double newmin, double newmax) {
        return (value-oldmin)/(oldmax-oldmin)*(newmax-newmin)+newmin;
    }


	double target_position[3];		// in world frame
	double target_yaw;
	double current_position[3];		// in world frame
	double current_orientation[3];	// roll, pitch, yaw
	states flight_state;

	void set_target(double _x, double _y, double _z);

public:
	Controller();
	~Controller();

	virtual void reset();
	virtual void initialize(const PID_Config_t _config_x,
                            const PID_Config_t _config_y,
                            const PID_Config_t _config_z);
	virtual void compute_outputs(geometry_msgs::Twist* _cmd);
    
    virtual void update_target(const geometry_msgs::Pose::ConstPtr& _target_position);
    virtual void update_position(const geometry_msgs::PoseStamped::ConstPtr& _current_position);

	virtual void switch_to_standby(void);
	virtual void switch_to_tracking(void);
	virtual void switch_to_emergency(void);
};

#endif
