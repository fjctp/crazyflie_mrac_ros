#include <ros/ros.h>
#include "AdaptiveController.hpp"
#include <Eigen/Dense>

#define RAD_TO_DEG 		(double)(180.0/M_PI)
#define DEG_TO_RAD 		(double)(M_PI/180.0)

#define MAX_ROLL_DEG	20.0
#define MAX_PITCH_DEG	20.0
#define MAX_YAW_DEG		160.0
#define MAX_THROTTLE	65000
#define TRIM_THROTTLE	40000

using namespace Eigen;

AdaptiveController::AdaptiveController(void (*_phi_functions) (VectorXd&, const VectorXd))
{
	(*this).phi_functions = _phi_functions;
}
AdaptiveController::~AdaptiveController() {}

void AdaptiveController::reset()
{
	this->model.reset();
	this->law.reset();
}
void AdaptiveController::initialize(const PID_Config_t _pid_x_config,
                					const PID_Config_t _pid_y_config,
                					const PID_Config_t _pid_z_config, 
                					const ReferenceModel_Config_t _model_config,
                                    const AdaptiveLaw_Config_t _law_config,
                                    const double _dt)
{
    Controller::initialize(_pid_x_config, _pid_y_config, _pid_z_config);
    this->model.initialize(_model_config);
    this->law.initialize(_law_config);

    this->last_zpos = 0.0;
    this->dt = _dt;

    switch_to_standby();
}



void AdaptiveController::compute_outputs(geometry_msgs::Twist* _cmd)
{
    // position error in world frame
    double p_err_world[3], p_err_body[3];
    for(int i=0;i<3;i++) {
    	p_err_world[i] = target_position[i] - current_position[i];
    }
	rotate(current_orientation[2], p_err_world, p_err_body);

	// PID controllers
	double roll_cmd  =  pid_x.get_output(-p_err_body[0]) * RAD_TO_DEG;
	double pitch_cmd = -pid_y.get_output(-p_err_body[1]) * RAD_TO_DEG;
	double yaw_cmd	 =  target_yaw* RAD_TO_DEG;
	double pid_throttle  =  pid_z.get_output( p_err_body[2]);

	// Z Adaptive controller
	VectorXd zRefCommands, zStates;		// Input to func, need to initialize
	VectorXd zRefOutputs, zPhi;	// Output from func
	MatrixXd zGains;

	// Note: model is z down (-z is above ground)
	zRefCommands = VectorXd(1);	
	zStates = VectorXd(2);
	zRefCommands(0) = -target_position[2];
	zStates(0) = -current_position[2];				// pose z
	zStates(1) = (zStates(0)-last_zpos)/this->dt;	// vel z
	last_zpos = zStates(0);

    this->model.update(zRefCommands);
    this->model.get_outputs(zRefOutputs);

	(*(*this).phi_functions) (zPhi, zStates);

    this->law.update(zStates - zRefOutputs, zPhi);
    this->law.get_gains(zGains);

    VectorXd adp_throttle = -zGains.transpose()*zPhi;
    if (adp_throttle.size() != 1)
        throw std::range_error("Adaptive gain is not a scaler!");
    adp_throttle *= -4.403669725e5; // convert to PWM from force (model)
    adp_throttle(0) = limit(adp_throttle(0), -3e4, 3e4);

    double total_throttle = pid_throttle + adp_throttle(0);
    //ROS_INFO("%.4f, %.4f, %.1lf, %.6lf, %.6lf", zStates(0), zRefOutputs(0),
    //											adp_throttle(0),
    //											zGains(0), zGains(1));
    
	// Send Commands
	_cmd->linear.y  = limit(roll_cmd, -MAX_ROLL_DEG, MAX_ROLL_DEG);
	_cmd->linear.x  = limit(pitch_cmd, -MAX_PITCH_DEG, MAX_PITCH_DEG);
	_cmd->angular.z = yaw_cmd;
	switch(flight_state) {
		case STANDBY:
			_cmd->linear.z  = limit(0, 0, MAX_THROTTLE);
			break;
		case EMERGENCY:
			_cmd->linear.z  = limit(0, 0, MAX_THROTTLE);
			break;
		default:
            _cmd->linear.z  = limit(total_throttle + TRIM_THROTTLE, 0.0, MAX_THROTTLE);
	}
}

void AdaptiveController::get_reference_states(VectorXd &_states) {
	this->model.get_states(_states);
}


void AdaptiveController::get_adaptive_gains(MatrixXd &gains)
{
	this->law.get_gains(gains);
}
