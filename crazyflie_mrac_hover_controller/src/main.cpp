/*
 * Waypoint controller
 * Given a list of 3D points, the controller generate altitude commands
 * and send it over to crazyflie using crazyflie_ros package. 
 *
 * Topics:
 *   Sub:
 *     /vrpn_client_node/body1/Pose		geometry_msgs::PoseStamped  Current position
 *     "~/target_position				geometry_msgs::Pose         Target position
 *     "~/state							std_msgs::UInt8             See controller.hpp
 * 
 *   Pub:
 *     /crazyflie/cmd_vel				geometry_msgs::Twist        Altitude commands
 *
 *   By Jason Chan
 *   Aug 18, 2017
 *
 */
 
#include <ros/ros.h>
#include <ros/console.h>

#include "std_msgs/UInt8.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include <cmath>
#include <Eigen/Dense>
#include "Controller.hpp"
#include "AdaptiveController.hpp"

using namespace Eigen;

// command parameters
#define POSITION_LOOP_RATE	200.0
#define POSITION_LOOP_DT	(double)(1.0/POSITION_LOOP_RATE)

////////////////////////////////////////////////////////
// * Adaptive controller related settings
////////////////////////////////////////////////////////
#define USE_ADAPTIVE_CONTROLLER 
#define USE_RBF
////////////////////////////////////////////////////////


////////////////////////////////////////////////////////
// * Phi function, used in AdaptiveController Class
////////////////////////////////////////////////////////
#ifndef USE_RBF
    void phi_functions(VectorXd &phi, const VectorXd states)
    {
        double z = states(0);
        phi = VectorXd::Zero(2);
        phi(0) = z;
        phi(1) = 1;
    }
#else
    const double rbf_center[] = {-0.0, -0.2, -0.4, -0.6, -0.8, -1.0};
    //const double rbf_center[] = {-0.0, -0.4, -0.8};
    const double rbf_width = 0.6;
	const int nrbf = sizeof(rbf_center)/sizeof(double) + 1;

    inline double rbf(double x, double center, double base_width)
    {
      	return exp(-pow((x-center)/(base_width/4), 2));
    }

    void phi_functions(VectorXd &phi, const VectorXd states)
    {
        double z = states(0);
        
        phi = VectorXd::Zero(nrbf);
        for (int i=0; i<(nrbf-1); i++)
        {
            phi(i) = rbf(z, rbf_center[i], rbf_width);
        	//ROS_INFO("%d, %.4f, %.4f, %.4f, %.4f", i, phi(i), z, rbf_center[i], rbf_width);
        }
        phi(nrbf-1) = 1;

    }
#endif

////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
// * Global Variabls
////////////////////////////////////////////////////////
static ros::Publisher pub_command;
#ifdef USE_ADAPTIVE_CONTROLLER
	static AdaptiveController hover_controller(phi_functions);
	static ros::Publisher pub_ref_states;
	static ros::Publisher pub_adp_gains;
#else
	static Controller hover_controller;
#endif
static uint8_t last_state = 0;
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
// * Callback functions
////////////////////////////////////////////////////////
void sub_flight_state_callback(const std_msgs::UInt8::ConstPtr& _state) {
	uint8_t new_state = _state->data;
	if(last_state != new_state)
	{
		switch (new_state) {
			case 0:
				hover_controller.switch_to_standby();
				break;
			case 1:
				hover_controller.switch_to_tracking();
				break;
			default:
				hover_controller.switch_to_emergency();
		}
		last_state = _state->data;
	}
}

#ifdef USE_ADAPTIVE_CONTROLLER
void pub_ref_states_callback(const ros::TimerEvent& e) {
	geometry_msgs::PoseStamped pose;

	VectorXd states;
	hover_controller.get_reference_states(states);

	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = 0.0;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = states(0);
	pose.pose.orientation.x = 0.0;
	pose.pose.orientation.y = 0.0;
	pose.pose.orientation.z = 0.0;
	pose.pose.orientation.w = 0.0;

	pub_ref_states.publish(pose);
}


void pub_adp_gains_callback(const ros::TimerEvent& e) {
	std_msgs::Float64MultiArray msg_gains;

	MatrixXd m_gains;
	hover_controller.get_adaptive_gains(m_gains);

	const int m = m_gains.rows();
	const int n = m_gains.cols();
	for (int i=0;i<m;i++)
	{
		for (int j=0;j<n;j++)
		{
			msg_gains.data.push_back(m_gains(i, j));
		}
	}

	pub_adp_gains.publish(msg_gains);
}
#endif

void pub_cmd_callback(const ros::TimerEvent& e) {
	geometry_msgs::Twist cmd_vel;

	hover_controller.compute_outputs(&cmd_vel);
	pub_command.publish(cmd_vel);
}
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
// * Main
////////////////////////////////////////////////////////
int main(int argc, char **argv) {
	ros::init(argc, argv, "hover_controller");
	ros::NodeHandle n("~");

	PID_Config_t config_x;
	PID_Config_t config_y;
	PID_Config_t config_z;

	ReferenceModel_Config_t config_model_z;
	AdaptiveLaw_Config_t config_law_z;

	// MatLab + tuned
	//n.param("Kpx", config_x.Kp,  -0.259246154*1.5);
	//n.param("Kix", config_x.Ki,  -0.049913621*1.5);
	//n.param("Kdx", config_x.Kd,  -0.336624385*1.5);
	//n.param("Kpy", config_y.Kp,  -0.259246154*1.5);
	//n.param("Kiy", config_y.Ki,  -0.049913621*1.5);
	//n.param("Kdy", config_y.Kd,  -0.336624385*1.5);
	//n.param("Kpz", config_z.Kp,  42692.24448);
	//n.param("Kiz", config_z.Ki,  8219.695803);
	//n.param("Kdz", config_z.Kd,  55434.76858);

	// MatLab
	n.param("Kpx", config_x.Kp,  -0.259246154*1.5);
	n.param("Kix", config_x.Ki,  -0.049913621*1.5);
	n.param("Kdx", config_x.Kd,  -0.336624385*1.5);
	n.param("Kpy", config_y.Kp,  -0.259246154*1.5);
	n.param("Kiy", config_y.Ki,  -0.049913621*1.5);
	n.param("Kdy", config_y.Kd,  -0.336624385*1.5);
	n.param("Kpz", config_z.Kp,  42692.24448);
	n.param("Kiz", config_z.Ki,  8219.695803);
	n.param("Kdz", config_z.Kd,  55434.76858);

	config_x.dt 		= POSITION_LOOP_DT;
	config_x.threshold	= 1.0e-4;
	config_x.max_output	= 4.0e4;
	config_y.dt 		= POSITION_LOOP_DT;
	config_y.threshold	= 1.0e-4;
	config_y.max_output	= 4.0e4;
	config_z.dt 		= POSITION_LOOP_DT;
	config_z.threshold	= 1.0e-4;
	config_z.max_output	= 4.0e4;
    
    // Reference model
	config_model_z.A		= MatrixXd::Zero(2,2);
    config_model_z.B		= MatrixXd::Zero(2,1);
	config_model_z.states0	= VectorXd::Zero(2);
	config_model_z.dt		= POSITION_LOOP_DT;
    config_model_z.A(0,0)	=  0.0;
    config_model_z.A(0,1)	=  1.0;
    config_model_z.A(1,0)	= -5.416;
    config_model_z.A(1,1)	= -7.027;
    config_model_z.B(0,0)	= -0.1145;
    config_model_z.B(1,0)	=  6.273;
	config_model_z.states0(0) = -0.03; // ground is not at zero
    
    // Adaptive Law
#ifndef USE_RBF
	config_law_z.Gamma	= 6.0e-4*MatrixXd::Identity(2,2);
#else
	config_law_z.Gamma	= 6.0e-4*MatrixXd::Identity(nrbf,nrbf);
#endif
	config_law_z.P		= MatrixXd::Zero(2,2);
	config_law_z.B		= MatrixXd::Zero(2,1);
	config_law_z.dt		= POSITION_LOOP_DT;
    config_law_z.P(0,0) = 72.373382399897039;
    config_law_z.P(0,1) =  9.231905465288030;
    config_law_z.P(1,0) =  9.231905465288030;
    config_law_z.P(1,1) =  1.384930335176893;
    config_law_z.B(0,0) =  0.0;
    config_law_z.B(1,0) = 26.232948583420782;
	
#ifdef USE_ADAPTIVE_CONTROLLER
    hover_controller.initialize(config_x, config_y, config_z,
								config_model_z, config_law_z, 
								POSITION_LOOP_DT);
#else
	hover_controller.initialize(config_x, config_y, config_z);
#endif
    
    ros::Subscriber sub_position = n.subscribe<geometry_msgs::PoseStamped>(
        "current_pose", 1, &Controller::update_position, (Controller*) &hover_controller);
    ros::Subscriber sub_target = n.subscribe<geometry_msgs::Pose>(
        "target_pose", 1, &Controller::update_target, (Controller*) &hover_controller);
	ros::Subscriber sub_flight_state = n.subscribe<std_msgs::UInt8>(
        "state", 1, sub_flight_state_callback);

	pub_command = n.advertise<geometry_msgs::Twist>("/crazyflie/cmd_vel", 1);
	ros::Timer timer_pub_cmd = n.createTimer(
        ros::Duration(POSITION_LOOP_DT), pub_cmd_callback);
#ifdef USE_ADAPTIVE_CONTROLLER
	pub_ref_states = n.advertise<geometry_msgs::PoseStamped>("reference_states", 1);
	ros::Timer timer_pub_ref_states = n.createTimer(
        ros::Duration(POSITION_LOOP_DT), pub_ref_states_callback);

	pub_adp_gains = n.advertise<std_msgs::Float64MultiArray>("adaptive_gains", 1);
	ros::Timer timer_pub_adp_gains = n.createTimer(
        ros::Duration(POSITION_LOOP_DT), pub_adp_gains_callback);
#endif
	
	ros::spin();

	return 0;
}
////////////////////////////////////////////////////////
