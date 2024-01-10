// ROS libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "boost/thread.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Wrench.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Point.h"

// Custom Library
#include "utils.h"

// C++ libraries
#include <Eigen/QR>
#include <eigen3/Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <fstream>
#include <math.h>

//Namespace Definition
using namespace std;
using namespace Eigen;

class CONTROLLER {
	public:
		CONTROLLER();
        void loop();

		// Publishers fun
		void republish();

		// Subscribers Callback fun
	    void ft_sens_cb( const gazebo_msgs::ContactsState ft_sens_msg );
	    void traj_cb( const geometry_msgs::Point msg );
 
		// Various fun
		void run();

	private:
		// ROS variables
		ros::NodeHandle _nh;
		ros::Publisher  _ft_repub;
		ros::Subscriber _arm_ft_sub, _traj_sub;

        gazebo_msgs::ContactsState _ft_sens;
        int _rate;
        // Interaction forces
		Eigen::VectorXd _F_ee;
		Eigen::Vector3d _f_ext, _m_ext, _f_ext_filtered, _f_ext_old, _m_ext_filtered, _m_ext_old;
		Eigen::Vector3d _traj;
};

// Class constructor
CONTROLLER::CONTROLLER() {

	// ROS publishers
	_ft_repub   = _nh.advertise< geometry_msgs::Wrench >("/ndt2/ft_sensor/repub", 0);
	// ROS subscribers
    _arm_ft_sub  = _nh.subscribe("/ndt2/ft_sensor", 1, &CONTROLLER::ft_sens_cb, this);
    _traj_sub  = _nh.subscribe("/ndt2/trajectory/lin_pos", 1, &CONTROLLER::traj_cb, this);
	
    _rate = 250;
    _F_ee.resize(6);
	_F_ee.setZero();
	_f_ext.setZero();
	_f_ext_old.setZero();
	_f_ext_filtered.setZero();
	_m_ext.setZero();
	_m_ext_old.setZero();
	_m_ext_filtered.setZero();
	_traj.setZero();
}

// Callback force sensor feedback
void CONTROLLER::ft_sens_cb( const gazebo_msgs::ContactsState ft_sens_msg ){

	_ft_sens = ft_sens_msg;
}

void CONTROLLER::traj_cb( const geometry_msgs::Point msg ){
	_traj << msg.x, msg.y, msg.z;
}


void CONTROLLER::loop() {

	ros::Rate r(_rate);
	
	while ( ros::ok() ) {

        // Force sensor feedback
        double alpha = 0.25; //Filter coefficient, adjust as needed
        // double alpha = 1.0; //Filter coefficient, adjust as needed
        
        if (!_ft_sens.states.empty()){
        _f_ext << _ft_sens.states[0].total_wrench.force.x,_ft_sens.states[0].total_wrench.force.y,_ft_sens.states[0].total_wrench.force.z;
        _m_ext << _ft_sens.states[0].total_wrench.torque.x,_ft_sens.states[0].total_wrench.torque.y,_ft_sens.states[0].total_wrench.torque.z;
        _f_ext_filtered = alpha * _f_ext + (1.0 - alpha) * _f_ext_old;
        _m_ext_filtered = alpha * _m_ext + (1.0 - alpha) * _m_ext_old;
        _F_ee << _m_ext, _f_ext;
        _F_ee << _m_ext_filtered, _f_ext_filtered;
		_f_ext_old = _f_ext_filtered;
		_m_ext_old = _m_ext_filtered;
        }
        else {
            // _f_ext << 0,0,0;
            // _F_ee.setZero();
			// _f_ext_old.setZero();
			// _m_ext_old.setZero();
        }
            
        r.sleep();
	}
}

// Arm torques command
void CONTROLLER::republish() {
    geometry_msgs::Wrench msg;
	ros::Rate r(_rate);
	
    while( ros::ok() ) {
        msg.force.x = _F_ee(3);
        msg.force.y = _F_ee(4);
        msg.force.z = _F_ee(5);
        msg.torque.x = _F_ee(0);
        msg.torque.y = _F_ee(1);
        msg.torque.z = _F_ee(2);
        _ft_repub.publish(msg);

    	r.sleep();
    }

}

// Mult-thread
void CONTROLLER::run() {
	boost::thread loop_t( &CONTROLLER::loop, this );
	boost::thread republish_t( &CONTROLLER::republish, this );

	ros::spin();
}

// Main
int main(int argc, char** argv ) {

	ros::init(argc, argv, "cmd_vel_ctrl");

	CONTROLLER kc;
	kc.run();

	return 0;
}
