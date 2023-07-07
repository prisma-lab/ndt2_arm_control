// ROS libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
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

// numerical integration methods
double wrapToRange(double value, double min_value, double max_value) {
    double range = max_value - min_value;
    double wrapped_value = fmod((value - min_value), range) + min_value;
    if (wrapped_value < min_value) {
        wrapped_value += range;
    }
    return wrapped_value;
}

// Class Definition
class CONTROLLER {
	public:
		CONTROLLER();

		// Trajectory fun
		void new_plan();
		Eigen::MatrixXd traj_generation(Eigen::Matrix4d, Eigen::Matrix4d, float, float, int);

		// Publishers fun
		void cmd_arm_ctrl();

		// Subscribers Callback fun
		void found_cb(const std_msgs::Bool found_msg);
		void image_cb(const geometry_msgs::Point fb_msg);
		void ndt2_state_cb( const nav_msgs::Odometry odometry_msg );
		void ft_sens_cb( const gazebo_msgs::ContactsState ft_sens_msg );
        void joint_state_cb( const sensor_msgs::JointState arm_state_msg );
		void camera_cb(geometry_msgs::Pose pose_msg);
		void camera_flag_cb(std_msgs::Bool flag_msg);
        void wrench_cb( const geometry_msgs::Wrench wrench_msg );
		void vs_cb(const std_msgs::Float64MultiArray corner_msg );
		void vs_flag_cb(const std_msgs::Bool flag_msg);

		// Screw Theory Modelling fun
		Eigen::Matrix4d dir_kin(Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd);
		Eigen::MatrixXd diff_kin(Eigen::VectorXd);
		Eigen::VectorXd recursive_inv_dyn(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::Vector3d, Eigen::VectorXd );
		Eigen::MatrixXd inertia_matrix(Eigen::VectorXd);
		Eigen::MatrixXd c_forces(Eigen::VectorXd,Eigen::VectorXd);
		Eigen::MatrixXd gravity_forces(Eigen::VectorXd);
		
		// Controller fun
		void arm_invdyn_control();

		// FileLog fun
		void fileclose();
        void writedata(const Vector3d & , std::ofstream & ofs);
        void writedata(const Vector2d & , std::ofstream & ofs);
		void writedata(const VectorXd & , std::ofstream & ofs);
        void writedata(const Vector4d & data,std::ofstream & ofs);
		// Various fun
		void run();
		void load_parameters();
		void coupled_feedback();
		void camera_parm_computation();

	private:
		// ROS variables
		ros::NodeHandle _nh;
		ros::Publisher  _extf_pub;
		ros::Publisher  _cmd_tau1_pub, _cmd_tau2_pub, _cmd_tau3_pub, _cmd_tau4_pub, _cmd_tau5_pub, _cmd_tau6_pub;
		ros::Publisher _vs_pub;
		ros::Publisher _pos_sp_pub, _vel_sp_pub, _acc_sp_pub;
        ros::Subscriber _model_state_sub;
        ros::Subscriber _ndt2_state_sub;
        ros::Subscriber _ndt2_wrench_sub;
		ros::Subscriber _arm_ft_sub;
        ros::Subscriber _joint_state_sub;
        ros::Subscriber _image_fb_sub, _blob_found_sub, _tag_fb_sub, _tag_flag_sub, _vs_ee_sub, _ndt2_vs_sub;
		ros::Subscriber _ft_sensor_sub;
		ros::ServiceClient _pauseGazebo;
		ros::ServiceClient _unpauseGazebo;

		Eigen::VectorXd _arm_cmd, _f_cmd, _pos_cmd, _vel_cmd, _acc_cmd;
        nav_msgs::Odometry _odom, _gaz_odom;
		gazebo_msgs::ContactsState _ft_sens;
        sensor_msgs::JointState _arm_state;
        Eigen::Vector3d _Eta;
        bool _first_odom_1, _first_odom_2;
		Eigen::VectorXd _arm_state_pos, _arm_state_vel, _tilt_state_pos, _tilt_state_vel;
		Eigen::Matrix3d _R_uav;
		Eigen::Matrix4d _T_uav, _T_arm,_NED_T_arm, _T_coupled;
		Eigen::MatrixXd _T_traj, _Td_traj;
		Eigen::Vector3d _p_uav;
		Eigen::MatrixXd _J_arm, _NED_J_arm, _J_coupled;
		Eigen::VectorXd _vel_mes, _vel_mes_coupled, _pose_mes_uav, _pose_mes_coupled, _arm_vel_mes, _vel_mes_uav, _ee_pos_to_world;
		bool _traj_ready, _goal_reached, _start_controller;

		//------kinematic/dynamic modeling
		int _N_JOINTS;
		Eigen::Vector3d _d_1_2, _d_2_3, _d_3_4, _d_4_5, _d_5_6, _d_6_7;
		Eigen::Matrix3d _R_1_2, _R_2_3, _R_3_4, _R_4_5, _R_5_6, _R_6_7;
		Eigen::Matrix4d _T_1_2, _T_2_3, _T_3_4, _T_4_5, _T_5_6, _T_6_7, _M;
		Eigen::Vector3d _w_1, _w_2, _w_3, _w_4, _w_5, _w_6;
		Eigen::Vector3d _v_1, _v_2, _v_3, _v_4, _v_5, _v_6;
		Eigen::Vector3d _q_1, _q_2, _q_3, _q_4, _q_5, _q_6;
		Eigen::Vector3d _d_1_base, _d_2_base, _d_3_base, _d_4_base, _d_5_base, _d_6_base;
		Eigen::VectorXd _S_1, _S_2, _S_3, _S_4, _S_5, _S_6;
		Eigen::MatrixXd _S_axis, _w, _v, _q, _d, _d_base;
		Eigen::VectorXd _curr_rot, _rot_vec;
		Eigen::Vector3d _gravity;
		Eigen::VectorXd _mass;
		Eigen::MatrixXd _Inertia, _inertial_disp;
		Eigen::MatrixXd _rot_mat, _rot_angle;

		//-----uav dynamic param
		Eigen::Matrix3d _uav_I;
		float _uav_mass, _ut;
		Eigen::Vector3d _tau_b, _thrust;

		// Camera variables
		geometry_msgs::Point _img_fb;
		Eigen::Vector2d _image_fb;
		Eigen::MatrixXd _K_camera, _P_camera, _T_cb,_T_cb_temp;
		bool _camera_on,_start_ibvs;
		Eigen::Vector3d _image_fb_aug;
		Eigen::VectorXd _ee_camera, _local_vel_6d_cf,_local_vel_6d_cf_temp;
		std_msgs::Bool _found_blob;
		bool _flag_blob, _found_tag;
		geometry_msgs::Pose _tag_pose_fb;
        Eigen::Vector4d _pose_tag_aug;
		Eigen::Vector3d _image_fb_aug_norm;
		Eigen::MatrixXd _L_matrix,_L_matrix_p1,_L_matrix_p2,_L_matrix_p3,_L_matrix_p4, _J_image, _gamma_matrix;
		Eigen::MatrixXd _s_dot,_s,_s_dot_EE,_s_EE, _arm_vel_mes_cam_fram, _arm_vel_mes_cam_fram_temp, ROT_Frame;
        Eigen::MatrixXd _pose_tag_corner_aug, _image_fb_aug_corner, _image_fb_aug_norm_corner, _pose_EE_corner_aug, _EE_fb_aug_corner, _EE_fb_aug_norm_corner, _L_matrix_4, _J_image_4, _gamma_matrix_4;
		Eigen::VectorXd _ee_tag_corners;

		// FileLog variables
		std::ofstream _esxFile;
		std::ofstream _esyFile;
		std::ofstream _epFile;
        std::ofstream _evFile;
        std::ofstream _erpyFile;
        std::ofstream _ewFile;
        std::ofstream _f_extFile;
        std::ofstream _tauFile;

		// Param from file
		int _rate, _iii;
		Eigen::MatrixXd _Kp, _Kd, _Kp_cam, _Kd_cam, _Ki_cam;
		std::vector<double> _Kp_pos_yaml, _Kd_pos_yaml,_Kp_att_yaml,_Kd_att_yaml,_Kp_pos_f_yaml,_Ki_pos_f_yaml, _Kp_cam_yaml, _Kd_cam_yaml, _Ki_cam_yaml;

		// Interaction forces
		Eigen::VectorXd _F_ee;
		Eigen::Vector3d _f_ext, _m_ext, _f_ext_filtered, _f_ext_old;

		//Reference frame param
		Eigen::Matrix4d _T_NED;

		Eigen::VectorXd _ext_wrench;
		bool _start_vs;

};

// Load params from file
void CONTROLLER::load_parameters(){
    if( !_nh.getParam("rate", _rate)) {
        _rate = 100;
    }
	_nh.getParam("Kp_pos", _Kp_pos_yaml);
    _nh.getParam("Kd_pos", _Kd_pos_yaml);
    _nh.getParam("Kp_att", _Kp_att_yaml);
    _nh.getParam("Kd_att", _Kd_att_yaml);
    _nh.getParam("Kp_pos_f", _Kp_pos_f_yaml);
    _nh.getParam("Kp_pos_f", _Ki_pos_f_yaml);
    _nh.getParam("Kp_cam", _Kp_cam_yaml);
    _nh.getParam("Kd_cam", _Kd_cam_yaml);
    _nh.getParam("Ki_cam", _Ki_cam_yaml);
	_Kp.resize(6,6);
	_Kd.resize(6,6);
    _Kp <<  _Kp_att_yaml[0],              0,               0,               0,               0,               0,
	                      0,_Kp_att_yaml[1],               0,               0,               0,               0,
						  0,              0, _Kp_att_yaml[2],               0,               0,               0,
						  0,              0,               0, _Kp_pos_yaml[0],               0,               0,
						  0,              0,               0,               0, _Kp_pos_yaml[1],               0,
						  0,              0,               0,               0,               0, _Kp_pos_yaml[2];

    _Kd <<  _Kd_att_yaml[0],              0,               0,               0,               0,               0,
	                      0,_Kd_att_yaml[1],               0,               0,               0,               0,
						  0,              0, _Kd_att_yaml[2],               0,               0,               0,
						  0,              0,               0, _Kd_pos_yaml[0],               0,               0,
						  0,              0,               0,               0, _Kd_pos_yaml[1],               0,
						  0,              0,               0,               0,               0, _Kd_pos_yaml[2];
	_Kp_cam.resize(8,8);
	_Kd_cam.resize(8,8);
	_Ki_cam.resize(8,8);
	_Kp_cam <<  _Kp_cam_yaml[0],              0,               0,               0,               0,               0,               0,               0,
	                          0,_Kp_cam_yaml[1],               0,               0,               0,               0,               0,               0,
						      0,              0, _Kp_cam_yaml[2],               0,               0,               0,               0,               0,
						      0,              0,               0, _Kp_cam_yaml[3],               0,               0,               0,               0,
						      0,              0,               0,               0, _Kp_cam_yaml[4],               0,               0,               0,
						      0,              0,               0,               0,               0, _Kp_cam_yaml[5],               0,               0,
						      0,              0,               0,               0,               0,               0, _Kp_cam_yaml[6],               0,
						      0,              0,               0,               0,               0,               0,               0, _Kp_cam_yaml[7];
	_Kd_cam <<  _Kd_cam_yaml[0],              0,               0,               0,               0,               0,               0,               0,
	                          0,_Kd_cam_yaml[1],               0,               0,               0,               0,               0,               0,
						      0,              0, _Kd_cam_yaml[2],               0,               0,               0,               0,               0,
						      0,              0,               0, _Kd_cam_yaml[3],               0,               0,               0,               0,
						      0,              0,               0,               0, _Kd_cam_yaml[4],               0,               0,               0,
						      0,              0,               0,               0,               0, _Kd_cam_yaml[5],               0,               0,
						      0,              0,               0,               0,               0,               0, _Kd_cam_yaml[6],               0,
						      0,              0,               0,               0,               0,               0,               0, _Kd_cam_yaml[7];
	_Ki_cam <<  _Ki_cam_yaml[0],              0,               0,               0,               0,               0,               0,               0,
	                          0,_Ki_cam_yaml[1],               0,               0,               0,               0,               0,               0,
						      0,              0, _Ki_cam_yaml[2],               0,               0,               0,               0,               0,
						      0,              0,               0, _Ki_cam_yaml[3],               0,               0,               0,               0,
						      0,              0,               0,               0, _Ki_cam_yaml[4],               0,               0,               0,
						      0,              0,               0,               0,               0, _Ki_cam_yaml[5],               0,               0,
						      0,              0,               0,               0,               0,               0, _Ki_cam_yaml[6],               0,
						      0,              0,               0,               0,               0,               0,               0, _Ki_cam_yaml[7];

}

// Class constructor
CONTROLLER::CONTROLLER() : _first_odom_1(false), _first_odom_2(false), _camera_on(false) {

	load_parameters();
	// ROS services
	_pauseGazebo   = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    _unpauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

	// ROS publishers
	_vs_pub   = _nh.advertise< std_msgs::Float64MultiArray >("/ndt2/ee_pos_corner", 0);
    _cmd_tau1_pub  = _nh.advertise< std_msgs::Float64 >("/Rev3_position_controller/command", 0);
    _cmd_tau2_pub  = _nh.advertise< std_msgs::Float64 >("/Rev4_position_controller/command", 0);
    _cmd_tau3_pub  = _nh.advertise< std_msgs::Float64 >("/Rev5_position_controller/command", 0);
    _cmd_tau4_pub  = _nh.advertise< std_msgs::Float64 >("/Rev6_position_controller/command", 0);
    _cmd_tau5_pub  = _nh.advertise< std_msgs::Float64 >("/Rev7_position_controller/command", 0);
    _cmd_tau6_pub  = _nh.advertise< std_msgs::Float64 >("/Rev8_position_controller/command", 0);
	_extf_pub      = _nh.advertise<geometry_msgs::Wrench>("/arm/wrench", 0);
	_pos_sp_pub    = _nh.advertise<std_msgs::Float64MultiArray>("/arm/pos_sp", 0);
	_vel_sp_pub    = _nh.advertise<std_msgs::Float64MultiArray>("/arm/vel_sp", 0);
	_acc_sp_pub    = _nh.advertise<std_msgs::Float64MultiArray>("/arm/acc_sp", 0);
	// ROS subscribers
    _ndt2_state_sub  = _nh.subscribe("/ndt2/odometry", 1, &CONTROLLER::ndt2_state_cb, this);
    _joint_state_sub = _nh.subscribe("/joint_states", 1, &CONTROLLER::joint_state_cb, this);
	_image_fb_sub    = _nh.subscribe("/image/poseblue", 1, &CONTROLLER::image_cb, this);
	_blob_found_sub  = _nh.subscribe("/image/result", 1, &CONTROLLER::found_cb, this);
	_ndt2_wrench_sub  = _nh.subscribe("/ndt2/wrench", 1, &CONTROLLER::wrench_cb, this);
	_arm_ft_sub  = _nh.subscribe("/ndt2/ft_sensor", 1, &CONTROLLER::ft_sens_cb, this);
    _tag_fb_sub = _nh.subscribe("/ndt2/camera/tag_pose", 1, &CONTROLLER::camera_cb, this);
	_tag_flag_sub = _nh.subscribe("/ndt2/camera/tag_flag", 1, &CONTROLLER::camera_flag_cb, this);
    _vs_ee_sub = _nh.subscribe("/ndt2/tag_pos_px_corner_ee", 1, &CONTROLLER::vs_cb, this);
	_ndt2_vs_sub = _nh.subscribe("/ndt2/ibvs/flag", 1, &CONTROLLER::vs_flag_cb, this);

	_arm_cmd.resize(6);
	_arm_cmd[0] = 0.0;
	_arm_cmd[1] = 0.0;
	_arm_cmd[2] = 0.0;
	_arm_cmd[3] = 0.0;
	_arm_cmd[4] = 0.0;
	_arm_cmd[5] = 0.0;

	_pos_cmd.resize(6);
	_pos_cmd[0] = 0.0;
	_pos_cmd[1] = 0.0;
	_pos_cmd[2] = 0.0;
	_pos_cmd[3] = 0.0;
	_pos_cmd[4] = 0.0;
	_pos_cmd[5] = 0.0;

	_vel_cmd.resize(6);
	_vel_cmd[0] = 0.0;
	_vel_cmd[1] = 0.0;
	_vel_cmd[2] = 0.0;
	_vel_cmd[3] = 0.0;
	_vel_cmd[4] = 0.0;
	_vel_cmd[5] = 0.0;

	_acc_cmd.resize(6);
	_acc_cmd[0] = 0.0;
	_acc_cmd[1] = 0.0;
	_acc_cmd[2] = 0.0;
	_acc_cmd[3] = 0.0;
	_acc_cmd[4] = 0.0;
	_acc_cmd[5] = 0.0;

	_first_odom_1 = false;
	_first_odom_2 = false;

	_tilt_state_pos.resize(4);
	_tilt_state_pos << 0,0,0,0;
	_tilt_state_vel.resize(4);
	_tilt_state_vel << 0,0,0,0;
	_arm_state_pos.resize(6);
	_arm_state_pos  << 0,0,0,0,0,0;
	_arm_state_vel.resize(6);
	_arm_state_vel  << 0,0,0,0,0,0;

	_arm_vel_mes.resize(6);
	_arm_vel_mes << 0,0,0,0,0,0;

	_T_uav.setIdentity();
	_T_arm.setIdentity();
	_NED_T_arm.setIdentity();
	_T_coupled.setIdentity();
	_R_uav.setIdentity();
	_p_uav<<0,0,0;
	_J_arm.resize(6,6);
	_J_arm.setIdentity();
	_NED_J_arm.resize(6,6);
	_NED_J_arm.setIdentity();
	_J_coupled.resize(12,12);
	_J_coupled.setIdentity();

	_pose_mes_uav.resize(6);
	_vel_mes_uav.resize(6);
	_pose_mes_coupled.resize(12);
	_pose_mes_uav << 0,0,0,0,0,0;
	_vel_mes_uav  << 0,0,0,0,0,0;
	_pose_mes_coupled << 0,0,0,0,0,0,0,0,0,0,0,0;

	_vel_mes.resize(12);
	_vel_mes_coupled.resize(12);
	_vel_mes << 0,0,0,0,0,0,0,0,0,0,0,0;
	_vel_mes_coupled << 0,0,0,0,0,0,0,0,0,0,0,0;
	_ee_pos_to_world.resize(6);
	_ee_pos_to_world << 0,0,0,0,0,0;
	_iii = 0;

	// +++++++++++++++++ kinematic/dynamic modeling params ++++++++++++++++++++++++++++
	_N_JOINTS = 6;

    _d_1_2	<<         0.0, 0.0, 0.0;
	_d_2_3	<<   -0.010, -0.035, 0.0;
	_d_3_4	<<    -0.020, 0.0, -0.20;
	_d_4_5	<<    0.010, 0.0, -0.190;
	_d_5_6	<<   -0.012, 0.0, -0.010;
	_d_6_7	<< 0.012, 0.0, -0.032022;

	_R_1_2	<<   0,   0, -1.0,
			  -1.0,   0,    0,
			     0, 1.0,    0;

	_R_2_3.setIdentity();
	_R_3_4.setIdentity();
	_R_4_5.setIdentity();
	_R_5_6.setIdentity();
	_R_6_7.setIdentity();

    _T_1_2.block<3,3>(0,0) = _R_1_2;
    _T_1_2.block<3,1>(0,3) = _d_1_2;
    _T_1_2.block<1,4>(3,0) << 0, 0, 0, 1;

	_T_2_3.block<3,3>(0,0) = _R_2_3;
    _T_2_3.block<3,1>(0,3) = _d_2_3;
    _T_2_3.block<1,4>(3,0) << 0, 0, 0, 1;

	_T_3_4.block<3,3>(0,0) = _R_3_4;
    _T_3_4.block<3,1>(0,3) = _d_3_4;
    _T_3_4.block<1,4>(3,0) << 0, 0, 0, 1;

	_T_4_5.block<3,3>(0,0) = _R_4_5;
    _T_4_5.block<3,1>(0,3) = _d_4_5;
    _T_4_5.block<1,4>(3,0) << 0, 0, 0, 1;

	_T_5_6.block<3,3>(0,0) = _R_5_6;
    _T_5_6.block<3,1>(0,3) = _d_5_6;
    _T_5_6.block<1,4>(3,0) << 0, 0, 0, 1;

	_T_6_7.block<3,3>(0,0) = _R_6_7;
    _T_6_7.block<3,1>(0,3) = _d_6_7;
    _T_6_7.block<1,4>(3,0) << 0, 0, 0, 1;

	_M = _T_1_2*_T_2_3*_T_3_4*_T_4_5*_T_5_6*_T_6_7;

    _w_1 <<  0,0,1;
	_w_2 << 0,-1,0;
    _w_3 << 0,-1,0;
	_w_4 << -1,0,0;
	_w_5 << 0,-1,0;
	_w_6 <<  1,0,0;

	_d_1_base = _M.block<3,3>(0,0)*_d_1_2;
	_d_2_base = _M.block<3,3>(0,0)*_d_2_3;
	_d_3_base = _M.block<3,3>(0,0)*_d_3_4;
	_d_4_base = _M.block<3,3>(0,0)*_d_4_5;
	_d_5_base = _M.block<3,3>(0,0)*_d_5_6;
	_d_6_base = _M.block<3,3>(0,0)*_d_6_7;

	_q_1 = _d_1_base;
	_q_2 = _d_2_base+_q_1;
	_q_3 = _d_3_base+_q_2;
	_q_4 = _d_4_base+_q_3;
	_q_5 = _d_5_base+_q_4;
	_q_6 = _d_6_base+_q_5;

	_v_1 = (-_w_1).cross(_q_1);
	_v_2 = (-_w_2).cross(_q_2);
	_v_3 = (-_w_3).cross(_q_3);
	_v_4 = (-_w_4).cross(_q_4);
	_v_5 = (-_w_5).cross(_q_5);
	_v_6 = (-_w_6).cross(_q_6);

	_S_1.resize(6);
	_S_1 << _w_1,_v_1;
	_S_2.resize(6);
	_S_2 << _w_2,_v_2;
	_S_3.resize(6);
	_S_3 << _w_3,_v_3;
	_S_4.resize(6);
	_S_4 << _w_4,_v_4;
	_S_5.resize(6);
	_S_5 << _w_5,_v_5;
	_S_6.resize(6);
	_S_6 << _w_6,_v_6;

	_S_axis.resize(6,6);
	_S_axis <<    _S_1.transpose(),
	              _S_2.transpose(),
				  _S_3.transpose(),
				  _S_4.transpose(),
				  _S_5.transpose(),
				  _S_6.transpose();

	_w.resize(6,3);
	_w      <<    _w_1.transpose(),
	              _w_2.transpose(),
				  _w_3.transpose(),
				  _w_4.transpose(),
				  _w_5.transpose(),
				  _w_6.transpose();

	_q.resize(6,3);
	_q      <<    _q_1.transpose(),
				  _q_2.transpose(),
				  _q_3.transpose(),
				  _q_4.transpose(),
				  _q_5.transpose(),
				  _q_6.transpose();

	_v.resize(6,3);
	_v      <<    _v_1.transpose(),
				  _v_2.transpose(),
				  _v_3.transpose(),
				  _v_4.transpose(),
				  _v_5.transpose(),
				  _v_6.transpose();

	_d.resize(6,3);
	_d      <<   _d_1_2.transpose(),
				 _d_2_3.transpose(),
				 _d_3_4.transpose(),
				 _d_4_5.transpose(),
				 _d_5_6.transpose(),
				 _d_6_7.transpose();

	_d_base.resize(6,3);
	_d_base <<   _d_1_base.transpose(),
				 _d_2_base.transpose(),
				 _d_3_base.transpose(),
				 _d_4_base.transpose(),
				 _d_5_base.transpose(),
				 _d_6_base.transpose();

    _curr_rot.resize(6);
    _rot_vec.resize(6);
	_rot_vec   << 0, 3,-2,-2,-1,-2;
	_curr_rot  << 3,-2,-2,-1,-2, 1;

	_gravity   << 0,0,-9.8;

	_rot_mat.resize(6,2);
	_rot_mat   << 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2;
	_rot_angle.resize(6,2);
	_rot_angle << M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2, M_PI/2, -M_PI/2;

	_mass.resize(6);
	_mass  <<  0.6539245694270445 + 0.18907008597028982,
			  					    0.04111040389630535,
								   0.031775151367082886,
								   0.019335321488620901,
								   0.017370138299177099,
								   0.016738716241950105;

	_Inertia.resize(18,18);
	_Inertia.setIdentity();
	_Inertia.block<3,3>(0,0)   <<    0.000289463446926973,   					0, 						0,
														0,   0.000081000023334641,   					0,
														0, 						0,   0.000267463443716636;

	_Inertia.block<3,3>(3,3)   <<    0.000659107447714460,   					0, 						0,
														0, 	 0.000663218619586273,   					0,
														0, 						0, 	 0.000007111171885526;

	_Inertia.block<3,3>(6,6)   <<    0.000286853595660124,   					0, 						0,
														0,   0.000290031211674448,   					0,
														0, 						0,   0.000005177616025130;

	_Inertia.block<3,3>(9,9)   <<  0.00000182410565073118, 						0, 						0,
													    0, 0.00000282410564798869, 						0,
														0, 						0, 0.00000100000000378813;

	_Inertia.block<3,3>(12,12) <<  0.00000192725542768134, 						0, 						0,
														0, 0.00000442862389132953, 						0,
														0, 						0, 0.00000350136846968257;

	_Inertia.block<3,3>(15,15) <<  0.00000194154222411342,                      0,                      0,
														0, 0.00000194154222165852,                      0,
														0,                      0, 0.00000100000000336012;

	_inertial_disp.resize(6,3);
	_inertial_disp <<  -0.000007295146686, -0.032723661239861,  0.000008378471211,
						0.010000159920573,  0.000000408390010, -0.100000414583312,
						0.010000158743883,  0.000000412357516, -0.070705746319131,
						0.000000164437176,  0.000000410947985, -0.009712908448932,
						0.012000164437172,  0.000000416772969, -0.007306309297656,
						0.000000164437176,  0.000000416772969,  0.007499957912211;

	_traj_ready = false;
	_goal_reached = true;
	_start_controller = false;

	//uav
	_uav_mass = 5,414 + 0.6539245694270445;
	_uav_I  << 0.29163, 0, 0, 0, 0.29163, 0, 0, 0, 0.3527;
	_ut = 0.0;
	_tau_b  << 0.0,0.0,0.0;
	_thrust << 0.0, 0.0, 0.0;

	_image_fb << 0,0;
	_image_fb_aug << 0,0,0;
	_K_camera.resize(3,3);
	_P_camera.resize(3,4);
	_T_cb.resize(4,4);
	_T_cb_temp.resize(4,4);

	// Camera intrinsic parameters
    _K_camera <<    476.7030,       0.0,     400.50,
						 0.0,  476.7030,     300.50,
						 0.0,       0.0,       1.00;

	_P_camera << 1,0,0,0,
		  		 0,1,0,0,
		  		 0,0,1,0; // PI matrix

	_T_cb_temp <<   1, 0, 0, 0.085, // From base_link frame to camera_temp frame
                	0, 1, 0,     0,
               		0, 0, 1, 0.060,
                	0, 0, 0,     1;

	_T_cb <<    0, 0, 1, 0, // From camera_temp frame to camera frame
               -1, 0, 0, 0,
                0,-1, 0, 0,
                0, 0, 0, 1;				

	_camera_on = false;
	_ee_camera.resize(4);
	_ee_camera << 0,0,0,0;

	_flag_blob = false;
	_found_tag = false;

	_tauFile.open("/home/simone/uav_ws/src/ndt2_arm_control/log/tau.txt", std::ios::out);
    _epFile.open("/home/simone/uav_ws/src/ndt2_arm_control/log/ep.txt", std::ios::out);
    _esxFile.open("/home/simone/uav_ws/src/ndt2_arm_control/log/esx.txt", std::ios::out);
    _esyFile.open("/home/simone/uav_ws/src/ndt2_arm_control/log/esy.txt", std::ios::out);
    _evFile.open("/home/simone/uav_ws/src/ndt2_arm_control/log/ev.txt", std::ios::out);
    _ewFile.open("/home/simone/uav_ws/src/ndt2_arm_control/log/ew.txt", std::ios::out);
    _erpyFile.open("/home/simone/uav_ws/src/ndt2_arm_control/log/erpy.txt", std::ios::out);
	_f_extFile.open("/home/simone/uav_ws/src/ndt2_arm_control/log/f_ext.txt", std::ios::out);

	_F_ee.resize(6);
	_F_ee.setZero();
	_f_ext.setZero();
	_f_ext_old.setZero();
	_f_ext_filtered.setZero();
	_m_ext.setZero();

	_T_NED = utilities::rotx_T(-M_PI);
	_ext_wrench.resize(6);
	_ext_wrench.setZero();
	_L_matrix.resize(8,6);
	_L_matrix.setZero();
	_L_matrix_p1.resize(2,6);
	_L_matrix_p1.setZero();
	_L_matrix_p2.resize(2,6);
	_L_matrix_p2.setZero();
	_L_matrix_p3.resize(2,6);
	_L_matrix_p3.setZero();
	_L_matrix_p4.resize(2,6);
	_L_matrix_p4.setZero();
	_J_image.resize(8,6);
	_J_image.setZero();
	_arm_vel_mes_cam_fram_temp.resize(6,1);
	_arm_vel_mes_cam_fram_temp.setZero();
	_arm_vel_mes_cam_fram.resize(6,1);
	_arm_vel_mes_cam_fram.setZero();
	_s_dot.resize(8,1);
	_s_dot.setZero();	
	_s.resize(8,1);
	_s.setZero();
	_s_dot_EE.resize(8,1);
	_s_dot_EE.setZero();	
	_s_EE.resize(8,1);
	_s_EE.setZero();
	ROT_Frame.resize(6,6);

	_gamma_matrix.resize(6,6);
	_gamma_matrix.setIdentity();
	_gamma_matrix = -1*_gamma_matrix;

	_pose_tag_corner_aug.resize(4,4);
	_image_fb_aug_corner.resize(3,4);
	_image_fb_aug_norm_corner.resize(3,4);

	_pose_EE_corner_aug.resize(4,4);
	_EE_fb_aug_corner.resize(3,4);
	_EE_fb_aug_norm_corner.resize(3,4);

	_local_vel_6d_cf.resize(6);
	_local_vel_6d_cf_temp.resize(6);
	_start_ibvs = false;
	_ee_tag_corners.resize(8);
	_ee_tag_corners.setZero();
	_start_vs=false;
}

// Close log-file
void CONTROLLER::fileclose(){

  _tauFile.close();
  _epFile.close();
  _esxFile.close();
  _esyFile.close();
  _evFile.close();
  _ewFile.close();
  _erpyFile.close();
  _f_extFile.close();

}

// Write log-file
void CONTROLLER::writedata(const Vector3d & data,std::ofstream & ofs){

    ofs<<data(0)<<','<<data(1)<<','<<data(2);
    ofs<<'\n';
}

void CONTROLLER::writedata(const Vector2d & data,std::ofstream & ofs){

    ofs<<data(0)<<','<<data(1);
    ofs<<'\n';
}

void CONTROLLER::writedata(const VectorXd & data,std::ofstream & ofs){

    ofs<<data(0)<<','<<data(1)<<','<<data(2)<<','<<data(3)<<','<<data(4)<<','<<data(5);
    ofs<<'\n';
}

void CONTROLLER::writedata(const Vector4d & data,std::ofstream & ofs){

    ofs<<data(0)<<','<<data(1)<<','<<data(2)<<','<<data(3);
    ofs<<'\n';
}

// ------ CALLBACK ROS SUBSCRIBER ------

// Callback ndt2 feedback
void CONTROLLER::ndt2_state_cb( const nav_msgs::Odometry odometry_msg ) {
    _odom = odometry_msg;
    _Eta = utilities::R2XYZ( utilities::QuatToMat ( Vector4d( _odom.pose.pose.orientation.w,  _odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z ) ) );
    _R_uav = utilities::QuatToMat ( Vector4d( _odom.pose.pose.orientation.w,  _odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z ));
	_p_uav << _odom.pose.pose.position.x,_odom.pose.pose.position.y, _odom.pose.pose.position.z;

	_vel_mes_uav << _odom.twist.twist.linear.x, _odom.twist.twist.linear.y, _odom.twist.twist.linear.z, _odom.twist.twist.angular.x, _odom.twist.twist.angular.z, _odom.twist.twist.angular.z;

	_first_odom_1 = true;
}

// Callback arm feedback
void CONTROLLER::joint_state_cb( const sensor_msgs::JointState arm_state_msg ) {

    _arm_state = arm_state_msg;
	for(int i=0; i < 6;i++){
		_arm_state_pos[i] = _arm_state.position[i];
		_arm_state_vel[i] = _arm_state.velocity[i];
	}
	for(int i=0; i < 4;i++){
		_tilt_state_pos[i] = _arm_state.position[i+6];
		_tilt_state_vel[i] = _arm_state.velocity[i+6];
	}

    _first_odom_2 = true;
}

// Callback force sensor feedback
void CONTROLLER::ft_sens_cb( const gazebo_msgs::ContactsState ft_sens_msg ){

	_ft_sens = ft_sens_msg;
}

void CONTROLLER::camera_flag_cb(std_msgs::Bool flag_msg){
	_found_tag = flag_msg.data;
}

// Callback image servoing
void CONTROLLER::camera_cb(geometry_msgs::Pose pose_msg){

    _tag_pose_fb.position.x = pose_msg.position.x;
    _tag_pose_fb.position.y = pose_msg.position.y;
    _tag_pose_fb.position.z = pose_msg.position.z;
    _tag_pose_fb.orientation.w = pose_msg.orientation.w;
    _tag_pose_fb.orientation.x = pose_msg.orientation.x;
    _tag_pose_fb.orientation.y = pose_msg.orientation.y;
    _tag_pose_fb.orientation.z = pose_msg.orientation.z;

	_camera_on = true;

}

// Callback ndt2 wrench feedback
void CONTROLLER::wrench_cb( const geometry_msgs::Wrench wrench_msg ) {

    _ext_wrench << wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z, wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z;
	// _ext_wrench = utilities::Ad_f(_T_arm.inverse())*(utilities::Ad_f(_T_NED.inverse())*_ext_wrench);
	_ext_wrench = (utilities::Ad_f(_T_NED.inverse())*_ext_wrench);
	// cout<<(_J_arm.transpose()*_ext_wrench).transpose()<<endl;
	// cout<<(_ext_wrench).transpose()<<endl;
}

// Callback camera feedback
void CONTROLLER::image_cb( const geometry_msgs::Point fb_msg ) {

    _img_fb = fb_msg;
	_image_fb << _img_fb.x, _img_fb.y;
	// _image_fb_aug << _img_fb.x, _img_fb.y, 1;
	_camera_on = true;
}

// Callback detection feedback
void CONTROLLER::found_cb( const std_msgs::Bool found_msg ) {

    _found_blob = found_msg;
	_flag_blob = _found_blob.data;
}

// Callback ee tag corners pos
void CONTROLLER::vs_cb(const std_msgs::Float64MultiArray corner_msg){
    _ee_tag_corners.resize(corner_msg.data[0]);
    for(int i = 0;i<_ee_tag_corners.size();i++){
        _ee_tag_corners(i) = corner_msg.data[i+1];
    }
}

void CONTROLLER::vs_flag_cb(const std_msgs::Bool flag_msg){
	_start_vs = flag_msg.data;
}

// Feedback computation
void CONTROLLER::coupled_feedback(){
	std_srvs::Empty pauseSrv;
	geometry_msgs::Wrench ext_f;
	Eigen::VectorXd f_cmd;
	f_cmd.resize(6);

	_T_uav.block<3,3>(0,0) = _R_uav;
	_T_uav.block<3,1>(0,3) = _p_uav;
	_T_arm = dir_kin(_arm_state_pos,_S_axis,_M);
	_J_arm = diff_kin(_arm_state_pos);
	_pose_mes_uav << _p_uav, _Eta;
	_pose_mes_coupled << _pose_mes_uav, _arm_state_pos;

	// _pauseGazebo.call(pauseSrv);

	// Force sensor feedback
	double alpha = 0.25; //Filter coefficient, adjust as needed
	if(_iii>2000){
		alpha = 0.05;
	}
	if (!_ft_sens.states.empty()){
	_f_ext << _ft_sens.states[0].total_wrench.force.x,_ft_sens.states[0].total_wrench.force.y,_ft_sens.states[0].total_wrench.force.z;
	_m_ext << _ft_sens.states[0].total_wrench.torque.x,_ft_sens.states[0].total_wrench.torque.y,_ft_sens.states[0].total_wrench.torque.z;
	_f_ext_filtered = alpha * _f_ext + (1.0 - alpha) * _f_ext_old;
	// _F_ee << _m_ext, _f_ext;
	_F_ee <<0,0,0,_f_ext_filtered;
	// _F_ee = utilities::Ad_f(_T_arm).transpose()*_F_ee;
	f_cmd = utilities::Ad_f((utilities::rotx_T(-M_PI)*_T_arm).inverse()).transpose()*_F_ee;
	}
	else {
		// _f_ext << 0,0,0;
		// _F_ee.setZero();
		f_cmd.setZero();
	}
	ext_f.force.x = f_cmd(3);
	ext_f.force.y = f_cmd(4);
	ext_f.force.z = f_cmd(5);
	ext_f.torque.x = f_cmd(0);
	ext_f.torque.y = f_cmd(1);
	ext_f.torque.z = f_cmd(2);
	_extf_pub.publish(ext_f);
	writedata(_f_ext_filtered,_f_extFile);
	_f_ext_old = _f_ext_filtered;
	// Coupled position feedback
	_NED_T_arm = utilities::rotx_T(-M_PI)*_T_arm; 					// arm dir-kin in ned frame
	_NED_J_arm = utilities::Ad_f(utilities::rotx_T(-M_PI))*_J_arm; 	// arm diff-kin in ned frame

	_T_coupled = _T_uav*_NED_T_arm;
	_ee_pos_to_world << _T_coupled.block<3,1>(0,3), utilities::R2XYZ(_T_coupled.block<3,3>(0,0));

	_arm_vel_mes = _J_arm*_arm_state_vel;
	_J_coupled.block<6,6>(6,6) = _NED_J_arm;
	_vel_mes << _vel_mes_uav, _arm_state_vel;
	_vel_mes_coupled = _J_coupled*_vel_mes;

	// Camera feedback 
	// if(_camera_on && _flag_blob){
	// 	// _ee_camera = (_T_cb.inverse()).block<3,3>(0,0).inverse()*(_K_camera.inverse()*_image_fb_aug-(_T_cb.inverse()).block<3,1>(0,3));
	// 	// _ee_camera = (((_K_camera*_P_camera*_T_cb).completeOrthogonalDecomposition()).pseudoInverse())*_image_fb_aug; //4x1 = 4x3*3x1
	// 	_ee_camera = _K_camera.inverse()*_image_fb_aug; // from pixel coordinates to normalized coordinates ( y down and x on the right (origin at the center of camera view))
	// 	// cout<<"blob pose in normalized coordinates: "<<_ee_camera.transpose()<<endl;
	// 	cout<<"blob pose in pixels coordinates: "<<_image_fb_aug.transpose()<<endl;
	// }
	if(_camera_on && _found_tag ){
		camera_parm_computation();

	}
}

// Camera parameters computation
void CONTROLLER::camera_parm_computation(){
    Eigen::VectorXd vel,vel_temp;
	Eigen::Vector4d corner_1,corner_2,corner_3,corner_4;
	Eigen::Vector4d T_arm_c1,T_arm_c2,T_arm_c3,T_arm_c4;
	Eigen::Vector4d pose_EE_aug;
	Eigen::Matrix4d pose_EE;
	double dim_tag, dim_ee;
    vel.resize(6);
    vel_temp.resize(6);

    _pose_tag_aug << _tag_pose_fb.position.x, _tag_pose_fb.position.y, _tag_pose_fb.position.z, 1; //object pose in camera frame (x,y,z,1)
    _image_fb_aug = _K_camera*_P_camera*_pose_tag_aug;
    _image_fb_aug = _image_fb_aug/_image_fb_aug(2);											       //object pose in pixels coordinates (X_i,Y_i,1)
    _image_fb_aug_norm = _K_camera.inverse()*_image_fb_aug;										   //object pose in normalized pixels coordinates (X,Y,1)

	// E-E pose in image plane
	// 4 Corners linear pose
	// dim_ee = 0.01;
	// dim_ee = 0.05;
	// dim_ee = 0.16;
	dim_ee = 0.02;

	// 4 points augmented linear position w.r.t. E-E frame
	corner_1 << -dim_ee/2, -dim_ee/2, 0.0, 1.0;
	corner_2 <<  dim_ee/2, -dim_ee/2, 0.0, 1.0;
	corner_3 <<  dim_ee/2,  dim_ee/2, 0.0, 1.0;
	corner_4 << -dim_ee/2,  dim_ee/2, 0.0, 1.0;
	T_arm_c1 = _T_arm*corner_1;
	T_arm_c2 = _T_arm*corner_2;
	T_arm_c3 = _T_arm*corner_3;
	T_arm_c4 = _T_arm*corner_4;

	pose_EE = _T_cb.inverse()*(_T_cb_temp.inverse()*_T_arm); 										//E-E pose in camera frame
	pose_EE_aug << pose_EE(0,3), pose_EE(1,3), pose_EE(2,3), 1.0;

	if(_iii<750){
	_pose_EE_corner_aug << pose_EE_aug(0)-dim_ee/2, pose_EE_aug(0)+dim_ee/2, pose_EE_aug(0)+dim_ee/2, pose_EE_aug(0)-dim_ee/2, 
						   pose_EE_aug(1)+dim_ee/2, pose_EE_aug(1)+dim_ee/2, pose_EE_aug(1)-dim_ee/2, pose_EE_aug(1)-dim_ee/2,
						//    pose_EE_aug(1)-0.025+dim_ee/2, pose_EE_aug(1)-0.025+dim_ee/2, pose_EE_aug(1)-0.025-dim_ee/2, pose_EE_aug(1)-0.025-dim_ee/2,
							        pose_EE_aug(2),          pose_EE_aug(2),          pose_EE_aug(2),          pose_EE_aug(2),
							  		             1,                       1,                       1,                       1;
	}
	else{
	// _pose_EE_corner_aug.block<4,1>(0,0) << pose_EE*corner_1;
	// _pose_EE_corner_aug.block<4,1>(0,1) << pose_EE*corner_2;
	// _pose_EE_corner_aug.block<4,1>(0,2) << pose_EE*corner_3;
	// _pose_EE_corner_aug.block<4,1>(0,3) << pose_EE*corner_4;
	_pose_EE_corner_aug.block<4,1>(0,0) << _T_cb.inverse()*_T_cb_temp.inverse()*T_arm_c1;
	_pose_EE_corner_aug.block<4,1>(0,1) << _T_cb.inverse()*_T_cb_temp.inverse()*T_arm_c2;
	_pose_EE_corner_aug.block<4,1>(0,2) << _T_cb.inverse()*_T_cb_temp.inverse()*T_arm_c3;
	_pose_EE_corner_aug.block<4,1>(0,3) << _T_cb.inverse()*_T_cb_temp.inverse()*T_arm_c4;
	// _pose_EE_corner_aug(2,0) = pose_EE(2,3);
	// _pose_EE_corner_aug(2,1) = pose_EE(2,3);
	// _pose_EE_corner_aug(2,2) = pose_EE(2,3);
	// _pose_EE_corner_aug(2,3) = pose_EE(2,3);
	}

	// Corners pixel coordinates (x,y)
	_EE_fb_aug_corner.block<3,1>(0,0) = _K_camera*_P_camera*_pose_EE_corner_aug.block<4,1>(0,0);
	_EE_fb_aug_corner.block<3,1>(0,0) = _EE_fb_aug_corner.block<3,1>(0,0)/_EE_fb_aug_corner(2,0);

	_EE_fb_aug_corner.block<3,1>(0,1) = _K_camera*_P_camera*_pose_EE_corner_aug.block<4,1>(0,1);
	_EE_fb_aug_corner.block<3,1>(0,1) = _EE_fb_aug_corner.block<3,1>(0,1)/_EE_fb_aug_corner(2,1);

	_EE_fb_aug_corner.block<3,1>(0,2) = _K_camera*_P_camera*_pose_EE_corner_aug.block<4,1>(0,2);
	_EE_fb_aug_corner.block<3,1>(0,2) = _EE_fb_aug_corner.block<3,1>(0,2)/_EE_fb_aug_corner(2,2);

	_EE_fb_aug_corner.block<3,1>(0,3) = _K_camera*_P_camera*_pose_EE_corner_aug.block<4,1>(0,3);
	_EE_fb_aug_corner.block<3,1>(0,3) = _EE_fb_aug_corner.block<3,1>(0,3)/_EE_fb_aug_corner(2,3);

	// VISPs Corners pixel coordinates (x,y)
	// _EE_fb_aug_corner.block<1,4>(2,0) << 1,1,1,1;
    // _EE_fb_aug_corner.block<2,1>(0,0) << _ee_tag_corners(1), _ee_tag_corners(0);
    // _EE_fb_aug_corner.block<2,1>(0,1) << _ee_tag_corners(3), _ee_tag_corners(2);
    // _EE_fb_aug_corner.block<2,1>(0,2) << _ee_tag_corners(5), _ee_tag_corners(4);
    // _EE_fb_aug_corner.block<2,1>(0,3) << _ee_tag_corners(7), _ee_tag_corners(6);
		
	if(_start_ibvs){
		std_msgs::Float64MultiArray corners_px;
		corners_px.data.resize(9);
		corners_px.data[0] = 8;
		corners_px.data[1] = _EE_fb_aug_corner(0,0);
		corners_px.data[2] = _EE_fb_aug_corner(1,0);
		corners_px.data[3] = _EE_fb_aug_corner(0,1);
		corners_px.data[4] = _EE_fb_aug_corner(1,1);
		corners_px.data[5] = _EE_fb_aug_corner(0,2);
		corners_px.data[6] = _EE_fb_aug_corner(1,2);
		corners_px.data[7] = _EE_fb_aug_corner(0,3);
		corners_px.data[8] = _EE_fb_aug_corner(1,3);
		_vs_pub.publish(corners_px);
	}
	// Augmented normalized pixels corners coordinates
	_EE_fb_aug_norm_corner.block<3,1>(0,0) = _K_camera.inverse()*_EE_fb_aug_corner.block<3,1>(0,0);
	_EE_fb_aug_norm_corner.block<3,1>(0,1) = _K_camera.inverse()*_EE_fb_aug_corner.block<3,1>(0,1);
	_EE_fb_aug_norm_corner.block<3,1>(0,2) = _K_camera.inverse()*_EE_fb_aug_corner.block<3,1>(0,2);
	_EE_fb_aug_norm_corner.block<3,1>(0,3) = _K_camera.inverse()*_EE_fb_aug_corner.block<3,1>(0,3);

	// _pose_EE_corner_aug(2,0) = pose_EE(2,3);
	// _pose_EE_corner_aug(2,1) = pose_EE(2,3);
	// _pose_EE_corner_aug(2,2) = pose_EE(2,3);
	// _pose_EE_corner_aug(2,3) = pose_EE(2,3);

	_L_matrix_p1 << -1/_pose_EE_corner_aug(2,0), 							  0,  	_EE_fb_aug_norm_corner(0,0)/_pose_EE_corner_aug(2,0),		_EE_fb_aug_norm_corner(0,0)*_EE_fb_aug_norm_corner(1,0), 							-(1+pow(_EE_fb_aug_norm_corner(0,0),2)),		 _EE_fb_aug_norm_corner(1,0),
											  0,    -1/_pose_EE_corner_aug(2,0),	_EE_fb_aug_norm_corner(1,0)/_pose_EE_corner_aug(2,0),							(1+pow(_EE_fb_aug_norm_corner(1,0),2)), 		-_EE_fb_aug_norm_corner(0,0)*_EE_fb_aug_norm_corner(1,0),		-_EE_fb_aug_norm_corner(0,0);

	_L_matrix_p2 << -1/_pose_EE_corner_aug(2,1), 							  0,  	_EE_fb_aug_norm_corner(0,1)/_pose_EE_corner_aug(2,1),		_EE_fb_aug_norm_corner(0,1)*_EE_fb_aug_norm_corner(1,1), 							-(1+pow(_EE_fb_aug_norm_corner(0,1),2)),		 _EE_fb_aug_norm_corner(1,1),
											  0,    -1/_pose_EE_corner_aug(2,1),	_EE_fb_aug_norm_corner(1,1)/_pose_EE_corner_aug(2,1),							(1+pow(_EE_fb_aug_norm_corner(1,1),2)), 		-_EE_fb_aug_norm_corner(0,1)*_EE_fb_aug_norm_corner(1,1),		-_EE_fb_aug_norm_corner(0,1);

	_L_matrix_p3 << -1/_pose_EE_corner_aug(2,2), 							  0,  	_EE_fb_aug_norm_corner(0,2)/_pose_EE_corner_aug(2,2),		_EE_fb_aug_norm_corner(0,2)*_EE_fb_aug_norm_corner(1,2), 							-(1+pow(_EE_fb_aug_norm_corner(0,2),2)),		 _EE_fb_aug_norm_corner(1,2),
											  0,    -1/_pose_EE_corner_aug(2,2),	_EE_fb_aug_norm_corner(1,2)/_pose_EE_corner_aug(2,2),							(1+pow(_EE_fb_aug_norm_corner(1,2),2)), 		-_EE_fb_aug_norm_corner(0,2)*_EE_fb_aug_norm_corner(1,2),		-_EE_fb_aug_norm_corner(0,2);

	_L_matrix_p4 << -1/_pose_EE_corner_aug(2,3), 							  0,  	_EE_fb_aug_norm_corner(0,3)/_pose_EE_corner_aug(2,3),		_EE_fb_aug_norm_corner(0,3)*_EE_fb_aug_norm_corner(1,3), 							-(1+pow(_EE_fb_aug_norm_corner(0,3),2)),		 _EE_fb_aug_norm_corner(1,3),
											  0,    -1/_pose_EE_corner_aug(2,3),	_EE_fb_aug_norm_corner(1,3)/_pose_EE_corner_aug(2,3),							(1+pow(_EE_fb_aug_norm_corner(1,3),2)), 		-_EE_fb_aug_norm_corner(0,3)*_EE_fb_aug_norm_corner(1,3),		-_EE_fb_aug_norm_corner(0,3);

	_L_matrix.block<2,6>(0,0) = _L_matrix_p1;
	_L_matrix.block<2,6>(2,0) = _L_matrix_p2;
	_L_matrix.block<2,6>(4,0) = _L_matrix_p3;
	_L_matrix.block<2,6>(6,0) = _L_matrix_p4;
	_gamma_matrix.block<3,3>(0,3) = utilities::skew(Eigen::Vector3d(pose_EE_aug(0),pose_EE_aug(1),pose_EE_aug(2)));

    _s_EE << _EE_fb_aug_norm_corner(0,0), _EE_fb_aug_norm_corner(1,0), _EE_fb_aug_norm_corner(0,1), _EE_fb_aug_norm_corner(1,1), _EE_fb_aug_norm_corner(0,2), _EE_fb_aug_norm_corner(1,2), _EE_fb_aug_norm_corner(0,3), _EE_fb_aug_norm_corner(1,3); 
	vel_temp = utilities::Ad_f(_T_cb.inverse()*_T_cb_temp.inverse())*(_J_arm*_arm_state_vel);
    vel << vel_temp(3), vel_temp(4), vel_temp(5), vel_temp(0), vel_temp(1), vel_temp(2);
	_s_dot_EE = (_L_matrix*_gamma_matrix.inverse())*vel;
	// cout<<"s E_E: "<<_s_EE.transpose()<<endl;
}

// Request new plan
void CONTROLLER::new_plan() {

	std_srvs::Empty pauseSrv;
    std_srvs::Empty unpauseSrv;
	// ros::Rate r(_rate);

	// while (ros::ok()) {

		string input;
		if(_goal_reached){
		cout<<"Want new trajectory?"<<endl;
		getline( cin, input);
		_goal_reached = false;
		_start_controller = false;
		}

		if( input == "y" && !_goal_reached) {
			_traj_ready = false;
		}
		else {
			_traj_ready = true;
		}

		if(!_traj_ready){
			_pauseGazebo.call(pauseSrv);
			int N = 1000;
			float Ti = 0;
			float Tf = 10;
			Eigen::MatrixXd Traj;
			Traj.resize(N*8,N*8);
			Eigen::Matrix4d T_s, T_g;
			Eigen::VectorXd theta_goal, theta_s;
			theta_s.resize(6);
			// theta_s << 0.0, -0.3499631000000001, 1.1510456000000002, 0.09989970000000037, -1.0027668000000003, 0.0;
			theta_s << 0.0, -0.6641131000000002, 1.2584849000000005, -0.0018849000000003002, -0.6169906000000003, 0.0;
			// T_s = _T_uav*utilities::rotx_T(-M_PI)*dir_kin(theta_s,_S_axis,_M);
			T_s = dir_kin(theta_s,_S_axis,_M);
			theta_goal.resize(6);
			// theta_goal << 0.0, -0.6641131000000002, 1.2584849000000005, -0.0018849000000003002, -0.6169906000000003, 0.0;
			theta_goal << 0.0, -1.1095777999999998, 1.4482315, 0.0, -0.28210669999999993, 0.0;

			// T_g = _T_uav*utilities::rotx_T(-M_PI)*dir_kin(theta_goal, _S_axis,_M);
			T_g = dir_kin(theta_goal, _S_axis,_M);

			// // set point to be in contact with the ground
			// T_g.setIdentity();
			// T_g.block<3,3>(0,0) = utilities::RpyToMat(Eigen::Vector3d(0, 0, -M_PI/2));
			// T_g.block<3,1>(0,3) << 0.344211, 0.0, -0.188278;

			// cout<<"T_start = \n"<<T_s<<endl;
		    // cout<<"rpy start = "<<utilities::MatToRpy((T_s.block<3,3>(0,0))).transpose()<<endl;

			// cout<<"T_goal = \n"<<T_g<<endl;
		    // cout<<"rpy goal = "<<utilities::MatToRpy((T_g.block<3,3>(0,0))).transpose()<<endl;
			cout<<"+++ Trajectory Planner starting now... +++"<<endl;
			// // ---------------------------------------------------------------
			// _T_traj.resize(8*N,8*N);
			// _Td_traj.resize(8*N,8*N);

			// Traj = traj_generation(T_s,T_g,Ti,Tf,N);
			// for (int i=0;i<4*N;i++){
			// 	for(int j=0;j<4*N;j++){
			// 		_T_traj(i,j) = Traj(i,j);
			// 		_Td_traj(i,j) = Traj(4*N+i,j+4*N);
			// 	}
			// }
			
			// Traj = traj_generation(T_g,T_s,Ti,Tf,N);
			// for (int i=0;i<4*N;i++){
			// 	for(int j=0;j<4*N;j++){
			// 		_T_traj(i+4*N,j+4*N) = Traj(i,j);
			// 		_Td_traj(i+4*N,j+4*N) = Traj(4*N+i,j+4*N);
			// 	}
			// }
			// //------------------------------------------------------------------
			// cout<<"p = [];"<<endl;
			// cout<<"pd = [];"<<endl;
			// cout<<"rpy = [];"<<endl;
			// Eigen::Vector3d rpy, rpy_d;
			// for(int i=0;i<2*N;i++){
			// 	rpy = utilities::MatToRpy(_T_traj.block<3,3>(4*i,4*i));
			// 	cout<<"p(:,"<<i+1<<") = ["<<_T_traj.block<1,1>(4*i+0,4*i+3).transpose()<<", "<<_T_traj.block<1,1>(4*i+1+0,4*i+3).transpose()<<", "<<_T_traj.block<1,1>(4*i+2+0,4*i+3).transpose()<<"];"<<endl;
			// 	cout<<"pd(:,"<<i+1<<") = ["<<_Td_traj.block<1,1>(4*i+0,4*i+3).transpose()<<", "<<_Td_traj.block<1,1>(4*i+1+0,4*i+3).transpose()<<", "<<_Td_traj.block<1,1>(4*i+2+0,4*i+3).transpose()<<"];"<<endl;
			// 	cout<<"rpy(:,"<<i+1<<") = ["<<rpy(0)<<", "<<rpy(1)<<", "<<rpy(2)<<"]; "<<endl;
			// }
			_traj_ready = true;
			_goal_reached = false;
			_start_controller = true;
			cout<<"Traj planned correctly!"<<endl;
			cout<<"+++ Controller is starting now... +++"<<endl;
	        // _unpauseGazebo.call(unpauseSrv);
		}

	// 	r.sleep();
	// }
}

// Parallel Vision/Force Controller (ALSO WITH CARTESIAN CONTROL BUT NOT USED FOR NOW. HERE ONLY FOR DEBUG)
void CONTROLLER::arm_invdyn_control() {

	std_srvs::Empty pauseSrv;
	std_srvs::Empty unpauseSrv;

	ros::Rate r(_rate);
	Eigen::Matrix4d T_eff, X_error_, Td_T, T_traj_act, Td_traj_act;
	Eigen::Matrix3d R_eff;
	Eigen::Vector3d p_eff;
	Eigen::MatrixXd J_be,B_in_mat_inverse,J_wpi,J_be_prev,J_be_d, Identity, sd_mat;
	Eigen::MatrixXd J_img, J_img_temp, J_img_pinv;
	Eigen::VectorXd V_eff, X_error, V_error, X_err_temp, V_d, V_des, V_des_prev, A_des, u_v, u_q, f_ee, u_mot;
	Eigen::VectorXd sd, e_s, edot_s, e_i, error, error_temp, vs;
	Eigen::Vector4d e_sx, e_sy;
	Eigen::VectorXd F_err, F_des, F_int;
	Eigen::MatrixXd P, P2, Kp_f, Ki_f, J_be_body, Lambda_inv;
	bool ibvs_flag = false;
	int i = 0;
	int ii = 0;
	int iii = 0;
	int N_ = 2000;
	float Ts = 0.01;
	double lambda_k = 0;
	double lambda_d = 0;
	double lambda_e = 0;
	double c_lambda = 0;

	Eigen::VectorXd vel_old, acc_old;
	vel_old.resize(6);
	acc_old.resize(6);
	
	V_eff.resize(6);
	X_err_temp.resize(6);
	X_err_temp<<0,0,0,0,0,0;
	V_error.resize(6);
	V_d.resize(6);
	V_des.resize(6);
	V_des_prev.resize(6);
	A_des.resize(6);
	u_v.resize(6);
	u_q.resize(6);
	B_in_mat_inverse.resize(_N_JOINTS,_N_JOINTS);
	J_be.resize(6,_N_JOINTS);
	J_be_d.resize(6,_N_JOINTS);
	J_be_prev.resize(6,_N_JOINTS);
	J_wpi.resize(_N_JOINTS,6);
	f_ee.resize(6);
	f_ee << 0,0,0,0,0,0;
	u_mot.resize(6);
	u_mot.setZero();

	error.resize(6,1);
	error.setZero();
    error_temp.resize(6,1);
	error_temp.setZero();
    vs.resize(8,1);
    sd.resize(8,1);
	sd.setZero();    
	e_s.resize(8,1);
	e_s.setZero();
	e_sx.setZero();
	e_sy.setZero();
	edot_s.resize(8,1);
	edot_s.setZero();
	e_i.resize(8,1);
	e_i.setZero();

	F_des.resize(6);
	F_des.setZero();
	F_err.resize(6);
	F_err.setZero();
	F_int.resize(6);
	F_int.setZero();

	P.resize(6,6);
	P2.resize(6,6);

	Kp_f.resize(6,6);
	Ki_f.resize(6,6);
	Kp_f.setIdentity();
	Ki_f.setIdentity();
	Kp_f(3,3) = _Kp_pos_f_yaml[0];
	Kp_f(4,4) = _Kp_pos_f_yaml[1];
	Kp_f(5,5) = _Kp_pos_f_yaml[2];
	Ki_f(3,3) = _Ki_pos_f_yaml[0];
	Ki_f(4,4) = _Ki_pos_f_yaml[1];
	Ki_f(5,5) = _Ki_pos_f_yaml[2];
	sd_mat.resize(3,4);

	J_be_body.resize(6,6);
	Lambda_inv.resize(6,6);

	Identity.resize(6,6);
	Identity.setIdentity();
	geometry_msgs::Wrench ext_f;
	ext_f.force.x = 0.0;
	ext_f.force.y = 0.0;
	ext_f.force.z = 0.0;

    ext_f.torque.x = 0.0;
    ext_f.torque.y = 0.0;
    ext_f.torque.z = 0.0;

	_arm_state_pos << 0.0, -0.3499631000000001, 1.1510456000000002, 0.09989970000000037, -1.0027668000000003, 0.0;
	_arm_state_vel.setZero();
	_T_arm = dir_kin(_arm_state_pos,_S_axis,_M);
	_J_arm = diff_kin(_arm_state_pos);
	ibvs_flag = true;

	// Desired feature vector
	// sd << -0.154781,   0.187575,  0.0281414,   0.187575,  0.0281414, 0.00465223,  -0.154781, 0.00465223; // tag 0.05
	// sd << -0.092651,   0.414493, -0.0556423,   0.414493, -0.0556423,   0.377484,  -0.092651,   0.377484; // tag 0.01
    // sd << -0.299125,  0.300783,  0.307344,  0.300783,  0.307344, -0.305687, -0.299125, -0.305687;    	// tag centrato 0.16
	// sd << -0.300623,   0.605069,    0.30677,   0.605069,    0.30677, -0.0023237,  -0.300623, -0.0023237; // tag basso 0.16
	// sd << -0.1211,  0.443594, -0.048403,  0.443594, -0.048403,  0.370897,   -0.1211,  0.370897; // tag 0.02
	sd << -0.0984,    0.4000,   -0.0328,    0.4000,   -0.0328,    0.3344,   -0.0984,    0.3344;
	
	// Completo 
	// sd << -0.12233,   0.452767,  -0.049245,    0.45431, -0.0456234,  0.382438,  -0.118839,   0.380565; // tag 0.02 no co-planari
	// sd << -0.0922813,   0.387486, -0.0197691,   0.393881, -0.0115061,   0.320912, -0.0838013,   0.314156; // tag 0.02 new
	// sd << -0.110462,    0.42888, -0.0363176,   0.428872, -0.0363522,   0.354765,  -0.110497,   0.354773; // tag 0.02 co-planari //sbagliato
	
	sd_mat << sd(0), sd(2), sd(4), sd(6), sd(1), sd(3), sd(5), sd(7), 1,1,1,1; 
	cout<<"sd matrix: "<< _K_camera*sd_mat<<endl;
		
	// Computing desired trajectory
	new_plan();
	
	while ( ros::ok() && i < N_ + 2000 ) {
		if(_start_controller){
			_pos_cmd = _arm_state_pos;
			// while( !_first_odom_1 && !_first_odom_2) usleep(0.1*1e6);
			if(ii>1 || i>1 || _iii>1){
				coupled_feedback();
			}
			if(ii<2*N_ && !_start_vs){
				if(ii==0){
					cout<<"Starting Phase 1...\nApproaching Apriltag..."<<endl;
				}
				// ------------- Regulation ------------------
				Eigen::VectorXd theta_goal, ep, ev;
				Eigen::MatrixXd kp,kd;
				theta_goal.resize(6);
				ep.resize(6);
				ev.resize(6);
				kp.resize(6,6);
				kd.resize(6,6);
				kp.setIdentity();
				kd.setIdentity();
				kp = 300*kp;
				kp(5,5) = 10000;
				kp(4,4) = 4000;
				kp(3,3) = 4000;
				// kp(3,3) = 1500;
				kd = 75*kd;

				// // // theta_goal << 0.3047255, -0.4574023999999999, 1.1893718999999998, -1.2811037000000003, 0.0, 0.0;
				// // theta_goal << 0.05, -0.3, 1.1, 0.5, 0, 0;
				// theta_goal << 0.0, -0.35, 1.15, 0.1, -1, 0;
				// theta_goal << 0, -0.66, 1.26, 0, -0.62, 0; //goal position
				// theta_goal << 0.0, -0.3499631000000001, 1.1510456000000002, 0.09989970000000037, -1.0027668000000003, 0.0;
				// theta_goal << 0.0, -0.6641131000000002, 1.2584849000000005, -0.0018849000000003002, -0.6169906000000003, 0.0; //<--
				theta_goal << 0.0, -0.81, 1.80, -0.0018849000000003002, -0.6169906000000003, 0.0;
				// theta_goal << 0.1, -0.32, 1.2584849000000005, -0.7, -0.9, 1.0;

				ep = theta_goal - _arm_state_pos;
				ev = - _arm_state_vel;
				u_q = kp*ep+kd*ev;

				f_ee <<0,0,0,0,0,0;
				_arm_cmd = CONTROLLER::recursive_inv_dyn(_arm_state_pos, _arm_state_vel, u_q, _gravity, f_ee);
				writedata(_arm_cmd,_tauFile);
				
				// save actual values for trapeze integration
				acc_old = u_q;
				vel_old = _vel_cmd;

				_acc_cmd = CONTROLLER::inertia_matrix(_arm_state_pos).inverse()*(_arm_cmd - CONTROLLER::c_forces(_arm_state_pos,_arm_state_vel) - CONTROLLER::gravity_forces(_arm_state_pos));
				
				// Integrations methods for velocity
				// _vel_cmd = utilities::euler_integration(u_q,_vel_cmd,0.01); 				    // EULER
        		_vel_cmd = utilities::trapeze_integration(u_q, acc_old, _vel_cmd, 0.01);		// TRAPEZE
				// _vel_cmd = utilities::simpsonIntegration(u_q, acc_old, _vel_cmd, 0.01);			// SIMPSON
				for(int i=0;i<6;i++){
					_vel_cmd(i) = wrapToRange(_vel_cmd(i), 0.0, 1.57);
				}	

				// Integrations methods for position
				// _pos_cmd = utilities::euler_integration(_vel_cmd,_pos_cmd,0.01); 				// EULER
        		_pos_cmd = utilities::trapeze_integration(_vel_cmd, vel_old, _pos_cmd, 0.01);	// TRAPEZE
				// _pos_cmd = utilities::simpsonIntegration(_vel_cmd, vel_old, _pos_cmd, 0.01);		// SIMPSON

				for(int i=0;i<6;i++){
					if (_pos_cmd(i)>0)
						_pos_cmd(i) = fmod(_pos_cmd(i)+M_PI, 2.0*M_PI)-M_PI;
					else
						_pos_cmd(i) = fmod(_pos_cmd(i)-M_PI, 2.0*M_PI)+M_PI;
				}	
				//
				ii++;
				_unpauseGazebo.call(unpauseSrv);
			}			
			else if(_camera_on && ibvs_flag && _iii<N_+4000 && (ii>2*N_-1 || _start_vs)){
				if(_iii==0){
					cout<<"Starting Phase 2...\nE-E IBVS..."<<endl;
				}
			// --------------------- IBVS tracking task -----------------------------------
				_start_ibvs = true;
				// hybrid controller switch
				P.setIdentity();
				P2.setIdentity();
				// switch discontinuo
				// if(_F_ee(5)!=0){
				// 	P(5,5)=0;
				// 	P2(3,3)=0;
				// }

				// switch continuo
				c_lambda = 0.05; //0.025
				if(_iii<N_){
					if((_iii>1000 &&  abs(e_s(0)))>0.005 && abs(e_s(0))<=0.1){
						if(_iii==1001){
							cout<<"Starting interaction control..."<<endl;
						}
						lambda_d = 0.5*(1+cos((e_s(0)-0.005)/(0.1-0.005)*M_PI));
					}
					else if(_iii>1000 &&  abs(e_s(0))<=0.005){
						lambda_d = 1;
					}
					else{
						lambda_d = 0;
						_F_ee.setZero();
						_f_ext.setZero();
					}

					// if(_F_ee(5)-F_des(5)>0.2 && _F_ee(5)-F_des(5)<=0.25){
					// 	lambda_e = 0.5*(1+cos((_F_ee(5)-F_des(5)-0.2)/(0.25-0.2)*M_PI));
					// }
					// else if(_F_ee(5)-F_des(5)<=0.2){
					// 	lambda_e = 1;
					// }
					// else{
					// 	lambda_e = 0;
					// }
					lambda_e = 1;

					if(_iii>1000 && F_des.norm()>0){
						lambda_k = c_lambda*lambda_d*lambda_e+(1-c_lambda)*lambda_k;
					}
					else{
						lambda_k = 0;
					}
				}	
				else{
						// lambda_k = 1;
						lambda_k = c_lambda*lambda_d*lambda_e+(1-c_lambda)*lambda_k;
				}
				P(5,5) = 1 - lambda_k;
				P2(3,3) = 1 - lambda_k;
				

				// Feedback

				T_eff = _T_arm;
				R_eff = T_eff.block<3,3>(0,0);
				p_eff = T_eff.block<3,1>(0,3);
				J_be = _J_arm;

				// image space trajectory
				// if(_iii>N_){
				// 	if(_iii==N_+1){
				// 		cout<<"Starting Phase 3\nSliding on surface..."<<endl;
				// 	}
				// 	if(_iii<(N_+N_/4)){
				// 		for(int j=0;j<8;j++){
				// 			if(j%2==0){
				// 				sd(j,1) = sd(j,1)+0.00016*(_iii/100-(N_+N_/4)/100); 
				// 			}
				// 		}
				// 		// cout<<"step 1"<<endl;
				// 	}
				// 	else if (_iii>(N_+N_/4) && _iii<(N_+N_*3/4)) {
				// 		for(int j=0;j<8;j++){
				// 			if(j%2==0){
				// 				sd(j,1) = sd(j,1)+0.00024;
				// 			}
				// 		}
				// 		// cout<<"step 2"<<endl;
				// 	}
				// 	else if (_iii>(N_+N_*3/4) && _iii<2*N_) {
				// 		for(int j=0;j<8;j++){
				// 			if(j%2==0){	
				// 				sd(j,1) = sd(j,1)+0.00016*((2*N_)/100-_iii/100); 
				// 			}
				// 		}
				// 		// cout<<"step 3"<<endl;
				// 	}
				// 	else {
				// 		for(int j=0;j<8;j++){
				// 			if(j%2==0){	
				// 				sd(j,1) = sd(j,1); 
				// 			}
				// 		}
				// 		// cout<<"step 4"<<endl;	
				// 	}

				// }
				// ---------------------SLIDING--------------------- 
				if(_iii>2*N_){
					if(_iii==2*N_+1){
						cout<<"Starting Phase 3\nSliding on surface..."<<endl;
					}
					if(_iii<(2*N_+N_/2)){

						sd(0,0) = sd(0,0)-0.0001*(_iii/100-2*N_/100); 
						sd(2,0) = sd(2,0)-0.0001*(_iii/100-2*N_/100); 
						sd(4,0) = sd(4,0)-0.0001*(_iii/100-2*N_/100); 
						sd(6,0) = sd(6,0)-0.0001*(_iii/100-2*N_/100); 
						
						if(_iii==2*N_+1){
							cout<<"step 1"<<endl;
						}
					}			
				}
				// if(_iii>N_){
				// 	if(_iii==N_+1){
				// 		cout<<"Starting Phase 3\nSliding on surface..."<<endl;
				// 	}
				// 	if(_iii<(N_+N_/3)){
				// 		for(int j=0;j<8;j++){
				// 			if(j%2==0){
				// 				sd(j,0) = sd(j,0)-0.00016*(_iii/100-N_/100); 
				// 			}
				// 		}
				// 		if(_iii==N_+1){
				// 			cout<<"step 1"<<endl;
				// 		}
				// 	}					
				// 	else if (_iii>(N_+N_/3) && _iii<(N_+N_/2)) {
				// 		for(int j=0;j<8;j++){
				// 			if(j%2!=0){
				// 				sd(j,0) = sd(j,0)+0.00016*(_iii/100-(N_+N_/3)/100);
				// 			}
				// 		}
				// 		if(_iii==N_+N_/3+1){
				// 			cout<<"step 2"<<endl;
				// 		}
				// 	}
				// 	else if (_iii>(N_+N_/2) && _iii<(N_+N_/2+N_/3)) {
				// 		for(int j=0;j<8;j++){
				// 			if(j%2==0){	
				// 				sd(j,0) = sd(j,0)+0.00016*(_iii/100-(N_+N_/2)/100); 
				// 			}
				// 		}
				// 		if(_iii==N_+N_/2+1){
				// 			cout<<"step 3"<<endl;
				// 		}
				// 	}
				// 	else {
				// 		for(int j=0;j<8;j++){
				// 			if(j%2==0){	
				// 				sd(j,0) = sd(j,0); 
				// 			}
				// 		}
				// 		if(_iii==N_+N_/2+N_/3+1){
				// 			cout<<"step 4"<<endl;
				// 		}
				// 	}
				// }
				//------------------------------------------
				// cout <<"sd: "<< sd.transpose() <<endl;
				// Errors 
				e_s = (sd -_s_EE);
    			edot_s = (-_s_dot_EE);
				e_i = e_i + e_s*0.01;
				e_sx << e_s(0), e_s(2), e_s(4), e_s(6);
				e_sy << e_s(1), e_s(3), e_s(5), e_s(7);
				writedata(e_sx,_esxFile);
				writedata(e_sy,_esyFile);
				// cout<<"Errors: "<<e_s.transpose()<<endl;
				// cout<<"integral error: "<<e_i.transpose()<<endl;
				// Computation of the Dinamically consistent pseudoinverse space Jacobian
				B_in_mat_inverse = (inertia_matrix(_arm_state_pos)).inverse();
				// J_wpi = (B_in_mat_inverse*J_be.transpose())*(J_be*B_in_mat_inverse*J_be.transpose()).inverse();
				J_wpi = J_be.inverse();
				
				// Initialization for numerical differentiation
				if (i == 1){
					J_be_prev = J_be;
				}

				// Numerical differentiation of the space Jacobian
				J_be_d = (J_be - J_be_prev)/Ts;
				J_be_prev = J_be;

				// Virtual acceleration cartesian space
				vs = (_Kp_cam*e_s+_Kd_cam*edot_s+_Ki_cam*e_i);
				J_img = _L_matrix*_gamma_matrix.inverse();
				J_img_temp = J_img.completeOrthogonalDecomposition().pseudoInverse();
				J_img_pinv = J_img_temp;
				error_temp = J_img_pinv*vs;

				// error_temp = (_L_matrix*_gamma_matrix.inverse()).completeOrthogonalDecomposition().pseudoInverse()*vs;
				error << error_temp(3), error_temp(4), error_temp(5), error_temp(0), error_temp(1), error_temp(2);	
 				u_q = J_wpi*(P2*(utilities::Ad_f(_T_cb.inverse()*_T_cb_temp.inverse()).inverse()*error) - (J_be_d*_arm_state_vel));

				f_ee <<0,0,0,0,0,0;
				u_mot = CONTROLLER::recursive_inv_dyn(_arm_state_pos, _arm_state_vel, u_q, _gravity, f_ee);
				// _arm_cmd = u_mot;
				writedata(_arm_cmd,_tauFile);

				// save actual values for trapeze integration
				acc_old = u_q;
				vel_old = _vel_cmd;

				_acc_cmd = CONTROLLER::inertia_matrix(_arm_state_pos).inverse()*(_arm_cmd - CONTROLLER::c_forces(_arm_state_pos,_arm_state_vel) - CONTROLLER::gravity_forces(_arm_state_pos));
				
				// Integrations methods for velocity
				// _vel_cmd = utilities::euler_integration(u_q,_vel_cmd,0.01); 				// EULER
        		_vel_cmd = utilities::trapeze_integration(u_q, acc_old, _vel_cmd, 0.01);		// TRAPEZE
				for(int i=0;i<6;i++){
					_vel_cmd(i) = wrapToRange(_vel_cmd(i), 0.0, 1.57);
				}	

				// Integrations methods for position
				// _pos_cmd = utilities::euler_integration(_vel_cmd,_pos_cmd,0.01); 			// EULER
        		_pos_cmd = utilities::trapeze_integration(_vel_cmd, vel_old, _pos_cmd, 0.01);	// TRAPEZE
				for(int i=0;i<6;i++){
					if (_pos_cmd(i)>0)
						_pos_cmd(i) = fmod(_pos_cmd(i)+M_PI, 2.0*M_PI)-M_PI;
					else
						_pos_cmd(i) = fmod(_pos_cmd(i)-M_PI, 2.0*M_PI)+M_PI;
				}	
				//

				// ---------- Direct Force Tracking ------------
				F_des << 0,0,0,0,0,2;
				F_err = - F_des + _F_ee;
				F_int = F_int + F_err*Ts;

				J_be_body = utilities::Ad_f(T_eff.inverse())*J_be; //Body Jacobian
				Lambda_inv = J_be_body*(inertia_matrix(_arm_state_pos)).inverse()*J_be_body.transpose();

				_arm_cmd = u_mot + J_be_body.transpose()*(Lambda_inv.inverse())*(Identity-P)*(-F_des+Kp_f*F_err+Ki_f*F_int);
				//-----------------------------------------------
								
				// save actual values for trapeze integration
				acc_old = u_q;
				vel_old = _vel_cmd;

				_acc_cmd = CONTROLLER::inertia_matrix(_arm_state_pos).inverse()*(_arm_cmd - CONTROLLER::c_forces(_arm_state_pos,_arm_state_vel) - CONTROLLER::gravity_forces(_arm_state_pos));
				
				// Integrations methods for velocity
				// _vel_cmd = utilities::euler_integration(u_q,_vel_cmd,0.01); 				// EULER
        		_vel_cmd = utilities::trapeze_integration(u_q, acc_old, _vel_cmd, 0.01);		// TRAPEZE
				for(int i=0;i<6;i++){
					_vel_cmd(i) = wrapToRange(_vel_cmd(i), 0.0, 1.57);
				}	

				// Integrations methods for position
				// _pos_cmd = utilities::euler_integration(_vel_cmd,_pos_cmd,0.01); 			// EULER
        		_pos_cmd = utilities::trapeze_integration(_vel_cmd, vel_old, _pos_cmd, 0.01);	// TRAPEZE
				for(int i=0;i<6;i++){
					if (_pos_cmd(i)>0)
						_pos_cmd(i) = fmod(_pos_cmd(i)+M_PI, 2.0*M_PI)-M_PI;
					else
						_pos_cmd(i) = fmod(_pos_cmd(i)-M_PI, 2.0*M_PI)+M_PI;
				}	
				//
				// // ----------------- ADMITTANCE TO RECONSTRUCT POSITION --------------
				// Eigen::VectorXd ddz, dz, z;
				// Eigen::MatrixXd mz, kpz, kdz;
				// ddz.resize(6);
				// dz.resize(6);
				// z.resize(6);
				// mz.resize(6,6);
				// kpz.resize(6,6);
				// kdz.resize(6,6);
				// ddz.setZero();
				// dz.setZero();
				// z.setZero();
				// mz.setIdentity();
				// kdz.setIdentity();
				// kpz.setIdentity();

				// mz = 1*mz;
				// kdz = 10*kdz;
				// kpz = 10*kpz;

				// ddz = mz.inverse()*((J_be.transpose()).inverse()*_arm_cmd - kpz*z - kdz*dz);
				// dz = dz + ddz*0.01; 
				// z = z + dz*0.01;
				// std_msgs::Float64MultiArray pose;
				// pose.data.resize(6);
				// pose.data[0] = z(0);
				// pose.data[1] = z(1);
				// pose.data[2] = z(2);
				// pose.data[3] = z(3);
				// pose.data[4] = z(4);
				// pose.data[5] = z(5);
				// _pos_sp_pub.publish(pose); 
				// ------------------------------------------------------------------
				_iii++;
				_unpauseGazebo.call(unpauseSrv);
			}			
			else if(i<N_+2000 && ii>N_-1 && !ibvs_flag) {
			// --------------------- Trajectory tracking task -----------------------------------
				if(i<N_){
					// T_traj_act = _T_uav*_T_NED*_T_traj.block<4,4>(4*i,4*i);
					// Td_traj_act = _T_uav*_T_NED*_Td_traj.block<4,4>(4*i,4*i);
					T_traj_act = _T_traj.block<4,4>(4*i,4*i);
					Td_traj_act = _Td_traj.block<4,4>(4*i,4*i);
				}

				// hybrid controller switch
				P.setIdentity();
				// switch discontinuo
				// if(_F_ee(5)!=0){
				// 	P(5,5)=0;
				// }

				// switch continuo
				c_lambda = 0.05;
				if((i>1400 &&  abs(X_error(5)))>0.03 && abs(X_error(5))<=0.1){
					lambda_d = 0.5*(1+cos((X_error(5)-0.02)/(0.2-0.02)*M_PI));
				}
				else if(i>1400 &&  abs(X_error(5))<=0.03){
					lambda_d = 1;
				}
				else{
					lambda_d = 0;
					// _F_ee.setZero();
					// _f_ext.setZero();
				}

				// if(_F_ee(5)-F_des(5)>0.2 && _F_ee(5)-F_des(5)<=0.25){
				// 	lambda_e = 0.5*(1+cos((_F_ee(5)-F_des(5)-0.2)/(0.25-0.2)*M_PI));
				// }
				// else if(_F_ee(5)-F_des(5)<=0.2){
				// 	lambda_e = 1;
				// }
				// else{
				// 	lambda_e = 0;
				// }
				lambda_e = 1;

				if(F_des.norm()>0){
					lambda_k = c_lambda*lambda_d*lambda_e+(1-c_lambda)*lambda_k;
				}
				else{
					lambda_k = 0;
				}
				P(5,5) = 1 - lambda_k;
				//

				// Feedback

				// Direct kinematic to compute effective pose of the ee from the sensed data
				// T_eff = _T_uav*_T_NED*_T_arm;
				T_eff = _T_arm;
				R_eff = T_eff.block<3,3>(0,0);
				p_eff = T_eff.block<3,1>(0,3);

				// Differential kinematic to compute the actual velocity of the ee from sensed data
				J_be = _J_arm;
				// J_be = utilities::Ad_f(_T_uav)*(utilities::Ad_f(_T_NED)*_J_arm);

				V_eff = J_be*_arm_state_vel;

				X_error_ = (T_eff.inverse()*T_traj_act).log();                                     			//4x4
				X_err_temp << X_error_(2,1), X_error_(0,2), X_error_(1,0), X_error_.block<3,1>(0,3);
				X_error = utilities::Ad_f(T_eff) * X_err_temp;     								 			//6x1

				Td_T = (T_traj_act).inverse()*Td_traj_act;

				V_d << Td_T(2,1), Td_T(0,2), Td_T(1,0), Td_T.block<3,1>(0,3);
				V_des = utilities::Ad_f(T_eff) *  (utilities::Ad_f(T_eff.inverse() * T_traj_act) * V_d);
				V_error = V_des - V_eff;

				if (i == 1){
					V_des_prev = V_des;
				}

				A_des = (V_des - V_des_prev)/Ts;
				V_des_prev = V_des;

				// Computation of the inverse Inertia Matrix
				B_in_mat_inverse = (inertia_matrix(_arm_state_pos)).inverse();

				// Computation of the Dinamically consistent pseudoinverse space Jacobian
				J_wpi = (B_in_mat_inverse*J_be.transpose())*(J_be*B_in_mat_inverse*J_be.transpose()).inverse();

				// Initialization for numerical differentiation
				if (i == 1){
					J_be_prev = J_be;
				}

				// Numerical differentiation of the space Jacobian
				J_be_d = (J_be - J_be_prev)/Ts;
				J_be_prev = J_be;

				// Virtual acceleration cartesian space
				u_v = (A_des + _Kd*V_error + _Kp*X_error);

				writedata(Eigen::Vector3d(X_error(0),X_error(1),X_error(2)),_erpyFile);
				writedata(Eigen::Vector3d(X_error(3),X_error(4),X_error(5)),_epFile);
				writedata(Eigen::Vector3d(V_error(0),V_error(1),V_error(2)),_ewFile);
				writedata(Eigen::Vector3d(V_error(3),V_error(4),V_error(5)),_evFile);

				u_q = J_wpi*(u_v - J_be_d*_arm_state_vel);

				f_ee <<0,0,0,0,0,0;
				u_mot = CONTROLLER::recursive_inv_dyn(_arm_state_pos, _arm_state_vel, u_q, _gravity, f_ee);
				_arm_cmd = u_mot;
				// _arm_cmd = CONTROLLER::inertia_matrix(_arm_state_pos)*u_q + CONTROLLER::c_forces(_arm_state_pos,_arm_state_vel)+CONTROLLER::gravity_forces(_arm_state_pos);
				writedata(_arm_cmd,_tauFile);

				// ---------- Direct Force Tracking ------------

				F_des << 0,0,0,0,0,3;
				F_err = - F_des + _F_ee;
				F_int = F_int + F_err*Ts;

				J_be_body = utilities::Ad_f(T_eff.inverse())*J_be; //Body Jacobian
				Lambda_inv = J_be_body*(inertia_matrix(_arm_state_pos)).inverse()*J_be_body.transpose();

				// _arm_cmd = u_mot + J_be_body.transpose()*(Lambda_inv.inverse())*(Identity-P)*(-F_des+Kp_f*F_err+Ki_f*F_int);
				//-----------------------------------------------

				i++;
				_unpauseGazebo.call(unpauseSrv);
			}
			else{
				_pauseGazebo.call(pauseSrv);
			}

			// _unpauseGazebo.call(unpauseSrv);
			r.sleep();
			// _pauseGazebo.call(pauseSrv);
		}
		else{
			_pauseGazebo.call(pauseSrv);
			r.sleep();
		}
	}
	cout<<"final pose: \n"<< dir_kin(_arm_state_pos, _S_axis, _M) <<endl;
    cout<<"final rpy = "<<utilities::MatToRpy(((dir_kin(_arm_state_pos, _S_axis, _M)).block<3,3>(0,0))).transpose()<<endl;

	_pauseGazebo.call(pauseSrv);
}

// KINEMATIC MODEL

// Direct Kinematic
Eigen::Matrix4d CONTROLLER::dir_kin(Eigen::VectorXd theta_fb, Eigen::MatrixXd S_axis, Eigen::MatrixXd rest_M){
	Eigen::MatrixXd T;
	T.resize(4,4);
	T=rest_M;
	Eigen::Matrix4d S_bracket;
	for (int i=5;i>=0;i--){
		S_bracket.block<3,3>(0,0) = utilities::skew(S_axis.block<1,3>(i,0));
		S_bracket.block<3,1>(0,3) = _S_axis.block<1,3>(i,3);
		S_bracket.block<1,4>(3,0) <<0,0,0,0;
		T = ((S_bracket*theta_fb[i]).exp())*T;

	}
	return T;
}

// Differential kinematic
Eigen::MatrixXd CONTROLLER::diff_kin(Eigen::VectorXd theta_fb){
    Eigen::MatrixXd J;
	Eigen::Matrix3d R_temp, R_curr;
	Eigen::Vector3d omega_vec, omega, v, q, q_disp, base_disp;

	R_curr.setIdentity();
	base_disp << 0,0,0;
	J.resize(6,6);
	J.setIdentity();

	for(int i=0;i<6;i++){
		if (_rot_vec[i]==0){
			R_temp.setIdentity();
		}
		else if (_rot_vec[i]==1){
			R_temp = utilities::rotx(theta_fb[i-1]);
		}
		else if (_rot_vec[i]==2 ){
			R_temp = utilities::roty(theta_fb[i-1]);
		}
		else if (_rot_vec[i]==3 ){
		    R_temp = utilities::rotz(theta_fb[i-1]);
		}
		else if (_rot_vec[i]==-1){
			R_temp = utilities::rotx(-theta_fb[i-1]);
		}
		else if (_rot_vec[i]==-2){
			R_temp = utilities::roty(-theta_fb[i-1]);
		}
		else if (_rot_vec[i]==-3){
		    R_temp = utilities::rotz(-theta_fb[i-1]);
		}

		R_curr = R_curr * R_temp;
		q_disp = _d_base.block<1,3>(i,0);
		q = base_disp + R_curr*q_disp;
		base_disp = q;

		if (_curr_rot[i] == 1){
			omega_vec << 1,0,0;
		}
		else if (_curr_rot[i] == 2){
			omega_vec << 0,1,0;
		}
		else if (_curr_rot[i] == 3){
			omega_vec << 0,0,1;
		}
		else if (_curr_rot[i] == -1){
			omega_vec << -1,0,0;
		}
		else if (_curr_rot[i] == -2){
			omega_vec << 0,-1,0;
		}
		else if (_curr_rot[i] == -3){
			omega_vec << 0,0,-1;
		}
        omega = R_curr * omega_vec;
		v = -omega.cross(q);

        J.block<3,1>(0,i) << omega;
        J.block<3,1>(3,i) << v;
	}
	return J;
}

// DYNAMIC MODEL

// Inverse Dynamic
Eigen::VectorXd CONTROLLER::recursive_inv_dyn(Eigen::VectorXd theta_fb, Eigen::VectorXd theta_d_fb, Eigen::VectorXd theta_dd_fb, Eigen::Vector3d gravity, Eigen::VectorXd f_ee){

	// param definition
	Eigen::VectorXd V_prev,Vd_prev, F_prev, tau;
	Vd_prev.resize(6);
	Vd_prev<<0,0,0,-gravity;
	V_prev.resize(6);
	V_prev<<0,0,0,0,0,0;

	F_prev.resize(6);
	F_prev = f_ee;
	tau.resize(_N_JOINTS);
	tau<<0,0,0,0,0,0;


	Eigen::Matrix4d M_prev, M, M_mutual, T_mutual;
	M_prev.setIdentity();
    Eigen::MatrixXd A_i, G_i, A_bracket, T, G, A, V, Vd, F;
	A_i.resize(6,1);
	G_i.resize(6,6);
	A_bracket.resize(4,4);
	A_bracket.setZero();
	T.resize(4*(_N_JOINTS+1),4*(_N_JOINTS+1));
	G.resize(6*_N_JOINTS,6*_N_JOINTS);
	A.resize(6*_N_JOINTS,_N_JOINTS);
	V.resize(6,_N_JOINTS);
	Vd.resize(6,_N_JOINTS);
	F.resize(6,_N_JOINTS);

	// cout<<"invdyn start: "<<endl;
	//forward cycle
	for (int i=0;i<_N_JOINTS;i++){
		M = utilities::M_f(_rot_mat.block<1,2>(i,0), _rot_angle.block<1,2>(i,0), _d_base, i);            //4x4
        A_i = utilities::Ad_f(M.inverse())*(_S_axis.block<1,6>(i,0)).transpose();                        //6x1
		G_i = utilities::G_f(_mass[i],_Inertia.block<3,3>(i*3,i*3),_inertial_disp.block<1,3>(i,0));      //6x6

		M_mutual = M_prev.inverse()*M;
		M_prev = M;

		A_bracket.block<3,3>(0,0) = utilities::skew(A_i.block<3,1>(0,0));
		A_bracket.block<3,1>(0,3) = A_i.block<3,1>(3,0);
        T_mutual = (-(A_bracket*theta_fb[i])).exp()*M_mutual.inverse();

		G.block<6,6>(i*6,i*6) = G_i;                                                                     //6nx6n
		T.block<4,4>(i*4,i*4) = T_mutual;                                                                //4(n+1)x4(n+1)
		A.block<6,1>(i*6,0) = A_i;                                                                       //6nxn

		V.block<6,1>(0,i) = utilities::Ad_f(T_mutual)*V_prev + A_i*theta_d_fb[i];                        //6nxn
		V_prev = V.block<6,1>(0,i);                                                                      //6x1

        Vd.block<6,1>(0,i) = utilities::Ad_f(T_mutual)*Vd_prev + utilities::ad_f_(V_prev)*A_i*theta_d_fb[i] + A_i*theta_dd_fb[i];
		Vd_prev = Vd.block<6,1>(0,i);                                                                    //6x1
	}

    (T.block<4,4>(4*_N_JOINTS,4*_N_JOINTS)).setIdentity();

	//backward cycle
	for (int i=_N_JOINTS-1;i>=0;i--){
		F.block<6,1>(0,i) =  utilities::Ad_f(T.block<4,4>((i+1)*4,(i+1)*4)).transpose()*F_prev
											+ G.block<6,6>(i*6,i*6)*Vd.block<6,1>(0,i)
												- utilities::ad_f_(V.block<6,1>(0,i)).transpose()*(G.block<6,6>(i*6,i*6)*V.block<6,1>(0,i));
		F_prev = F.block<6,1>(0,i);

		tau[i] = F_prev.transpose()*A.block<6,1>(i*6,0);                                                 //1x6*6x1
	}

	return tau;
}

// Inertia Matrix computation from inv-dyn
Eigen::MatrixXd CONTROLLER::inertia_matrix(Eigen::VectorXd theta_fb){
	int N_JOINTS = 6;
    Eigen::MatrixXd M;
	Eigen::VectorXd fake_acc, zero_vel, f_ee;
	Eigen::Vector3d gravity;
	f_ee.resize(6);
	f_ee<<0,0,0,0,0,0;
	gravity<<0,0,0;
	fake_acc.resize(6);
	M.resize(6,6);
	M.setZero();
	zero_vel.resize(6);
	zero_vel<<0,0,0,0,0,0;

    for (int i=0;i<N_JOINTS;i++){
        fake_acc << 0,0,0,0,0,0;
        fake_acc[i] = 1;
        M.block<6,1>(0,i) = recursive_inv_dyn(theta_fb, zero_vel, fake_acc, gravity, f_ee);
    }
	return M;
}

// Non-Linear dyn terms computation from inv-dyn
Eigen::MatrixXd CONTROLLER::c_forces(Eigen::VectorXd theta_fb, Eigen::VectorXd theta_d_fb){
	int N_JOINTS = 6;
	Eigen::VectorXd zero_acc, zero_vel, f_ee, c_q_dot;
	Eigen::Vector3d gravity;
	f_ee.resize(6);
	f_ee<<0,0,0,0,0,0;
	gravity<<0,0,0;
	zero_acc.resize(6);
	zero_acc.setZero();
	c_q_dot.resize(6);
	c_q_dot.setZero();

    c_q_dot = recursive_inv_dyn(theta_fb, theta_d_fb, zero_acc, gravity, f_ee);

	return c_q_dot;
}

// Gravity forces computation from inv-dyn
Eigen::MatrixXd CONTROLLER::gravity_forces(Eigen::VectorXd theta_fb){
	int N_JOINTS = 6;
	Eigen::VectorXd zero_acc, zero_vel, f_ee, g_q;
	Eigen::Vector3d gravity;
	f_ee.resize(6);
	f_ee<<0,0,0,0,0,0;
	gravity<<0,0,-9.8;
	zero_acc.resize(6);
	zero_acc.setZero();
	zero_vel.resize(6);
	zero_vel<<0,0,0,0,0,0;
	g_q.resize(6);
	g_q.setZero();

    g_q = recursive_inv_dyn(theta_fb, zero_vel, zero_acc, gravity, f_ee);

	return g_q;

}

// Quintic Polynomial Planner
Eigen::MatrixXd CONTROLLER::traj_generation(Eigen::Matrix4d T_s, Eigen::Matrix4d T_g, float ti, float tf, int N){

	// Starting and goal position and orientation
    Eigen::Matrix3d Rs = T_s.block<3,3>(0,0);
    Eigen::Vector3d ps = T_s.block<3,1>(0,3);

    Eigen::Matrix3d Rg = T_g.block<3,3>(0,0);
    Eigen::Vector3d pg = T_g.block<3,1>(0,3);

	float t, s, sd, sdd, th, thd, thdd;
	Eigen::Vector3d p, pd, pdd;
	Eigen::Vector3d omega_i, omegad_i, omega_e, omegad_e;
	Eigen::Matrix3d Re, Re_dot, Ri;
	Eigen::MatrixXd Te_dot, Te, Traj;

	Te.resize(4*N,4*N);
	Te.setIdentity();
	Te_dot.resize(4*N,4*N);
	Te_dot.setZero();
	Traj.resize(8*N,8*N);
	Traj.setZero();

    // Time interval
    float T = tf - ti;

    // Coefficients of the fifth order polinomial
    float a1, a2, a3;
	a1 = 10/pow(T,3);
    a2 = -15/pow(T,4);
    a3 = 6/pow(T,5);

    // Displacements parameters among Rs and Rg
    Eigen::Matrix3d Rsg;
	Rsg = Rs.transpose()*Rg;
    // These two equations can be used instead of rotm2axang
	float th_f;
	Eigen::Vector3d r, r_t;
    th_f = acos((Rsg(0,0) + Rsg(1,1) + Rsg(2,2) -1)/2);
	r_t << Rsg(2,1) - Rsg(1,2), Rsg(0,2) - Rsg(2,0), Rsg(1,0) - Rsg(0,1);
    r = 1/(2*sin(th_f)) * r_t;

    for (int i=0;i<N;i++){
        t = T/(N-1)*(i);

        // Arc length for the rectilinear path
        s = a1*pow(t, 3.0) + a2*pow(t, 4.0) + a3*pow(t, 5.0);
        sd = 3*a1*pow(t, 2.0) + 4*a2*pow(t, 3.0) + 5*a3*pow(t, 4.0);
        sdd = 6*a1*t + 12*a2*pow(t, 2.0) + 20*a3*pow(t, 3.0);

        // Rectilinear path
        p = ps + s*(pg-ps);
        pd = sd*(pg-ps);
        pdd = sdd*(pg-ps);

        // Angle between Rs and Rg
        th = a1*pow(t, 3.0)*th_f + a2*pow(t, 4.0)*th_f + a3*pow(t, 5.0)*th_f;
        thd = 3*a1*pow(t, 2.0)*th_f + 4*a2*pow(t, 3.0)*th_f + 5*a3*pow(t, 4.0)*th_f;
        thdd = 6*a1*t*th_f + 12*a2*pow(t, 2.0)*th_f + 20*a3*pow(t, 3.0)*th_f;

        // Angular velocity and acceleration of the "middle frame" R^i
        omega_i = thd*r;
        omegad_i = thdd*r;

        // Angular path
		Ri << pow(r[0],2)*(1-cos(th))+cos(th), r[0]*r[1]*(1-cos(th))-r[2]*sin(th), r[0]*r[2]*(1-cos(th))+r[1]*sin(th), r[0]*r[1]*(1-cos(th))+r[2]*sin(th), pow(r[1],2)*(1-cos(th))+cos(th), r[1]*r[2]*(1-cos(th))-r[0]*sin(th), r[0]*r[2]*(1-cos(th))-r[1]*sin(th), r[1]*r[2]*(1-cos(th))+r[0]*sin(th), pow(r[2],2)*(1-cos(th))+cos(th);
        Re = Rs*Ri;
        omega_e = Rs*omega_i;
        omegad_e = Rs*omegad_i;

        // Te_dot and Te
        Re_dot = utilities::skew(omega_e)*Re;

        Te_dot.block<3,3>(4*i,4*i)=Re_dot;
		Te_dot.block<3,1>(4*i,3+4*i) = pd;

		Te.block<3,3>(4*i,4*i)=Re;
		Te.block<3,1>(4*i,4*i+3) = p;
	}

	for (int k=0;k<4*N;k++){
			for(int j=0;j<4*N;j++){
				Traj(k,j) = Te(k,j);
				Traj(4*N+k,4*N+j) = Te_dot(k,j);
			}
		}
	return Traj;
}

// ROS - PUBLISHERS

// Arm torques command
void CONTROLLER::cmd_arm_ctrl() {

	ros::Rate r(_rate);
	std_msgs::Float64 ctrl_input_1, ctrl_input_2, ctrl_input_3, ctrl_input_4, ctrl_input_5, ctrl_input_6;
	std_msgs::Float64MultiArray pose, vel, acc;
	Eigen::VectorXd joint_cmd, joint_cmd_old;
	joint_cmd.resize(6);
	joint_cmd_old.resize(6);
	double alpha, beta;
	alpha = 0.05;
	beta = 0.01;
    while( ros::ok() ) {

		while( !_first_odom_1 && !_first_odom_2) usleep(0.1*1e6);


        ctrl_input_1.data = _arm_cmd[0];
        ctrl_input_2.data = _arm_cmd[1];
        ctrl_input_3.data = _arm_cmd[2];
        ctrl_input_4.data = _arm_cmd[3];
        ctrl_input_5.data = _arm_cmd[4];
        ctrl_input_6.data = _arm_cmd[5];

        _cmd_tau1_pub.publish(ctrl_input_1);
        _cmd_tau2_pub.publish(ctrl_input_2);
        _cmd_tau3_pub.publish(ctrl_input_3);
        _cmd_tau4_pub.publish(ctrl_input_4);
        _cmd_tau5_pub.publish(ctrl_input_5);
        _cmd_tau6_pub.publish(ctrl_input_6);

		
		pose.data.resize(6);
		vel.data.resize(6);
		acc.data.resize(6);
		for(int i=0;i<6;i++){
		// 	if (_pos_cmd(i)>0)
		// 		_pos_cmd(i) = fmod(_pos_cmd(i)+M_PI, 2.0*M_PI)-M_PI;
		// 	else
		// 		_pos_cmd(i) = fmod(_pos_cmd(i)-M_PI, 2.0*M_PI)+M_PI;
		// 	if (i<3)	
		// 		joint_cmd(i) = alpha * _pos_cmd(i) + (1.0 - alpha) * joint_cmd_old(i);
		// 	else
		// 		joint_cmd(i) = beta * _pos_cmd(i) + (1.0 - beta) * joint_cmd_old(i);

		// 	joint_cmd_old(i) = joint_cmd(i);
			joint_cmd(i) = _pos_cmd(i);
		}
		pose.data[0] = joint_cmd(0);
		pose.data[1] = joint_cmd(1);
		pose.data[2] = joint_cmd(2);
		pose.data[3] = joint_cmd(3);
		pose.data[4] = joint_cmd(4);
		pose.data[5] = joint_cmd(5);
		_pos_sp_pub.publish(pose); 

		acc.data[0] = _acc_cmd(0);
		acc.data[1] = _acc_cmd(1);
		acc.data[2] = _acc_cmd(2);
		acc.data[3] = _acc_cmd(3);
		acc.data[4] = _acc_cmd(4);
		acc.data[5] = _acc_cmd(5);
		_acc_sp_pub.publish(acc); 

		vel.data[0] = _vel_cmd(0);
		vel.data[1] = _vel_cmd(1);
		vel.data[2] = _vel_cmd(2);
		vel.data[3] = _vel_cmd(3);
		vel.data[4] = _vel_cmd(4);
		vel.data[5] = _vel_cmd(5);
		_vel_sp_pub.publish(vel); 

    	r.sleep();
    }

}

// Mult-thread
void CONTROLLER::run() {
	// boost::thread cmd_input_t( &CONTROLLER::new_plan, this );
	boost::thread ctrl_law_t( &CONTROLLER::arm_invdyn_control, this );
	boost::thread cmd_arm_ctrl_t( &CONTROLLER::cmd_arm_ctrl, this );

	ros::spin();
}

// Main
int main(int argc, char** argv ) {

	ros::init(argc, argv, "cmd_vel_ctrl");

	CONTROLLER kc;
	kc.run();

	return 0;
}