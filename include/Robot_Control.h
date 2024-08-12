#pragma once

#include <chrono>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include "Simulator/Robot_Simulate.hpp"

#include "ARBMLlib/ARBML.h"
#include "Trajectory/JointTrajectory.h"
#include "SplineLibs/CubicSpline.h"
#include "NullControl/NullControl.hpp"
#include "csvpp/Write.h"

using namespace std;

//	Time Variables
constexpr sysReal TORQUE_ON = 1.0;				//	Second
constexpr sysReal INITIAL_POSE = 2.0;				//	Second

//	Multiple of Simulation Step(Forward dynamics) > 1
constexpr int CONTROL_RATE = 1;

typedef enum {
	TORQ_OFF	= 0X01,
	TORQ_ON 	= 0X02,
	READY		= 0X03,
	CLIK		= 0X04,
}TaskCmdType;

typedef enum {
	JOINT_PD 	= 0X01,
	INV_DYN	 	= 0X02,
	TORQUE		= 0X03,
	GRAV_COMP	= 0X04,
}CtrlType;

class CRobotControl
{
private:

public:
	
	CARBML robot;
	CRobotControl();
	~CRobotControl() {}

	unsigned count_sim;
	unsigned count_ctrl;
	unsigned CtrlFlag;
	unsigned TaskFlag;
	unsigned TaskScheduleFlag;

	//////////	Active joint variables	//////////
	Eigen::Matrix<double, ACTIVE_DOF, 1>			joint_torq;					//	Active joint torque
	Eigen::Matrix<double, ACTIVE_DOF, 1>			qpos_d, qvel_d, qacc_d;		//	Desired position & velocity of active joint

	///////////////////////////////////////////////////////////////////////////
	/////	Motion parameters for body frame : NOT NECESSARY !!!!
	///////////////////////////////////////////////////////////////////////////
	Eigen::Vector3d								p_lnk[NO_OF_BODY];			//	Position vector of i-th link frame in {I}
	Eigen::Matrix3d								R_lnk[NO_OF_BODY];			//	Rotation matrix of i-th link frame in {I}

	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnk[NO_OF_BODY];			//	Linear Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnk[NO_OF_BODY];			//	Angular Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_lnkCoM[NO_OF_BODY];		//	CoM Jacobian of i-th link in {I}

	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnk[NO_OF_BODY];		//	Time derivative of Jp_lnk
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_lnk[NO_OF_BODY];		//	Time derivative of Jr_lnk
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdot_lnkCoM[NO_OF_BODY];	//	Time derivative of Jp_lnkCoM


	///////////////////////////////////////////////////////////////////////////
	/////	Motion parameters for end-effectors : Expressed in {I}
	///////////////////////////////////////////////////////////////////////////
	int													no_of_EE;			//	Number of end-effectors
	vector<int>											id_body_EE;			//	End-effector ID
	vector<Eigen::Vector3d>								p0_lnk2EE;			//	Local position offset from link frame to end-effector
	vector<Eigen::Matrix3d>								R0_lnk2EE;			//	Local rotation offset from link frame to end-effector

	vector<Eigen::Vector3d>								p_EE;				//	Position vector of i-th end-effector
	vector<Eigen::Vector3d>								pdot_EE;			//	Linear velocity of i-th end-effector
	vector<Eigen::Vector3d>								omega_EE;			//	Angular velocity of i-th end-effector
	vector<Eigen::Matrix3d>								R_EE;				//	End-effector rotation matrix
	vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>		Jp_EE;				//	i-th End-effector linear Jacobian
	vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>		Jr_EE;				//	i-th End-effector angular Jacobian
	vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>		Jdotp_EE;			//	Time derivative of Jp_EE
	vector<Eigen::Matrix<double, DOF3, TOTAL_DOF>>		Jdotr_EE;			//	Time derivative of Jr_EE

	////////////////////////////////////////////
	////////////////	JY CODE	////////////////
	////////////////////////////////////////////
	double sim_time;
	TaskCmdType TaskCmd, PrevTaskCmd;
	Eigen::Matrix<double, ACTIVE_DOF, ACTIVE_DOF> 	K_qp, K_qv;		//	Joint gain matrices for active joint
	
	// Polynomial Trajectory
	CP2P_Traj<ACTIVE_DOF, double> 			Joint_Traj;
	Eigen::Matrix<double, ACTIVE_DOF, 1> 	qpos_ref;

	CP2P_Traj<DOF3, double> 				lEE_Traj;
	CP2P_Traj<DOF3, double> 				rEE_Traj;
	CP2P_Traj<DOF3, double> 				lEE_OriTraj;
	CP2P_Traj<DOF3, double> 				rEE_OriTraj;
	SPLINE::CCubicSpline					lEE_CubicSpline_Traj;
	SPLINE::CCubicSpline					rEE_CubicSpline_Traj;
	double 									lEE_local_t = 0.0;
	double 									rEE_local_t = 0.0;

	// remap p_EE -> p_XEE
	Eigen::Vector3d p_lEE, p_rEE;
	Eigen::Vector3d pdot_lEE, pdot_rEE;
	Eigen::Matrix<double, DOF3, TOTAL_DOF> Jp_lEE, Jr_lEE, Jdotp_lEE, Jdotr_lEE;
	Eigen::Matrix<double, DOF3, TOTAL_DOF> Jp_rEE, Jr_rEE, Jdotp_rEE, Jdotr_rEE;

	Eigen::Vector3d p_lEE_d, p_rEE_d;
	Eigen::Vector3d pdot_lEE_d, pdot_rEE_d;
	Eigen::Vector3d pddot_lEE_d, pddot_rEE_d;

	// cmd variables
	Eigen::Matrix<double, TOTAL_DOF, 1> 	qddot_cmd;
	Eigen::Matrix<double, TOTAL_DOF, 1> 	torq_cmd;

	// Nullspace Control related
	Null::NullSpaceControl 					NullCtrl;
	Eigen::Matrix<double, ACTIVE_DOF, 1>	pre_qvel_d;		//	Desired position & velocity of active joint

	// CLIK related variables
	bool initial_clik_flag = false;
	Eigen::Matrix<double, TOTAL_DOF, 1> qpos_init;

    csvpp::Writer<double, double, double, double, double, double, double> lEE_d{
        // destination file
        CMAKE_SOURCE_DIR"/log/data/lEE_d.csv",
        // headers
        "dt",
		"p_lEE_x","p_lEE_y","p_lEE_z",
		"pdot_lEE_x","pdot_lEE_y","pdot_lEE_z"
    };

	void getUserCommand(mujoco::Simulate& sim);
	void computeMotionTasks();
	void mapEEvar();
	void JointPlanner(double duration);
	void LeftEEPlanner(double duration);
	void RightEEPlanner(double duration);
	void assignSelectedJointTask(std::vector<int> selectedJoints);
	void CLIK();
	void CTC(); // computed torque control
	void RAC();	// resolved acceleration control
	void NullSpacePlanner();
	void OptimalControl();
	void computeJointTorque(CtrlType type);
	////////////////////////////////////////////
	////////////////	JY CODE	////////////////
	////////////////////////////////////////////


	//////////	Functions	//////////
	void InitializeSystem(const mjModel* model_mj);
	void initEEParameters(const mjModel* model);
	void initCtrlParameters(const mjModel* model_mj);
	void outputEEInformation();

	void UserControl(mjModel* uModel, mjData* uData);
	void getFeedbackInformation(const mjData* data);
	void computeControlInput();

	void computeCoMMotion();
	void computeEEKinematics(Eigen::Matrix<double, TOTAL_DOF, 1>& xidot);
	void computeLinkKinematics();	//	Compute kinematics and Jacobian, Jdot, etc

	//	Check code validaty
	void compareModelComputation(const mjModel* model, mjData* data, const int& count);
};