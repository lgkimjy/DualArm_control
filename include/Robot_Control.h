#pragma once

#include <chrono>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include "ARBMLlib/ARBML.h"
#include "Trajectory/JointTrajectory.h"

using namespace std;

//	Time Variables
constexpr sysReal TORQUE_ON = 1.0;				//	Second
constexpr sysReal INITIAL_POSE = 2.0;				//	Second

//	Multiple of Simulation Step(Forward dynamics) > 1
constexpr int CONTROL_RATE = 1;

typedef enum {
	JOINT_PD = 0X01,
	TORQUE	 = 0X02,
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


	////////////////	JY CODE	////////////////
	double sim_time;
	Eigen::Matrix<double, ACTIVE_DOF, ACTIVE_DOF> 	K_qp, K_qv;		//	Joint gain matrices for active joint
	
	// Polynomial Trajectory
	CP2P_Traj<ACTIVE_DOF, double> 			Joint_Traj;
	CP2P_Traj<DOF3, double> 				LPos_Traj;
	CP2P_Traj<DOF3, double> 				RPos_Traj;
	Eigen::Matrix<double, ACTIVE_DOF, 1> 	qpos_ref;

	void computeMotionTasks();
	void JointPlanner(double duration);
	void computeJointTorque(CtrlType type);
	////////////////	JY CODE	////////////////


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