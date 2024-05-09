//
//
#include "Robot_Control.h"


CRobotControl::CRobotControl() : count_sim(0), count_ctrl(0), CtrlFlag(0), TaskFlag(0), TaskScheduleFlag(0)
{
	qpos_d.setZero();
	qvel_d.setZero();
	qacc_d.setZero();

	joint_torq.setZero();


	no_of_EE = 0;
	id_body_EE.clear();

	p0_lnk2EE.clear();
	R0_lnk2EE.clear();

	p_EE.clear();
	R_EE.clear();
	pdot_EE.clear();
	omega_EE.clear();

	Jp_EE.clear();
	Jr_EE.clear();
	Jdotp_EE.clear();
	Jdotr_EE.clear();
}



////////////////////////////////////////////////////////////////////////////////
//	Initialize control related parameters !
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::InitializeSystem(const mjModel* model_mj)
{
	robot.initRobot(model_mj);

	/////	Set joint range of motion, end-effector, etc
	initEEParameters(model_mj);

	/////	Set controller parameters
	initCtrlParameters(model_mj);

#ifdef PRINT_END_EFFECTOR_INFO
	outputEEInformation();
#endif
}



////////////////////////////////////////////////////////////////////////////////
//	Initialize End-effector Parameters !
//	 * Joint motion range
//	 * Local End-effector pose !
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::initEEParameters(const mjModel* model)
{
	int i;
	Eigen::Vector3d temp_vec;
	Eigen::Vector4d temp_quat;


	//////////	Get body ID for end-effectors (defined in XML file via model->site !)
	no_of_EE = model->nsite;

	for (i = 0; i < no_of_EE; i++) {
		id_body_EE.push_back(model->site_bodyid[i] - 1);

		temp_vec = {(sysReal)model->site_pos[i * 3],
					(sysReal)model->site_pos[i * 3 + 1],
					(sysReal)model->site_pos[i * 3 + 2]};
		p0_lnk2EE.push_back(temp_vec);				//	Set rel. position of end-effector ???

		temp_quat = {(sysReal)model->site_quat[i * 4],
					 (sysReal)model->site_quat[i * 4 + 1],
					 (sysReal)model->site_quat[i * 4 + 2],
					 (sysReal)model->site_quat[i * 4 + 3]};
		R0_lnk2EE.push_back(_Quat2Rot(temp_quat));	//	Set rel. orientation of end-effector
	}

	id_body_EE.shrink_to_fit();
	p0_lnk2EE.shrink_to_fit();
	R0_lnk2EE.shrink_to_fit();


	/////	Initialize transformation matrix about base, end-effector, contact wheel
	p_EE.reserve(no_of_EE);
	R_EE.reserve(no_of_EE);
	pdot_EE.reserve(no_of_EE);
	omega_EE.reserve(no_of_EE);

	Jp_EE.reserve(no_of_EE);		//	Linear Jacobian of end-effectors
	Jr_EE.reserve(no_of_EE);		//	Angular Jacobian of end-effectors
	Jdotp_EE.reserve(no_of_EE);		//	Time derivative of Jp_EE
	Jdotr_EE.reserve(no_of_EE);		//	Time derivative of Jr_EE

	for (int i = 0; i < no_of_EE; i++) {
		p_EE[i].setZero();
		R_EE[i].setIdentity();
		pdot_EE[i].setZero();
		omega_EE[i].setZero();

		Jp_EE[i].setZero();
		Jr_EE[i].setZero();
		Jdotp_EE[i].setZero();
		Jdotr_EE[i].setZero();
	}
}



void CRobotControl::outputEEInformation()
{
	int i;

	cout.precision(3);
	cout << endl << "No. of End-effector : " << no_of_EE;
	cout << endl << "ID of End-effectors : ";
	for (auto& it : id_body_EE)
		cout << it << " ";
	cout << endl;

	cout << endl << "Local Position of End-effector : p0_lnk2EE()" << endl;
	for (i = 0; i < p0_lnk2EE.size(); i++) {
		cout << "[" << i << "] : ";
		cout << " [ " << p0_lnk2EE[i].transpose() << " ]" << endl;
	}

	cout << endl << "Local Rotation of End-effector : R0_lnk2EE()" << endl;
	for (i = 0; i < R0_lnk2EE.size(); i++) {
		cout << "[" << i << "] : " << endl;
		cout << R0_lnk2EE[i] << endl;
	}

	cout << "================================================================================" << endl;
}



void CRobotControl::initCtrlParameters(const mjModel* model_mj)
{
	K_qp = 1000.0 * K_qp.setIdentity();
	K_qv = 100.0 * K_qv.setIdentity();

	//	Task selection
	TaskFlag = TaskScheduleFlag = task_standing;

	//CtrlFlag = (unsigned)PointJointPD;
	CtrlFlag = (unsigned)TrackingJointPD;

	//cout << "Task Flag : " << TaskFlag << endl;
	//cout << "Control Flag : " << CtrlFlag << endl;
}



////////////////////////////////////////////////////////////////////////////////
//	USER Control Core !!
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::UserControl(mjModel* model, mjData* data)
{
	int i;

	//mj_warning(data, warning_index, warning_info);
	if ((data->warning[mjWARN_INERTIA].lastinfo != 0) || (data->warning[mjWARN_BADQPOS].lastinfo != 0) ||
		(data->warning[mjWARN_BADQVEL].lastinfo != 0) || (data->warning[mjWARN_BADQACC].lastinfo != 0)) {
		_ErrorMsg("Check Inertia, Position, Velocity & Acceleration !!");
	}

	/////	Check the kinematics and dynamics of model
	if (count_sim > 0)	compareModelComputation(model, data, count_ctrl);

	////////////////////	Main Control Routine	////////////////////
	if (count_sim % CONTROL_RATE == 0) {
		//	Time Stamp Routine : Start
		std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();

		/////	01. Input feedback variables(joint pos/vel, base pos/ori/vel)
		getFeedbackInformation(data);

		/////	02. Compute Kinematic Motion Core (w.r.t Base frame) - Called BEFORE All others !!!
		robot.computeMotionCore();

		/////	03. Compute CoM Kinematics : "computeMotionCore()" is followed by "computeCoMKinematics()"
		robot.computeCoMKinematics();

		//computeCoMMotion(robot);

		/////	Compute Dynamics : "computeCoMKinematics()" is followes by "computeDynamics()"
		robot.computeDynamics();

		/////	Compute Link/CoM Kinematics (w.r.t Inertial frame) 
		computeLinkKinematics();

		/////	Compute End-effector Kinematics : "computeMotionCore()" is followed by "computeEEKinematics()"
		computeEEKinematics(robot.xidot);

		/////	Compute Control Input & Output Control Input
		computeControlInput();

		for (i = 0; i < model->nu; i++) {
			data->ctrl[i] = joint_torq(i);
		}
		++count_ctrl;
	}
	++count_sim;
}



///////////////////////////////////////////////////////////////////////////
/////	Read Devices : State Feedback !
///////////////////////////////////////////////////////////////////////////
void CRobotControl::getFeedbackInformation(const mjData* data)
{
#ifdef _FLOATING_BASE
	/////	Position vector of floating-base body w.r.t {I}
	robot.p_B(0) = data->qpos[0];
	robot.p_B(1) = data->qpos[1];
	robot.p_B(2) = data->qpos[2];

	/////	Orientation of floating-base body w.r.t {I}
	robot.quat_B(0) = data->qpos[3];
	robot.quat_B(1) = data->qpos[4];
	robot.quat_B(2) = data->qpos[5];	
	robot.quat_B(3) = data->qpos[6];

	robot.R_B = _Quat2Rot(robot.quat_B);

	/////	Linear velocity of floating-base body w.r.t {I}
	robot.pdot_B(0) = data->qvel[0];
	robot.pdot_B(1) = data->qvel[1];
	robot.pdot_B(2) = data->qvel[2];

	/////	Angular velocity of floating-base body expressed in {B}
	robot.varphi_B(0) = data->qvel[3];
	robot.varphi_B(1) = data->qvel[4];
	robot.varphi_B(2) = data->qvel[5];

	/////	Convert to absolute angular velocity
	robot.omega_B = robot.R_B * robot.varphi_B;

	/////	Set generalized coordinates
	robot.xi_quat.segment(0, DOF3) = robot.p_B;
	robot.xi_quat.segment(DOF3, DOF4) = robot.quat_B;

	robot.xidot.segment(0, DOF3) = robot.pdot_B;
	robot.xidot.segment(DOF3, DOF3) = robot.omega_B;
#endif

	for (int i = 0; i < ACTIVE_DOF; i++) {
		robot.q(i) = data->qpos[DOF_BASEBODY_QUAT + i];
		robot.qdot(i) = data->qvel[DOF_BASEBODY + i];
	}

	/////	Set joint coordinates and joint velocity
	robot.xi_quat.segment(DOF_BASEBODY_QUAT, ACTIVE_DOF) = robot.q;
	robot.xidot.segment(DOF_BASEBODY, ACTIVE_DOF) = robot.qdot;

	/////	Compute joint acceleration by numerical diff.
	robot.xiddot = (robot.xidot - robot.xidot_tmp) / robot.getSamplingTime();
	robot.xidot_tmp = robot.xidot;
}



void CRobotControl::computeControlInput()
{
	// joint_torq = K_qp * (0.0 - robot.q) + K_qv * (0.0 - robot.qdot);
}



////////////////////////////////////////////////////////////////////////////////
//	Compute pose, Jacobian and its time derivative for ALL link w.r.t {I}
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::computeLinkKinematics()
{
	for (int i = 0; i < NO_OF_BODY; i++) {
		robot.getLinkPose(i, p_lnk[i], R_lnk[i]);

		robot.getBodyJacob(i, p_lnk[i], Jp_lnk[i], Jr_lnk[i]);

		robot.getBodyJacobDeriv(i, Jdotp_lnk[i], Jdotr_lnk[i]);
	}
}



void CRobotControl::computeCoMMotion()
{
	///	Time Stamp Routine : Start
	std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();

	int i;
	int error;
	sysReal m_G = robot.getTotalMass();
	Eigen::Vector3d								p_lnkCoM[NO_OF_BODY];
	Eigen::Matrix3d								R_lnkCoM[NO_OF_BODY];
	Eigen::Vector3d								p_G;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_G;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdot_G;
	p_G.setZero();
	J_G.setZero();
	Jdot_G.setZero();

	for (int k = 0; k < 20; k++) {
		p_G.setZero();
		J_G.setZero();
		Jdot_G.setZero();

		for (i = 0; i < NO_OF_BODY; i++) {
			error = 0;
			robot.getLinkCoMPose(i, p_lnkCoM[i], R_lnkCoM[i]);
			p_G += robot.body[i].get_mass() * p_lnkCoM[i];

			robot.getBodyJacob(i, p_lnkCoM[i], J_lnkCoM[i]);
			J_G += robot.body[i].get_mass() * J_lnkCoM[i];

			robot.getBodyJacobDeriv(i, Jdot_lnkCoM[i]);
			Jdot_G += robot.body[i].get_mass() * Jdot_lnkCoM[i];
		}

		p_G = p_G / m_G;
		J_G = J_G / m_G;
		Jdot_G = Jdot_G / m_G;

		robot.p_B2CoM = p_G - robot.p_B;
		robot.pdot_CoM = J_G * robot.xidot;
		robot.pdot_B2CoM = robot.pdot_CoM - robot.pdot_B;
	}
}


////////////////////////////////////////////////////////////////////////////////
//	Compute End-effector Kinematics w.r.t {I}
//	 * Pose, CoM Jacobian and its time derivative
//	 * Linear/angular velocity vectors
////////////////////////////////////////////////////////////////////////////////
void CRobotControl::computeEEKinematics(Eigen::Matrix<double, TOTAL_DOF, 1>& xidot)
{
	int i, j, k;

	/////	Position / Rotation matrix of end-effector w.r.t {I}
	for (i = 0; i < id_body_EE.size(); i++) {
		robot.getBodyPose(id_body_EE[i], p0_lnk2EE[i], R0_lnk2EE[i], p_EE[i], R_EE[i]);
	}

	/////	End-effector Jacobian & its time derivative w.r.t {I}  (Geometric Jacobian NOT analytic Jacobian !)
	for (i = 0; i < id_body_EE.size(); i++) {
		robot.getBodyJacob(id_body_EE[i], p_EE[i], Jp_EE[i], Jr_EE[i]);
		robot.getBodyJacobDeriv(id_body_EE[i], Jdotp_EE[i], Jdotr_EE[i]);
	}

	/////	Compute end-effector velocity expressed in {I}
	for (i = 0; i < id_body_EE.size(); i++) {
		pdot_EE[i].setZero();
		omega_EE[i].setZero();

#ifdef _FLOATING_BASE
		for (j = 0; j < DOF3; j++) {
			for (k = 0; k < DOF6; k++) {
				pdot_EE[i](j) += Jp_EE[i](j, k) * xidot(k);
				omega_EE[i](j) += Jr_EE[i](j, k) * xidot(k);
			}
		}
#endif
		for (j = 0; j < DOF3; j++) {
			for (unsigned& idx : robot.kinematic_chain[id_body_EE[i]]) {
				k = idx + DOF_BASEBODY - robot.BodyID_ActJntStart();
				pdot_EE[i](j) += Jp_EE[i](j, k) * xidot(k);
				omega_EE[i](j) += Jr_EE[i](j, k) * xidot(k);
			}
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, DOF3, TOTAL_DOF> Pre_Jp_EE_mj[NO_OF_ENDEFFECTOR], Pre_Jr_EE_mj[NO_OF_ENDEFFECTOR];
Eigen::Matrix<double, DOF3, TOTAL_DOF> Pre_Jp_lnk_mj[NO_OF_BODY], Pre_Jr_lnk_mj[NO_OF_BODY];
Eigen::Matrix<double, DOF3, TOTAL_DOF> Pre_Jp_lnkCoM_mj[NO_OF_BODY];

void CRobotControl::compareModelComputation(const mjModel* uModel, mjData* uData, const int& count)
{
	int i, j, k;
	int error = 0;
	int mjBodyID;
	int start_BodyID = robot.BodyID_ActJntStart();

	sysReal dT = robot.getSamplingTime();
	sysReal error_precision = uModel->opt.tolerance;

	mjtNum* jacp = new mjtNum[DOF3 * TOTAL_DOF]{ 0 };
	mjtNum* jacr = new mjtNum[DOF3 * TOTAL_DOF]{ 0 };
	mjtNum* dense_M = new mjtNum[TOTAL_DOF * TOTAL_DOF]{ 0 };
	mjtNum* temp_vec3 = new mjtNum[DOF3]{ 0 };

	Eigen::Vector3d								p_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix3d								R_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_EE[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_EE[NO_OF_ENDEFFECTOR];

	Eigen::Vector3d								p_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix3d								R_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_EE_mj[NO_OF_ENDEFFECTOR];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_EE_mj[NO_OF_ENDEFFECTOR];

	Eigen::Vector3d								p_lnk[NO_OF_BODY];
	Eigen::Vector3d								p_lnkCoM[NO_OF_BODY];
	Eigen::Matrix3d								R_lnk[NO_OF_BODY];
	Eigen::Matrix3d								R_lnkCoM[NO_OF_BODY];

	Eigen::Vector3d								p_lnk_mj[NO_OF_BODY];
	Eigen::Vector3d								p_lnkCoM_mj[NO_OF_BODY];
	Eigen::Matrix3d								R_lnk_mj[NO_OF_BODY];
	Eigen::Matrix3d								R_lnkCoM_mj[NO_OF_BODY];

	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnk[NO_OF_BODY];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnk[NO_OF_BODY];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_lnkCoM[NO_OF_BODY];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnk[NO_OF_BODY];
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_lnk[NO_OF_BODY];

	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnk_mj[NO_OF_BODY];		//	Linear Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnk_mj[NO_OF_BODY];		//	Angular Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jp_lnkCoM_mj[NO_OF_BODY];	//	CoM Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jr_lnkCoM_mj[NO_OF_BODY];	//	CoM Jacobian of i-th link in {I}
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnk_mj[NO_OF_BODY];	//	Time derivative of Jp_lnk
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotr_lnk_mj[NO_OF_BODY];	//	Time derivative of Jr_lnk
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdotp_lnkCoM_mj[NO_OF_BODY];

	Eigen::Matrix<double, TOTAL_DOF, 1>			hvec;
	Eigen::Matrix<double, TOTAL_DOF, 1>			hvec_mj;
	Eigen::Matrix<double, TOTAL_DOF, 1>			gvec_mj;
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF>	Mmat_mj;

	Eigen::Vector3d								p_G;
	Eigen::Vector3d								p_CoM_mj;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_G;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		J_G_mj;
	Eigen::Matrix<double, DOF3, TOTAL_DOF>		Jdot_G;

	Eigen::Vector3d								tempVec3_1, tempVec3_2, tempVec3_3, tempVec3_4;
	Eigen::Vector3d								tmpPosVec1, tmpPosVec2;
	Eigen::Matrix3d								TmpRot_Mat1, TmpRot_Mat2;
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> X_O;

	for(int i = 0; i<NO_OF_ENDEFFECTOR; i++) {
		p_EE[i].setZero();
		R_EE[i].setZero();
		Jp_EE[i].setZero();
		Jr_EE[i].setZero();
		Jdotp_EE[i].setZero();
		Jdotr_EE[i].setZero();
		p_EE_mj[i].setZero();
		R_EE_mj[i].setZero();
		Jp_EE_mj[i].setZero();
		Jr_EE_mj[i].setZero();
		Jdotp_EE_mj[i].setZero();
		Jdotr_EE_mj[i].setZero();
	}

	for(int i = 0; i<NO_OF_BODY; i++) {
		Jp_lnk[i].setZero();
		Jr_lnk[i].setZero();
		J_lnkCoM[i].setZero();
		Jdotp_lnk[i].setZero();
		Jdotr_lnk[i].setZero();
		Jp_lnk_mj[i].setZero();
		Jr_lnk_mj[i].setZero();
		Jp_lnkCoM_mj[i].setZero();
		Jr_lnkCoM_mj[i].setZero();
		Jdotp_lnk_mj[i].setZero();
		Jdotr_lnk_mj[i].setZero();
		Jdotp_lnkCoM_mj[i].setZero();
	}

	X_O.setIdentity();

#ifdef _FLOATING_BASE
	X_O.block(DOF3, DOF3, DOF3, DOF3) = robot.R_B;
#endif

	//////////////////////////////////////////////////////////////////////
	/////	Get All information from MuJoCo !! --> Convert to {I}
	//////////////////////////////////////////////////////////////////////
	for (i = 0; i < NO_OF_BODY; i++) {
		mjBodyID = i + 1;

		/////	01. Position & rotation of each body/link frame and link CoM w.r.t {I}
		for (j = 0; j < DOF3; j++) {
			p_lnk_mj[i](j) = uData->xpos[mjBodyID * DOF3 + j];						//	Position of Each Link from MuJoCo
			p_lnkCoM_mj[i](j) = uData->xipos[mjBodyID * DOF3 + j];					//	CoM Position of Each Link from MuJoCo

			for (k = 0; k < DOF3; k++) {
				R_lnk_mj[i](j, k) = uData->xmat[mjBodyID * 9 + j * DOF3 + k];		//	Link Rotation from MuJoCo
				R_lnkCoM_mj[i](j, k) = uData->ximat[mjBodyID * 9 + j * DOF3 + k];	//	Link CoM Rotation from MuJoCo
			}
		}

		/////	02. ALL Link Jacobian
		mj_jacBody(uModel, uData, jacp, jacr, mjBodyID);
		for (j = 0; j < DOF3; j++) {
			for (k = 0; k < TOTAL_DOF; k++) {
				Jp_lnk_mj[i](j, k) = jacp[j * TOTAL_DOF + k];
				Jr_lnk_mj[i](j, k) = jacr[j * TOTAL_DOF + k];
			}
		}
		//	Transform from MuJoCo coordinate to inertial coordinate
		Jp_lnk_mj[i] = Jp_lnk_mj[i] * Eigen::Transpose(X_O);
		Jr_lnk_mj[i] = Jr_lnk_mj[i] * Eigen::Transpose(X_O);


		/////	03. Time derivative of Link Jacobian via numerical diff.
		Jdotp_lnk_mj[i] = (Jp_lnk_mj[i] - Pre_Jp_lnk_mj[i]) / dT;
		Jdotr_lnk_mj[i] = (Jr_lnk_mj[i] - Pre_Jr_lnk_mj[i]) / dT;
		Pre_Jp_lnk_mj[i] = Jp_lnk_mj[i];
		Pre_Jr_lnk_mj[i] = Jr_lnk_mj[i];


		/////	04. ALL Link CoM Jacobian
		mj_jacBodyCom(uModel, uData, jacp, jacr, mjBodyID);
		for (j = 0; j < DOF3; j++) {
			for (k = 0; k < TOTAL_DOF; k++) {
				Jp_lnkCoM_mj[i](j, k) = jacp[j * TOTAL_DOF + k];
				Jr_lnkCoM_mj[i](j, k) = jacr[j * TOTAL_DOF + k];
			}
		}
		//	Transform from MuJoCo coordinate to inertial coordinate
		Jp_lnkCoM_mj[i] = Jp_lnkCoM_mj[i] * Eigen::Transpose(X_O);
		Jr_lnkCoM_mj[i] = Jr_lnkCoM_mj[i] * Eigen::Transpose(X_O);


		/////	05. Time derivative of Link CoM Jacobian via numerical diff.
		Jdotp_lnkCoM_mj[i] = (Jp_lnkCoM_mj[i] - Pre_Jp_lnkCoM_mj[i]) / dT;
		Pre_Jp_lnkCoM_mj[i] = Jp_lnkCoM_mj[i];
	}

	/////	06. CoM Position of the system from MuJoCo !
	p_CoM_mj(0) = uData->subtree_com[start_BodyID * DOF3];
	p_CoM_mj(1) = uData->subtree_com[start_BodyID * DOF3 + 1];
	p_CoM_mj(2) = uData->subtree_com[start_BodyID * DOF3 + 2];


	/////	07. CoM Jacobian of the system from MuJoCo
	mj_jacSubtreeCom(uModel, uData, jacp, start_BodyID);
	for (j = 0; j < DOF3; j++) {
		for (k = 0; k < TOTAL_DOF; k++) {
			J_G_mj(j, k) = jacp[j * TOTAL_DOF + k];
		}
	}
	//	Transform from MuJoCo coordinate to inertial coordinate
	J_G_mj = J_G_mj * Eigen::Transpose(X_O);


	for (i = 0; i < id_body_EE.size(); i++) {
		/////	08. Compute End-effector position & rotation from MuJoCo !!
		for (j = 0; j < DOF3; j++) {
			p_EE_mj[i](j) = uData->site_xpos[i * DOF3 + j];						//	End-effecor Position from MuJoCo
			for (k = 0; k < DOF3; k++) {
				R_EE_mj[i](j, k) = uData->site_xmat[i * 9 + j * DOF3 + k];		//	End-effector Rotation from MuJoCo
			}
		}

		///// 09. End-Effector Jacobian & end-effector velocity from MuJoCo
		temp_vec3[0] = p_EE_mj[i](0);
		temp_vec3[1] = p_EE_mj[i](1);
		temp_vec3[2] = p_EE_mj[i](2);

		//	Linear & angular end-effector Jacobian matrix
		mj_jac(uModel, uData, jacp, jacr, temp_vec3, id_body_EE[i] + 1);
		for (j = 0; j < DOF3; j++) {
			for (k = 0; k < TOTAL_DOF; k++) {
				Jp_EE_mj[i](j, k) = jacp[j * TOTAL_DOF + k];
				Jr_EE_mj[i](j, k) = jacr[j * TOTAL_DOF + k];
			}
		}
		//	Transform from MuJoCo coordinate to inertial coordinate
		Jp_EE_mj[i] = Jp_EE_mj[i] * Eigen::Transpose(X_O);
		Jr_EE_mj[i] = Jr_EE_mj[i] * Eigen::Transpose(X_O);

		/////	10. Time derivative of end-effector Jacobian
		Jdotp_EE_mj[i] = (Jp_EE_mj[i] - Pre_Jp_EE_mj[i]) / dT;
		Jdotr_EE_mj[i] = (Jr_EE_mj[i] - Pre_Jr_EE_mj[i]) / dT;

		Pre_Jp_EE_mj[i] = Jp_EE_mj[i];
		Pre_Jr_EE_mj[i] = Jr_EE_mj[i];
	}


	/////	11. Compute Inertia Matrix
	mj_fullM(uModel, dense_M, uData->qM);
	// Mmat_mj = Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF>::CstMatrix('a', dense_M);
	int temp_k = 0;
	for (int i=0;i<TOTAL_DOF;i++)
	{
		for(int j=0;j<TOTAL_DOF;j++)
		{
			Mmat_mj(i,j) = dense_M[temp_k++];
		}
	}
	Mmat_mj = X_O * Mmat_mj * Eigen::Transpose(X_O);


	/////	12. Compute Gravity Force
	gvec_mj = mj_getTotalmass(uModel) * robot.getGravityConst() * J_G_mj.row(Z_AXIS);

	/////	13. Compute Coriolis & Centrifugl Forces
	for (i = 0; i < TOTAL_DOF; i++) {
		hvec_mj(i) = uData->qfrc_bias[i];
	}
	hvec_mj = X_O * hvec_mj - gvec_mj;

	////////////////////////////////////////////////////////////
	//	Check Error between MuJoCo & ARBML
	////////////////////////////////////////////////////////////
	if (count_ctrl % int(3.0 / dT) == 0) {
		//////////	01. Check Total Mass
		error = 0;
		sysReal m_G = robot.getTotalMass();
		error = max(error, (abs(m_G - mj_getTotalmass(uModel)) > error_precision ? 1 : 0));
		if (error == 1) {
			cout << "# 01 : System mass error = " << m_G - mj_getTotalmass(uModel) << endl;
		}

		//////////	02 : Kinematics(pose, Jacob and Jdot) of ALL body frames expressed in {I}
		for (i = 0; i < NO_OF_BODY; i++) {
			///	02-1. Pose(position & rotation)
			error = 0;
			robot.getLinkPose(i, p_lnk[i], R_lnk[i]);
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_lnk_mj[i](j) - p_lnk[i](j)) > error_precision ? 1 : 0));
				for (k = 0; k < DOF3; k++) {
					error = max(error, (abs(R_lnk_mj[i](j, k) - R_lnk[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 02-1. " << i << "-th Link : Position error (p_lnk_mj - p_lnk)" << endl;
				cout << (p_lnk_mj[i] - p_lnk[i])[2] << endl;
				cout << "# 02-1. " << i << "-th Link : Rotation error (R_lnk_mj - R_lnk)" << endl;
				cout << (R_lnk_mj[i] - R_lnk[i]) << endl;
			}


			///	02-2. Link Jacobian
			error = 0;
			robot.getBodyJacob(i, p_lnk[i], Jp_lnk[i], Jr_lnk[i]);
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jp_lnk_mj[i](j, k) - Jp_lnk[i](j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(Jr_lnk_mj[i](j, k) - Jr_lnk[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 02-2. " << i << "-th Link : Linear Jacobian error (Jp_lnk_mj - Jp_lnk)" << endl;
				cout << (Jp_lnk_mj[i] - Jp_lnk[i]) << endl;
				cout << "# 02-2. " << i << "-th Link : Angular Jacobian error (Jr_lnk_mj - Jr_lnk)" << endl;
				cout << (Jr_lnk_mj[i] - Jr_lnk[i]) << endl;
			}

			///	02-3. Time derivative of link Jacobian
			error = 0;
			robot.getBodyJacobDeriv(i, Jdotp_lnk[i], Jdotr_lnk[i]);
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jdotp_lnk_mj[i](j, k) - Jdotp_lnk[i](j, k)) > 0.01 ? 1 : 0));
					error = max(error, (abs(Jdotr_lnk_mj[i](j, k) - Jdotr_lnk[i](j, k)) > 0.01 ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 02-3. " << i << "-th Link : Time derivative of linear Jacobian error (Jdotp_lnk_mj - Jdotp_lnk)" << endl;
				cout << (Jdotp_lnk_mj[i] - Jdotp_lnk[i]) << endl;
				cout << "# 02-3. " << i << "-th Link : Time derivative of angular Jacobian error (Jdotr_lnk_mj - Jdotr_lnk)" << endl;
				cout << (Jdotr_lnk_mj[i] - Jdotr_lnk[i]) << endl;
			}
		}

		//////////	03 : Link CoM Kinematics (pose, J_CoM, Jdot_CoM) for ALL bodies expressed in {I}
		p_G.setZero();
		J_G.setZero();
		Jdot_G.setZero();
		for (i = 0; i < NO_OF_BODY; i++) {
			///	03-1. Link CoM pose(position & rotation)
			error = 0;
			robot.getLinkCoMPose(i, p_lnkCoM[i], R_lnkCoM[i]);
			p_G += robot.body[i].get_mass() * p_lnkCoM[i];
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_lnkCoM_mj[i](j) - p_lnkCoM[i](j)) > error_precision ? 1 : 0));
				for (k = 0; k < DOF3; k++) {
					error = max(error, (abs(R_lnkCoM_mj[i](j, k) - R_lnkCoM[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 03-1. " << i << "-th Link : CoM position error (p_lnkCoM_mj - p_lnkCoM)" << endl;
				cout << (p_lnkCoM_mj[i] - p_lnkCoM[i])[2] << endl;
				cout << "# 03-1. " << i << "-th Link : CoM rotation error (R_lnkCoM_mj - R_lnkCoM)" << endl;
				cout << (R_lnkCoM_mj[i] - R_lnkCoM[i]) << endl;
			}

			///	03-2. Link CoM Jacobian
			error = 0;
			robot.getBodyJacob(i, p_lnkCoM[i], J_lnkCoM[i]);
			J_G += robot.body[i].get_mass() * J_lnkCoM[i];
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jp_lnkCoM_mj[i](j, k) - J_lnkCoM[i](j, k)) > error_precision ? 1 : 0));
					// error = max(error, (abs(Jr_lnkCoM_mj[i](j, k) - Jr_lnk[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 03-2. " << i << "-th Link : Linear CoM Jacobian error (Jp_lnkCoM_mj - J_lnkCoM)" << endl;
				cout << (Jp_lnkCoM_mj[i] - J_lnkCoM[i]) << endl;
			}

			///	03-3. Time derivative of link CoM Jacobian
			error = 0;
			robot.getBodyJacobDeriv(i, Jdot_lnkCoM[i]);
			Jdot_G += robot.body[i].get_mass() * Jdot_lnkCoM[i];
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jdotp_lnkCoM_mj[i](j, k) - Jdot_lnkCoM[i](j, k)) > 0.01 ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 03-3. " << i << "-th Link : Time derivative of CoM Jacobian error (Jdot_lnkCoM_mj - Jdot_lnkCoM)" << endl;
				cout << (Jdotp_lnkCoM_mj[i] - Jdot_lnkCoM[i]) << endl;
			}
		}

		//////////	04 : CoM Kinematics of the System expressed in {I}
		{
			///	04-1. CoM pose of the system
			error = 0;
			p_G = p_G / m_G;
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_CoM_mj(j) - robot.p_CoM(j)) > error_precision ? 1 : 0));
				error = max(error, (abs(robot.p_CoM(j) - p_G(j)) > error_precision ? 1 : 0));
			}
			if (error == 1) {
				cout << "# 04-1. CoM position error (p_CoM_mj - p_CoM)" << endl;
				cout << (p_CoM_mj - robot.p_CoM) << endl;
				cout << "# 04-1. CoM position error (p_CoM - p_G)" << endl;
				cout << (robot.p_CoM - p_G) << endl;
			}

			///	04-2. CoM Jacobian of system
			error = 0;
			J_G = J_G / m_G;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(J_G_mj(j, k) - robot.J_CoM(j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(J_G_mj(j, k) - J_G(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 04-2. CoM Jacobian error (J_G_mj - J_CoM)" << endl;
				cout << (J_G_mj - robot.J_CoM) << endl;
				cout << "# 04-2. CoM Jacobian error (J_G_mj - J_G)" << endl;
				cout << (J_G_mj - J_G) << endl;
			}

			///	04-3. Time derivative of CoM Jacobian of system
			error = 0;
			Jdot_G = Jdot_G / m_G;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jdot_G(j, k) - robot.Jdot_CoM(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 04-3 : Time derivative CoM Jacobian error (Jdot_G -  Jdot_CoM)" << endl;
				cout << (Jdot_G - robot.Jdot_CoM) << endl;
			}
#ifdef _FLOATING_BASE
			error = 0;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(robot.C_p(j, k) - m_G * robot.Jdot_CoM(j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(Jdot_G(j, k) - robot.Jdot_CoM(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 04-3. Time derivative CoM Jacobian error (C_p -  m_G * Jdot_CoM)" << endl;
				cout << (robot.C_p - m_G * robot.Jdot_CoM) << endl;
				cout << "# 04-3. Time derivative CoM Jacobian error (Jdot_G -  Jdot_CoM)" << endl;
				cout << (Jdot_G - robot.Jdot_CoM) << endl;
			}
#endif
		}

		//////////	05 : End-effector Kinematics expressed in {I}
		for (i = 0; i < id_body_EE.size(); i++) {
			///	05-1. Global end-effector position & rotation
			error = 0;
			robot.getBodyPose(id_body_EE[i], p0_lnk2EE[i], R0_lnk2EE[i], p_EE[i], R_EE[i]);
			for (j = 0; j < DOF3; j++) {
				error = max(error, (abs(p_EE_mj[i](j) - p_EE[i](j)) > error_precision ? 1 : 0));
				for (k = 0; k < DOF3; k++) {
					error = max(error, (abs(R_EE_mj[i](j, k) - R_EE[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 05-1. " << i << "-th End-effector : Position error (p_EE_mj - p_EE)" << endl;
				cout << (p_EE_mj[i] - p_EE[i]) << endl;
				cout << "# 05-1. " << i << "-th End-effector : Rotation error (R_EE_mj - R_EE)" << endl;
				cout << (R_EE_mj[i] - R_EE[i]) << endl;
			}

			///	05-2. End-effector Jacobian expressed in {I}
			error = 0;
			robot.getBodyJacob(id_body_EE[i], p_EE[i], Jp_EE[i], Jr_EE[i]);
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jp_EE_mj[i](j, k) - Jp_EE[i](j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(Jr_EE_mj[i](j, k) - Jr_EE[i](j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 05-2. " << i << "-th End-effector : Linear Jacobian error (Jp_EE_mj - Jp_EE)" << endl;
				cout << (Jp_EE_mj[i] - Jp_EE[i]) << endl;
				cout << "# 05-2. " << i << "-th End-effector : Angular Jacobian error (Jr_EE_mj - Jr_EE)" << endl;
				cout << (Jr_EE_mj[i] - Jr_EE[i]) << endl;
			}

			///	05-3. Check Time derivative of End-effector Jacobian : ��� �� �ٸ� �������, �� �� ��Ȯ�ϰ�~~~
			error = 0;
			robot.getBodyJacobDeriv(id_body_EE[i], Jdotp_EE[i], Jdotr_EE[i]);
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Jdotp_EE_mj[i](j, k) - Jdotp_EE[i](j, k)) > 0.01 ? 1 : 0));
					error = max(error, (abs(Jdotr_EE_mj[i](j, k) - Jdotr_EE[i](j, k)) > 0.01 ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 05-3. " << i << "-th End-effector : Time derivative of linear Jacobian error (Jdotp_EE - Jdotp_EE_mj)" << endl;
				cout << (Jdotp_EE[i] - Jdotp_EE_mj[i]) << endl;
				cout << "# 05-3. " << i << "-th End-effector : Time derivative of angular Jacobian error (Jdotr_EE - Jdotr_EE_mj)" << endl;
				cout << (Jdotr_EE[i] - Jdotr_EE_mj[i]) << endl;
			}
		}

		//////////	06 : Joint Inertia Matrix (M_mat)
		{
			error = 0;
			for (j = 0; j < TOTAL_DOF; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Mmat_mj(j, k) - robot.M_mat(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 06 : Inertia matrix error (Mmat_mj - M_mat)" << endl;
				cout << (Mmat_mj - robot.M_mat) << endl;
			}
		}

		//////////	07. Gravity Forces Vector : g_vec
		{
			error = 0;
			for (j = 0; j < TOTAL_DOF; j++) {
				error = max(error, (abs(gvec_mj(j) - robot.g_vec(j)) > error_precision ? 1 : 0));
			}
			if (error == 1) {
				cout << "# 07 : Gravity force error (gvec_mj - gvec)" << endl;
				cout << (gvec_mj - robot.g_vec) << endl;
			}
		}

		//////////	08. Check Coriolis & Centrifugal forces : C_mat
		{
			error = 0;
			hvec = robot.C_mat * robot.xidot;
			for (j = 0; j < TOTAL_DOF; j++) {
				error = max(error, (abs(hvec_mj(j) - hvec(j)) > error_precision ? 1 : 0));
			}
			if (error == 1) {
				cout << "# 08 : Coriolis & Centrifugal force error (hvec_mj - C_mat * xidot)" << endl;
				cout << (hvec_mj - hvec) << endl;
			}
		}

#ifdef INERTIADOT
		//////////	09. Time derivative of Inertia matrix, Mdot = C + C^T
		{
			error = 0;
			for (j = 0; j < TOTAL_DOF; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(robot.Mdot_mat(j, k) - robot.C_mat(j, k) - robot.C_mat(k, j)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 09 : Mdot - C_mat - C_mat^T error" << endl;
				cout << (robot.Mdot_mat - robot.C_mat - Eigen::Transpose(robot.C_mat)) << endl;
			}
		}

#ifdef _FLOATING_BASE
		Eigen::Matrix<double, DOF3, TOTAL_DOF> Mdot_p, Mdot_o;

		Mdot_p = robot.Mdot_mat.block(0, 0, DOF3, TOTAL_DOF);
		Mdot_o = robot.Mdot_mat.block(DOF3, 0, DOF3, TOTAL_DOF);

		//////////	10 : Mdot_p = C_p = m_G * Jdot_CoM
		{
			error = 0;
			for (j = 0; j < DOF3; j++) {
				for (k = 0; k < TOTAL_DOF; k++) {
					error = max(error, (abs(Mdot_p(j, k) - robot.C_p(j, k)) > error_precision ? 1 : 0));
					error = max(error, (abs(m_G * Jdot_G(j, k) - robot.C_p(j, k)) > error_precision ? 1 : 0));
				}
			}
			if (error == 1) {
				cout << "# 10-1. Mdot_p - C_p" << endl;
				cout << (Mdot_p - robot.C_p) << endl;
				cout << "# 10-1. m_G * Jdot_G - C_p" << endl;
				cout << (m_G* Jdot_G - robot.C_p) << endl;
			}
		}

		//////////	11 : Mdot_o * xidot = (pdot_bG x M_p + C_o) * xidot !!!
		{
			error = 0;
			Eigen::Matrix3d Sk_pdot_BG = Skew(robot.pdot_B2CoM);
			tempVec3_1 = Mdot_o * robot.xidot;
			tempVec3_2 = (Sk_pdot_BG * robot.M_p + robot.C_o) * robot.xidot;
			for (j = 0; j < DOF3; j++) {
				error = max(error, (tempVec3_1(j) - tempVec3_2(j)) > error_precision ? 1 : 0);
			}
			if (error == 1) {
				cout << "# 11-1 : Mdot_o * xidot - (pdot_B2CoM x M_p + C_o) * xidot" << endl;
				cout << (tempVec3_1 - tempVec3_2) << endl;
			}

			error = 0;
			tempVec3_2 = Sk_pdot_BG * robot.M_p * robot.xidot;
			tempVec3_2(0) += hvec_mj(3);
			tempVec3_2(1) += hvec_mj(4);
			tempVec3_2(2) += hvec_mj(5);
			for (j = 0; j < DOF3; j++) {
				error = max(error, (tempVec3_1(j) - tempVec3_2(j)) > error_precision ? 1 : 0);
			}
			if (error == 1) {
				cout << "# 11-2 : Mdot_o * xidot - (Sk_pdot_BG * M_p * xidot + hvec_o_mj)" << endl;
				cout << (tempVec3_1 - tempVec3_2) << endl;
			}
		}
#endif
#endif
		cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl << endl;
	}

	/////	memory free !!!
	delete[] jacp;
	delete[] jacr;
	delete[] dense_M;
	delete[] temp_vec3;
}
