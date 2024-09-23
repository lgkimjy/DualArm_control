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

	sim_time = 0.0;
	K_qp.setZero();
	K_qv.setZero();
	qpos_ref.setZero();
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

	p_lEE_d.setZero();
	p_rEE_d.setZero();
	pdot_lEE_d.setZero();
	pdot_rEE_d.setZero();
	pddot_lEE_d.setZero();
	pddot_rEE_d.setZero();

	TaskCmd = TORQ_OFF;
	PrevTaskCmd = TORQ_OFF;

	qddot_cmd.setZero();
	torq_cmd.setZero();

	pre_qvel_d.setZero();

	initial_clik_flag = false;
	qpos_init.setZero();
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
	YAML::Node yaml_node;
#if defined(YORI)
	std::string path = CMAKE_SOURCE_DIR"/config/YORI.yaml";
#elif defined(ArmHand)
	std::string path = CMAKE_SOURCE_DIR"/config/ArmHand.yaml";
#elif defined(DualArm)
	std::string path = CMAKE_SOURCE_DIR"/config/DualArm.yaml";
#elif defined(DualArmHand)
	std::string path = CMAKE_SOURCE_DIR"/config/DualArmHand.yaml";
#endif
	yaml_node = YAML::LoadFile(path.c_str());

	try
	{
		for(int i=0; i<ACTIVE_DOF; i++)
		{
			K_qp(i, i) = yaml_node["PD_gain"]["kp"][i].as<double>();
			K_qv(i, i) = yaml_node["PD_gain"]["kd"][i].as<double>();
			qpos_ref(i) = D2R(yaml_node["initial_pos"][i].as<double>());
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Config File Parse ERROR" << std::endl;
	}
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
		mapEEvar();

		/////	Compute Control Input & Output Control Input
		computeControlInput(data);

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
	sim_time = data->time;
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

	if(sim_time < TORQUE_ON + INITIAL_POSE) {
		data->mocap_pos[0] = p_EE[0](0);
		data->mocap_pos[1] = p_EE[0](1);
		data->mocap_pos[2] = p_EE[0](2);
		data->mocap_quat[0] = _Rot2Quat(R_EE[0])[0];
		data->mocap_quat[1] = _Rot2Quat(R_EE[0])[1];
		data->mocap_quat[2] = _Rot2Quat(R_EE[0])[2];
		data->mocap_quat[3] = _Rot2Quat(R_EE[0])[3];
	// }

	// if(sim_time < TORQUE_ON + INITIAL_POSE) {
		data->mocap_pos[3] = p_EE[1](0);
		data->mocap_pos[4] = p_EE[1](1);
		data->mocap_pos[5] = p_EE[1](2);
		data->mocap_quat[4] = _Rot2Quat(R_EE[1])[0];
		data->mocap_quat[5] = _Rot2Quat(R_EE[1])[1];
		data->mocap_quat[6] = _Rot2Quat(R_EE[1])[2];
		data->mocap_quat[7] = _Rot2Quat(R_EE[1])[3];
	}
}

void CRobotControl::mapEEvar()
{
	p_lEE = p_EE[0];
	R_lEE = R_EE[0];
	p_rEE = p_EE[1];
	R_rEE = R_EE[1];

	pdot_lEE = pdot_EE[0];
	omega_lEE = omega_EE[0];
	pdot_rEE = pdot_EE[1];
	omega_rEE = omega_EE[1];
}

void CRobotControl::getUserCommand(mujoco::Simulate& sim)
{
	if(sim.mode == 1) {
		TaskCmd = TORQ_ON;
	} else if(sim.mode == 2) {
		TaskCmd = READY;
	} else {
		TaskCmd = TORQ_ON;
	}
}

void CRobotControl::computeControlInput(mjData* data)
{
	if(sim_time < TORQUE_ON){
		p_lEE_d = p_lEE;
		p_rEE_d = p_rEE;
		computeJointTorque(JOINT_PD);
	}
	else if(sim_time < TORQUE_ON + INITIAL_POSE) {
		p_lEE_d = p_lEE;
		p_rEE_d = p_rEE;
		JointPlanner(INITIAL_POSE);
		computeJointTorque(JOINT_PD);
	}
	else
	{	
		// LeftEEPlanner(14.0);
		// RightEEPlanner(14.0);
		// CLIK(data);
		// if(sim_time > 18.0)	computeJointTorque(GRAV_COMP);
		// else				computeJointTorque(INV_DYN);

		OperationalSpaceControl(data);
		computeJointTorque(TORQUE);		// JOINT_PD, TORQUE, GRAV_COMP, INV_DYN

		// NullSpacePlanner();
		// OptimalControl();
		// computeJointTorque(TORQUE);		// JOINT_PD, TORQUE, GRAV_COMP, INV_DYN
		lEE_d.add(sim_time, p_lEE_d(0), p_lEE_d(1), p_lEE_d(2), pdot_lEE_d(0), pdot_lEE_d(1), pdot_lEE_d(2));
	}
}

void CRobotControl::JointPlanner(double duration)
{
	if(Joint_Traj.is_moving_ == false)
	{
		std::cout << "[" << sim_time << "] At JointPlanner" << std::endl;
		Joint_Traj.setTargetPosition(robot.q, qpos_ref, duration, 1 / 1000.0, CUBIC);
	}
	Joint_Traj.computeTraj(qpos_d, qvel_d, qacc_d);
}

void CRobotControl::LeftEEPlanner(double duration)
{
	// if(lEE_Traj.is_moving_ == false)
	// {
	// 	std::cout << "[" << sim_time << "] At LeftEEPlanner" << std::endl;
	// 	lEE_Traj.setTargetPosition(p_EE[0], p_EE[0] + Eigen::Vector3d(0.3, -0.02, 0.25), duration, 1 / 1000.0, CUBIC);
	// }
	// lEE_Traj.computeTraj(p_lEE_d, pdot_lEE_d, pddot_lEE_d);

	if(lEE_local_t == 0.0) {
		std::cout << "[" << sim_time << "] At LeftEEPlanner" << std::endl;
		vector<Eigen::Vector3d> ctrl_pts = {p_lEE, {0.3, 0.1, 0.7}, {0.3, 0.3, 0.9}, {0.3, 0.5, 0.7},  {0.3, 0.3, 0.5}, 
													{0.3, 0.1, 0.7}, {0.3, 0.3, 0.9}, {0.3, 0.5, 0.7}, {0.3, 0.3, 0.5}, 
													{0.3, 0.1, 0.7}, {0.3, 0.3, 0.9}, {0.3, 0.5, 0.7}, {0.3, 0.3, 0.5}, 
													{0.3, 0.1, 0.7}, {0.3, 0.3, 0.9}, {0.3, 0.5, 0.7}, {0.3, 0.3, 0.5}, 
													{0.3, 0.1, 0.7}, {0.3, 0.3, 0.9}, {0.3, 0.5, 0.7}, {0.3, 0.3, 0.5}, 
													{0.3, 0.1, 0.7}, {0.3, 0.3, 0.9}, {0.3, 0.5, 0.7}, {0.3, 0.3, 0.5}, 
													{0.3, 0.1, 0.7}, {0.3, 0.3, 0.9}, {0.3, 0.5, 0.7}, {0.3, 0.3, 0.5}, p_lEE};
		int SpType = CLAMPED_SPLINE;
		lEE_CubicSpline_Traj.initSpline(duration, ctrl_pts, SpType);
	}
	p_lEE_d = lEE_CubicSpline_Traj.getPositionAt(lEE_local_t);
	pdot_lEE_d = lEE_CubicSpline_Traj.getVelocityAt(lEE_local_t);
	pddot_lEE_d = lEE_CubicSpline_Traj.getAccelerationAt(lEE_local_t);
	lEE_local_t += robot.getSamplingTime();
}

void CRobotControl::RightEEPlanner(double duration)
{
	// if(rEE_Traj.is_moving_ == false)
	// {
	// 	std::cout << "[" << sim_time << "] At RightEEPlanner" << std::endl;
	// 	rEE_Traj.setTargetPosition(p_EE[1], p_EE[1] + Eigen::Vector3d(0.3, 0.02, 0.25), duration, 1 / 1000.0, CUBIC);
	// }
	// rEE_Traj.computeTraj(p_rEE_d, pdot_rEE_d, pddot_rEE_d);

	if(rEE_local_t == 0.0) {
		std::cout << "[" << sim_time << "] At RightEEPlanner" << std::endl;
		vector<Eigen::Vector3d> ctrl_pts = {p_rEE, {0.3, -0.1, 0.6}, {0.3, -0.3, 0.8}, {0.3, -0.5, 0.6},  {0.3, -0.3, 0.4}, 
													{0.3, -0.1, 0.6}, {0.3, -0.3, 0.8}, {0.3, -0.5, 0.6}, {0.3, -0.3, 0.4}, 
													{0.3, -0.1, 0.6}, {0.3, -0.3, 0.8}, {0.3, -0.5, 0.6}, {0.3, -0.3, 0.4}, 
													{0.3, -0.1, 0.6}, {0.3, -0.3, 0.8}, {0.3, -0.5, 0.6}, {0.3, -0.3, 0.4}, 
													{0.3, -0.1, 0.6}, {0.3, -0.3, 0.8}, {0.3, -0.5, 0.6}, {0.3, -0.3, 0.4}, 
													{0.3, -0.1, 0.6}, {0.3, -0.3, 0.8}, {0.3, -0.5, 0.6}, {0.3, -0.3, 0.4}, 
													{0.3, -0.1, 0.6}, {0.3, -0.3, 0.8}, {0.3, -0.5, 0.6}, {0.3, -0.3, 0.4}, p_rEE};
		int SpType = CLAMPED_SPLINE;
		rEE_CubicSpline_Traj.initSpline(duration, ctrl_pts, SpType);
	}
	p_rEE_d = rEE_CubicSpline_Traj.getPositionAt(rEE_local_t);
	pdot_rEE_d = rEE_CubicSpline_Traj.getVelocityAt(rEE_local_t);
	pddot_rEE_d = rEE_CubicSpline_Traj.getAccelerationAt(rEE_local_t);
	rEE_local_t += robot.getSamplingTime();
}

void CRobotControl::OperationalSpaceControl(mjData* data)
{
	if(initial_clik_flag == false)
	{
		qpos_init = robot.q;
		qpos_d = qpos_init;
		initial_clik_flag = true;
	}

	Eigen::MatrixXd InvJ;

	// Eigen::Matrix<double, 6, TOTAL_DOF> Aug_Jp_EE;
	// Eigen::Matrix<double, 6, 1> X_cmd;
	// Eigen::Matrix<double, 6, 1> Xdot_cmd;
	// Eigen::Matrix<double, 6, 1> X_curr;
	// Eigen::Matrix<double, 6, 1> Xdot_curr;
	// Eigen::Matrix<double, 6, 1> X_err;
	// Eigen::Matrix<double, 6, 1> Xdot_err;
	// Aug_Jp_EE.block(0, 0, 3, TOTAL_DOF) = Jp_EE[0];
	// Aug_Jp_EE.block(3, 0, 3, TOTAL_DOF) = Jp_EE[1];

	// X_cmd(0) = data->mocap_pos[0];
	// X_cmd(1) = data->mocap_pos[1];
	// X_cmd(2) = data->mocap_pos[2];
	// X_cmd(3) = data->mocap_pos[3];
	// X_cmd(4) = data->mocap_pos[4];
	// X_cmd(5) = data->mocap_pos[5];

	// Xdot_cmd.segment<3>(0).setZero();
	// Xdot_cmd.segment<3>(3).setZero();

	// X_curr.segment<3>(0) = p_lEE;
	// X_curr.segment<3>(3) = p_rEE;
	// Xdot_curr.segment<3>(0) = pdot_lEE;
	// Xdot_curr.segment<3>(3) = pdot_rEE;

	// X_err = X_cmd - X_curr;
	// Xdot_err = Xdot_cmd - Xdot_curr;

	// NullCtrl.svd_pseudoInverse(Aug_Jp_EE, InvJ);

	// // torq_cmd = robot.M_mat * Aug_Jp_EE.transpose() * (250 * Xdot_err + 1500 * X_err);		// jacobian transpose
	// torq_cmd = robot.M_mat * InvJ * (250 * Xdot_err + 1500 * X_err);							// jacobian pseudo-inverse
	// torq_cmd += robot.C_mat * robot.qdot + robot.g_vec;										// operational space control (only control position of end-effector)
	
	// Eigen::Matrix<double, ACTIVE_DOF, ACTIVE_DOF> I34;
	// I34.setIdentity();
	// torq_cmd += (I34 - InvJ * Aug_Jp_EE) * (K_qp * (qpos_init - robot.q) - K_qv * robot.qdot);	// Null space control


	Eigen::Matrix<double, 12, TOTAL_DOF> Aug_Jp_EE;
	Eigen::Matrix<double, 12, 1> X_cmd;
	Eigen::Matrix<double, 12, 1> Xdot_cmd;
	Eigen::Matrix<double, 12, 1> X_curr;
	Eigen::Matrix<double, 12, 1> Xdot_curr;
	Eigen::Matrix<double, 12, 1> X_err;
	Eigen::Matrix<double, 12, 1> Xdot_err;
	Aug_Jp_EE.block(0, 0, 3, TOTAL_DOF) = Jp_EE[0];
	Aug_Jp_EE.block(3, 0, 3, TOTAL_DOF) = Jr_EE[0];
	Aug_Jp_EE.block(6, 0, 3, TOTAL_DOF) = Jp_EE[1];
	Aug_Jp_EE.block(9, 0, 3, TOTAL_DOF) = Jr_EE[1];

	X_cmd.setZero();
	X_cmd(0) = data->mocap_pos[0];
	X_cmd(1) = data->mocap_pos[1];
	X_cmd(2) = data->mocap_pos[2];
	X_cmd(6) = data->mocap_pos[3];
	X_cmd(7) = data->mocap_pos[4];
	X_cmd(8) = data->mocap_pos[5];

	Xdot_cmd.segment<3>(0).setZero();
	Xdot_cmd.segment<3>(3).setZero();
	Xdot_cmd.segment<3>(6).setZero();
	Xdot_cmd.segment<3>(9).setZero();

	X_curr.setZero();
	Xdot_curr.setZero();
	X_curr.segment<3>(0) = p_lEE;
	X_curr.segment<3>(6) = p_rEE;
	Xdot_curr.segment<3>(0) = pdot_lEE;
	Xdot_curr.segment<3>(6) = pdot_rEE;

	X_err = X_cmd - X_curr;
	Xdot_err = Xdot_cmd - Xdot_curr;

	Eigen::Vector4d quat_tmp;
	quat_tmp << data->mocap_quat[0], data->mocap_quat[1], data->mocap_quat[2], data->mocap_quat[3];
    Eigen::Matrix3d errorMatrix = _Quat2Rot(quat_tmp) * R_lEE.transpose();
    Eigen::AngleAxisd errorAngleAxis(errorMatrix);
	errorAngleAxis.angle() * errorAngleAxis.axis().transpose();
	X_err.segment<3>(3) = errorAngleAxis.angle() * errorAngleAxis.axis().transpose();

	quat_tmp << data->mocap_quat[4], data->mocap_quat[5], data->mocap_quat[6], data->mocap_quat[7];
	errorMatrix = _Quat2Rot(quat_tmp) * R_rEE.transpose();
	errorAngleAxis = Eigen::AngleAxisd(errorMatrix);

	X_err.segment<3>(9) = errorAngleAxis.angle() * errorAngleAxis.axis().transpose();
	Xdot_err.segment<3>(3) = -omega_lEE;
	Xdot_err.segment<3>(9) = -omega_rEE;

	NullCtrl.svd_pseudoInverse(Aug_Jp_EE, InvJ);

	// torq_cmd = robot.M_mat * Aug_Jp_EE.transpose() * (250 * Xdot_err + 1500 * X_err);		// jacobian transpose
	torq_cmd = robot.M_mat * InvJ * (250 * Xdot_err + 1500 * X_err);							// jacobian pseudo-inverse
	torq_cmd += robot.C_mat * robot.qdot + robot.g_vec;											// operational space control (6D)
	
	Eigen::Matrix<double, ACTIVE_DOF, ACTIVE_DOF> I34;
	I34.setIdentity();
	torq_cmd += (I34 - InvJ * Aug_Jp_EE) * (K_qp * (qpos_init - robot.q) - K_qv * robot.qdot);	// added for null space control
}

void CRobotControl::CLIK(mjData* data)
{
	if(initial_clik_flag == false)
	{
		qpos_init = robot.q;
		qpos_d = qpos_init;
		initial_clik_flag = true;
	}

	Eigen::MatrixXd InvJ;

	Eigen::Matrix<double, 12, TOTAL_DOF> Aug_Jp_EE;
	Eigen::Matrix<double, 12, 1> X_cmd;
	Eigen::Matrix<double, 12, 1> Xdot_cmd;
	Eigen::Matrix<double, 12, 1> X_curr;
	Aug_Jp_EE.block(0, 0, 3, TOTAL_DOF) = Jp_EE[0];
	Aug_Jp_EE.block(3, 0, 3, TOTAL_DOF) = Jr_EE[0];
	Aug_Jp_EE.block(6, 0, 3, TOTAL_DOF) = Jp_EE[1];
	Aug_Jp_EE.block(9, 0, 3, TOTAL_DOF) = Jr_EE[1];

	X_cmd(0) = data->mocap_pos[0];
	X_cmd(1) = data->mocap_pos[1];
	X_cmd(2) = data->mocap_pos[2];
	Eigen::Vector3d temp1 = _Rot2Eul(_Quat2Rot({data->mocap_quat[0], data->mocap_quat[1], data->mocap_quat[2], data->mocap_quat[3]}));
	X_cmd.segment<3>(3) = temp1;
	X_cmd(6) = data->mocap_pos[3];
	X_cmd(7) = data->mocap_pos[4];
	X_cmd(8) = data->mocap_pos[5];
	Eigen::Vector3d temp2 = _Rot2Eul(_Quat2Rot({data->mocap_quat[4], data->mocap_quat[5], data->mocap_quat[6], data->mocap_quat[7]}));
	X_cmd.segment<3>(9) = temp2;

	Xdot_cmd.segment<3>(0).setZero();
	Xdot_cmd.segment<3>(3).setZero();
	Xdot_cmd.segment<3>(6).setZero();
	Xdot_cmd.segment<3>(9).setZero();

	// R_lEE_d = R_lEE_d.setIdentity() * _Rot_Z(D2R(90)) * _Rot_Y(D2R(-90));
	// R_rEE_d = R_rEE_d.setIdentity() * _Rot_Z(D2R(90)) * _Rot_Y(D2R(-90));

	X_curr.segment<3>(0) = p_lEE;
	// X_curr.segment<3>(3) = getPhi(R_lEE, R_lEE_d);
	X_curr.segment<3>(3) = _Rot2Eul(R_lEE);
	X_curr.segment<3>(6) = p_rEE;
	// X_curr.segment<3>(9) = getPhi(R_rEE, R_rEE_d);
	X_curr.segment<3>(9) = _Rot2Eul(R_rEE);

	NullCtrl.svd_pseudoInverse(Aug_Jp_EE, InvJ);
	
	qpos_d += InvJ * (Xdot_cmd + 10 * (X_cmd - X_curr)) * robot.getSamplingTime();
	torq_cmd = robot.M_mat * (40 * (InvJ * (Xdot_cmd + 10 * (X_cmd - X_curr)) - robot.qdot) + 1000 * (qpos_d - robot.q)) + robot.C_mat * robot.qdot + robot.g_vec;
	
	// base body joint pd control
	// qpos_d(0) = 0.0;
	// qvel_d(0) = 0.0;
	// torq_cmd(0) = 20000 * (qpos_d(0) - robot.q(0)) + 2000 * (qvel_d(0) - robot.qdot(0));
}


void CRobotControl::NullSpacePlanner()
{
	// NullCtrl.clearTask();

	// //// Task 0 : Set Contact Jacobian w/ 4 point contact & no swing trajectory
	// // NullCtrl.setContactJacobianSize(1);
	// // NullCtrl.J_c.block(0, 0, 3, TOTAL_DOF) = Jp_EE[1];	// right toe
	// // NullCtrl.Jdot_c.block(0, 0, 3, TOTAL_DOF) = Jdotp_EE[1];

	// // assignSelectedJointTask({0});
	Eigen::Vector3d X_ee_cmd = pddot_lEE_d + 20 * (pdot_lEE_d - pdot_lEE) + 1 * (p_lEE_d - p_lEE);
	// NullCtrl.assignTask(3, Jp_EE[0], Jdotp_EE[0], p_EE[0], p_lEE_d, pdot_lEE_d, X_ee_cmd);
	// // NullCtrl.assignTask(3, Jp_EE[1], Jdotp_EE[1], p_EE[1], p_rEE_d, pdot_rEE_d, pddot_rEE_d);

	// NullCtrl.computeNullSpaceControl(robot.xidot);
	// qpos_d = NullCtrl.qpos_d;
	// qvel_d = NullCtrl.qvel_d;
	// qacc_d = NullCtrl.qacc_d;
	// std::cout << "[" << sim_time << "] qpos_d : " << qpos_d.transpose() << std::endl;
	// std::cout << "[" << sim_time << "] qvel_d : " << qvel_d.transpose() << std::endl;
	// std::cout << "[" << sim_time << "] qacc_d : " << qacc_d.transpose() << std::endl << std::endl;
	Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> N;
	Eigen::MatrixXd Inv_J1;
	Eigen::MatrixXd Inv_J2;
	NullCtrl.svd_pseudoInverse(Jp_EE[0], Inv_J1);
	NullCtrl.BuildProjectionMatrix(Jp_EE[0], N);
	NullCtrl.svd_pseudoInverse(Jp_EE[1] * N, Inv_J2);
	
	qvel_d = Inv_J1 * pdot_lEE_d + Inv_J2 * (pdot_rEE_d - Jp_EE[1] * Inv_J1 * pdot_lEE_d);
	// qpos_d = qpos_d + qvel_d * robot.getSamplingTime();
	qacc_d = (qvel_d - pre_qvel_d) / robot.getSamplingTime();

	// qacc_d = Inv_J2 * (X_ee_cmd - Jdotp_EE[1] * robot.xidot);

	pre_qvel_d = qvel_d;
	qddot_cmd = qacc_d;
}

void CRobotControl::OptimalControl()
{
    int numConstraints = 10;
    int numDecisionVars = 10;

    // Constraints (inequality, equality)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, numDecisionVars);
    Eigen::VectorXd lbA(numConstraints), ubA(numConstraints);  // clang-format off
    // A << constraints.a_,
    //     constraints.d_;

    // lbA << constraints.b_,
    //         constraints.f_;
    // ubA << constraints.b_,
    //         qpOASES::INFTY * Eigen::VectorXd::Ones(constraints.f_.size());

    // Objective Function (Cost)
    // WBCTask weightTasks = formulateWeightedTask();
    // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weightTasks.a_.transpose() * weightTasks.a_;
    // Eigen::VectorXd g = -weightTasks.a_.transpose() * weightTasks.b_;

    // Solve
    auto qpProblem = qpOASES::QProblem(numDecisionVars, numConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    options.enableEqualities = qpOASES::BT_TRUE;
    qpProblem.setOptions(options);
    int nWsr = 50;

    // qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
    Eigen::VectorXd qpSol(numDecisionVars);
    qpProblem.getPrimalSolution(qpSol.data());

	torq_cmd.setZero();	
}

void CRobotControl::computeJointTorque(CtrlType type)
{
	if(type == JOINT_PD) {
		// joint_torq = K_qp * (qpos_d - robot.q) + K_qv * (qvel_d - robot.qdot);
		joint_torq = qacc_d + K_qp * (qpos_d - robot.q) + K_qv * (qvel_d - robot.qdot);
	} else if(type == INV_DYN) {
		joint_torq = robot.M_mat * qddot_cmd + robot.C_mat * robot.qdot + robot.g_vec;
	}
	else if(type == GRAV_COMP) 
	{
		qpos_d = robot.q;
		qvel_d = robot.qdot;
		joint_torq = robot.M_mat * (500 * (qpos_d - robot.q) + 20 * (qvel_d - robot.qdot)) + robot.C_mat * robot.qdot + robot.g_vec;
	}
	else if(type == TORQUE)
	{
		joint_torq = torq_cmd;
	}
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
