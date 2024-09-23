//
//  RobotConfig.h
//
//	It describes the information of the robot configuration.
//
//	Department : Cognitive & Collaborative Robotics Research Group
//				@ Korea Institute of Science and Technology (KIST)
// 
//	============================================================================
//
//	# NOTE
//	* KEY variables
//      _FLOATING_BASE : Flag for floating-base body system
//      NO_OF_ENDEFFECTOR : Number of end-effector
//      ACTIVE_DOF : = mjModel->njnt - 1 (Floaing-base body)
//      NO_OF_BODY : The number of total moving bodies = mjModel->nbody - 1
//      DOF_BASEBODY : Degrees of freedom of base body
//      TOTAL_DOF : Total DoF of the system
//	============================================================================
// 
//	# Revision History
//		* 2023.03 : Originated by Dr Yonghwan Oh
//
#pragma once

/////////////////////////////////////////////////
//	�����ϸ� �ǵ��� ���� ��~~
/////////////////////////////////////////////////
constexpr int DOF2 = 2;
constexpr int DOF3 = 3;
constexpr int DOF4 = 4;
constexpr int DOF6 = 6;

constexpr int ZMP_DOF = 3;
constexpr double g_const = 9.81;


/////////////////////////////////////////////////
//  User variables.
/////////////////////////////////////////////////

/////	Configuration Setup
// #define FILE_OUT
#define INERTIADOT
#define PRINT_MODEL_INFO
#define PRINT_END_EFFECTOR_INFO


/////////////////////////////////////////////////
//	Select Robot Model : Currently, 6-case !!!!
/////////////////////////////////////////////////
// #define YORI        //	YORI
// #define ArmHand		//  KIST 6-DoF Arm & Hand
// #define DualArm		//  KIST 7-DoF Dual Arm + Torso
#define DualArmHand	//  KIST 7-DoF Dual Arm + Hand/Gripper w Torso



/////   YORI    /////
#if defined(YORI)
constexpr int NO_OF_BASEBODY = 1;						   	//	Number of floating/fixed body
constexpr int NO_OF_ARM = 2;								//	Number of arm
constexpr int DOF_ARM = 4;									//	DOF of arm

#define _FLOATING_BASE
constexpr int NO_OF_ENDEFFECTOR = NO_OF_ARM;				//	Number of end-effector
constexpr int ACTIVE_DOF = DOF_ARM * NO_OF_ARM;             //  = mjModel->njnt - 1 (Floaing-base body)
constexpr int NO_OF_BODY = (NO_OF_BASEBODY + ACTIVE_DOF);   //  = mjModel->nbody - 1

/////   KIST 6-DoF Arm and Hand    /////
#elif defined(ArmHand)
constexpr int NO_OF_TORSO = 1;
constexpr int NO_OF_ARM = 1;
constexpr int NO_OF_HAND = 1;
constexpr int NO_OF_FINGER = 4;

constexpr int DOF_TORSO = 1;
constexpr int DOF_ARM = 6;
constexpr int DOF_FINGER = 4;
constexpr int DOF_HAND = (NO_OF_FINGER * DOF_FINGER);

constexpr int NO_OF_ENDEFFECTOR = (NO_OF_TORSO + NO_OF_ARM + NO_OF_HAND * NO_OF_FINGER);
constexpr int ACTIVE_DOF = (NO_OF_TORSO * DOF_TORSO + NO_OF_ARM * DOF_ARM + NO_OF_HAND * DOF_HAND);
constexpr int NO_OF_BODY = ACTIVE_DOF;

/////   KIST 7-DoF Dual Arm     /////
#elif defined(DualArm)
constexpr int NO_OF_ARM = 2;

constexpr int DOF_TORSO = 1;
constexpr int DOF_ARM = 7;

constexpr int NO_OF_ENDEFFECTOR = 2;
constexpr int ACTIVE_DOF = (NO_OF_ARM * DOF_ARM + DOF_TORSO);
constexpr int NO_OF_BODY = ACTIVE_DOF;

/////   KIST 7-DoF Dual Arm & Hand System w/ Prismatic Torso   /////
#elif defined(DualArmHand)
constexpr int NO_OF_ARM = 2;
constexpr int NO_OF_HAND = 2;
constexpr int NO_OF_R_FINGER = 4;
constexpr int NO_OF_L_FINGER = 2;

constexpr int DOF_TORSO = 0;
constexpr int DOF_ARM = 7;
constexpr int DOF_R_FINGER = 4;
constexpr int DOF_L_FINGER = 2;
constexpr int DOF_HAND = (NO_OF_R_FINGER * DOF_R_FINGER + NO_OF_L_FINGER * DOF_L_FINGER);

constexpr int NO_OF_ENDEFFECTOR = 4;
constexpr int ACTIVE_DOF = (DOF_HAND + NO_OF_ARM * DOF_ARM + DOF_TORSO);
constexpr int NO_OF_BODY = ACTIVE_DOF;


#endif


///////////////////////////////////////////////////////////////////////////
#ifdef _FLOATING_BASE
constexpr int DOF_BASEBODY = 6;	               		    //	Floating-base Body DoF
constexpr int DOF_BASEBODY_QUAT = 7;               		//	Floating-base Body DoF w/ quaternion

constexpr int TOTAL_DOF = (DOF_BASEBODY + ACTIVE_DOF);	//	= mjModel->nv
constexpr int TOTAL_DOF_QUAT = (TOTAL_DOF + 1);			//	Orientation is quaternion
#else
constexpr int DOF_BASEBODY = 0;						 	//	Fixed-base Body joint DoF
constexpr int DOF_BASEBODY_QUAT = 0;	                //	Fixed-base Body joint DoF w/ quaternion

constexpr int TOTAL_DOF = ACTIVE_DOF;                  	//	= mjModel->nv
constexpr int TOTAL_DOF_QUAT = TOTAL_DOF;
#endif // FLOATING_BASE