#ifndef ROBOT_TASK_HPP_
#define ROBOT_TASK_HPP_

typedef enum {
	STAY	 	= 0x00,
	READY		= 0X01,
	G_COMPENSATE= 0X02,
	CL_IK		= 0X03,
	OCS_POS		= 0X04,
	OCS_POSE	= 0X05,
	BIMANUAL	= 0X06,
}TaskCmdType;

typedef enum {
	JOINT_PD 	= 0X01,
	INV_DYN	 	= 0X02,
	TORQUE		= 0X03,
	GRAV_COMP	= 0X04,
}CtrlType;

#endif