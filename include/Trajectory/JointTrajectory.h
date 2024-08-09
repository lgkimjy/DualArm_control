//
//	initPos   : Current Joint Position(rad)
//	targetPos : target Joint Position(rad)
//	velocity : for get via point (rad/s)

#pragma once

#include "ARBMLlib/CommUtil.h"

using namespace std;
using namespace Eigen;


enum TrajectoryType_list
{
	LINEAR = 0,
	CUBIC,
	QUINTIC,
	COSINE
};



template <unsigned DIM, typename T = sysReal>
class CP2P_Traj
{
private:
	int		TrajType;
	sysReal dT, local_t, Time_End;
	sysReal w_n;

	Eigen::Matrix<sysReal, DIM, 1> c0, c1, c2, c3, c4, c5;
	Eigen::Matrix<sysReal, DIM, 1> pos_init, pos_end;
	Eigen::Matrix<sysReal, DIM, 1> vel_init, vel_end;
	Eigen::Matrix<sysReal, DIM, 1> acc_init, acc_end;

public:

	bool is_moving_ = false;
	CP2P_Traj() : dT(0), local_t(0), Time_End(0), TrajType(0), w_n(0)
	{
		c0.setZero();
		c1.setZero();
		c2.setZero();
		c3.setZero();
		c4.setZero();
		c5.setZero();

		pos_init.setZero();
		vel_init.setZero();
		acc_init.setZero();

		pos_end.setZero();
		vel_end.setZero();
		acc_end.setZero();
	}
	~CP2P_Traj() {}

	void setTargetPosition(const Eigen::Matrix<sysReal, DIM, 1>& q_i, const Eigen::Matrix<sysReal, DIM, 1>& q_f,
						const sysReal& Time_Exec, const sysReal& del_T, const int& trjType);

	void computeTraj(Eigen::Matrix<sysReal, DIM, 1>& q_d, Eigen::Matrix<sysReal, DIM, 1>& qdot_d, Eigen::Matrix<sysReal, DIM, 1>& qddot_d);
};


//============================== Class Implementation ==============================//

////////////////////////////////////////////////////////////////////////////////
/////	Initialize Joint Trajectory (P2P)
////////////////////////////////////////////////////////////////////////////////
template<unsigned DIM, typename T>
inline void CP2P_Traj<DIM, T>:: setTargetPosition(const Matrix<sysReal, DIM, 1>& q_i, const Matrix<sysReal, DIM, 1>& q_f,
												const sysReal& Time_Exec, const sysReal& del_T, const int& trjType)
{
	local_t = 0;

	dT = del_T;
	Time_End = Time_Exec;

	pos_init = q_i;
	pos_end = q_f;

	TrajType = trjType;

	switch (TrajType) {
	case LINEAR:
		c0 = q_i;
		c1 = (q_f - q_i) / Time_End;
		break;

	case CUBIC:
		c0 = q_i;
		c2 = 3 * (q_f - q_i) / SQR(Time_End);
		c3 = 2 * (q_i - q_f) / CUBE(Time_End);
		break;

	case QUINTIC:
		c0 = q_i;
		c2.setZero();
		c3 = 10 * (q_f - q_i) / CUBE(Time_End);
		c4 = 15 * (q_i - q_f) / (SQR(Time_End) * SQR(Time_End));
		c5 = 6 * (q_f - q_i) / (SQR(Time_End) * CUBE(Time_End));
		break;

	case COSINE:
		c0 = (q_f - q_i) / 2;
		w_n = M_PI / Time_End;
		break;
	}

	is_moving_ = true;
}



template<unsigned DIM, typename T>
inline void CP2P_Traj<DIM, T>::computeTraj(Eigen::Matrix<sysReal, DIM, 1>& q_d, Eigen::Matrix<sysReal, DIM, 1>& qdot_d,
										Eigen::Matrix<sysReal, DIM, 1>& qddot_d)
{
	if (local_t < Time_End) {
		local_t += dT;

		switch (TrajType) {
		case LINEAR:		//	Linear interpolation
			q_d = c0 + c1 * local_t;
			qdot_d = c1;
			qddot_d.setZero();
			break;

		case CUBIC:			//	3rd order time polynomial
			q_d = c0 + c1 * local_t + c2 * SQR(local_t) + c3 * CUBE(local_t);
			qdot_d = c1 + 2 * c2 * local_t + 3 * c3 * SQR(local_t);
			qddot_d = 2 * c2 + 6 * c3 * local_t;
			break;

		case QUINTIC:		//	5th order time polynomial
			q_d = c0 + c1 * local_t + (c2 + c3 * local_t + c4 * SQR(local_t) + c5 * CUBE(local_t)) * SQR(local_t);
			qdot_d = c1 + 2 * c2 * local_t + (3 * c3 + 4 * c4 * local_t + 5 * c5 * SQR(local_t)) * SQR(local_t);
			qddot_d = 2 * c2 + 6 * c3 * local_t + 12 * c4 * SQR(local_t) + 20 * c5 * CUBE(local_t);
			break;

		case COSINE:		//	Cosine function
			q_d = c0 * (1 - cos(w_n * local_t)) + pos_init;
			qdot_d = c0 * w_n * sin(w_n * local_t);
			qddot_d = c0 * SQR(w_n) * cos(w_n * local_t);
			break;
		}
	}
}

