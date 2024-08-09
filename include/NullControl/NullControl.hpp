#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "ARBMLlib/ARBML.h"
#include "NullTask.hpp"

#define RESET "\033[0m"
#define RED "\033[31m"  /* Red */
#define BLUE "\033[34m" /* Blue */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define CYAN "\033[36m" /* Cyan */
#define MAGENTA "\033[35m" /* Magenta */

// #define FUNC_TAG "[" << __func__ << "] "
// const Eigen::IOFormat fmt(Eigen::StreamPrecision, 0, ", ", "\n", "[ ", " ]");
// const Eigen::IOFormat full_fmt(Eigen::FullPrecision, 0, ", ", "\n", "[ ", " ]");

typedef enum{
  POSITION		= 0X00, // Joint PD
  ORIENTATION   = 0X01, // Purly Inverse Dynamics calculated torque
  JOINT     	= 0X02, // Joint PD + feedforward torque
  MIXED     	= 0X03, // Cartesain PD + feedforward torque
}TaskTypeDef;

namespace Null
{

class NullSpaceControl
{
private:
public:

    int n_tasks;
	Eigen::MatrixXd 	J_c;            // Contact Jacobian
	Eigen::MatrixXd 	Jdot_c;         // Contact Jacobian derivative
	std::vector<Task> 	tasks;

    Eigen::VectorXd 	delta_q;
    Eigen::VectorXd 	qdot;
    Eigen::VectorXd 	qddot;

    // Eigen::VectorXd     xi_d;
    // Eigen::VectorXd     xidot_d;
    // Eigen::VectorXd     xiddot_d;
    Eigen::VectorXd     qpos_d;
    Eigen::VectorXd     qvel_d;
    Eigen::VectorXd     qacc_d;
    Eigen::Matrix<double, TOTAL_DOF, 1> qddot_cmd;
    Eigen::Matrix<double, TOTAL_DOF, 1> qdot_cmd;
    Eigen::Matrix<double, TOTAL_DOF, 1> q_cmd;

    static constexpr int dim = 0;
    static constexpr int active_dim = 0;

    // functions
    NullSpaceControl();
    ~NullSpaceControl();

    void setContactJacobianSize(int size);
    void clearTask();
    void assignTask(int task_DoF, Eigen::MatrixXd J, Eigen::MatrixXd Jdot, Eigen::VectorXd x, Eigen::VectorXd x_d, Eigen::VectorXd xdot_d, Eigen::VectorXd xddot_d);
    void assignJointTask(std::vector<int> joint_task, Eigen::VectorXd x, Eigen::VectorXd x_d, Eigen::VectorXd xdot_d, Eigen::VectorXd xddot_d);
    void computeNullSpaceControl(Eigen::Matrix<double, TOTAL_DOF, 1> xidot);
    void checkComputation();

    void BuildProjectionMatrix(const Eigen::MatrixXd & J, Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> & N);
	void svd_pseudoInverse(Eigen::MatrixXd Matrix, Eigen::MatrixXd& invMatrix);
};

}