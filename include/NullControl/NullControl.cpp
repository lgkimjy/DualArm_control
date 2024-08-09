#include "NullControl.hpp"

namespace Null
{
    NullSpaceControl::NullSpaceControl()
    {
        n_tasks = 0;
    }

    NullSpaceControl::~NullSpaceControl()
    {
        std::cout << RED << "[ NullSpaceControl ] " << "NullSpaceControl Destructor Called" << RESET << std::endl;
    }

    void NullSpaceControl::setContactJacobianSize(int size)
    {
        J_c = Eigen::MatrixXd(3 * size, TOTAL_DOF).setZero();
        Jdot_c = Eigen::MatrixXd(3 * size, TOTAL_DOF).setZero();
    }

    void NullSpaceControl::clearTask()
    {
        tasks.clear();
        n_tasks = 0;
    }

    void NullSpaceControl::assignTask(int task_DoF, Eigen::MatrixXd J, Eigen::MatrixXd Jdot, Eigen::VectorXd x, \
                                    Eigen::VectorXd x_d, Eigen::VectorXd xdot_d, Eigen::VectorXd xddot_d)
    {
        Task task;
        task.setTaskSize(task_DoF, TOTAL_DOF);

        task.J = J;
        task.Jdot = Jdot;
        task.x = x;
        task.x_d = x_d;
        task.err = x_d - x;
        task.xdot_d = xdot_d;
        task.xddot_d = xddot_d;
        
        tasks.push_back(task);
        // n_tasks++;
    }

    void NullSpaceControl::assignJointTask(std::vector<int> joint_task, Eigen::VectorXd x, Eigen::VectorXd x_d, Eigen::VectorXd xdot_d, Eigen::VectorXd xddot_d)
    {
        Task task;
        task.setTaskSize(joint_task.size(), TOTAL_DOF);

        task.x = x;
        task.x_d = x_d;
        task.err = x_d - x;
        task.xdot_d = xdot_d;
        task.xddot_d = xddot_d;

        for(int i=0; i<joint_task.size(); i++) {
            task.J(i, joint_task[i]) = 1.0;
        }
        task.Jdot.setZero();

        // std::cout << "Joint task Jacobian" << std::endl << task.J << std::endl;
        // std::cout << "Joint task Jacobian dot" << std::endl << task.Jdot << std::endl;

        tasks.push_back(task);
    }

    void NullSpaceControl::computeNullSpaceControl(Eigen::Matrix<double, TOTAL_DOF, 1> xidot)
    {
        // Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> N_c;
        // BuildProjectionMatrix(J_c, N_c);

        Eigen::Matrix<double, TOTAL_DOF, 1> delta_q, qdot, qddot, qddot_0;
        Eigen::MatrixXd JtPre, JtPre_pinv, Jc_pinv_tmp;
        Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> N_nx, N_pre;

        // First Task
        JtPre = tasks[0].J ;//* N_c;
        svd_pseudoInverse(JtPre, JtPre_pinv);
        
        delta_q = JtPre_pinv * (tasks[0].err);
        qdot = JtPre_pinv * tasks[0].xdot_d;

        // option1: prev
        qddot = JtPre_pinv * tasks[0].xddot_d;
        // option2: Ref: Highly dynamic locomotion
        // svd_pseudoInverse(J_c, Jc_pinv_tmp);
        // // std::cout << Jc_pinv_tmp << std::endl;
        // qddot_0 = Jc_pinv_tmp * (-J_c * robot.xidot);
        // qddot = JtPre_pinv * (tasks[0].xddot_d - tasks[0].Jdot * robot.xidot - tasks[0].J * qddot_0);


        Eigen::Matrix<double, TOTAL_DOF, 1> prev_delta_q = delta_q;
        Eigen::Matrix<double, TOTAL_DOF, 1> prev_qdot = qdot;
        Eigen::Matrix<double, TOTAL_DOF, 1> prev_qddot = qddot;

        BuildProjectionMatrix(JtPre, N_pre);

        // std::cout << "Contact Vel: " << (J_c * delta_q).transpose().format(fmt) << std::endl;
        // std::cout << "Jt0: \n" << tasks[0].J.format(fmt) << std::endl;
        // std::cout << "J_C: \n" << J_c.format(fmt) << std::endl;
        // std::cout << "Nc: \n" << N_c.format(fmt) << std::endl;
        // std::cout << "JtNc: \n" << JtPre.format(fmt) << std::endl;
        // std::cout << "JtNc_pinv: \n" << JtPre_pinv.format(fmt) << std::endl;
        // std::cout << "delta q: \n" << delta_q.transpose().format(fmt) << std::endl;

        for(int i = 1; i<tasks.size(); i++) 
        {
            std::cout << "A" << i << std::endl;
            JtPre = tasks[i].J * N_pre;

            svd_pseudoInverse(JtPre, JtPre_pinv);
            // delta_q = prev_delta_q + JtPre_pinv * (task_x_d[i] - task_x[i] - J_task[i] * prev_delta_q);
            delta_q = prev_delta_q + JtPre_pinv * (tasks[i].err - tasks[i].J * prev_delta_q);
            qdot = prev_qdot + JtPre_pinv * (tasks[i].xdot_d - tasks[i].J * prev_qdot);
            qddot = prev_qddot + JtPre_pinv * (tasks[i].xddot_d - tasks[i].Jdot * xidot - tasks[i].J * prev_qddot);

            // std::cout << "Jt" << i << std::endl << J_task[i].format(fmt) << std::endl;s
            // std::cout << "JtN" << i << std::endl << JtPre.format(fmt) << std::endl;
            // std::cout << "J_pre = J" << i << "N" << i-1 << std::endl << J_pre[i].format(fmt) << std::endl;
            // std::cout << "J_pre_pinv" << std::endl << J_pre_pinv.format(fmt) << std::endl;
            // std::cout << JtPre_pinv * JtPre << std::endl;
            // std::cout << "delta_q" << i << std::endl << delta_q[i].transpose().format(fmt) << std::endl;

            BuildProjectionMatrix(JtPre, N_nx);
            N_pre *= N_nx;
            prev_delta_q = delta_q;
            prev_qdot = qdot;
            prev_qddot = qddot;
        }
        qpos_d = delta_q;
        qvel_d = qdot;
        qacc_d = qddot;
    }

    void NullSpaceControl::BuildProjectionMatrix(const Eigen::MatrixXd & J, Eigen::Matrix<double, TOTAL_DOF, TOTAL_DOF> & N)
    {
        Eigen::MatrixXd J_pinv;
        Eigen::MatrixXd I_mtx(TOTAL_DOF, TOTAL_DOF);
        I_mtx.setIdentity();
        svd_pseudoInverse(J, J_pinv);
        N = I_mtx  - J_pinv * J;
    }

    // void NullSpaceControl::svd_pseudoInverse(Eigen::MatrixXd Matrix, Eigen::MatrixXd& invMatrix)
    // {
    //     Eigen::JacobiSVD<Eigen::MatrixXd> svd(Matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //     Eigen::VectorXd S = svd.singularValues();

    //     Eigen::VectorXd Sinv = S;
    //     for (int i = 0; i < S.size(); i++)
    //     {
    //         if (S(i) > 0.0001)
    //             Sinv(i) = 1.0 / S(i);
    //         else
    //             Sinv(i) = 0.0;
    //     }

    //     invMatrix = svd.matrixV() * Sinv.asDiagonal() * svd.matrixU().transpose();		
    // }

    void NullSpaceControl::svd_pseudoInverse(Eigen::MatrixXd Matrix, Eigen::MatrixXd& invMatrix)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        int const nrows(svd.singularValues().rows());
        Eigen::MatrixXd invS;
        invS = Eigen::MatrixXd::Zero(nrows, nrows);
        for(int ii(0); ii < nrows; ++ii) {
            if(svd.singularValues().coeff(ii) > 0.0001) {
                invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
            }
            else {
                invS.coeffRef(ii, ii) = 1.0 / 0.0001;
            }
        }
        invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
    }

    void NullSpaceControl::checkComputation()
    {
        // std::cout << "[ NullSpaceControl ] " << "Contact Jacbi Size: " << J_c.rows() << "x" << J_c.cols() << std::endl;
        // std::cout << "[ NullSpaceControl ] " << "Number of Assinged Tasks: " << tasks.size() << std::endl;
        // std::cout << "[ NullSpaceControl ] " << "qpos_d: " << qpos_d.transpose().format(fmt) << std::endl;
        // std::cout << "[ NullSpaceControl ] " << "qvel_d: " << qvel_d.transpose().format(fmt) << std::endl;
        // std::cout << "[ NullSpaceControl ] " << "qacc_d: " << qacc_d.transpose().format(fmt) << std::endl;
    }

} // namespace NullControl