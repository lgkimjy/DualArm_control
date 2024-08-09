#include <Eigen/Dense>

namespace Null
{
    class Task
    {
    private:

    public:
        Task() {};
        ~Task() {};

        Eigen::VectorXd 	x;			
        Eigen::VectorXd 	x_d;		
        Eigen::VectorXd 	err;		
        Eigen::VectorXd 	xdot_d;	
        Eigen::VectorXd 	xddot_d;	
        Eigen::MatrixXd 	J;			
        Eigen::MatrixXd 	Jdot;

        void setTaskSize(int size, int DoF) 
        {
            x = Eigen::VectorXd(size).setZero();
            x_d = Eigen::VectorXd(size).setZero();
            err = Eigen::VectorXd(size).setZero();
            xdot_d = Eigen::VectorXd(size).setZero();
            xddot_d = Eigen::VectorXd(size).setZero();
            J = Eigen::MatrixXd(size, DoF).setZero();
            Jdot = Eigen::MatrixXd(size, DoF).setZero();
        }
    };
} // namespace Null