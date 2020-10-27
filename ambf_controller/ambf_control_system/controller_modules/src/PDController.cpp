#include "controller_modules/PDController.h"


PDController::PDController(const Eigen::MatrixXd& _Kp, const Eigen::MatrixXd& _Kd):
   Kp(validateMat(_Kp, _Kd)),
   Kd(validateMat(_Kd, _Kp)) //validate the size of the matrix
{
    dimensions = Kp.rows();
}


PDController::~PDController()
{

}

bool PDController::setKp(const Eigen::MatrixXd& mat)
{
    int r = mat.rows();
    int c = mat.cols();

    if(r == dimensions && c == dimensions )
    {
        Kp = mat;
        return true;
    }
    else
    {
        return false;
    }

}

bool PDController::setKd(const Eigen::MatrixXd& mat)
{

    int r = mat.rows();
    int c = mat.cols();

    if(r == dimensions && c == dimensions )
    {
        Kd = mat;
        return true;
    }
    else
    {
        return false;
    }

}

Eigen::MatrixXd PDController::getKp()

{
    return Kp;
}

Eigen::MatrixXd PDController::getKd()
{
    return Kd;
}


Eigen::MatrixXd PDController::validateMat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
{
    int r1 = mat1.rows();
    int c1 = mat1.cols();

    int r2 = mat2.rows();
    int c2 = mat2.cols();

    if( r1 == c1 && r1 == r2 && c1 == c2){ //make sure it is square and both are equal
       return mat1;
    }
    else{

         throw std::invalid_argument{"!"};
    }

}

void PDController::calculate_torque(const Eigen::VectorXd &e, const Eigen::VectorXd &ed, Eigen::VectorXd &torque)
{
    torque = Kp*e+ Kd*ed;
}


void PDController::update(const trajectory_msgs::JointTrajectoryPoint& actual, const trajectory_msgs::JointTrajectoryPoint& desired, std::vector<double>& torque)
{
    
    Eigen::VectorXd e = VectToEigen(desired.positions) - VectToEigen(actual.positions);
    Eigen::VectorXd ed = VectToEigen(desired.velocities) - VectToEigen(actual.velocities); 
    Eigen::VectorXd tau_(e.rows());
    calculate_torque(e, ed, tau_);
    std::vector<double> my_tau(&tau_[0], tau_.data()+tau_.cols()*tau_.rows());
    tau = my_tau;
    //error = std::vector<double>(&e[0], e.data()+e.cols()*e.rows());
    torque = tau;
    
}
























