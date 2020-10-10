#include "controller_modules/PDController.h"


PDController::PDController(const Eigen::Ref<const Eigen::MatrixXd> & _Kp, const Eigen::Ref<const Eigen::MatrixXd> & _Kd):
    Kp(_Kp),
    Kd(_Kd)
{

}



PDController::~PDController()
{

}

void PDController::calculate( const Eigen::VectorXd& e, const  Eigen::VectorXd& ed, Eigen::VectorXd& tau)
{
    //need to calculate the PD control
    //Kp*e + Kd*ed;
    tau = Kp.cross(e) + Kd.cross(ed);

}
