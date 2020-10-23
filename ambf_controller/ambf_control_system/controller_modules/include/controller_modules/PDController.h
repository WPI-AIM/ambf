#ifndef PDCONTROLLER_H
#define PDCONTROLLER_H
#include <Eigen/Core>
#include "controller_modules/ControllerBase.h"
#include <iostream>


class PDController : public ControllerBase
{
public:

     PDController(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
     bool setKp(const Eigen::MatrixXd&);
     bool setKd(const Eigen::MatrixXd&);
     Eigen::MatrixXd getKp();
     Eigen::MatrixXd getKd();
     void calc_tau( const Eigen::VectorXd& e, const  Eigen::VectorXd& ed, Eigen::VectorXd& tau);
    ~PDController(void);

private:
     Eigen::MatrixXd Kp;
     Eigen::MatrixXd Kd;
     int dimensions;

     static Eigen::MatrixXd validateMat(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

};


#endif
