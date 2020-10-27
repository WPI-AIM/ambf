#ifndef PDCONTROLLER_H
#define PDCONTROLLER_H
#include <Eigen/Core>
#include "controller_modules/ControllerBase.h"
#include <iostream>

class PDController : public ControllerBase
{
     public:

          PDController(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
          ~PDController(void);
          bool setKp(const Eigen::MatrixXd&);
          bool setKd(const Eigen::MatrixXd&);
          Eigen::MatrixXd getKp();
          Eigen::MatrixXd getKd();
          void calculate_torque( const Eigen::VectorXd& e, const  Eigen::VectorXd& ed, Eigen::VectorXd& tau);
          void update(const trajectory_msgs::JointTrajectoryPoint&, const  trajectory_msgs::JointTrajectoryPoint&, std::vector<double>&);
          

     private:

          Eigen::MatrixXd Kp;
          Eigen::MatrixXd Kd;
          int dimensions;
          std::vector<double> error;
          std::vector<double> tau;
          static Eigen::MatrixXd validateMat(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

};


#endif
