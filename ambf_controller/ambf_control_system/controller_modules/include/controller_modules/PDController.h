#include <Eigen/Core>

#include <iostream>


class PDController
{
public:

     PDController(const Eigen::MatrixXd&, const Eigen::MatrixXd&);
     bool setKp(const Eigen::MatrixXd&);
     bool setKd(const Eigen::MatrixXd&);
     Eigen::MatrixXd getKp();
     Eigen::MatrixXd getKd();
     void calculate( const Eigen::VectorXd& e, const  Eigen::VectorXd& ed, Eigen::VectorXd& tau);
    ~PDController(void);

private:
     Eigen::MatrixXd Kp;
     Eigen::MatrixXd Kd;
     int dimensions;

     static Eigen::MatrixXd validateMat(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

};

