#include <Eigen/Core>




class PDController
{
public:

     PDController(const Eigen::Ref<const Eigen::MatrixXd>& Kp, const Eigen::Ref<const Eigen::MatrixXd>& Kd);
     void setKp(const Eigen::Ref<const Eigen::MatrixXd>& Kp);
     void setKd(const Eigen::Ref<const Eigen::MatrixXd>& Kd);
     Eigen::MatrixXd getKp();
     Eigen::MatrixXd getKd();
     void calculate( const Eigen::VectorXd& e, const  Eigen::VectorXd& ed, Eigen::VectorXd& tau);
    ~PDController(void);

private:
     Eigen::MatrixXd Kp;
     Eigen::MatrixXd Kd;
//     Eigen::EigenBase<Derived>& Kp;
//     Eigen::EigenBase<Derived>& Kd;

};

