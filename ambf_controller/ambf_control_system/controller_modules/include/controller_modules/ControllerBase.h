#ifndef CONTROLLERBASE_H
#define CONTROLLERBASE_H

#include <Eigen/Core>

class ControllerBase
{
   public:
    ControllerBase(){}
    ~ControllerBase(){}

   private:
        virtual void calc_tau(const Eigen::VectorXd&, const  Eigen::VectorXd&, Eigen::VectorXd&)=0;

   protected:
   


};


#endif // CONTROLLERBASE_H
