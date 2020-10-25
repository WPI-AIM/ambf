#include "controller_modules/ControllerManager.h"



ControllerManager::ControllerManager(ros::NodeHandle* n):nh(*n)
{
 calc_tau =  nh.advertiseService("CalcTau", &ControllerManager::calcTau, this);
}

ControllerManager::~ControllerManager()
{

}


void ControllerManager::addController(std::string name, ControllerBase& controller )
{
    boost::shared_ptr<ControllerBase> cntl(&controller ) ;
    control_list[name] = cntl;
}
// void ControllerManager::getControllerList();
bool ControllerManager::calcTau(controller_modules::JointControlRequest& req, controller_modules::JointControlResponse& res )
{
    
}
