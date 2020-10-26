#include "controller_modules/ControllerManager.h"



ControllerManager::ControllerManager(ros::NodeHandle* n):nh(*n)
{
    calc_tau_srv =  nh.advertiseService("CalcTau", &ControllerManager::calcTauSrv, this);
    controller_list_srv = nh.advertiseService("ControllerList", &ControllerManager::getControllersListSrv, this);
}

ControllerManager::~ControllerManager()
{

}

bool ControllerManager::getControllersListSrv(controller_modules::ControllerListRequest& req, controller_modules::ControllerListResponse& res)
{   
    res.controllers = getControllerList();
}

std::vector<std::string> ControllerManager::getControllerList()
{
    std::vector<std::string> names;
    for( std::pair<std::string,  boost::shared_ptr<ControllerBase>>  node: controller_list )
    {
        names.push_back(node.first);
    }
    return names;
}

void ControllerManager::addController(std::string name, boost::shared_ptr<ControllerBase> controller )
{
    //boost::shared_ptr<ControllerBase> cntl(*controller) ;
    controller_list[name] = controller;
}

// void ControllerManager::getControllerList();
bool ControllerManager::calcTauSrv(controller_modules::JointControlRequest& req, controller_modules::JointControlResponse& res )
{
    controller_list[req.controller_name];

}
