#ifndef CONTROLLERMANAGER_H
#define CONTROLLERMANAGER_H
#include "ros/ros.h"
#include "ControllerBase.h"
#include "boost/shared_ptr.hpp"
#include <unordered_map>
#include <string>
#include "controller_modules/JointControl.h"
#include "controller_modules/ControllerList.h"
#include <vector>

class ControllerManager
{
    public:
        ControllerManager(ros::NodeHandle*);
        ~ControllerManager();
        void addController(std::string, boost::shared_ptr<ControllerBase>  );
        std::vector<std::string> getControllerList();
       
    private:
        ros::NodeHandle nh;
        ros::ServiceServer calc_tau_srv, controller_list_srv;
        std::unordered_map<std::string, boost::shared_ptr<ControllerBase>> controller_list;
        bool calcTauSrv(controller_modules::JointControlRequest&, controller_modules::JointControlResponse& );
        bool getControllersListSrv(controller_modules::ControllerListRequest&, controller_modules::ControllerListResponse& );
};  


#endif
