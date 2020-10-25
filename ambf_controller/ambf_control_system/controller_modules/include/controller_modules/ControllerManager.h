#ifndef CONTROLLERMANAGER_H
#define CONTROLLERMANAGER_H
#include "ros/ros.h"
#include "ControllerBase.h"
#include "boost/shared_ptr.hpp"
#include <unordered_map>
#include <string>
#include "controller_modules/JointControl.h"

class ControllerManager
{
    public:
        ControllerManager(ros::NodeHandle*);
        ~ControllerManager();
        void addController(std::string, ControllerBase& );
        void getControllerList();
        bool calcTau(controller_modules::JointControlRequest&, controller_modules::JointControlResponse& );

    private:
        ros::NodeHandle nh;
        ros::ServiceServer calc_tau;
        std::unordered_map<std::string, boost::shared_ptr<ControllerBase>> control_list;
};


#endif
