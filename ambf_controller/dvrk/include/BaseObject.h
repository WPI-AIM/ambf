#ifndef BASEOBJECT_H
#define BASEOBJECT_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <ros/callback_queue.h>
#include <ros/duration.h>
#include "CmdWatchDog.h"
#include "RosComBase.h"

#include "ambf_msgs/ActuatorCmd.h"
#include "ambf_msgs/ActuatorState.h"
#include "ambf_msgs/CameraState.h"
#include "ambf_msgs/CameraCmd.h"
#include "ambf_msgs/LightState.h"
#include "ambf_msgs/LightCmd.h"
#include "ambf_msgs/ObjectCmd.h"
#include "ambf_msgs/ObjectState.h"
#include "ambf_msgs/RigidBodyCmd.h"
#include "ambf_msgs/RigidBodyState.h"
#include "ambf_msgs/SensorCmd.h"
#include "ambf_msgs/SensorState.h"
#include "ambf_msgs/VehicleCmd.h"
#include "ambf_msgs/VehicleState.h"

class IBaseObject
{
  // list of cell methods
};

//class ObjectRosCom: public BaseObject<ambf_msgs::ObjectState, ambf_msgs::ObjectCmd>{
template <class T_state, class T_cmd>
class BaseObject: public RosComBase<T_state, T_cmd>, public IBaseObject {
//public:
//    BaseObject(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
//    ~BaseObject();
//    virtual void init();

////protected:
////    virtual void reset_cmd();
////    void sub_cb(ambf_msgs::ObjectCmdConstPtr msg);
//};


//class BaseObject: public RosComBase<T_state, T_cmd> {
public:
    BaseObject(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out):
        RosComBase<T_state, T_cmd>(a_name, a_namespace, a_freq_min, a_freq_max, time_out)
    {
//        m_name = a_name;
//        m_namespace = a_namespace;

//        m_freq_min = a_freq_min;
//        m_freq_max = a_freq_max;

//        BaseObject<T_state, T_cmd>::RosComBase(a_name, a_namespace, a_freq_min, a_freq_max, time_out);

//        int argc = 0;
//        char **argv = 0;
//        ros::init(argc, argv, "ambf_client");
//        nodePtr.reset(new ros::NodeHandle);
//        aspinPtr.reset(new ros::AsyncSpinner(1));
//        nodePtr->setCallbackQueue(&m_custom_queue);
//        m_watchDogPtr.reset(new CmdWatchDog(a_freq_min, a_freq_max, time_out));
    }


//    ~BaseObject();
//    virtual void init() = 0;
//    virtual void run_publishers();
    virtual void cleanUp();
//    virtual T_cmd get_command(){return m_Cmd;}

//    int m_freq_min;
//    int m_freq_max;

protected:
//    boost::shared_ptr<ros::NodeHandle> nodePtr;
//    boost::shared_ptr<ros::AsyncSpinner> aspinPtr;
//    boost::shared_ptr<CmdWatchDog> m_watchDogPtr;

    std::string m_namespace;
    std::string m_name;
//    ros::Publisher m_pub;
//    ros::Subscriber m_sub;

    tf::Transform m_trans;
    T_state m_State;
    T_cmd m_Cmd;
    T_cmd m_CmdPrev;

//    boost::thread m_thread;
//    ros::CallbackQueue m_custom_queue;

    virtual void reset_cmd() = 0;
};

//template<class T_state, class T_cmd>
//void BaseObject<T_state, T_cmd>::run_publishers(){
//    RosComBase<T_state, T_cmd>::run_publishers();
//    while(nodePtr->ok()){
//        m_pub.publish(m_State);
//        m_custom_queue.callAvailable();
//        if(m_watchDogPtr->is_wd_expired()){
//            m_watchDogPtr->consolePrint(m_name);
//            reset_cmd();
//        }
//        m_watchDogPtr->m_ratePtr->sleep();
//    }
//}




#endif // BASEOBJECT_H
