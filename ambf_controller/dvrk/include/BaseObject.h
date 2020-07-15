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
public:
    virtual ~IBaseObject() = default;
};

//class ObjectRosCom: public BaseObject<ambf_msgs::ObjectState, ambf_msgs::ObjectCmd>{
template <class T_state, class T_cmd>
class BaseObject: public RosComBase<T_state, T_cmd>, public IBaseObject {
public:
    BaseObject(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out):
        RosComBase<T_state, T_cmd>(a_name, a_namespace, a_freq_min, a_freq_max, time_out) {

        freq_min_ = a_freq_min;
        freq_max_ = a_freq_max;
        time_out_ = time_out;
    }

    void destroy_at(T_state* t_s, T_cmd* t_c);
//    void destroy_at(T_state* t_s, T_cmd* t_c) {
//        t_s->~T_state();
//        t_c->~T_cmd();
//    }

    virtual void cleanUp();
    virtual ~BaseObject(void){}

protected:

    tf::Transform m_trans;
    T_state m_State;
    T_cmd m_Cmd;
    T_cmd m_CmdPrev;

private:
    std::string name_;
    std::string namespace_;
    int freq_min_;
    int freq_max_;
    double time_out_;
};


#endif // BASEOBJECT_H
