//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
    (https://github.com/WPI-AIM/ambf)
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
    \author    <amunawar@wpi.edu, schandrasekhar@wpi.edu>
    \author    Adnan Munawar, Shreyas Chandra Sekhar
    \version   1.0$
*/
//==============================================================================

#include "ambf_client.h"
#include <string>

Client::Client(){
//    m_numObjects = 0;

    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "ambf_client");
}


void Client::connect() {
    this->createObjsFromRostopics();

}

void Client::createObjsFromRostopics()
{
    this->getPublishedTopics();

//    string topic_name = "lights/light_left";
//    IBaseObject* handler = new Light(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);

//    Light* l = dynamic_cast<Light*>(handler);
//    l->get_command();

//    LightRosCom* lr = dynamic_cast<LightRosCom*>(handler);
//    lr->~LightRosCom();

    for (itr_ = objects_map_.begin(); itr_ != objects_map_.end(); itr_++) {
        string msg_type = itr_->first;

        for (ptr_ = itr_->second.begin(); ptr_ != itr_->second.end(); ptr_++) {
            string topic_name = ptr_->first.c_str();

            if (msg_type == "ambf_msgs/ActuatorState") {
                objects_map_[msg_type][topic_name] = new Actuator(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/CameraState") {
                objects_map_[msg_type][topic_name] = new Camera(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/LightState") {
                objects_map_[msg_type][topic_name] = new Light(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/ObjectState") {
                objects_map_[msg_type][topic_name] = new Object(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/RigidBodyState") {
                objects_map_[msg_type][topic_name] = new RigidBody(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/SensorState") {
                objects_map_[msg_type][topic_name] = new Sensor(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/VehicleState") {
                objects_map_[msg_type][topic_name] = new Vehicle(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if(msg_type == "ambf_msgs/WorldState") {
                objects_map_[msg_type][topic_name] = new World(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            }
        }
    }

}

/////
///// \brief Client::getAActuator
///// \param a_name
///// \param suppress_warning
///// \return
/////
actuatorPtr Client::getAActuator(std::string a_name, bool suppress_warning){
    string msg_type = "ambf_msgs/ActuatorState";
    if(!checkMessageType(msg_type)) return NULL;

    return dynamic_cast<actuatorPtr>(getObject<iBaseObjectPtr, iBaseObjectMap>(a_name, &objects_map_[msg_type], suppress_warning));
}

/////
///// \brief Client::getACamera
///// \param a_name
///// \param suppress_warning
///// \return
/////
cameraPtr Client::getACamera(std::string a_name, bool suppress_warning){
    string msg_type = "ambf_msgs/CameraState";
    if(!checkMessageType(msg_type)) return NULL;

    return dynamic_cast<cameraPtr>(getObject<iBaseObjectPtr, iBaseObjectMap>(a_name, &objects_map_[msg_type], suppress_warning));
}

/////
///// \brief Client::getAFLight
///// \param a_name
///// \param suppress_warning
///// \return
/////
lightPtr Client::getALight(std::string a_name, bool suppress_warning){
    string msg_type = "ambf_msgs/LightState";
    if(!checkMessageType(msg_type)) return NULL;
    return dynamic_cast<lightPtr>(getObject<iBaseObjectPtr, iBaseObjectMap>(a_name, &objects_map_[msg_type], suppress_warning));
}

/////
///// \brief Client::getAObject
///// \param a_name
///// \param suppress_warning
///// \return
/////
objectPtr Client::getAObject(std::string a_name, bool suppress_warning){
    string msg_type = "ambf_msgs/ObjectState";
    if(!checkMessageType(msg_type)) return NULL;
    return dynamic_cast<objectPtr>(getObject<iBaseObjectPtr, iBaseObjectMap>(a_name, &objects_map_[msg_type], suppress_warning));
}

/////
///// \brief Client::getRightBody
///// \param a_name
///// \param suppress_warning
///// \return
/////
rigidBodyPtr Client::getARigidBody(std::string a_name, bool suppress_warning){
    string msg_type = "ambf_msgs/RigidBodyState";
    if(!checkMessageType(msg_type)) return NULL;
    return dynamic_cast<rigidBodyPtr>(getObject<iBaseObjectPtr, iBaseObjectMap>(a_name, &objects_map_[msg_type], suppress_warning));
}

/////
///// \brief Client::getSensor
///// \param a_name
///// \param suppress_warning
///// \return
/////
sensorPtr Client::getASensor(std::string a_name, bool suppress_warning){
    string msg_type = "ambf_msgs/SensorState";
    if(!checkMessageType(msg_type)) return NULL;
    return dynamic_cast<sensorPtr>(getObject<iBaseObjectPtr, iBaseObjectMap>(a_name, &objects_map_[msg_type], suppress_warning));
}

/////
///// \brief Client::getVehicle
///// \param a_name
///// \param suppress_warning
///// \return
/////
vehiclePtr Client::getAVehicle(std::string a_name, bool suppress_warning){
    string msg_type = "ambf_msgs/VehicleState";
    if(!checkMessageType(msg_type)) return NULL;
    return dynamic_cast<vehiclePtr>(getObject<iBaseObjectPtr, iBaseObjectMap>(a_name, &objects_map_[msg_type], suppress_warning));
}

/////
///// \brief Client::getAWorld
///// \param a_name
///// \param suppress_warning
///// \return
/////
worldPtr Client::getAWorld(std::string a_name, bool suppress_warning){
    string msg_type = "ambf_msgs/WorldState";
    if(!checkMessageType(msg_type)) return NULL;

    return dynamic_cast<worldPtr>(getObject<iBaseObjectPtr, iBaseObjectMap>(a_name, &objects_map_[msg_type], suppress_warning));
}

bool Client::checkMessageType(std::string msg_type){
    itr_ = objects_map_.find (msg_type);
    if ( itr_ == objects_map_.end() ) {
        ROS_INFO("Object type not found");
        return false;
    }
    ROS_INFO("Object type found");
    return true;
}

bool Client::getPublishedTopics(){
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getTopicTypes", args, result, payload, true)){
        std::cout << "Failed!" << std::endl;
        return false;
    }

////    ros_topics_.clear();

    ROS_INFO("%d", payload.size());
    string trim_topic = "/State";
    for (int i = 0; i < payload.size(); ++i) {
       string topic_name = (string(payload[i][0])).c_str();
       string msg_type = (string(payload[i][1])).c_str();

       if(endsWith(topic_name, trim_topic)) {
           topic_name.erase (topic_name.begin(), topic_name.begin() + a_namespace_.length());
           topic_name.erase (topic_name.end() - trim_topic.length(), topic_name.end());
           ROS_INFO("%s - %s", msg_type.c_str(), topic_name.c_str());

           objects_map_.insert(make_pair(topic_name, std::unordered_map<string, IBaseObject *>()));
           objects_map_[msg_type].insert(make_pair(topic_name, nullptr));

       }
    }
    return true;
}


bool Client::endsWith(const std::string& stack, const std::string& needle) {
    return stack.find(needle, stack.size() - needle.size()) != std::string::npos;
}

template <typename T, typename TMap>
///
/// \brief IBaseObject::getObject
/// \param a_name
/// \param map
/// \param suppress_warning
/// \return
///
T Client::getObject(std::string a_name, TMap* a_map, bool suppress_warning){
    if (a_map->find(a_name) != a_map->end()){
        return ((*a_map)[a_name]);
    }
    // We didn't find the object using the full name, try checking if the name is a substring of the fully qualified name
    int matching_obj_count = 0;
    std::vector<std::string> matching_obj_names;
    T objHandle;
    typename TMap::iterator oIt = a_map->begin();
    for (; oIt != a_map->end() ; ++oIt){
        if (oIt->first.find(a_name) != std::string::npos){
            matching_obj_count++;
            matching_obj_names.push_back(oIt->first);
            objHandle = oIt->second;
        }
    }

    if (matching_obj_count == 1){
        // If only one object is found, return that object
        return objHandle;
    }
    else if(matching_obj_count > 1){
        std::cerr << "WARNING: MULTIPLE OBJECTS WITH SUB-STRING: \"" << a_name << "\" FOUND. PLEASE SPECIFY FURTHER\n";
        for (int i = 0 ; i < matching_obj_names.size() ; i++){
            std::cerr << "\t" << i << ") " << matching_obj_names[i] << std::endl;
        }
        return NULL;
    }
    else{
        if (!suppress_warning){
            std::cerr << "WARNING: CAN'T FIND ANY OBJECTS NAMED: \"" << a_name << "\"\n";

            std::cerr <<"Existing OBJECTS in Map: " << a_map->size() << std::endl;
            typename TMap::iterator oIt = a_map->begin();
            for (; oIt != a_map->end() ; ++oIt){
                std::cerr << oIt->first << std::endl;
            }
        }
        return NULL;
    }
}




//World* Client::get_world_handle() {
//    return world_handle_;
//}




//void Client::add_object(std::string name, std::string a_namespace, int a_min_freq, int a_max_freq, double time_out){
//    if(!object_exists(name)){
//        m_objectMap[name] = boost::shared_ptr<ambf_client::Object>(new ambf_client::Object(name, a_namespace, a_min_freq, a_max_freq, time_out));
//    }
//    else{
//        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" ALREADY EXISTS. IGNORING" << std::endl;
//    }
//}

//ambf_client::Object* Client::get_object_handle(std::string name){
//    if(object_exists(name)){
//        return m_objectMap[name].get();
//    }
//    else{
//        return NULL;
//    }
//}

//bool Client::object_exists(std::string name){
//    m_objectIt = m_objectMap.find(name);
//    if(m_objectIt != m_objectMap.end()){
//        return true;
//    }
//    else{
//        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" DOESN'T EXIST" << std::endl;
//        return false;
//    }
//}

//bool Client::object_cur_position(std::string name, double px, double py, double pz){
//    if(object_exists(name)){
//        m_objectMap[name]->cur_position(px, py, pz);
//        return true;
//    }
//    else{
//        return false;
//    }
//}

//bool Client::object_cur_orientation(std::string name, double roll, double pitch, double yaw){
//    if(object_exists(name)){
//        m_objectMap[name]->cur_orientation(roll, pitch, yaw);
//        return true;
//    }
//    else{
//        return false;
//    }
//}

//bool Client::object_cur_force(std::string name, double fx, double fy, double fz){
//    if(object_exists(name)){
//        m_objectMap[name]->cur_force(fx, fy, fz);
//        return true;
//    }
//    else{
//        return false;
//    }
//}

//bool Client::object_cur_torque(std::string name, double nx, double ny, double nz){
//    if(object_exists(name)){
//        m_objectMap[name]->cur_torque(nx, ny, nz);
//        return true;
//    }
//    else{
//        return false;
//    }
//}


void Client::cleanUp() {
//    ros::spin();



    for (itr_ = objects_map_.begin(); itr_ != objects_map_.end(); itr_++) {
        string msg_type = itr_->first;

        for (ptr_ = itr_->second.begin(); ptr_ != itr_->second.end(); ptr_++) {
            IBaseObject * handler = ptr_->second;
            if (msg_type == "ambf_msgs/ActuatorState") {
                (dynamic_cast<ActuatorRosCom*>(handler))->~ActuatorRosCom();
            } else if (msg_type == "ambf_msgs/CameraState") {
                (dynamic_cast<CameraRosCom*>(handler))->~CameraRosCom();
            } else if (msg_type == "ambf_msgs/LightState") {
                (dynamic_cast<LightRosCom*>(handler))->~LightRosCom();
            } else if (msg_type == "ambf_msgs/ObjectState") {
                (dynamic_cast<ObjectRosCom*>(handler))->~ObjectRosCom();
            } else if (msg_type == "ambf_msgs/RigidBodyState") {
                (dynamic_cast<RigidBodyRosCom*>(handler))->~RigidBodyRosCom();
            } else if (msg_type == "ambf_msgs/SensorState") {
                (dynamic_cast<SensorRosCom*>(handler))->~SensorRosCom();
            } else if (msg_type == "ambf_msgs/VehicleState") {
                (dynamic_cast<VehicleRosCom*>(handler))->~VehicleRosCom();
            } else if(msg_type == "ambf_msgs/WorldState") {
                (dynamic_cast<WorldRosCom*>(handler))->~WorldRosCom();
            }
        }
    }
}

Client::~Client(void){

}
