/*897//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019, AMBF
    (www.aimlab.wpi.edu)

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

    \author    <http://www.aimlab.wpi.edu>
    \author    <amunawar@wpi.edu, schandrasekhar@wpi.edu>
    \author    Adnan Munawar, Shreyas Chandra Sekhar
    \version   $
*/
//==============================================================================

#include "ambf_client.h"
#include <string>

Client::Client(){
    m_numObjects = 0;

    int argc = 0;
    char **argv = 0;
    ros::init(argc, argv, "ambf_client");
}


void Client::connect() {
    this->create_objs_from_rostopics();

}



void Client::create_objs_from_rostopics()
{
    this->getPublishedTopics();

    for (itr_ = objects_map_.begin(); itr_ != objects_map_.end(); itr_++) {
        string msg_type = itr_->first;

        for (ptr_ = itr_->second.begin(); ptr_ != itr_->second.end(); ptr_++) {
            string topic_name = ptr_->first.c_str();

            if(msg_type == "ambf_msgs/WorldState") {
                world_handle_ = new World(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/ObjectState") {
                ROS_INFO("ObjectState - %s", topic_name.c_str());
                objects_map_[msg_type].insert(make_pair(topic_name, new Object(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_)));
//                objects_map_[topic_name.c_str()] =  new Object(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/LightState") {
                ROS_INFO("LightBodyState - %s %s", topic_name.c_str(), a_namespace_.c_str());
                objects_map_[msg_type].insert(make_pair(topic_name, new Light(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_)));
//                objects_map_[topic_name.c_str()] = new Light(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            } else if (msg_type == "ambf_msgs/RigidBodyState") {
                ROS_INFO("RightBodyState - %s", topic_name.c_str());
                objects_map_[msg_type].insert(make_pair(topic_name, new RigidBody(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_)));
//                objects_map_[topic_name.c_str()] =  new RigidBody(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
            }
        }
    }

}

bool Client::getPublishedTopics(){
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getTopicTypes", args, result, payload, true)){
        std::cout << "Failed!" << std::endl;
        return false;
    }

//    ros_topics_.clear();

    ROS_INFO("%d", payload.size());
    string trim_topic = "/State";
    for (int i = 0; i < payload.size(); ++i) {
       string topic_name = (string(payload[i][0])).c_str();
       string msg_type = (string(payload[i][1])).c_str();

       if(endsWith(topic_name, trim_topic)) {
           topic_name.erase (topic_name.begin(), topic_name.begin() + a_namespace_.length());
           topic_name.erase (topic_name.end() - trim_topic.length(), topic_name.end());
//           ROS_INFO("%s - %s", msg_type.c_str(), topic_name.c_str());

           objects_map_.insert(make_pair(topic_name, std::unordered_map<string, IBaseObject *>()));
           objects_map_[msg_type].insert(make_pair(topic_name, nullptr));
//        ros_topics_.emplace_back(ros::master::TopicInfo(string(payload[i][0]), string(payload[i][1])));
       }
    }
    return true;
}


bool Client::endsWith(const std::string& stack, const std::string& needle) {
    return stack.find(needle, stack.size() - needle.size()) != std::string::npos;
}

void Client::add_object(std::string name, std::string a_namespace, int a_min_freq, int a_max_freq, double time_out){
    if(!object_exists(name)){
        m_objectMap[name] = boost::shared_ptr<ambf_client::Object>(new ambf_client::Object(name, a_namespace, a_min_freq, a_max_freq, time_out));
    }
    else{
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" ALREADY EXISTS. IGNORING" << std::endl;
    }
}

ambf_client::Object* Client::get_object_handle(std::string name){
    if(object_exists(name)){
        return m_objectMap[name].get();
    }
    else{
        return NULL;
    }
}

bool Client::object_exists(std::string name){
    m_objectIt = m_objectMap.find(name);
    if(m_objectIt != m_objectMap.end()){
        return true;
    }
    else{
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" DOESN'T EXIST" << std::endl;
        return false;
    }
}

bool Client::object_cur_position(std::string name, double px, double py, double pz){
    if(object_exists(name)){
        m_objectMap[name]->cur_position(px, py, pz);
        return true;
    }
    else{
        return false;
    }
}

bool Client::object_cur_orientation(std::string name, double roll, double pitch, double yaw){
    if(object_exists(name)){
        m_objectMap[name]->cur_orientation(roll, pitch, yaw);
        return true;
    }
    else{
        return false;
    }
}

bool Client::object_cur_force(std::string name, double fx, double fy, double fz){
    if(object_exists(name)){
        m_objectMap[name]->cur_force(fx, fy, fz);
        return true;
    }
    else{
        return false;
    }
}

bool Client::object_cur_torque(std::string name, double nx, double ny, double nz){
    if(object_exists(name)){
        m_objectMap[name]->cur_torque(nx, ny, nz);
        return true;
    }
    else{
        return false;
    }
}


void Client::clean_up() {
//    ros::spin();

//    world_handle_->~World();

//    for (itr_ = objects_map_.begin(); itr_ != objects_map_.end(); itr_++) {
//        string msg_type = itr_->first;

//        for (ptr_ = itr_->second.begin(); ptr_ != itr_->second.end(); ptr_++) {
//            string topic_name = ptr_->first.c_str();

//            if(msg_type == "ambf_msgs/WorldState") {
//                world_handle_ = new World(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
//            } else if (msg_type == "ambf_msgs/ObjectState") {
////                ROS_INFO("ObjectState - %s", topic_name.c_str());
////                objects_map_[msg_type].insert(make_pair(topic_name, new Object(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_)));
////                objects_map_[topic_name.c_str()] =  new Object(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
//            } else if (msg_type == "ambf_msgs/LightState") {
////                ROS_INFO("LightBodyState - %s %s", topic_name.c_str(), a_namespace_.c_str());
////                objects_map_[msg_type].insert(make_pair(topic_name, new Light(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_)));
////                objects_map_[topic_name.c_str()] = new Light(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
//            } else if (msg_type == "ambf_msgs/RigidBodyState") {
////                ROS_INFO("RightBodyState - %s", topic_name.c_str());
////                objects_map_[msg_type].insert(make_pair(topic_name, new RigidBody(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_)));
////                objects_map_[topic_name.c_str()] =  new RigidBody(topic_name, a_namespace_, a_freq_min_, a_freq_max_, time_out_);
//            }
//        }
//    }
}

Client::~Client(void){

}
