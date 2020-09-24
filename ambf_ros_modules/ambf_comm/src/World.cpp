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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   1.0$
*/
//==============================================================================

#include "ambf_comm/World.h"

namespace ambf_comm{

const std::string world_param_enum_to_str(WorldParamsEnum enumVal)
{
    if (enumVal == WorldParamsEnum::point_cloud_topics) return "point_cloud_topics";
    else if (enumVal == WorldParamsEnum::point_cloud_radii) return "point_cloud_radii";

}


///
/// \brief WorldParams::WorldParams
///
WorldParams::WorldParams(){
    m_paramsChanged = false;
}


int WorldParams::get_num_point_cloud_handlers(){
    return m_pointCloudHandlerMap.size();
}

///
/// \brief WorldParams::get_all_point_cloud_handlers
/// \return
///
std::vector<PointCloudHandlerPtr> WorldParams::get_all_point_cloud_handlers(){
    PointCloudHandlerVec pVec;
    PointCloudHandlerMap::iterator pIt;
    for (pIt = m_pointCloudHandlerMap.begin() ; pIt != m_pointCloudHandlerMap.end() ; ++ pIt){
        pVec.push_back(pIt->second);
    }

    return pVec;
}


PointCloudHandlerPtr WorldParams::get_point_clound_handler(std::string topic_name){
    PointCloudHandlerPtr pchPtr;
    if (m_pointCloudHandlerMap.find(topic_name) != m_pointCloudHandlerMap.end()){
        pchPtr = m_pointCloudHandlerMap[topic_name];
    }
    else{
        std::cerr << "ERROR: CAN'T FIND ANY PC HANDLER NAMED: " << topic_name << std::endl;
    }
    return pchPtr;
}


///
/// \brief World::set_params_on_server
///
void World::set_params_on_server(){
    nodePtr->setParam(m_base_prefix + "/" + world_param_enum_to_str(WorldParamsEnum::point_cloud_topics), m_point_cloud_topics);
    nodePtr->setParam(m_base_prefix + "/" + world_param_enum_to_str(WorldParamsEnum::point_cloud_radii), m_point_cloud_radii);
}

///
/// \brief World::update_params_from_server
///
void World::update_params_from_server(){
    std::vector<std::string> topic_names;
    std::vector<double> topic_radii;

    nodePtr->getParamCached(m_base_prefix + "/" + world_param_enum_to_str(WorldParamsEnum::point_cloud_topics), topic_names);
    nodePtr->getParamCached(m_base_prefix + "/" + world_param_enum_to_str(WorldParamsEnum::point_cloud_radii), topic_radii);

    std::vector<bool> keep_active_idx;

    if (m_new_topic_names <= m_point_cloud_topics){
        m_new_topic_names.clear();
    }
    else{
        nodePtr->setParam(m_base_prefix + "/" + world_param_enum_to_str(WorldParamsEnum::point_cloud_topics), m_new_topic_names);
    }
    m_defunct_topic_names.clear();

    keep_active_idx.resize(m_point_cloud_topics.size());
    for (int i = 0 ; i < keep_active_idx.size() ; i++){
        keep_active_idx[i] = false;
    }

    // Check each topic name against previously stored copy
    for(int i = 0 ; i < topic_names.size() ; i++){
        std::string new_topic = topic_names[i];

        bool is_new_topic = true;
        for(int j = 0 ; j < m_point_cloud_topics.size() ; j++){
            std::string old_topic = m_point_cloud_topics[j];
            if(new_topic.compare(old_topic) == 0){
                // Match Found. Thus keep this topic active
                is_new_topic = false;
                keep_active_idx[i] = true;
                break;
            }
        }

        if (is_new_topic == true){
            // No match found thus it is a new topic. Mark to be created anew
            m_new_topic_names.push_back(new_topic);
        }
    }

    for (int i = 0 ; i < m_point_cloud_topics.size() ; i++){
        if (keep_active_idx[i] == false){
            m_defunct_topic_names.push_back(m_point_cloud_topics[i]);
        }
    }

    // Lets remove topics that have been marked for removal.
    for (int i = 0 ; i < m_defunct_topic_names.size() ; i++){
        std::string topic_name = m_defunct_topic_names[i];
        if (m_pointCloudHandlerMap.find(topic_name) != m_pointCloudHandlerMap.end()){
            (*m_pointCloudHandlerMap[topic_name]).remove();
            m_pointCloudHandlerMap.erase(topic_name);
        }
    }

    // Now add new topics
    for (int i = 0 ; i < m_new_topic_names.size() ; i++){
        std::string topic_name = m_new_topic_names[i];
        if (m_pointCloudHandlerMap.find(topic_name) == m_pointCloudHandlerMap.end()){
            // Sanity check to see if the topic isn't already in the map
            PointCloudHandlerPtr pcHandler(new PointCloundHandler());
            pcHandler->init(nodePtr, topic_name);
            m_pointCloudHandlerMap[topic_name] = pcHandler;
        }
    }

    // If any topic names need to be removed or added, then update the variable containing the list of topic names
    if (m_defunct_topic_names.size() > 0 || m_new_topic_names.size() > 0){
        m_paramsChanged = true;
        m_point_cloud_topics.clear();
        PointCloudHandlerMap::iterator pcIt;
        for (pcIt = m_pointCloudHandlerMap.begin() ; pcIt != m_pointCloudHandlerMap.end() ; ++pcIt){
            m_point_cloud_topics.push_back( pcIt->first );
        }
    }

    // Update the topic radii
    PointCloudHandlerMap::iterator pcIt;
    int rIdx = 0;
    for (pcIt = m_pointCloudHandlerMap.begin() ; pcIt != m_pointCloudHandlerMap.end() ; ++pcIt){
        if (rIdx < topic_radii.size()){
            pcIt->second->set_radius(topic_radii[rIdx]);
        }
        rIdx++;
    }

    // DEBUG
//    std::cerr << "-----------------------\n";

//    for (int i = 0 ; i < m_point_cloud_topics.size() ; i++){
//        std::cerr << i << ")\t" << m_point_cloud_topics[i] << "\n";
//    }
}


///
/// \brief World::World
/// \param a_name
/// \param a_namespace
/// \param a_freq_min
/// \param a_freq_max
/// \param time_out
///
World::World(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): WorldRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
    m_num_skip_steps = 10;
    m_skip_steps_ctr = 0;
    m_base_prefix = a_namespace + '/' + a_name;
}


///
/// \brief World::set_wall_time
/// \param a_sec
///
void World::set_wall_time(double a_sec){
    m_State.wall_time = a_sec;
    increment_sim_step();
    m_State.header.stamp = ros::Time::now();
}


///
/// \brief World::increment_sim_step
///
void World::increment_sim_step(){
    if(m_enableSimThrottle){
        m_skip_steps_ctr++;
        if (m_skip_steps_ctr == m_num_skip_steps){
            m_stepSim = false;
            m_skip_steps_ctr = 0;
        }
        if (m_skip_steps_ctr > m_num_skip_steps){
            std::cerr << "WARN, Skipped " << m_skip_steps_ctr << " steps, Default skip limit " << m_num_skip_steps << std::endl;
        }
    }
    m_State.sim_step++;
}

extern "C"{

World* create_world(std::string a_name, std::string a_namespace="/ambf_comm/", int a_min_freq=50, int a_max_freq=1000, double time_out=10.0){
    return new World(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_world(World* obj){
    delete obj;
}

}
}
