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

#include "ambf_comm/ambf_comm.h"
#include <string>

ChaiEnv::ChaiEnv(){
    m_numObjects = 0;
}


void ChaiEnv::add_object(std::string name, std::string a_namespace, int a_min_freq, int a_max_freq, double time_out){
    if(!object_exists(name)){
        m_objectMap[name] = boost::shared_ptr<ambf_comm::Object>(new ambf_comm::Object(name, a_namespace, a_min_freq, a_max_freq, time_out));
    }
    else{
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" ALREADY EXISTS. IGNORING" << std::endl;
    }
}

ambf_comm::Object* ChaiEnv::get_object_handle(std::string name){
    if(object_exists(name)){
        return m_objectMap[name].get();
    }
    else{
        return NULL;
    }
}

bool ChaiEnv::object_exists(std::string name){
    m_objectIt = m_objectMap.find(name);
    if(m_objectIt != m_objectMap.end()){
        return true;
    }
    else{
        std::cerr<< "ERROR!, OBJECT: \""<< name << "\" DOESN'T EXIST" << std::endl;
        return false;
    }
}

bool ChaiEnv::object_cur_position(std::string name, double px, double py, double pz){
    if(object_exists(name)){
        m_objectMap[name]->cur_position(px, py, pz);
        return true;
    }
    else{
        return false;
    }
}

bool ChaiEnv::object_cur_orientation(std::string name, double roll, double pitch, double yaw){
    if(object_exists(name)){
        m_objectMap[name]->cur_orientation(roll, pitch, yaw);
        return true;
    }
    else{
        return false;
    }
}

bool ChaiEnv::object_cur_force(std::string name, double fx, double fy, double fz){
    if(object_exists(name)){
        m_objectMap[name]->cur_force(fx, fy, fz);
        return true;
    }
    else{
        return false;
    }
}

bool ChaiEnv::object_cur_torque(std::string name, double nx, double ny, double nz){
    if(object_exists(name)){
        m_objectMap[name]->cur_torque(nx, ny, nz);
        return true;
    }
    else{
        return false;
    }
}

ChaiEnv::~ChaiEnv(){

}
