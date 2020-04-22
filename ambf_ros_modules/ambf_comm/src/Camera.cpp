//==============================================================================
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
    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   $
*/
//==============================================================================

#include "ambf_comm/Camera.h"
namespace ambf_comm{

const std::string ProjectionEnumToStr(int enumVal)
{
  return std::string(ProjectionEnumStr[enumVal]);
}

const std::string ViewTypeEnumToStr(int enumVal)
{
  return std::string(ViewEnumStr[enumVal]);
}

const std::string CameraParamEnumToStr(int enumVal)
{
  return std::string(CameraParamEnumStr[enumVal]);
}

CameraParams::CameraParams(){

    m_up.resize(3);
    m_look_at.resize(3);
    m_paramsChanged = false;
    m_projectionType = ProjectionEnumToStr(ProjectionType::PERSPECTIVE);
    m_viewType = ViewTypeEnumToStr(ViewType::MONO);
}

void Camera::set_params_on_server(){
    nodePtr->setParam(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::look_at), m_look_at);
    nodePtr->setParam(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::up), m_up);
    nodePtr->setParam(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::near_plane), m_near_plane);
    nodePtr->setParam(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::far_plane), m_far_plane);
    nodePtr->setParam(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::projection), m_projectionType);
    nodePtr->setParam(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::type), m_viewType);


}

void Camera::update_params_from_server(){
    nodePtr->getParamCached(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::look_at), m_look_at);
    nodePtr->getParamCached(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::up), m_up);
    nodePtr->getParamCached(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::near_plane), m_near_plane);
    nodePtr->getParamCached(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::far_plane), m_far_plane);
    nodePtr->getParamCached(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::projection), m_projectionType);
    nodePtr->getParamCached(m_base_prefix + "/" + CameraParamEnumToStr(CameraParamsEnum::type), m_viewType);
}

Camera::Camera(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): CameraRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
    m_base_prefix = a_namespace + '/' + a_name;
}

void Camera::cur_position(double px, double py, double pz){
    m_trans.setOrigin(tf::Vector3(px, py, pz));
    m_State.pose.position.x = px;
    m_State.pose.position.y = py;
    m_State.pose.position.z = pz;
}

void Camera::cur_orientation(double roll, double pitch, double yaw){
    tf::Quaternion rot_quat;
    rot_quat.setRPY(roll, pitch, yaw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_State.pose.orientation);
}

void Camera::cur_orientation(double qx, double qy, double qz, double qw){
    tf::Quaternion rot_quat(qx, qy, qz, qw);
    m_trans.setRotation(rot_quat);
    tf::quaternionTFToMsg(rot_quat, m_State.pose.orientation);
}

void Camera::set_wall_time(double a_sec){
    m_State.wall_time = a_sec;
    increment_sim_step();
    m_State.header.stamp = ros::Time::now();
}

ambf_msgs::CameraCmd Camera::get_command(){
    ambf_msgs::CameraCmd temp_cmd = m_Cmd;
    return temp_cmd;
}


extern "C"{

Camera* create_camera(std::string a_name, std::string a_namespace="/ambf_comm/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5){
    return new Camera(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_camera(Camera* obj){
    delete obj;
}

}
}
