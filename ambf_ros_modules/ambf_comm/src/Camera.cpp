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

const std::string projection_type_enum_to_str(int enumVal)
{
  return std::string(ProjectionTypeEnumStr[enumVal]);
}

const std::string view_mode_enum_to_str(int enumVal)
{
  return std::string(ViewModeEnumStr[enumVal]);
}

const std::string camera_param_enum_to_str(int enumVal)
{
  return std::string(CameraParamEnumStr[enumVal]);
}

CameraParams::CameraParams(){
    m_paramsChanged = false;
    m_projectionType = ProjectionType::PERSPECTIVE;
    m_viewMode = ViewMode::MONO;
}

//void CameraParams::set_near_plane(double val){
//    m_near_plane = val;
//}

//void CameraParams::set_far_plane(double val){
//    m_far_plane = val;
//}

//void CameraParams::set_field_view_angle(double val){
//    m_field_view_angle = val;
//}

//void CameraParams::set_projection_type(ProjectionType type){
//    m_projectionType = type;
//}

//void CameraParams::set_view_mode(ViewMode type){
//    m_viewMode = type;
//}

//double CameraParams::get_near_plane(){
//    return m_near_plane;
//}

//double CameraParams::get_far_plane(){
//    return m_far_plane;
//}

//double CameraParams::get_field_view_angle(){
//    return m_field_view_angle;
//}

//ProjectionType CameraParams::get_projection_type(){
//    return m_projectionType;
//}

//ViewMode CameraParams::get_view_mode(){
//    return m_viewMode;
//}

void Camera::set_params_on_server(){
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::near_plane), m_near_plane);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::far_plane), m_far_plane);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::field_view_angle), m_field_view_angle);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::orthographic_view_width), m_orthographic_view_width);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::stereo_eye_separation), m_stereo_eye_separation);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::stereo_focal_length), m_stereo_focal_length);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::parent_name), m_parentName);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::projection), projection_type_enum_to_str(m_projectionType));
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::mode), view_mode_enum_to_str(m_viewMode));
}

void Camera::update_params_from_server(){
    std::string projection_type, view_mode;
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::near_plane), m_near_plane);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::far_plane), m_far_plane);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::field_view_angle), m_field_view_angle);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::orthographic_view_width), m_orthographic_view_width);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::stereo_eye_separation), m_stereo_eye_separation);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::stereo_focal_length), m_stereo_focal_length);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::parent_name), m_parentName);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::projection), projection_type);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::mode), view_mode);

    if (projection_type.compare(projection_type_enum_to_str(ProjectionType::PERSPECTIVE)) == 0){
        m_projectionType = ProjectionType::PERSPECTIVE;
    }
    else if (projection_type.compare(projection_type_enum_to_str(ProjectionType::ORTHOGRAPHIC)) == 0){
        m_projectionType = ProjectionType::ORTHOGRAPHIC;
    }
    else{
        std::cerr << "ERROR! FOR CAMERA \"" << m_name << "\" PROJECTION TYPE \"" << projection_type << "\" NOT UNDERSTOOD\n";
        std::cerr << "VALID TYPES ARE: \n" <<
                     projection_type_enum_to_str(ProjectionType::PERSPECTIVE) <<
                     "\n" <<
                     projection_type_enum_to_str(ProjectionType::ORTHOGRAPHIC) <<
                     "\n";
    }


    if (view_mode.compare(view_mode_enum_to_str(ViewMode::MONO)) == 0){
        m_viewMode = ViewMode::MONO;
    }
    else if (view_mode.compare(view_mode_enum_to_str(ViewMode::STEREO)) == 0){
        m_viewMode = ViewMode::STEREO;
    }
    else{
        std::cerr << "ERROR! FOR CAMERA \"" << m_name << "\" VIEW MODE \"" << view_mode << "\" NOT UNDERSTOOD\n";
        std::cerr << "VALID MODES ARE: \n" <<
                     view_mode_enum_to_str(ViewMode::MONO) <<
                     "\n" <<
                     view_mode_enum_to_str(ViewMode::STEREO) <<
                     "\n";
    }
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
