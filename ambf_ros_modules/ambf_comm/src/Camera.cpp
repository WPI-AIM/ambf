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

#include "ambf_comm/Camera.h"
namespace ambf_comm{

const std::string projection_type_enum_to_str(ProjectionType enumVal)
{
    if (enumVal == ProjectionType::PERSPECTIVE) return "PERSPECTIVE";
    else if (enumVal == ProjectionType::ORTHOGRAPHIC) return "ORTHOGRAPHIC";
}

const std::string view_mode_enum_to_str(ViewMode enumVal)
{
    if (enumVal == ViewMode::MONO) return "MONO";
    else if (enumVal == ViewMode::STEREO) return "STEREO";
}

const std::string camera_param_enum_to_str(CameraParamsEnum enumVal)
{
    if (enumVal == CameraParamsEnum::near_plane) return "near_plane";
    else if (enumVal == CameraParamsEnum::far_plane) return "far_plane";
    else if (enumVal == CameraParamsEnum::field_view_angle) return "field_view_angle";
    else if (enumVal == CameraParamsEnum::orthographic_view_width) return "orthographic_view_width";
    else if (enumVal == CameraParamsEnum::stereo_eye_separation) return "stereo_eye_separation";
    else if (enumVal == CameraParamsEnum::stereo_focal_length) return "stereo_focal_length";
    else if (enumVal == CameraParamsEnum::parent_name) return "parent_name";
    else if (enumVal == CameraParamsEnum::projection) return "projection";
    else if (enumVal == CameraParamsEnum::mode) return "mode";

}

CameraParams::CameraParams(){
    m_paramsChanged = false;
    m_projection_type = ProjectionType::PERSPECTIVE;
    m_view_mode = ViewMode::MONO;
}

void Camera::set_params_on_server(){
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::near_plane), m_near_plane);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::far_plane), m_far_plane);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::field_view_angle), m_field_view_angle);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::orthographic_view_width), m_orthographic_view_width);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::stereo_eye_separation), m_stereo_eye_separation);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::stereo_focal_length), m_stereo_focal_length);
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::projection), projection_type_enum_to_str(m_projection_type));
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::mode), view_mode_enum_to_str(m_view_mode));
    nodePtr->setParam(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::parent_name), m_State.parent_name.data);
}

void Camera::update_params_from_server(){
    double np, fp, fva, ovw, ses, sfl;
    std::string pn;
    std::string pt, vm;
    ProjectionType pt_enum;
    ViewMode vm_enum;

    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::near_plane), np);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::far_plane), fp);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::field_view_angle), fva);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::orthographic_view_width), ovw);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::stereo_eye_separation), ses);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::stereo_focal_length), sfl);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::projection), pt);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::mode), vm);
    nodePtr->getParamCached(m_base_prefix + "/" + camera_param_enum_to_str(CameraParamsEnum::parent_name), pn);

    if (pt.compare(projection_type_enum_to_str(ProjectionType::PERSPECTIVE)) == 0){
        pt_enum = ProjectionType::PERSPECTIVE;
    }
    else if (pt.compare(projection_type_enum_to_str(ProjectionType::ORTHOGRAPHIC)) == 0){
        pt_enum = ProjectionType::ORTHOGRAPHIC;
    }
    else{
        std::cerr << "ERROR! FOR CAMERA \"" << m_name << "\" PROJECTION TYPE \"" << pt << "\" NOT UNDERSTOOD\n";
        std::cerr << "VALID TYPES ARE: \n" <<
                     projection_type_enum_to_str(ProjectionType::PERSPECTIVE) <<
                     "\n" <<
                     projection_type_enum_to_str(ProjectionType::ORTHOGRAPHIC) <<
                     "\n";
    }


    if (vm.compare(view_mode_enum_to_str(ViewMode::MONO)) == 0){
        vm_enum = ViewMode::MONO;
    }
    else if (vm.compare(view_mode_enum_to_str(ViewMode::STEREO)) == 0){
        vm_enum = ViewMode::STEREO;
    }
    else{
        std::cerr << "ERROR! FOR CAMERA \"" << m_name << "\" VIEW MODE \"" << vm << "\" NOT UNDERSTOOD\n";
        std::cerr << "VALID MODES ARE: \n" <<
                     view_mode_enum_to_str(ViewMode::MONO) <<
                     "\n" <<
                     view_mode_enum_to_str(ViewMode::STEREO) <<
                     "\n";
    }

    if (np != m_near_plane ||
            fp != m_far_plane ||
            fva != m_field_view_angle ||
            ovw != m_orthographic_view_width ||
            ses != m_stereo_eye_separation ||
            sfl != m_stereo_focal_length ||
            pt_enum != m_projection_type ||
            vm_enum != m_view_mode ||
            pn.compare(m_State.parent_name.data) !=0){
        m_paramsChanged = true;
        std::cerr << "INFO! PARAMS CHANGED FOR \"" << m_name << "\"\n";
    }

    // Finally update the local copies of the params
    m_near_plane = np;
    m_far_plane = fp;
    m_field_view_angle = fva;
    m_orthographic_view_width = ovw;
    m_stereo_eye_separation = ses;
    m_stereo_focal_length = sfl;
    m_projection_type = pt_enum;
    m_view_mode = vm_enum;
    m_State.parent_name.data = pn;
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
