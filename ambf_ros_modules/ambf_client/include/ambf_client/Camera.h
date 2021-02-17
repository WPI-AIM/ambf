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

#ifndef AFCAMERACOMM_H
#define AFCAMERACOMM_H

#include <string>
#include "ambf_client/CameraRosCom.h"

namespace ambf_client{

enum class ProjectionType{
    PERSPECTIVE,
    ORTHOGRAPHIC
};


enum class ViewMode{
    MONO,
    STEREO
};


enum class CameraParamsEnum{
    near_plane,
    far_plane,
    field_view_angle,
    orthographic_view_width,
    stereo_eye_separation,
    stereo_focal_length,
    parent_name,
    projection,
    mode
};


class CameraParams{

    friend class Camera;

public:

    CameraParams();

    inline void set_qualified_namespace(std::string a_base_prefix){m_base_prefix = a_base_prefix;}

    // Setters
    void set_near_plane(double val){m_near_plane = val;}
    void set_far_plane(double val){m_far_plane = val;}
    void set_field_view_angle(double val){m_field_view_angle = val;}
    void set_orthographic_view_width(double val){m_orthographic_view_width = val;}
    void set_steteo_eye_separation(double val){m_stereo_eye_separation = val;}
    void set_steteo_focal_length(double val){m_stereo_focal_length = val;}
    void set_projection_type(ProjectionType type){m_projection_type = type;}
    void set_view_mode(ViewMode view_mode){m_view_mode = view_mode;}

    // Getters
    double get_near_plane(){return m_near_plane;}
    double get_far_plane(){return m_far_plane;}
    double get_field_view_angle(){return m_field_view_angle;}
    double get_orthographic_view_width(){return m_orthographic_view_width;}
    double get_steteo_eye_separation(){return m_stereo_eye_separation;}
    double get_steteo_focal_length(){return m_stereo_focal_length;}
    ProjectionType get_projection_type(){return m_projection_type;}
    ViewMode get_view_mode(){return m_view_mode;}

    // This a flag to check if any param has been updated
    bool m_paramsChanged;

protected:

    // Namespace + obj_name is the base_prefix. E.g. /ambf/env/ + Camera1 = /ambf/env/Camera1 -> Base Prefix
    std::string m_base_prefix;

    // Datatyped Variables for params defined on the server
    double m_near_plane;
    double m_far_plane;
    double m_field_view_angle;
    double m_orthographic_view_width;
    double m_stereo_eye_separation;
    double m_stereo_focal_length;

    ProjectionType m_projection_type;
    ViewMode m_view_mode;
};

class Camera: public CameraRosCom, public CameraParams{
public:
    Camera(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
    ambf_msgs::CameraCmd get_command();
    void set_position(double px, double py, double pz);
    void set_orientation(double roll, double pitch, double yaw);
    void set_orientation(double qx, double qy, double qz, double qw);
    std::string get_parent_name(){return m_State.parent_name.data;}

    // This method updates from the ROS param server instead of topics
    void update_params_from_server();
    // This method may be called when AMBF starts to load the existing
    void set_params_on_server();
};
}

#endif
