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

#ifndef AFCAMERACOMM_H
#define AFCAMERACOMM_H

#include <string>
#include "ambf_comm/CameraRosCom.h"

namespace ambf_comm{

enum ProjectionType{
  PERSPECTIVE, ORTHOGONAL
};

static const char* ProjectionEnumStr[] = {"PERSPECTIVE", "ORTHOGONAL"};


enum ViewType{
  MONO, STEREO
};

static const char* ViewEnumStr[] = {"MONO", "STEREO"};


enum CameraParamsEnum{
    look_at, up, near_plane, far_plane, parent, projection, type
};

static const char* CameraParamEnumStr[] = {"look_at", "up", "near_plane", "far_plane", "parent", "projection", "type"};


class CameraParams{

    friend class Camera;

public:

    CameraParams();

    inline void set_qualified_namespace(std::string a_base_prefix){m_base_prefix = a_base_prefix;}
    inline bool have_params_changed(){return m_paramsChanged;}

    // Setters
    void set_up_vector(double x, double y, double z);
    void set_look_vector(double x, double y, double z);
    void set_near_plane(double val);
    void set_far_plane(double val);
    void set_projection_type(ProjectionType type);
    void set_view_type(ViewType type);

    // Getters
    std::vector<double> get_up_vector();
    std::vector<double> get_look_vector();
    double get_near_plane();
    double get_far_plane();
    ProjectionType get_projection_type(ProjectionType type);
    ViewType get_view_type(ViewType type);

    // This a flag to check if any param has been updated
    bool m_paramsChanged;

protected:
    // Namespace + obj_name is the base_prefix. E.g. /ambf/env/ + Camera1 = /ambf/env/Camera1 -> Base Prefix
    std::string m_base_prefix;

    // Datatyped Variables for params defined on the server
    std::vector<double> m_up;
    std::vector<double> m_look_at;
    double m_near_plane, m_far_plane;

    std::string m_projectionType;
    std::string m_viewType;
};

class Camera: public CameraRosCom, public CameraParams{
public:
    Camera(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
    ambf_msgs::CameraCmd get_command();
    inline void set_name(std::string name){m_State.name.data = name;}
    void cur_position(double px, double py, double pz);
    void cur_orientation(double roll, double pitch, double yaw);
    void cur_orientation(double qx, double qy, double qz, double qw);
    void set_wall_time(double a_sec);
    inline void set_sim_time(double a_sec){ m_State.sim_time = a_sec;}
    inline void increment_sim_step(){m_State.sim_step++;}
    inline void set_sim_step(uint step){m_State.sim_step = step;}

    // This method updates from the ROS param server instead of topics
    void update_params_from_server();
    // This method may be called when AMBF starts to load the existing
    void set_params_on_server();
};
}

#endif
