//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2024, AMBF
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
*/
//==============================================================================

#include "ambf_server/Light.h"
namespace ambf_comm{

const std::string light_type_enum_to_str(LightType enumVal)
{
    if (enumVal == LightType::SPOT) return "SPOT";
    else if (enumVal == LightType::POINT) return "POINT";
    else if (enumVal == LightType::DIRECTIONAL) return "DIRECTIONAL";
    return "";
}

const std::string light_param_enum_to_str(LightParamsEnum enumVal)
{
    if (enumVal == LightParamsEnum::cuttoff_angle) return "cutoff_angle";
    else if (enumVal == LightParamsEnum::parent_name) return "parent_name";
    else if (enumVal == LightParamsEnum::type) return "type";
    else if (enumVal == LightParamsEnum::attenuation) return "attenuation";
    return "";
}

LightParams::LightParams(){
    m_paramsChanged = false;
    m_attenuation["constant"] = 1.0;
    m_attenuation["linear"] = 0.0;
    m_attenuation["quadratic"] = 0.0;
}

void LightParams::set_attenuation(double cons, double lin, double quad)
{
    m_attenuation["constant"] = cons;
    m_attenuation["linear"] = lin;
    m_attenuation["quadratic"] = quad;
}

void Light::set_params_on_server(){
    ambf_ral::set_parameter(m_nodePtr,
                            m_base_prefix + light_param_enum_to_str(LightParamsEnum::cuttoff_angle),
                            m_cuttoff_angle);
    ambf_ral::set_parameter(m_nodePtr,
                            m_base_prefix + light_param_enum_to_str(LightParamsEnum::parent_name),
                            m_State.parent_name.data);
    ambf_ral::set_parameter(m_nodePtr,
                            m_base_prefix + light_param_enum_to_str(LightParamsEnum::type),
                            light_type_enum_to_str(m_light_type));

    for (auto const& mIt : m_attenuation){
        ambf_ral::set_parameter(m_nodePtr,
                                m_base_prefix + light_param_enum_to_str(LightParamsEnum::attenuation) + "/" + mIt.first, mIt.second);
    }

}

void Light::update_params_from_server(){
    double ca;
    std::string pn;
    std::string lt;
    std::map<std::string, double> att;
    LightType lt_enum;
    ambf_ral::get_parameter(m_nodePtr,
                            m_base_prefix + light_param_enum_to_str(LightParamsEnum::cuttoff_angle),
                            ca);
    ambf_ral::get_parameter(m_nodePtr,
                            m_base_prefix + light_param_enum_to_str(LightParamsEnum::parent_name),
                            pn);
    ambf_ral::get_parameter(m_nodePtr,
                            m_base_prefix + light_param_enum_to_str(LightParamsEnum::type),
                            lt);
    for (auto & mIt : m_attenuation){
        double val;
        ambf_ral::get_parameter(m_nodePtr,
                                m_base_prefix + light_param_enum_to_str(LightParamsEnum::attenuation) + "/" + mIt.first, val);
        if (val != mIt.second){
            // std::cerr << "INFO! PARAM CHANGED " << mIt.first << " OLD: " << mIt.second << " NEW: " << val << std::endl;
            m_paramsChanged = true;
            mIt.second = val;
        }
    }

    if (lt.compare(light_type_enum_to_str(LightType::SPOT)) == 0){
        lt_enum = LightType::SPOT;
    }
    else if (lt.compare(light_type_enum_to_str(LightType::POINT)) == 0){
        lt_enum = LightType::POINT;
    }
    else if (lt.compare(light_type_enum_to_str(LightType::DIRECTIONAL)) == 0){
        lt_enum = LightType::DIRECTIONAL;
    }
    else{
        std::cerr << "ERROR! FOR LIGHT \"" << m_name << "\" LIGHT TYPE \"" << lt << "\" NOT UNDERSTOOD\n";
        std::cerr << "VALID TYPES ARE: \n" <<
                     light_type_enum_to_str(LightType::SPOT) <<
                     "\n" <<
                     light_type_enum_to_str(LightType::POINT) <<
                     "\n" <<
                     light_type_enum_to_str(LightType::DIRECTIONAL) <<
                     "\n";
    }

    if (ca != m_cuttoff_angle ||
            lt_enum != m_light_type ||
            pn.compare(m_State.parent_name.data) !=0){
        m_paramsChanged = true;
        std::cerr << "INFO! PARAMS CHANGED FOR \"" << m_name << "\"\n";
    }


    // Finally update the local copies of the params
    m_cuttoff_angle = ca;
    m_State.parent_name.data = pn;
    m_light_type = lt_enum;
}

Light::Light(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out):
    LightRosCom(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
    m_base_prefix = a_namespace + a_name + '/';
}

void Light::cur_position(double px, double py, double pz){
    m_trans.setOrigin(tf2::Vector3(px, py, pz));
    m_State.pose.position.x = px;
    m_State.pose.position.y = py;
    m_State.pose.position.z = pz;
}

void Light::cur_orientation(double roll, double pitch, double yaw){
    tf2::Quaternion rot_quat;
    rot_quat.setRPY(roll, pitch, yaw);
    m_trans.setRotation(rot_quat);
    m_State.pose.orientation = tf2::toMsg(rot_quat);
}

void Light::cur_orientation(double qx, double qy, double qz, double qw){
    tf2::Quaternion rot_quat(qx, qy, qz, qw);
    m_trans.setRotation(rot_quat);
    m_State.pose.orientation = tf2::toMsg(rot_quat);
}


extern "C"{

Light* create_light(std::string a_name, std::string a_namespace="/ambf_comm/", int a_min_freq=50, int a_max_freq=1000, double time_out=0.5){
    return new Light(a_name, a_namespace, a_min_freq, a_max_freq, time_out);
}

void destroy_light(Light* obj){
    delete obj;
}

}
}
