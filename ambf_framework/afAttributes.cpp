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

#include "afAttributes.h"

using namespace ambf;
using namespace chai3d;

///
/// \brief afShapeGeometry::setScale
/// \param a_scale
///
afPrimitiveShapeAttributes::afPrimitiveShapeAttributes()
{
    m_posOffset.set(0, 0, 0);
    m_rotOffset.identity();
    m_shapeType = afPrimitiveShapeType::INVALID;
    m_axisType = afAxisType::Z;
}

///
/// \brief afShapeGeometry::copyShapeOffsetData
/// \param offsetNode
/// \return
///
bool afPrimitiveShapeAttributes::copyShapeOffsetData(YAML::Node *offset_node)
{
    bool valid = true;

    YAML::Node offsetNode = *offset_node;

    if (offsetNode.IsDefined()){

        if (offsetNode["position"].IsDefined()){
            double px = offsetNode["position"]["x"].as<double>();
            double py = offsetNode["position"]["y"].as<double>();
            double pz = offsetNode["position"]["z"].as<double>();
            m_posOffset.set(px, py, pz);
        }

        if (offsetNode["orientation"].IsDefined()){
            double roll =  offsetNode["orientation"]["r"].as<double>();
            double pitch = offsetNode["orientation"]["p"].as<double>();
            double yaw =   offsetNode["orientation"]["y"].as<double>();
            m_rotOffset.setExtrinsicEulerRotationRad(roll,pitch,yaw,cEulerOrder::C_EULER_ORDER_XYZ);
        }
    }
    else{
        valid = false;
    }

    return valid;
}

///
/// \brief afPrimitiveShape::copyPrimitiveShapeData
/// \param shape_node
/// \return
///
bool afPrimitiveShapeAttributes::copyPrimitiveShapeData(YAML::Node *shape_node)
{
    bool valid = true;

    YAML::Node shapeNode = *shape_node;

    if(shapeNode["axis"].IsDefined()){
        std::string axis = shapeNode["axis"].as<std::string>();

        if (axis.compare("x") == 0 || axis.compare("X") == 0){
            m_axisType = afAxisType::X;
        }
        else if (axis.compare("y") == 0 || axis.compare("Y") == 0){
            m_axisType = afAxisType::Y;
        }
        else if (axis.compare("z") == 0 || axis.compare("Z") == 0){
            m_axisType = afAxisType::Z;
        }
        else{
            std::cerr << "WARNING: Axis string \"" << axis << "\" not understood!\n";
            m_axisType = afAxisType::Z;
        }
    }

    switch (m_shapeType) {
    case afPrimitiveShapeType::BOX:{
        double dx, dy, dz;
        dx = shapeNode["x"].as<double>();
        dy = shapeNode["y"].as<double>();
        dz = shapeNode["z"].as<double>();
        m_dimensions.set(dx, dy, dz);
        break;
    }
    case afPrimitiveShapeType::SPHERE:{
        m_radius = shapeNode["radius"].as<double>();
        break;
    }
    case afPrimitiveShapeType::CAPSULE:{
        m_radius = shapeNode["radius"].as<double>();
        m_height = shapeNode["height"].as<double>();
        break;
    }
    case afPrimitiveShapeType::CONE:{
        m_radius = shapeNode["radius"].as<double>();
        m_height = shapeNode["height"].as<double>();
        break;
    }
    case afPrimitiveShapeType::PLANE:{
        double nx, ny, nz;
        nx = shapeNode["normal"]["x"].as<double>();
        ny = shapeNode["normal"]["y"].as<double>();
        nz = shapeNode["normal"]["z"].as<double>();
        m_planeNormal.set(nx, ny, nz);
        m_planeConstant = shapeNode["offset"].as<double>();
        break;
    }
    default:{
        valid = false;
        break;
    }
    }

    return valid;
}


///
/// \brief afPrimitiveGeometry::setPlaneData
/// \param normal_x
/// \param normal_y
/// \param normal_z
/// \param plane_constant
///
void afPrimitiveShapeAttributes::setPlaneData(double normal_x, double normal_y, double normal_z, double plane_constant)
{
    m_planeNormal.set(normal_x, normal_y, normal_z);
    m_planeConstant = plane_constant;
    m_shapeType = afPrimitiveShapeType::PLANE;
}


////
/// \brief afPrimitiveGeometry::setBoxData
/// \param dimension_x
/// \param dimension_y
/// \param dimension_z
///
void afPrimitiveShapeAttributes::setBoxData(double dimension_x, double dimension_y, double dimension_z)
{
    m_dimensions.set(dimension_x, dimension_y, dimension_z);
    m_shapeType = afPrimitiveShapeType::BOX;
}


///
/// \brief afPrimitiveGeometry::setSphereData
/// \param radius
///
void afPrimitiveShapeAttributes::setSphereData(double radius)
{
    m_radius = radius;
    m_shapeType = afPrimitiveShapeType::SPHERE;
}


///
/// \brief afPrimitiveGeometry::setCapsuleData
/// \param radius
/// \param height
/// \param axis
///
void afPrimitiveShapeAttributes::setCapsuleData(double radius, double height, afAxisType axis)
{
    m_radius = radius;
    m_height = height;
    m_axisType = axis;
    m_shapeType = afPrimitiveShapeType::CAPSULE;
}


///
/// \brief afPrimitiveGeometry::setConeData
/// \param radius
/// \param height
/// \param axis
///
void afPrimitiveShapeAttributes::setConeData(double radius, double height, afAxisType axis)
{
    m_radius = radius;
    m_height = height;
    m_axisType = axis;
    m_shapeType = afPrimitiveShapeType::CONE;
}


///
/// \brief afPrimitiveGeometry::setPosOffset
/// \param px
/// \param py
/// \param pz
///
void afPrimitiveShapeAttributes::setPosOffset(double px, double py, double pz)
{
    m_posOffset.set(px, py, pz);
}


///
/// \brief afPrimitiveGeometry::setRotOffset
/// \param roll
/// \param pitch
/// \param yaw
///
void afPrimitiveShapeAttributes::setRotOffset(double roll, double pitch, double yaw)
{
    m_rotOffset.setExtrinsicEulerRotationRad(roll,pitch,yaw,cEulerOrder::C_EULER_ORDER_XYZ);
}


///
/// \brief afPrimitiveGeometry::setScale
/// \param a_scale
///
void afPrimitiveShapeAttributes::setScale(double a_scale)
{
    m_scale = a_scale;
}
