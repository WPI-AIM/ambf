//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
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

//------------------------------------------------------------------------------
#ifndef AF_ENUMS_H
#define AF_ENUMS_H
//------------------------------------------------------------------------------

///
/// \brief The afActuatorType enum
///
enum class afActuatorType{
    CONSTRAINT = 0,
    INVALID = 1
};


///
/// \brief The afAxisType enum
///
enum class afAxisType{
    X = 0,
    Y = 1,
    Z = 2
};


///
/// \brief The afBodyType enum
///
enum class afBodyType{
    RIGID_BODY=0,
    SOFT_BODY=1
};


///
/// \brief The afType enum
///
enum class afType{
    INVALID,
    OBJECT,
    ACTUATOR,
    CAMERA,
    INPUT_DEVICE,
    LIGHT,
    RIGID_BODY,
    SOFT_BODY,
    GHOST_OBJECT,
    SENSOR,
    VEHICLE,
    VOLUME,
    MODEL,
    POINT_CLOUD,
    WORLD,
    JOINT
};


///
/// \brief The afPluginType enum
///
enum afPluginType{
    SIMULATOR,
    WORLD,
    MODEL,
    OBJECT
};


///
/// \brief The afControlType enum
///
enum class afControlType{
  POSITION=0,
  FORCE=1,
  VELOCITY=2
};


///
/// \brief The afGeometryType enum
///
enum class afGeometryType{
    INVALID=0,
    MESH=1,
    SINGLE_SHAPE=2,
    COMPOUND_SHAPE=3
};


///
/// \brief The JointType enum
///
enum class afJointType{
    REVOLUTE = 0,
    PRISMATIC = 1,
    LINEAR_SPRING = 2,
    TORSION_SPRING = 3,
    P2P = 4,
    FIXED = 5,
    CONE_TWIST = 6,
    SIX_DOF = 7,
    SIX_DOF_SPRING = 8,
    INVALID = 9
};


///
/// \brief The afPrimitiveShapeType enum
///
enum class afPrimitiveShapeType{
    INVALID = 0,
    PLANE = 1,
    BOX = 2,
    SPHERE = 3,
    CYLINDER = 4,
    CAPSULE = 5,
    CONE = 6,
};


enum class afCollisionMeshShapeType{
    CONCAVE_MESH=0,
    CONVEX_MESH=1,
    CONVEX_HULL=2,
    POINT_CLOUD=3
};


///
/// \brief The afSensorType enum
///
enum class afSensorType{
    RAYTRACER = 0,
    RANGE = 1,
    RESISTANCE = 2,
    INVALID = 3
};


///
/// \brief The afSensactorSpecificationType enum
///
enum class afSensactorSpecificationType{
    ARRAY = 0,
    MESH = 1,
    PARAMETRIC = 2,
    INVALID = 3
};


///
/// \brief The ShadowQuality enum
///
enum class afShadowQualityType{
    NO_SHADOW=0,
    VERY_LOW=1,
    LOW=2,
    MEDIUM=3,
    HIGH=4,
    VERY_HIGH=5
};


///
/// \brief The afWheelRepresentationType enum
///
enum class afWheelRepresentationType{
    MESH=0,
    RIGID_BODY=1,
    INVALID=2
};

#endif
