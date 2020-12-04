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

    \author    Adnan Munawar
    \version   1.0$
*/
//==============================================================================

#include <adf_loader_base.h>

using namespace std;

ADFLoaderBase::ADFLoaderBase()
{
    m_loader = nullptr;
}

ADFLoaderBase::~ADFLoaderBase()
{
    cleanUp();
}

string ADFLoaderBase::getVersion()
{
    assert(m_loader == nullptr);
    return m_loader->getVersion();
}

bool ADFLoaderBase::setLoader(ADFLoaderBase *a_loader)
{
    m_loader = a_loader;
    return true;
}

bool ADFLoaderBase::loadObjectAttribs(string a_filepath, string a_objName, afObjectType a_type, afBaseObjectAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadObjectAttribs(a_filepath, a_objName, a_type, attribs);
}

bool ADFLoaderBase::loadLightAttribs(YAML::Node *a_node, afLightAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadLightAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadCameraAttribs(YAML::Node *a_node, afCameraAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadCameraAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadRigidBodyAttribs(YAML::Node *a_node, afRigidBodyAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadRigidBodyAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadSoftBodyAttribs(YAML::Node *a_node, afSoftBodyAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadSoftBodyAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadJointAttribs(YAML::Node *a_node, afJointAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadJointAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadRayTracerSensorAttribs(YAML::Node *a_node, afSensorAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadRayTracerSensorAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadActuatorAttribs(YAML::Node *a_node, afActuatorAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadActuatorAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadVehicleAttribs(YAML::Node *a_node, afVehicleAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadVehicleAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadInputDeviceAttributes(YAML::Node *a_node, afInputDeviceAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadInputDeviceAttributes(a_node, attribs);
}

bool ADFLoaderBase::loadMultiBodyAttribs(YAML::Node *a_node, afMultiBodyAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadMultiBodyAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadWorldAttribs(YAML::Node *a_node, afWorldAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadMultiBodyAttribs(a_node, attribs);
}

bool ADFLoaderBase::loadLaunchFileAttribs(string a_filepath, afLaunchAttributes *attribs)
{
    assert(m_loader == nullptr);
    return m_loader->loadLaunchFileAttribs(a_filepath, attribs);
}

bool ADFLoaderBase::cleanUp()
{
    if (m_loader != nullptr){
        delete m_loader;
    }
    return true;
}
