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

    \author    <amunawa2@jh.edu>
    \author    Adnan Munawar
*/
//==============================================================================
#include "WorldPluginsLoader.h"
#include <yaml-cpp/yaml.h>

int afWorldPluginsLoader::init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs){
    m_worldPtr = a_afWorld;

    for (auto obj : m_worldPtr->getCameras()) {
        onObjectAdd(obj);
    }

    for (auto obj : m_worldPtr->getLights()) {
        onObjectAdd(obj);
    }

    for (auto obj : m_worldPtr->getRigidBodies()) {
        onObjectAdd(obj);
    }

    afPluginAttributes commPluginAttribs;

    commPluginAttribs.m_name = "world_comm";
    commPluginAttribs.m_filename = "libworld_comm_plugin.so";
    m_worldPluginsAttribs.push_back(commPluginAttribs);

    commPluginAttribs.m_name = "object_comm";
    commPluginAttribs.m_filename = "libobject_comm_plugin.so";
    m_objectPluginsAttribs.push_back(commPluginAttribs);

    m_worldPtr->loadPlugins(&m_worldPluginsAttribs);

    return 1;
}

void afWorldPluginsLoader::graphicsUpdate()
{

}

void afWorldPluginsLoader::physicsUpdate(double dt)
{

}

bool afWorldPluginsLoader::close()
{

    return true;
}

void afWorldPluginsLoader::onObjectAdd(const afBaseObjectPtr a_objectPtr){
    cerr << "INFO! WORLD PLUGIN! OBJECT ADDED:\n " << a_objectPtr->getName() << endl;
    if (!a_objectPtr->getAttributes()->m_communicationAttribs.m_passive) {
        a_objectPtr->loadPlugins(&m_objectPluginsAttribs);
    }
    
    if (a_objectPtr->getType() == afType::CAMERA){
        vector<afPluginAttributes> morePlugins;
        afPluginAttributes videoPlugin;
        videoPlugin.m_filename = "libvideo_streamer_plugin.so";
        videoPlugin.m_name = "video_plugin";
        afPluginAttributes depthPlugin;
        depthPlugin.m_filename = "libdepth_streamer_plugin.so";
        depthPlugin.m_name = "depth_plugin";

        afCameraPtr camPtr = (afCameraPtr) a_objectPtr;
        afCameraAttributes* camAttribs = (afCameraAttributes*)camPtr->getAttributes();
        if (camAttribs->m_publishImage) morePlugins.push_back(videoPlugin);
        if (camAttribs->m_publishDepth) morePlugins.push_back(depthPlugin);

        a_objectPtr->loadPlugins(&morePlugins);
    }
}

void afWorldPluginsLoader::onModelAdd(const afModelPtr a_modelPtr){
    cerr << "INFO! WORLD PLUGIN! MODEL ADDED:\n " << a_modelPtr->getName() << endl;
//    a_modelPtr->loadPlugins(a_modelPtr, a_modelPtr->getAttributes(), &m_objectPluginsAttribs);
}

