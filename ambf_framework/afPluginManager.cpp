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

#include <afPluginManager.h>
#include <afFramework.h>

using namespace std;
using namespace  ambf;

void afSimulatorPluginManager::init(int argc, char** argv, const afWorldPtr a_afWorld){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ;){
        if ((*it)->init(argc, argv, a_afWorld) == afInitStatus::ERROR){
            cerr << "ERROR! PLUGIN " << (*it)->getFilename() << " FAILED ON INITIALIZATION THEREFORE IGNORING!\n";
            m_plugins.erase(it);
        }
        else{
            ++it;
        }
    }
}

void afSimulatorPluginManager::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->keyboardUpdate(a_window, a_key, a_scancode, a_action, a_mods);
    }
}

void afSimulatorPluginManager::mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->mouseBtnsUpdate(a_window, a_button, a_action, a_modes);
    }
}

void afSimulatorPluginManager::mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->mousePosUpdate(a_window, x_pos, y_pos);
    }
}

void afSimulatorPluginManager::mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->mouseScrollUpdate(a_window, x_pos, y_pos);
    }
}

void afSimulatorPluginManager::graphicsUpdate(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->graphicsUpdate();
    }
}

void afSimulatorPluginManager::physicsUpdate(double dt){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->physicsUpdate(dt);
    }
}

void afSimulatorPluginManager::reset(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool afSimulatorPluginManager::close(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
    return true;
}

bool afWorldPluginManager::loadPlugin(afWorldPtr worldPtr, afWorldAttribsPtr attribs, string lib_name, string plugin_name, string path){
    bool res = load<afWorld, afWorldAttributes>(worldPtr, attribs, lib_name, plugin_name, path);
    return res;
}

bool afWorldPluginManager::loadPlugin(afWorldPtr worldPtr, afWorldAttribsPtr attribs, afWorldPlugin *plugin){
    bool res = false;
    if (plugin){
        if (plugin->init(worldPtr, attribs)){
            res = add(plugin);
        }
    }
    return res;
}

void ambf::afWorldPluginManager::init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ;){
        if ((*it)->init(a_afWorld, a_worldAttribs) == afInitStatus::ERROR){
            cerr << "ERROR! PLUGIN " << (*it)->getFilename() << " FAILED ON INITIALIZATION THEREFORE IGNORING!\n";
            m_plugins.erase(it);
        }
        else{
            ++it;
        }
    }
}

void afWorldPluginManager::onModelAdd(const afModelPtr a_modelPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onModelAdd(a_modelPtr);
    }
}

void afWorldPluginManager::onModelRemoval(const afModelPtr a_modelPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onModelRemoval(a_modelPtr);
    }
}

void afWorldPluginManager::onObjectAdd(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectAdd(a_objectPtr);
    }
}

void afWorldPluginManager::onObjectRemoval(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectRemoval(a_objectPtr);
    }
}

void afWorldPluginManager::graphicsUpdate()
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->graphicsUpdate();
    }
}

void ambf::afWorldPluginManager::physicsUpdate(double dt)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->physicsUpdate(dt);
    }
}

void ambf::afWorldPluginManager::reset()
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool ambf::afWorldPluginManager::close()
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
    return true;
}

bool afModelPluginManager::loadPlugin(afModelPtr modelPtr, afModelAttribsPtr attribs, string lib_name, string plugin_name, string path){
    bool res = load<afModel, afModelAttributes>(modelPtr, attribs, lib_name, plugin_name, path);
    return res;
}

bool afModelPluginManager::loadPlugin(afModelPtr modelPtr, afModelAttribsPtr attribs, afModelPlugin *plugin)
{
    bool res = false;
    if (plugin){
        if (plugin->init(modelPtr, attribs)){
            res = add(plugin);
        }
    }
    return res;
}

void afModelPluginManager::init(const afModelPtr a_afModel, const afModelAttribsPtr a_modelAttribs)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ;){
        if ((*it)->init(a_afModel, a_modelAttribs) == afInitStatus::ERROR){
            cerr << "ERROR! PLUGIN " << (*it)->getFilename() << " FAILED ON INITIALIZATION THEREFORE IGNORING!\n";
            m_plugins.erase(it);
        }
        else{
            ++it;
        }
    }
}

void afModelPluginManager::onObjectAdd(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectAdd(a_objectPtr);
    }
}

void afModelPluginManager::onObjectRemoval(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectRemoval(a_objectPtr);
    }
}

void afModelPluginManager::graphicsUpdate()
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->graphicsUpdate();
    }
}

void afModelPluginManager::physicsUpdate(double dt)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->physicsUpdate(dt);
    }
}

void afModelPluginManager::reset()
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool afModelPluginManager::close()
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
    return true;
}

bool afBaseObjectPluginManager::loadPlugin(afBaseObjectPtr objPtr, afBaseObjectAttribsPtr attribs, string lib_name, string plugin_name, string path){
    bool res = load<afBaseObject, afBaseObjectAttributes>(objPtr, attribs, lib_name, plugin_name, path);
    return res;
}

bool afBaseObjectPluginManager::loadPlugin(afBaseObjectPtr objPtr, afBaseObjectAttribsPtr attribs, afObjectPlugin* plugin){
    bool res = false;
    if (plugin){
        if (plugin->init(objPtr, attribs)){
            res = add(plugin);
        }
    }
    return res;
}

void afBaseObjectPluginManager::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ;){
        if ((*it)->init(a_afObjectPtr, a_objectAttribs) == afInitStatus::ERROR){
            cerr << "ERROR! PLUGIN " << (*it)->getFilename() << " FAILED ON INITIALIZATION THEREFORE IGNORING\n";
            m_plugins.erase(it);
        }
        else{
            ++it;
        }
    }
}

void afBaseObjectPluginManager::graphicsUpdate()
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->graphicsUpdate();
    }
}

void afBaseObjectPluginManager::physicsUpdate(double dt)
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->physicsUpdate(dt);
    }
}

void afBaseObjectPluginManager::reset()
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool afBaseObjectPluginManager::close()
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
    return true;
}
