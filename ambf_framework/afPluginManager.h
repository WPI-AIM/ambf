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
#ifndef AF_PLUGIN_MANAGER_H
#define AF_PLUGIN_MANAGER_H

#include <afPluginInterface.h>

using namespace std;

namespace ambf {


// Forward declaration
class afWorld;
class afModel;
class afBaseObject;


typedef afWorld* afWorldPtr;
typedef afModel* afModelPtr;
typedef afBaseObject* afBaseObjectPtr;

class afWorldAttributes;
class afModelAttributes;
class afBaseObjectAttributes;

typedef afWorldAttributes* afWorldAttribsPtr;
typedef afModelAttributes* afModelAttribsPtr;
typedef afBaseObjectAttributes* afBaseObjectAttribsPtr;


template <class T>
class afBasePluginManager{
public:
    ~afBasePluginManager(){
        for (typename vector<T*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
            delete (*it);
        }
    }

    vector<T*>* getPlugins(){
        return &m_plugins;
    }


protected:
    virtual bool add(string lib_name, string plugin_name, string path=""){
        T* plugin = T::Create(lib_name, plugin_name, path);
        return add(plugin);
    }

    virtual bool add(T* plugin){
        if (plugin){
            m_plugins.push_back(plugin);
            return true;
        }
        else{
            return false;
        }
    }

protected:
    vector<T*> m_plugins;
};

class afSimulatorPluginManager: public afBasePluginManager<afSimulatorPlugin>{
public:
    bool loadPlugin(int argc, char** argv, const afWorldPtr a_afWorld, string lib_name, string plugin_name, string path="");

    bool loadPlugin(int argc, char** argv, const afWorldPtr a_afWorld, afSimulatorPlugin*);

    void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

    void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes);

    void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos);

    void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos);

    void graphicsUpdate();

    void physicsUpdate(double dt);

    void reset();

    bool close();
};


class afWorldPluginManager: public afBasePluginManager<afWorldPlugin>{
public:
    bool loadPlugin(afWorldPtr, afWorldAttribsPtr, string lib_name, string plugin_name, string path="");

    bool loadPlugin(afWorldPtr, afWorldAttribsPtr, afWorldPlugin*);

    void onModelAdd(const afModelPtr a_modelPtr);

    void onModelRemoval(const afModelPtr a_modelPtr);

    void onObjectAdd(const afBaseObjectPtr a_objectPtr);

    void onObjectRemoval(const afBaseObjectPtr a_objectPtr);

    void graphicsUpdate();

    void physicsUpdate(double dt);

    void reset();

    bool close();

};


class afModelPluginManager: public afBasePluginManager<afModelPlugin>{
public:
    bool loadPlugin(afModelPtr, afModelAttribsPtr, string lib_name, string plugin_name, string path="");

    bool loadPlugin(afModelPtr, afModelAttribsPtr, afModelPlugin*);

    void onObjectAdd(const afBaseObjectPtr a_objectPtr);

    void onObjectRemoval(const afBaseObjectPtr a_objectPtr);

    void graphicsUpdate();

    void physicsUpdate(double dt);

    void reset();

    bool close();

};


class afBaseObjectPluginManager: public afBasePluginManager<afObjectPlugin>{
public:
    bool loadPlugin(afBaseObjectPtr, afBaseObjectAttribsPtr, string lib_name, string plugin_name, string path="");

    bool loadPlugin(afBaseObjectPtr, afBaseObjectAttribsPtr, afObjectPlugin*);

    void graphicsUpdate();

    void physicsUpdate(double dt);

    void reset();

    bool close();
};

}


#endif // AF_PLUGIN_CALLBACK_HELPERS_H
