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

// ACKNOWLEDGEMENT: (OSRF GAZEBO SIM https://www.openrobotics.org/)
// Some of the source code in this file is taken from
// https://github.com/osrf/gazebo/blob/gazebo11/gazebo/common/Plugin.hh

#ifndef AF_PLUGIN_INTERFACE_H
#define AF_PLUGIN_INTERFACE_H

#include <afSystem.h>
#include <string>
#include <afPath.h>
#include <afEnums.h>

#ifndef _WIN32
  #include <unistd.h>
#endif
#include <dlfcn.h>

#include <sys/stat.h>
#include <iostream>

using namespace std;

// Forward declaration outside ambf namespace
class GLFWwindow;

namespace ambf {


enum afInitStatus{
  SUCCESS=1, ERROR=-1
};


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

template<class T>
class afPluginBase
{
public: typedef T* TPtr;

public: afPluginBase()
    {
        this->dlHandle = nullptr;
    }

public: virtual ~afPluginBase()
    {
    }

public: string getFilename() const
    {
        return this->filename;
    }

public: string getHandle() const
    {
        return this->handleName;
    }

public: static TPtr Create(const std::string &_filename,
                           const std::string &_name,
                           const std::string &_path)
    {
        TPtr result = nullptr;
        // PluginPtr result;
        struct stat st;
        bool found = false;
        string fullname, filename(_filename);
        list<string>::iterator iter;
        list<string> pluginPaths = afSystemPaths::getPluginPath();
        if (_path.empty() == false){
            pluginPaths.push_front(_path);
        }

#ifdef __APPLE__
        {
            size_t soSuffix = filename.rfind(".so");
            if (soSuffix != std::string::npos)
            {
                const std::string macSuffix(".dylib");
                filename.replace(soSuffix, macSuffix.length(), macSuffix);
            }
        }
#elif _WIN32
        {
            // replace .so with .dll
            size_t soSuffix = filename.rfind(".so");
            if (soSuffix != std::string::npos)
            {
                const std::string winSuffix(".dll");
                filename.replace(soSuffix, winSuffix.length(), winSuffix);
            }
            size_t libPrefix = filename.find("lib");
            if (libPrefix == 0)
            {
                // remove the lib prefix
                filename.erase(0, 3);
            }
        }
#endif  // ifdef __APPLE__

        for (iter = pluginPaths.begin();
             iter!= pluginPaths.end(); ++iter)
        {
            fullname = (*iter)+string("/")+filename;
            fullname = afPath(fullname).c_str();
            if (stat(fullname.c_str(), &st) == 0)
            {
                found = true;
                break;
            }
        }

        if (!found){
            cerr << "ERROR! PLUGIN: " << fullname << " NOT FOUND. TRYING WITHOUT PATH!"  "\n";
            fullname = filename;
        }

        fptr_union_t registerFunc;
        std::string registerName = "CreatePlugin";

        void *dlHandle = dlopen(fullname.c_str(), RTLD_LAZY|RTLD_GLOBAL);
        if (!dlHandle)
        {
            cerr << "ERROR! FAILED TO LOAD PLUGIN: " << fullname << ": "
                  << dlerror() << "!\n";
            return result;
        }

        registerFunc.ptr = dlsym(dlHandle, registerName.c_str());

        if (!registerFunc.ptr)
        {
            cerr << "ERROR! FAILED TO RESOLVE " << registerName
                  << ": " << dlerror() << "!\n";
            return result;
        }

        // Register the new controller.
        result = registerFunc.func();
        result->dlHandle = dlHandle;

        result->handleName = _name;
        result->filename = filename;

        return result;
    }

public: afPluginType GetType() const
    {
        return this->type;
    }

protected: afPluginType type;

protected: string filename;

protected: string handleName;

private: typedef union
    {
        T *(*func)();
        void *ptr;
    } fptr_union_t;

private: void *dlHandle;
};


///
/// \brief The afWorldPlugin class
///
class afSimulatorPlugin: public afPluginBase<afSimulatorPlugin>{
public:
    afSimulatorPlugin(){
        this->type = afPluginType::SIMULATOR;
    }

    virtual int init(int argc, char** argv, const afWorldPtr a_afWorld){return -1;}
    virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){}
    virtual void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes){}
    virtual void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos){}
    virtual void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos){}
    virtual void graphicsUpdate(){}
    virtual void physicsUpdate(double dt){}
    virtual void reset(){}
    virtual bool close(){return 0;}

protected:
    afWorldPtr m_worldPtr;
};


///
/// \brief The afWorldPlugin class
///
class afWorldPlugin: public afPluginBase<afWorldPlugin>{
public:
    afWorldPlugin(){
        this->type = afPluginType::WORLD;
    }

    virtual int init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs){return -1;}
    virtual void onModelAdd(const afModelPtr a_modelPtr){}
    virtual void onModelRemoval(const afModelPtr a_modelPtr){}
    virtual void onObjectAdd(const afBaseObjectPtr a_objectPtr){}
    virtual void onObjectRemoval(const afBaseObjectPtr a_objectPtr){}
    virtual void graphicsUpdate(){}
    virtual void physicsUpdate(double dt){}
    virtual void reset(){}
    virtual bool close(){return 0;}

protected:
    afWorldPtr m_worldPtr;
    afWorldAttribsPtr a_attribs;
};


///
/// \brief The afWorldPlugin class
///
class afModelPlugin: public afPluginBase<afModelPlugin>{
public:
    afModelPlugin(){
        this->type = afPluginType::MODEL;
    }

    virtual int init(const afModelPtr a_afModel, const afModelAttribsPtr a_modelAttribs){return -1;}
    virtual void onObjectAdd(const afBaseObjectPtr a_objectPtr){}
    virtual void onObjectRemoval(const afBaseObjectPtr a_objectPtr){}
    virtual void graphicsUpdate(){}
    virtual void physicsUpdate(double dt){}
    virtual void reset(){}
    virtual bool close(){return 0;}

protected:
    afModelPtr m_modelPtr;
    afModelAttribsPtr a_attribs;
};

///
/// \brief The afWorldPlugin class
///
class afObjectPlugin: public afPluginBase<afObjectPlugin>{
public:
    afObjectPlugin(){
        this->type = afPluginType::OBJECT;
    }

    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs){return -1;}
    virtual void graphicsUpdate(){}
    virtual void physicsUpdate(double dt){}
    virtual void reset(){}
    virtual bool close(){return 0;}

protected:
    afBaseObjectPtr m_objectPtr;
    afBaseObjectAttribsPtr a_attribs;
};

}

#define AF_REGISTER_SIMULATOR_PLUGIN(PluginClass) \
    extern "C" ambf::afSimulatorPlugin* CreatePlugin() \
    {\
        return new PluginClass(); \
    }

#define AF_REGISTER_MODEL_PLUGIN(PluginClass) \
    extern "C" ambf::afModelPlugin* CreatePlugin() \
    {\
        return new PluginClass(); \
    }

#define AF_REGISTER_WORLD_PLUGIN(PluginClass) \
    extern "C" ambf::afWorldPlugin* CreatePlugin() \
    {\
        return new PluginClass(); \
    }

#define AF_REGISTER_OBJECT_PLUGIN(PluginClass) \
    extern "C" ambf::afObjectPlugin* CreatePlugin() \
    {\
        return new PluginClass(); \
    }

#endif // AF_PLUGIN_INTERFACE_H
