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
#include <adf_loader_interface.h>
#include <version_1_0/adf_loader_1_0.h>

#include <assert.h>
#include <iostream>

using namespace std;
using namespace adf_loader_1_0;



///
/// \brief afConfigHandler::get_color_rgba
/// \param a_color_name
/// \return
///
adfVersion ADFLoaderInterface::getFileVersion(string a_filepath)
{
    try{
        YAML::Node node = YAML::LoadFile(a_filepath);
        return getFileVersion(&node, a_filepath);
    }
    catch(YAML::Exception& e){
        cerr << e.what() << endl;
        cerr << "Failed to Load " << a_filepath << endl;
        return adfVersion::INVALID;
    }
}

adfVersion ADFLoaderInterface::getFileVersion(YAML::Node *a_node, string a_filepath)
{
    assert(a_node != nullptr);

    YAML::Node &node = *a_node;

    YAML::Node versionNode = node["adf version"];

    if (versionNode.IsDefined()){
        string versionStr = versionNode.as<string>();

        return getVersionFromString(versionStr);
    }
    else{
        cerr << "WARNING! For File \"" << a_filepath  << "\", ADF version not defined thus assuming VERSION_1_0" << endl;
        return adfVersion::VERSION_1_0;
    }
}

bool ADFLoaderInterface::setLoaderVersion(adfVersion a_version){
    switch (a_version) {
    case adfVersion::VERSION_1_0:{
        if (m_loader == nullptr){
            ADFLoader_1_0* loader = new ADFLoader_1_0;
            setLoader(loader);
        }

        if (getVersionFromString(getLoaderVersion()) == a_version){
            // Have already loaded the right version. Ignore
            break;
        }
        else{
            ADFLoader_1_0* loader = new ADFLoader_1_0;
            setLoader(loader);
        }
        break;
    }
    default:
        break;
    }

    return true;
}

bool ADFLoaderInterface::setLoaderVersionForFile(string a_filepath)
{
    adfVersion version = getFileVersion(a_filepath);

    if (version == adfVersion::INVALID){
        cerr << "ERROR! COULDN'T DETERMINE THE CORRECT ADF LOADER FOR THE FILE \"" << a_filepath << "\"" << endl;
        return 0;
    }
    setLoaderVersion(version);
    return true;
}

adfVersion ADFLoaderInterface::getVersionFromString(string a_str){
    if (a_str.compare("1.0") == 0){
        return adfVersion::VERSION_1_0;
    }
    else{
        return adfVersion::INVALID;
    }
}

vector<double> ADFLoaderInterface::getColorRGBA(string a_color_name){
    vector<double> rgba = {0.5, 0.5, 0.5, 1.0};
    // Help from https://stackoverflow.com/questions/15425442/retrieve-random-key-element-for-stdmap-in-c
    if(strcmp(a_color_name.c_str(), "random") == 0 || strcmp(a_color_name.c_str(), "RANDOM") == 0){
        YAML::const_iterator it = m_colorsNode.begin();
        advance(it, rand() % m_colorsNode.size());
        rgba[0] = it->second["r"].as<int>() / 255.0;
        rgba[1] = it->second["g"].as<int>() / 255.0;
        rgba[2] = it->second["b"].as<int>() / 255.0;
        rgba[3] = it->second["a"].as<int>() / 255.0;
    }
    else if(m_colorsNode[a_color_name].IsDefined()){
        rgba[0] = m_colorsNode[a_color_name]["r"].as<int>() / 255.0;
        rgba[1] = m_colorsNode[a_color_name]["g"].as<int>() / 255.0;
        rgba[2] = m_colorsNode[a_color_name]["b"].as<int>() / 255.0;
        rgba[3] = m_colorsNode[a_color_name]["a"].as<int>() / 255.0;
    }
    else{
        cerr << "WARNING! COLOR NOT FOUND, RETURNING DEFAULT COLOR\n";
    }
    return rgba;
}

bool ADFLoaderInterface::loadObjectAttribs(string a_filepath, string a_objName, afType a_type, afBaseObjectAttributes *attribs)
{
    if (!setLoaderVersionForFile(a_filepath)){
        return false;
    }
    else{
        try{
            YAML::Node node = YAML::LoadFile(a_filepath);
            return m_loader->loadObjectAttribs(&node, a_objName, a_type, attribs);
        }
        catch(YAML::Exception& e){
            cerr << e.what() << endl;
            cerr << "Failed to Load " << a_filepath << endl;
            return false;
        }
    }
}


ADFLoaderInterface::ADFLoaderInterface()
{
    m_loader = nullptr;
}

ADFLoaderInterface::~ADFLoaderInterface()
{
    if (m_loader){
        delete m_loader;
    }
}

bool ADFLoaderInterface::loadWorldAttribs(string a_filepath, afWorldAttributes *attribs)
{
    if (!setLoaderVersionForFile(a_filepath)){
        return false;
    }
    else{
        bool result = m_loader->loadWorldAttribs(a_filepath, attribs);
        return result;
    }
}

bool ADFLoaderInterface::loadModelAttribs(string a_filepath, afModelAttributes *attribs)
{
    if (!setLoaderVersionForFile(a_filepath)){
        return false;
    }
    else{
        bool result = m_loader->loadModelAttribs(a_filepath, attribs);
        return result;
    }
}

bool ADFLoaderInterface::loadTeleRoboticUnitsAttribs(std::string a_filepath, vector<afTeleRoboticUnitAttributes> *attribs, vector<int> dev_indexes)
{
    if (!setLoaderVersionForFile(a_filepath)){
        return false;
    }
    else{
        bool result = m_loader->loadTeleRoboticUnitsAttribs(a_filepath, attribs, dev_indexes);
        return result;
    }

}

bool ADFLoaderInterface::loadLaunchFileAttribs(string a_filepath, afLaunchAttributes *attribs)
{
    if (!setLoaderVersionForFile(a_filepath)){
        return false;
    }
    else{
        bool result = m_loader->loadLaunchFileAttribs(a_filepath, attribs);
        return result;
    }
}


string ADFLoaderInterface::getLoaderVersion()
{
    isLoaderValid();
    return m_loader->getLoaderVersion();
}


bool ADFLoaderInterface::setLoader(ADFLoaderBase *a_loader)
{
    m_loader = a_loader;
    return true;
}


bool ADFLoaderInterface::cleanUp()
{
    if (m_loader != nullptr){
        delete m_loader;
    }
    return true;
}

bool ADFLoaderInterface::isLoaderValid()
{
    assert(m_loader != nullptr);
    if (m_loader == nullptr){
        return false;
    }
    else{
        return true;
    }
}




