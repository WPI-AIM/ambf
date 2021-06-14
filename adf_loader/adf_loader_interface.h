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
#ifndef AF_ADF_LOADER_INTERFACE_H
#define AF_ADF_LOADER_INTERFACE_H
//------------------------------------------------------------------------------

#include "adf_loader_base.h"

enum class adfVersion{
    VERSION_1_0,
    INVALID

};

using namespace ambf;

class ADFLoaderInterface{
public:
    ADFLoaderInterface();
    ~ADFLoaderInterface();

    bool loadObjectAttribs(std::string a_filepath, std::string a_objectName, afType a_type, afBaseObjectAttributes* attribs);

    // Load World Attribs
    bool loadWorldAttribs(std::string a_filepath, afWorldAttributes* attribs);

    // Load Model Attribs
    bool loadModelAttribs(std::string a_filepath, afModelAttributes* attribs);

    // Load TU Attribs
    bool loadTeleRoboticUnitsAttribs(std::string a_filepath, vector<afTeleRoboticUnitAttributes> *attribs, vector<int> dev_indexes);

    // Load the Launch file Attribs
    bool loadLaunchFileAttribs(std::string a_filepath, afLaunchAttributes* attribs);

    adfVersion getFileVersion(std::string a_filepath);

    adfVersion getFileVersion(YAML::Node *a_node, string a_filepath);

    adfVersion getVersionFromString(std::string a_str);

    // Get color's rgba values from the name of the color. Color names are defined
    // in the color config file
    std::vector<double> getColorRGBA(std::string a_color_name);

    bool setLoaderVersion(adfVersion);

    bool setLoaderVersionForFile(std::string a_filepath);

    bool cleanUp();

    bool isLoaderValid();

    bool setLoader(ADFLoaderBase* a_loader);

    // Get the version of the this loader
    std::string getLoaderVersion();

protected:


    YAML::Node m_colorsNode;

private:
    ADFLoaderBase* m_loader;
};

#endif
