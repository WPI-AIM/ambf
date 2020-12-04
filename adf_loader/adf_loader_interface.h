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

//------------------------------------------------------------------------------
#ifndef AF_ADF_LOADER_INTERFACE_H
#define AF_ADF_LOADER_INTERFACE_H
//------------------------------------------------------------------------------

#include "adf_loader_base.h"
#include "version_1_0/adf_loader_1_0.h"

enum class adfVersion{
    VERSION_1_0,
    INVALID

};

// To use or not use boost::filesystem?
typedef boost::filesystem bf;

using namespace ambf;

class ADFLoaderInterface{
public:
    ADFLoaderInterface(){}

    // Load World Attribs
    bool loadWorldAttribs(std::string a_filepath, afWorldAttributes* attribs);

    // Load Multibody Attribs
    bool loadMultiBodyAttribs(std::string a_filepath, afMultiBodyAttributes* attribs);

    // Load the Launch file Attribs
    bool loadLaunchFileAttribs(std::string a_filepath, afLaunchAttributes* attribs);

    adfVersion getFileVersion(std::string a_filepath);

    adfVersion getFileVersion(YAML::Node *a_node);

    adfVersion getVersionFromString(std::string a_str);

    // The the multibody config file name at specifc index
    bf::path getMultiBodyFilepath(uint i=0);

    // Get color's rgba values from the name of the color. Color names are defined
    // in the color config file
    std::vector<double> getColorRGBA(std::string a_color_name);

    bool setLoaderVersion(adfVersion);

    bool setLoaderVersionForFile(std::string a_filepath);

protected:
    YAML::Node m_colorsNode;
};

#endif
