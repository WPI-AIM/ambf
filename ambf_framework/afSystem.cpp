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


#include <afSystem.h>
#include <iterator>
#include <algorithm>
#include <afUtils.h>

using namespace std;
using namespace ambf;

string afSystemPaths::s_rootPath;
list<string> afSystemPaths::s_pluginPaths;

string afSystemPaths::getSeparator()
{
#ifdef _WIN32
    return "\\";
#else
    return "/";
#endif
}

string afSystemPaths::getPathSeparator()
{
#ifdef _WIN32
    return ";";
#else
    return ":";
#endif
}

const list<string> &afSystemPaths::getPluginPath()
{
    string path;

    char *pathCStr = getenv("AMBF_PLUGIN_PATH");
    if (!pathCStr || *pathCStr == '\0')
    {
        // No env var; take the compile-time default.
        path = AMBF_PLUGIN_PATH;
    }
    else
    {
        path = pathCStr;
    }

    list<string> delimitedPaths = ambf::afUtils::splitString<list<string>>(path, getPathSeparator());

    for (auto pathIt : delimitedPaths)
    {
        if (!pathIt.empty())
        {
            if (find(s_pluginPaths.begin(), s_pluginPaths.end(), pathIt) == s_pluginPaths.end()) {
              s_pluginPaths.push_back(pathIt);
            }
        }
    }
    return s_pluginPaths;
}

string &afSystemPaths::getRootPath()
{
    string path;

    char *pathCStr = getenv("AMBF_ROOT_PATH");
    if (!pathCStr || *pathCStr == '\0')
    {
        // No env var; take the compile-time default.
        path = AMBF_ROOT_PATH;
    }
    else
    {
        path = pathCStr;
    }

    s_rootPath = path;

    return s_rootPath;
}
