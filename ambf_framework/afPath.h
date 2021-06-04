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

#ifndef AF_PATH_H
#define AF_PATH_H

#include <boost/filesystem/path.hpp>
#include <string>

using namespace std;
using namespace boost::filesystem;

class afPath{
public:
    afPath(){
        m_path = "";
    }

    afPath(string a_path){m_path = a_path;}

    afPath(path a_path){m_path = a_path;}

    string c_str(){
        return m_path.c_str();
    }

    afPath parent_path(){
        return afPath(m_path.parent_path());
    }

    afPath filename(){
        return afPath(m_path.filename());
    }

    bool is_relative(){
        return m_path.is_relative();
    }

    bool is_absolute(){
        return m_path.is_absolute();
    }

    bool is_complete(){
        return m_path.is_complete();
    }

    // If this path is relative, it will be appended to the provided parent path
    bool resolvePath(const afPath& a_parentPath){
        if (is_relative()){
            m_path = a_parentPath.getWrappedObject() / m_path;
            return true;
        }
        else{
            return false;
        }
    }

    path& getWrappedObject(){
        return m_path;
    }

    path getWrappedObject() const{
        return m_path;
    }


    afPath operator/= (afPath a_path){
        m_path = m_path / a_path.getWrappedObject();
        return m_path;
    }

    afPath operator/= (std::string a_path){
        m_path = m_path / path(a_path);
        return m_path;
    }

    void operator= ( string a_path){
        m_path = a_path;
    }

private:
    path m_path;
};

inline afPath operator/ (const afPath& a_path1, const afPath& a_path2){
    afPath outPath(a_path1.getWrappedObject() / a_path2.getWrappedObject());
    return outPath;
}

#endif
