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
#ifndef AF_UTILS_H
#define AF_UTILS_H

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <list>

using namespace std;

//------------------------------------------------------------------------------
namespace ambf{

///
/// \brief The afUtils class
///
class afUtils{
public:

    afUtils(){}
    template<typename T1, typename T2>
    static T1 getRotBetweenVectors(const T2 &v1, const T2 &v2);

    template <typename T>
    static string getNonCollidingIdx(string a_body_name, const T* tMap){
        int occurances = 0;
        std::string remap_string = "" ;
        std::stringstream ss;
        if (tMap == nullptr){
            return remap_string;
        }
        if (tMap->find(a_body_name) == tMap->end()){
            return remap_string;
        }
        do{
            ss.str(std::string());
            occurances++;
            ss << occurances;
            remap_string = ss.str();
        }
        while(tMap->find(a_body_name + remap_string) != tMap->end() && occurances < 100);
        return remap_string;
    }

    static string removeAdjacentBackSlashes(string a_name){
        std::string cleaned_name;
        int last_back_slash_idx = -2;
        for (int i = 0; i < a_name.length() ; i++){
            if (a_name[i] == '/'){
                if (i - last_back_slash_idx > 1){
                    cleaned_name.push_back(a_name[i]);
                }
                last_back_slash_idx = i;
            }
            else{
                cleaned_name.push_back(a_name[i]);
            }
        }
        return cleaned_name;
    }

    static string mergeNamespace(string a_namespace1, string a_namespace2){
        a_namespace1 = removeAdjacentBackSlashes(a_namespace1);
        a_namespace2 = removeAdjacentBackSlashes(a_namespace2);

        if(a_namespace2.find('/') == 0){
            return a_namespace2;
        }
        else{
            return a_namespace1 + a_namespace2;
        }
    }

    static void debugPrint(int line, string filename){
        cerr << "Line: "<< line << ", File: " << filename << endl;
    }

    template <class T>
    ///
    /// \brief splitString: Set the template as std::list<string> or std::vector<string>
    /// \param a_str
    /// \param delimiter
    /// \return
    ///
    static const T splitString(string &a_str, const string &delimiter)
    {
        T split_str;
        size_t pos = 0;
        std::string token;
        while ((pos = a_str.find(delimiter)) != std::string::npos) {
            token = a_str.substr(0, pos);
            split_str.push_back(token);
            a_str.erase(0, pos + delimiter.length());
        }
        // To account for the case if a single path was specified without adding any delimiter
        if (a_str.empty() == false){
            split_str.push_back(a_str);
        }
        return split_str;
    }


    static string loadFileContents(const string &a_filepath);
};

}

#endif
