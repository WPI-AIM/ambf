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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   1.0$
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef AF_UTILS_H
#define AF_UTILS_H

#include <string>
#include <iostream>

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

    template<typename T1, typename T2>
    static T1 convertDataType(const T2 &r);

    template <typename T>
    static string getNonCollidingIdx(string a_body_name, const T* a_tMap);

    static string removeAdjacentBackSlashes(string a_name);

    static string mergeNamespace(string a_namespace1, string a_namespace2);

    static void debugPrint(int line, string filename){
        cerr << "Line: "<< line << ", File: " << filename << endl;
    }

//    static cMesh* createVisualShape(const afPrimitiveShapeAttributes* a_primitiveShape);

//    static btCollisionShape* createCollisionShape(const afPrimitiveShapeAttributes* a_primitiveShape);

//    static btCompoundShape* createCollisionShapeFromMesh(const cMultiMesh* a_collisionMesh, btTransform T_offset, double a_margin);
};


}
#endif
