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

#include "afUtils.h"

#include <sstream>
#include <math/CMaths.h>
#include <math/CQuaternion.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>
#include <afMath.h>
#include <fstream>

using namespace ambf;
using namespace chai3d;

template<>
///
/// \brief afUtils::getRotBetweenVectors<cQuaternion, cVector3d>
/// \param v1
/// \param v2
/// \return
///
cQuaternion afUtils::getRotBetweenVectors<cQuaternion, cVector3d>(const cVector3d &v1, const cVector3d &v2){
    cQuaternion quat;
    double rot_angle = cAngle(v1, v2);
    if ( abs(rot_angle) < 0.001){
        quat.fromAxisAngle(cVector3d(0, 0, 1), rot_angle);
    }
    else if ( cAbs(rot_angle) > 3.14 ){
        cVector3d nx(1, 0, 0);
        double temp_ang = cAngle(v1, nx);
        if ( abs(temp_ang) > 0.001 && abs(temp_ang) < 3.14 ){
            cVector3d rot_axis = cCross(v1, nx);
            quat.fromAxisAngle(rot_axis, rot_angle);
        }
        else{
            cVector3d ny(0, 1, 0);
            cVector3d rot_axis = cCross(v2, ny);
            quat.fromAxisAngle(rot_axis, rot_angle);
        }
    }
    else{
        cVector3d rot_axis = cCross(v1, v2);
        quat.fromAxisAngle(rot_axis, rot_angle);
    }

    return quat;
}


template<>
///
/// \brief afUtils::getRotBetweenVectors<cMatrix3d, cVector3d>
/// \param v1
/// \param v2
/// \return
///
cMatrix3d afUtils::getRotBetweenVectors<cMatrix3d, cVector3d>(const cVector3d &v1, const cVector3d &v2){
    cMatrix3d rot_mat;
    cQuaternion quat = getRotBetweenVectors<cQuaternion, cVector3d>(v1, v2);
    quat.toRotMat(rot_mat);
    return rot_mat;
}


template<>
///
/// \brief afUtils::getRotBetweenVectors<btQuaternion, btVector3>
/// \param v1
/// \param v2
/// \return
///
btQuaternion afUtils::getRotBetweenVectors<btQuaternion, btVector3>(const btVector3 &v1, const btVector3 &v2){
    btQuaternion quat;
    cVector3d va, vb;
    va.set(v1.x(), v1.y(), v1.z());
    vb.set(v2.x(), v2.y(), v2.z());
    cQuaternion cQaut = afUtils::getRotBetweenVectors<cQuaternion, cVector3d>(va, vb);

    quat.setX(cQaut.x); quat.setY(cQaut.y); quat.setZ(cQaut.z); quat.setW(cQaut.w);
    return quat;
}


template<>
///
/// \brief afUtils::getRotBetweenVectors<btMatrix3x3, btVector3>
/// \param v1
/// \param v2
/// \return
///
btMatrix3x3 afUtils::getRotBetweenVectors<btMatrix3x3, btVector3>(const btVector3 &v1, const btVector3 &v2){
    btMatrix3x3 rot_mat;
    btQuaternion quat = getRotBetweenVectors<btQuaternion, btVector3>(v1, v2);
    rot_mat.setRotation(quat);
    return rot_mat;
}


///
/// \brief afUtils::loadFileContents
/// \param a_filepath
/// \return
///
string afUtils::loadFileContents(const string &a_filepath)
{
    ifstream fileStream;
    fileStream.open(a_filepath.c_str());
    stringstream stringStream;
    stringStream << fileStream.rdbuf();
    fileStream.close();
    string contents = stringStream.str();
    return contents;
}
