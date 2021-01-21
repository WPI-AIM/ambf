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

#include "afUtils.h"

#include <sstream>
#include <math/CMaths.h>
#include <math/CQuaternion.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>
#include <afMath.h>

using namespace ambf;
using namespace chai3d;

template<typename T>
///
/// \brief afUtils::getNonCollidingIdx
/// \param a_body_name
/// \param tMap
/// \return
///
std::string afUtils::getNonCollidingIdx(std::string a_body_name, const T* tMap){
    int occurances = 0;
    std::string remap_string = "" ;
    std::stringstream ss;
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


template<>
///
/// \brief afUtils::getRotBetweenVectors<btQuaternion, btVector3>
/// \param v1
/// \param v2
/// \return
///
btQuaternion afUtils::getRotBetweenVectors<btQuaternion, btVector3>(const btVector3 &v1, const btVector3 &v2){
    btQuaternion quat;
    double rot_angle = v1.angle(v2);
    if ( abs(rot_angle) < 0.1){
        quat.setEulerZYX(0,0,0);
    }
    else if ( abs(rot_angle) > 3.13 ){
        btVector3 nx(1, 0, 0);
        double temp_ang = v1.angle(nx);
        if ( abs(temp_ang) > 0.1 && abs(temp_ang) < 3.13 ){
            btVector3 rot_axis = v1.cross(nx);
            quat.setRotation(rot_axis, rot_angle);
        }
        else{
            btVector3 ny(0, 1, 0);
            btVector3 rot_axis = v2.cross(ny);
            quat.setRotation(rot_axis, rot_angle);
        }
    }
    else{
        btVector3 rot_axis = v1.cross(v2);
        quat.setRotation(rot_axis, rot_angle);
    }

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

template<>
///
/// \brief afUtils::convertDataTypes<btVector3, cVector3d>
/// \param p
/// \return
///
btVector3 afUtils::convertDataType<btVector3, afVector3d>(const afVector3d &p){
    btVector3 btPos(p(0), p(1), p(2));
    return btPos;
}


template<>
///
/// \brief afUtils::convertDataTypes<btMatrix3x3, cMatrix3d>
/// \param r
/// \return
///
btMatrix3x3 afUtils::convertDataType<btMatrix3x3, afMatrix3d>(const afMatrix3d &mat){
    btMatrix3x3 btMat;
    double r, p, y;
    mat.getRPY(r, p, y);
    btMat.setEulerYPR(y, p, r);
    return btMat;
}


template<>
///
/// \brief afUtils::convertDataTypes<btTransform, cTransform>
/// \param t
/// \return
///
btTransform afUtils::convertDataType<btTransform, afTransform>(const afTransform &t){
    btMatrix3x3 btRot = afUtils::convertDataType<btMatrix3x3, afMatrix3d>(t.getRotation());
    btVector3 btPos = afUtils::convertDataType<btVector3, afVector3d>(t.getPosition());
    btTransform btMat(btRot, btPos);
    return btMat;
}


template<>
///
/// \brief afUtils::convertDataTypes<btVector3, cVector3d>
/// \param p
/// \return
///
btVector3 afUtils::convertDataType<btVector3, cVector3d>(const cVector3d &p){
    btVector3 btPos(p.x(), p.y(), p.z());
    return btPos;
}


template<>
///
/// \brief afUtils::convertDataTypes<cQuaternion, btQuaternion>
/// \param q
/// \return
///
btQuaternion afUtils::convertDataType<btQuaternion, cQuaternion>(const cQuaternion &q){
    btQuaternion btQuat(q.x, q.y, q.z, q.w);
    return btQuat;
}


template<>
///
/// \brief afUtils::convertDataTypes<btMatrix3x3, cMatrix3d>
/// \param r
/// \return
///
btMatrix3x3 afUtils::convertDataType<btMatrix3x3, cMatrix3d>(const cMatrix3d &r){
    cQuaternion cQuat;
    cQuat.fromRotMat(r);

    btQuaternion btQuat(cQuat.x, cQuat.y, cQuat.z, cQuat.w);
    btMatrix3x3 btMat;
    btMat.setRotation(btQuat);

    return btMat;
}


template<>
///
/// \brief afUtils::convertDataTypes<cVector3d, afVector3d>
/// \param p
/// \return
///
cVector3d afUtils::convertDataType<cVector3d, afVector3d>(const afVector3d &p){
    cVector3d cPos(p(0), p(1), p(2));
    return cPos;
}


template<>
///
/// \brief afUtils::convertDataTypes<cMatrix3d, afMatrix3d>
/// \param r
/// \return
///
cMatrix3d afUtils::convertDataType<cMatrix3d, afMatrix3d>(const afMatrix3d &r){
    cMatrix3d cMat;
    cMat.set(r(0,0), r(0,1), r(0,2),
             r(1,0), r(1,1), r(1,2),
             r(2,0), r(2,1), r(2,2));
    return cMat;
}


template<>
///
/// \brief afUtils::convertDataTypes<btTransform, cTransform>
/// \param t
/// \return
///
btTransform afUtils::convertDataType<btTransform, cTransform>(const cTransform &t){
    btMatrix3x3 btRot = afUtils::convertDataType<btMatrix3x3, cMatrix3d>(t.getLocalRot());
    btVector3 btPos = afUtils::convertDataType<btVector3, cVector3d>(t.getLocalPos());
    btTransform btMat(btRot, btPos);
    return btMat;
}





template<>
///
/// \brief afUtils::convertDataTypes<cTransform, afTransform>
/// \param t
/// \return
///
cTransform afUtils::convertDataType<cTransform, afTransform>(const afTransform &t){
    cMatrix3d cMat = afUtils::convertDataType<cMatrix3d, afMatrix3d>(t.getRotation());
    cVector3d cPos = afUtils::convertDataType<cVector3d, afVector3d>(t.getPosition());
    cTransform cTrans(cPos, cMat);
    return cTrans;
}


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
    if ( abs(rot_angle) < 0.1){
        quat.fromAxisAngle(cVector3d(0, 0, 1), rot_angle);
    }
    else if ( cAbs(rot_angle) > 3.13 ){
        cVector3d nx(1, 0, 0);
        double temp_ang = cAngle(v1, nx);
        if ( abs(temp_ang) > 0.1 && abs(temp_ang) < 3.13 ){
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
/// \brief afUtils::convertDataTypes<cVector3d, btVector3>
/// \param p
/// \return
///
cVector3d afUtils::convertDataType<cVector3d, btVector3>(const btVector3 &p){
    cVector3d cPos(p.x(), p.y(), p.z());
    return cPos;
}


template<>
///
/// \brief afUtils::convertDataTypes<cQuaternion, btQuaternion>
/// \param q
/// \return
///
cQuaternion afUtils::convertDataType<cQuaternion, btQuaternion>(const btQuaternion &q){
    cQuaternion cQuat(q.w(), q.x(), q.y(), q.z());
    return cQuat;
}


template<>
///
/// \brief afUtils::convertDataTypes<cMatrix3d, btMatrix3x3>
/// \param r
/// \return
///
cMatrix3d afUtils::convertDataType<cMatrix3d, btMatrix3x3>(const btMatrix3x3 &r){
    btQuaternion btQuat;
    r.getRotation(btQuat);

    cQuaternion cQuat(btQuat.w(), btQuat.x(), btQuat.y(), btQuat.z());

    cMatrix3d cMat;
    cQuat.toRotMat(cMat);

    return cMat;
}



template<>
///
/// \brief afUtils::convertDataTypes<cTransform, btTransform>
/// \param t
/// \return
///
cTransform afUtils::convertDataType<cTransform, btTransform>(const btTransform &t){
    cMatrix3d cRot = afUtils::convertDataType<cMatrix3d, btMatrix3x3>(t.getBasis());
    cVector3d cPos = afUtils::convertDataType<cVector3d, btVector3>(t.getOrigin());
    cTransform cMat(cPos, cRot);
    return cMat;
}


template<>
///
/// \brief afUtils::convertDataTypes<afVector3d, cVector3d>
/// \param p
/// \return
///
afVector3d afUtils::convertDataType<afVector3d, cVector3d>(const cVector3d &p){
    afVector3d aPos(p.x(), p.y(), p.z());
    return aPos;
}

