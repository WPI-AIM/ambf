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
#ifndef AF_CONVERSIONS_H
#define AF_CONVERSIONS_H
//------------------------------------------------------------------------------

#include <LinearMath/btTransform.h>
#include <chai3d.h>
#include <afMath.h>

using namespace ambf;
using namespace chai3d;
using namespace std;

class afConversions{
  public:
    afConversions(){}

    template<typename T1, typename T2>
    static T1 convertDataType(const T2 &r);

};

////
/// AF MATH

afVector3d& operator<< (afVector3d& lhs, const btVector3& rhs);
afVector3d& operator<< (afVector3d& lhs, const cVector3d& rhs);
afVector3d& operator* (cTransform& lhs, afVector3d& rhs);
afVector3d& operator* (btTransform& lhs, afVector3d& rhs);
afMatrix3d& operator<< (afMatrix3d& lhs, const btMatrix3x3& rhs);
afMatrix3d& operator<< (afMatrix3d& lhs, const cMatrix3d& rhs);
afTransform& operator<< (afTransform& lhs, const btTransform& rhs);
afTransform& operator<< (afTransform& lhs, const cTransform& rhs);

////
/// BULLET MATH

btVector3& operator<< (btVector3& lhs, const afVector3d& rhs);
btVector3& operator<< (btVector3& lhs, const cVector3d& rhs);
btVector3& operator* (cTransform& lhs, btVector3& rhs);
btTransform& operator<< (btTransform& lhs, const afTransform& rhs);
btTransform& operator<< (btTransform& lhs, const cTransform& rhs);

////
/// CHAI MATH

cVector3d &operator<<(cVector3d &lhs, const afVector3d &rhs);
cVector3d &operator<<(cVector3d &lhs, const btVector3 &rhs);
cVector3d &operator*(btTransform &lhs, cVector3d &rhs);
cMatrix3d &operator<<(cMatrix3d &lhs, const afMatrix3d &rhs);
cMatrix3d &operator<<(cMatrix3d &lhs, const btMatrix3x3 &rhs);
cTransform &operator<<(cTransform &lhs, const afTransform &rhs);
cTransform &operator<<(cTransform &lhs, const btTransform &rhs);


///
/// \brief to_btVector
/// \param vec
/// \return
///
btVector3 to_btVector(const cVector3d &vec);

///
/// \brief to_btVector
/// \param vec
/// \return
///
btVector3 to_btVector(const afVector3d &vec);

///
/// \brief to_btTransform
/// \param trans
/// \return
///
btTransform to_btTransform(const cTransform &trans);


///
/// \brief to_btTransform
/// \param trans
/// \return
///
btTransform to_btTransform(const afTransform &trans);

///
/// \brief to_cVector3d
/// \param vec
/// \return
///
cVector3d to_cVector3d(const btVector3 &vec);

///
/// \brief to_cVector3d
/// \param vec
/// \return
///
cVector3d to_cVector3d(const afVector3d &vec);


///
/// \brief to_cMatrix3d
/// \param mat
/// \return
///
cTransform to_cMatrix3d(const btMatrix3x3 &mat);


///
/// \brief to_cMatrix3d
/// \param mat
/// \return
///
cMatrix3d to_cMatrix3d(const afMatrix3d &mat);


/////
///// \brief toCtransform
///// \param btTrans
///// \return
/////
cTransform to_cTransform(const btTransform &trans);


///
/// \brief to_cTransform
/// \param trans
/// \return
///
cTransform to_cTransform(const afTransform &trans);


#endif
