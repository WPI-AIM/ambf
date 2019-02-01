//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

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

    * Neither the name of CHAI3D nor the names of its contributors may
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

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CConstantsH
#define CConstantsH
//------------------------------------------------------------------------------
#include "system/CGlobals.h"
//------------------------------------------------------------------------------
#ifdef C_USE_EIGEN
#include "Eigen/Eigen"
#endif
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CConstants.h
    \ingroup    math

    \brief
    Implements mathematical constants.
*/
//==============================================================================

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// ERROR CONSTANTS
//------------------------------------------------------------------------------

//! Function returns with an error.
const bool C_ERROR          = false;

//! Function returns successfully.
const bool C_SUCCESS        = true;


//------------------------------------------------------------------------------
// MATH CONSTANTS
//------------------------------------------------------------------------------

//! Two PI constant.
const double C_TWO_PI       = 6.28318530717958647692;

//! PI constant.
const double C_PI           = 3.14159265358979323846;

//! PI constant divided by two.
const double C_PI_DIV_2     = 1.57079632679489661923;

//! Conversion from degrees to radians.
const double C_DEG2RAD      = 0.01745329252;

//! Conversion from radians to degrees.
const double C_RAD2DEG      = 57.2957795131;

//! Smallest value near zero for a double.
const double C_TINY         = 1e-49;

//! Small value near zero.
const double C_SMALL        = 0.000000001;

//! Biggest value for a double.
#ifdef DBL_MAX
const double C_LARGE        = DBL_MAX;
#else
const double C_LARGE        = 1e+49;
#endif

//! Biggest value for a float.
#ifdef FLOAT_MAX
const double C_LARGE_FLOAT  = FLOAT_MAX;
#else
const double C_LARGE_FLOAT  = 1e37;
#endif

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
