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
    \version   3.2.0 $Rev: 2163 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CMathsH
#define CMathsH
//------------------------------------------------------------------------------
#include "math/CTransform.h"
#include <math.h>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMaths.h
    \ingroup    math

    \brief
    Implements general math utility functions.
*/
//==============================================================================

//------------------------------------------------------------------------------
/*!
    \addtogroup math
*/
//------------------------------------------------------------------------------

//@{

//==============================================================================
/*!
    \brief
    This function checks if a given bit is enabled.

    \details
    This function checks if a given bit is enabled. The function takes by 
    argument an unsigned integer \p a_value and the position of the bit 
    \p a_bitPosition to be tested. \n

    If the selected bit is enabled, the function returns __true__, otherwise __false__.

    \param  a_value        Unsigned int value.
    \param  a_bitPosition  Bit position [0..31].

    \return __true__ if the selected bit is enabled, __false__ otherwise.
*/
//==============================================================================
inline bool cCheckBit(const unsigned int& a_value, const unsigned int& a_bitPosition)
{
    if ((a_value & (1<<a_bitPosition)) > 0)
    {
        return (true);
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    \brief
    This function sets the value of a specified bit of an unsigned integer.

    \details
    This function sets the value of a specified bit of an unsigned integer. \n

    The function takes by argument an unsigned integer \p a_value, the position
    of the bit to be tested \p a_bitPosition, and its new value \p a_value as a 
    boolean. \n

    The function returns the modified unsigned integer as a result.

    \param  a_data         Input unsigned integer data.
    \param  a_bitPosition  Bit position [0..31]
    \param  a_value        New state of bit (__true__ = 1, __false__ = 0).

    \return Input data with modified bit.
*/
//==============================================================================
inline unsigned int cSetBit(const unsigned int& a_data, const unsigned int& a_bitPosition, const bool a_value)
{
    if (a_value)
    {
        return (a_data | (1<<a_bitPosition));
    }
    else
    {
        return (a_data & !(1<<a_bitPosition));
    }
}


//==============================================================================
/*!
    \brief
    This function checks if a value is equal to or almost near zero.

    \details
    This function checks if a value passed by argument a_value is equal to 
    or almost near zero.

    \param  a_value  Value to be checked.

    \return __true__ if the value is equal to zero, __false__ otherwise.
*/
//==============================================================================
inline bool cZero(const double& a_value)
{
    return ((a_value < C_TINY) && (a_value > -C_TINY));
}


//==============================================================================
/*!
    \brief
    This function computes an absolute value.

    \details
    This function computes the absolute value of a value \p a_value passed 
    by argument.

    \param  a_value  Input value.

    \return Absolute value.
*/
//==============================================================================
template<class T> inline T cAbs(const T& a_value)
{
    return (a_value >= 0 ? a_value : -a_value);
}


//==============================================================================
/*!
    \brief
    This function computes the __sign__ of a value.

    \details
    This function computes the __sign__ of a value passed by argument \p a_value.

    \param  a_value  Value to be evaluated.

    \return 1.0 if the value is zero or positive, -1.0 otherwise.
*/
//==============================================================================
template<class T> inline double cSign(const T& a_value)
{
    if (a_value < 0) 
    {
        return (-1);
    }
    else
    {
        return (1);
    }
}


//==============================================================================
/*!
    \brief
    This function computes the maximum value between two values.

    \details
    This function returns the maximum value between two values \p a_value1 and
    \p a_value2 passed by argument.

    \param  a_value1  First value.
    \param  a_value2  Second value.

    \return Maximum value between \p a_value1 and \p a_value2.
*/
//==============================================================================
template<class T> inline T cMax(const T& a_value1, 
                                const T& a_value2)
{
    return (a_value1 >= a_value2 ? a_value1 : a_value2);
}


//==============================================================================
/*!
    \brief
    This function computes the minimum value between two values.

    \details
    This function returns the minimum value between two values \p a_value1 and
    \p a_value2 passed by argument.

    \param  a_value1  First value.
    \param  a_value2  Second value.

    \return Minimum value between \p a_value1 and \p a_value2.
*/
//==============================================================================
template<class T> inline T cMin(const T& a_value1, 
                                const T& a_value2)
{
    return (a_value1 <= a_value2 ? a_value1 : a_value2);
}


//==============================================================================
/*!
    \brief
    This function computes the maximum value between three values.

    \details
    This function returns the maximum value between three values \p a_value1, 
    \p a_value2, and \p a_value3 passed by argument.

    \param  a_value1  First value.
    \param  a_value2  Second value.
    \param  a_value3  Third value.

    \return Maximum value between \p a_value1, \p a_value2, and \p a_value3.
*/
//==============================================================================
template<class T> inline T cMax3(const T& a_value1,
                                 const T& a_value2,
                                 const T& a_value3)
{
    return (cMax(a_value1, cMax(a_value2, a_value3)));
}


//==============================================================================
/*!
    \brief
    This function computes the minimum value between three values.

    \details
    This function returns the minimum value between three values \p a_value1, 
    \p a_value2, and \p a_value3 passed by argument.

    \param  a_value1  First value.
    \param  a_value2  Second value.
    \param  a_value3  Third value.

    \return Minimum value between \p a_value1, \p a_value2, and \p a_value3.
*/
//==============================================================================
template<class T> inline T cMin3(const T& a_value1, 
                                 const T& a_value2, 
                                 const T& a_value3)
{
    return (cMin(a_value1, cMin(a_value2, a_value3)));
}


//==============================================================================
/*!
    \brief
    This function swaps two elements.

    \details
    This function swaps two elements \p a_value1 and \p a_value2 passed by argument.

    \param  a_value1  First value.
    \param  a_value2  Second value.
*/
//==============================================================================
template<class T> inline void cSwap(T& a_value1, 
                                    T& a_value2)
{
    T value = a_value1;
    a_value1 = a_value2;
    a_value2 = value;
}


//==============================================================================
/*!
    \brief
    This function computes a linear interpolation between two values.

    \details
    This function computes a linear interpolation between values \p a_value1 
    (when  \p a_level = 0.0) and \p a_value2 (when \p a_level = 1.0). \n

    \code
    result = (1.0-a_level) * a_value1 + a_level * a_value2
    \endcode

    \param  a_level   Interpolation factor ranging from 0.0 to 1.0.
    \param  a_value1  First value.
    \param  a_value2  Second value.

    \return Interpolated value.
*/
//==============================================================================
template<class T> inline T cLerp(const double& a_level, 
                                 const T& a_value1, 
                                 const T& a_value2)
{
    return (a_value2 * a_level + a_value1 * (1 - a_level));
}


//==============================================================================
/*!
    \brief
    This function clamps a value to a specified range.

    \details
    This function clamps an input value \p a_value to a specified range.\n
    The range is specified by a lower bound \p a_low and upper bound \p a_high.

    \param  a_value  Value to be clamped.
    \param  a_low    Lower boundary value.
    \param  a_high   Upper boundary value.

    \return Clamped value.
*/
//==============================================================================
template<class T> inline T cClamp(const T& a_value, 
                                  const T& a_low, 
                                  const T& a_high)
{
    if      (a_value < a_low)  return a_low;
    else if (a_value > a_high) return a_high;
    else                       return a_value;
}


//==============================================================================
/*!
    \brief
    This function clamps a value to a value ranged between __0__ and __infinity__.

    \details
    This function clamps an input value \p a_value to a value ranged
    between __0__ and __infinity__.

    \param  a_value  Value to be clamped.

    \return Clamped value.
*/
//==============================================================================
template<class T> inline T cClamp0(const T& a_value)
{
    return cMax<T>(0, a_value);
}


//==============================================================================
/*!
    \brief
    This function clamps a value to a value ranged between __0.0__ and __1.0__.

    \details
    This function clamps an input value \p a_value to a value ranged
    between __0.0__ and __1.0__.

    \param  a_value  Value to be clamped.

    \return Clamped value.
*/
//==============================================================================
inline double cClamp01(const double& a_value)
{
    return (cClamp(a_value, 0.0, 1.0));
}


//==============================================================================
/*!
    \brief
    This function checks whether a value is within a specified range.

    \details
    This function checks whether a value \p a_value is within a range specified
    by arguments \p a_low and \p a_high.

    \param  a_value  Value to be tested.
    \param  a_low    Lower boundary value.
    \param  a_high   Upper boundary value.

    \return __true__ if the value is within the specified range, 
            __false__ otherwise.
*/
//==============================================================================
template<class T, class V> inline bool cContains(const T& a_value, 
                                                 const V& a_low, 
                                                 const V& a_high)
{
    return ((a_value >= a_low) && (a_value <= a_high));
}


//==============================================================================
/*!
    \brief
    This function computes the square of a scalar.

    \details
    This function computes the square of a scalar \p a_value passed by argument. \n

    \code
    result = a_value * a_value
    \endcode

    \param  a_value  Input value.

    \return Square of \p a_value.
*/
//==============================================================================
inline double cSqr(const double& a_value)
{
    return(a_value*a_value);
}


//==============================================================================
/*!
    \brief
    This function computes the square root of a scalar.

    \details
    This function computes the square root of a scalar \p a_value passed by 
    argument.

    \param  a_value  Input value.

    \return Square root of \p a_value.
*/
//==============================================================================
inline double cSqrt(const double& a_value)
{
    return(sqrt(a_value));
}


//==============================================================================
/*!
    \brief
    This function computes the cubic root of a scalar.

    \details
    This function computes the cubic root of a scalar \p a_value passed by argument.

    \param  a_value  Input value.

    \return Cubic root of a_value.
*/
//==============================================================================
inline double cCbrt(const double& a_value)
{
    return (pow(a_value, 1.0/3.0));
}


//==============================================================================
/*!
    \brief
    Compute the cosine of an angle defined in degrees.

    \details
    This function computes the cosine of an angle defined in degrees. \n
    The angle \p a_angleDeg is passed by argument.

    \param  a_angleDeg  Angle in degrees.

    \return Cosine of angle \p a_angleDeg.
*/
//==============================================================================
inline double cCosDeg(const double& a_angleDeg)
{
    return (cos(a_angleDeg * C_DEG2RAD));
}


//==============================================================================
/*!
    \brief
    This function computes the sine of an angle defined in degrees.

    \details
    This function computes the sine of an angle defined in degrees. \n
    The angle \p a_angleDeg is passed by argument.

    \param  a_angleDeg  Angle in degrees.

    \return Sine of angle \p a_angleDeg.
*/
//==============================================================================
inline double cSinDeg(const double& a_angleDeg)
{
    return (sin(a_angleDeg * C_DEG2RAD));
}


//==============================================================================
/*!
    \brief
    This function computes the tangent of an angle defined in degrees.

    \details
    This function computes the tangent of an angle defined in degrees. \n
    The angle \p a_angleDeg is passed by argument.

    \param  a_angleDeg  Angle in degrees.

    \return Tangent of angle \p a_angleDeg.
*/
//==============================================================================
inline double cTanDeg(const double& a_angleDeg)
{
    return (tan(a_angleDeg * C_DEG2RAD));
}


//==============================================================================
/*!
    \brief
    This function computes the cosine of an angle defined in radians.

    \details
    This function computes the cosine of an angle defined in radians. \n
    The angle \p a_angleRad is passed by argument.

    \param  a_angleRad  Angle in radians.

    \return Cosine of angle \p a_angleRad.
*/
//==============================================================================
inline double cCosRad(const double& a_angleRad)
{
    return (cos(a_angleRad));
}


//==============================================================================
/*!
    \brief
    This function computes the sine of an angle defined in radians.

    \details
    This function computes the sine of an angle defined in radians. \n
    The angle \p a_angleRad is passed by argument.

    \param  a_angleRad  Angle in radians.

    \return Sine of angle \p a_angleRad.
*/
//==============================================================================
inline double cSinRad(const double& a_angleRad)
{
    return (sin(a_angleRad));
}


//==============================================================================
/*!
    \brief
    This function computes the tangent of an angle defined in radians.

    \details
    This function computes the tangent of an angle defined in radians. \n
    The angle \p a_angleRad is passed by argument.

    \param  a_angleRad  Angle in radians.

    \return Tangent of angle \p a_angleRad.
*/
//==============================================================================
inline double cTanRad(const double& a_angleRad)
{
    return (tan(a_angleRad));
}


//==============================================================================
/*!
    \brief
    This function converts an angle from degrees to radians.

    \details
    This function converts an angle \p a_angleDeg expressed in degrees to an
    angle expressed in radians.

    \param  a_angleDeg  Angle in degrees.

    \return Converted angle in radians.
*/
//==============================================================================
inline double cDegToRad(const double& a_angleDeg)
{
    return (a_angleDeg * C_DEG2RAD);
}


//==============================================================================
/*!
    \brief
    This function converts an angle from radians to degrees.

    \details
    This function converts an angle \p a_angleRad expressed in radians to an
    angle expressed in degrees.

    \param  a_angleRad  Angle in radians.

    \return Converted angle in degrees.
*/
//==============================================================================
inline double cRadToDeg(const double& a_angleRad)
{
    return (a_angleRad * C_RAD2DEG);
}


//==============================================================================
/*!
    \brief
    This function computes the number of digits that compose a given integer.

    \details
    This function returns the number of digits that compose a given integer
    \p a_value passed by argument.

    \param  a_value  Value to be evaluated.

    \return Number of digits.
*/
//==============================================================================
inline int cNumDigits(int a_value)
{
    int digits = 0;

    if (a_value == 0) { return (1); }

    double value = (double)(abs(a_value));
    while (value >= 1.0)
    {
        value /= 10;
        digits++;
    }
    return (digits);
}


//==============================================================================
/*!
    \brief
    This function computes the addition of two vectors.

    \details
    This function computes the addition of two vectors. \n
    The vectors are passed by argument as \p a_vector1 and \p a_vector2. \n

    \code
    result = a_vector1 + a_vector2
    \endcode

    \param  a_vector1  First vector.
    \param  a_vector2  Second vector.

    \return Addition of two vectors.
*/
//==============================================================================
inline cVector3d cAdd(const cVector3d& a_vector1,
                      const cVector3d& a_vector2)
{
    return cVector3d(
      a_vector1(0)+a_vector2(0),
      a_vector1(1)+a_vector2(1),
      a_vector1(2)+a_vector2(2));
}


//==============================================================================
/*!
    \brief
    This function computes the addition of three vectors.

    \details
    This function computes the addition of three vectors. \n
    The vectors are passed by argument as \p a_vector1, \p a_vector2, and 
    \p a_vector3. \n

    \code
    result = a_vector1 + a_vector2 + a_vector3
    \endcode

    \param  a_vector1  First vector.
    \param  a_vector2  Second vector.
    \param  a_vector3  Third vector.

    \return Result of operation.
*/
//==============================================================================
inline cVector3d cAdd(const cVector3d& a_vector1,
                      const cVector3d& a_vector2,
                      const cVector3d& a_vector3)
{
    return cVector3d(
      a_vector1(0) +a_vector2(0) +a_vector3(0) ,
      a_vector1(1) +a_vector2(1) +a_vector3(1) ,
      a_vector1(2) +a_vector2(2) +a_vector3(2) );
}


//==============================================================================
/*!
    \brief
    This function computes the subtraction of two vectors.

    \details
    This function computes the subtraction of two vectors. \n
    The vectors are passed by argument as \p a_vector1 and \p a_vector2. \n

    \code
    result = a_vector1 - a_vector2
    \endcode

    \param  a_vector1  First vector.
    \param  a_vector2  Second vector.

    \return Result of operation.
*/
//==============================================================================
inline cVector3d cSub(const cVector3d& a_vector1,
                      const cVector3d& a_vector2)
{
  return cVector3d(
    a_vector1(0) -a_vector2(0) ,
    a_vector1(1) -a_vector2(1) ,
    a_vector1(2) -a_vector2(2) );
}


//==============================================================================
/*!
    \brief
    This function computes the negated vector.

    \details
    This function computes the negated vector of a vector \p a_vector passed 
    by argument. \n

    \code
    result = -a_vector
    \endcode

    \param  a_vector  Vector to be negated.

    \return Negated vector.
*/
//==============================================================================
inline cVector3d cNegate(const cVector3d& a_vector)
{
    return cVector3d(-a_vector(0),
                     -a_vector(1),
                     -a_vector(2));
}


//==============================================================================
/*!
    \brief
    This function computes the multiplication of a vector by a scalar.

    \details
    This function computes the multiplication of a vector by a scalar. \n
    The vector \p a_vector and scalar \p a_scalar are passed by argument. \n

    \code
    result = a_value * a_vector
    \endcode

    \param  a_value   Scalar.
    \param  a_vector  Vector to be scaled.

    \return Result of operation.
*/
//==============================================================================
inline cVector3d cMul(const double& a_value,
                      const cVector3d& a_vector)
{
    return cVector3d(a_vector(0) *a_value,
                     a_vector(1) *a_value,
                     a_vector(2) *a_value);
}


//==============================================================================
/*!
    \brief
    This function computes the division of a vector by a scalar.

    \details
    This function computes the division of a vector by a scalar. \n
    The vector \p a_vector and scalar \p a_scalar are passed by argument. \n

    \code
     result = (1.0 / a_value) * a_vector
    \endcode

    \param  a_value   Scalar.
    \param  a_vector  Vector to be scaled.

    \return Result of operation.
*/
//==============================================================================
inline cVector3d cDiv(const double& a_value, 
                      const cVector3d& a_vector)
{
    double factor = 1.0 / a_value;
    return (cVector3d(factor * a_vector(0),
                      factor * a_vector(1),
                      factor * a_vector(2)));
}


//==============================================================================
/*!
    \brief
    This function computes the cross product between two vectors.

    \details
    This function compute the cross product between two vectors. \n
    Both vectors are passed by argument as \p a_vector1 and \p a_vector2. \n

    \code
    result = a_vector1 x a_vector2 
    \endcode

    \param  a_vector1  Vector1.
    \param  a_vector2  Vector2.

    \return Cross product between both vectors.
*/
//==============================================================================
inline cVector3d cCross(const cVector3d& a_vector1,
                        const cVector3d& a_vector2)
{
    cVector3d result;
    a_vector1.crossr(a_vector2, result);
    return (result);
}


//==============================================================================
/*!
    \brief
    This function computes the dot product between two vectors.

    \details
    This function computes the dot product between two vectors. \n
    Both vectors are passed by argument as \p a_vector1 and \p a_vector2. \n

    \code
     result = a_vector1 * a_vector2
    \endcode

    \param  a_vector1  Vector1.
    \param  a_vector2  Vector2.

    \return Dot product between both vectors.
*/
//==============================================================================
inline double cDot(const cVector3d& a_vector1, 
                   const cVector3d& a_vector2)
{
    return(a_vector1.dot(a_vector2));
}


//==============================================================================
/*!
    \brief
    This function computes the normalized vector.

    \details
    This function computes the normalized vector of a vector \p a_vector
    passed by argument. \n

    \param  a_vector  Vector to be normalized.

    \return Normalized vector.
*/
//==============================================================================
inline cVector3d cNormalize(const cVector3d& a_vector)
{
    cVector3d result;
    a_vector.normalizer(result);
    return (result);
}


//==============================================================================
/*!
    \brief
    This function computes the Euclidean distance between two points.

    \details
    This function computes the distance between two points \p a_point1 and
    \p a_point2 passed by argument.

    \param  a_point1  First point.
    \param  a_point2  Second point.

    \return Euclidean Distance between both points.
*/
//==============================================================================
inline double cDistance(const cVector3d& a_point1, 
                        const cVector3d& a_point2)
{
    return ( a_point1.distance(a_point2) );
}


//==============================================================================
/*!
    \brief
    This function computes the squared distance between two points.

    \details
    This function computes the squared distance between two points \p a_point1 and
    \p a_point2 passed by argument.

    \param  a_point1  First point.
    \param  a_point2  Second point.

    \return Squared distance between both points.
*/
//==============================================================================
inline double cDistanceSq(const cVector3d& a_point1, 
                          const cVector3d& a_point2)
{
    return ( a_point1.distancesq(a_point2) );
}


//==============================================================================
/*!
    \brief
    This function determines whether two vectors are equal 
    (i.e. represent the same point).

    \details
    This function determine whether two vectors represent the same point. \n
    Both points, \p a_point1 and \p a_point2, are passed by argument. \n

    A parameter \p a_epsilon defines the distance tolerance between both points 
    such that if the computed distance is smaller than \p a_epsilon, then both 
    points are considered to be equal. \n

    The function returns __true__ if both points are determined equal, or 
    __false__ otherwise.

    \param  a_point1   First point.
    \param  a_point2   Second point.
    \param  a_epsilon  Distance tolerance. Defaults value is set to \e C_SMALL.

    \return __true__ if both points are determined equal, __false__ otherwise.
*/
//==============================================================================
bool inline cEqualPoints(const cVector3d& a_point1,
                         const cVector3d& a_point2,
                         const double a_epsilon = C_SMALL)
{
    // accelerated path for exact equality
    if (a_epsilon == 0.0) 
    {
        if ((a_point1(0)  == a_point2(0)) && (a_point1(1)  == a_point2(1)) && (a_point1(2)  == a_point2(2)))
        {
            return (true);
        }
        else
        {
            return (false);
        }
    }

    if ((fabs(a_point1(0) - a_point2(0)) < a_epsilon) &&
        (fabs(a_point1(1) - a_point2(1)) < a_epsilon) &&
        (fabs(a_point1(2) - a_point2(2)) < a_epsilon))
    {
        return (true);
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    \brief
    This function return a 3x3 identity matrix.

    \details
    The function returns the identity matrix.

    \return Identity matrix.
*/
//==============================================================================
inline cMatrix3d cIdentity3d(void)
{
    cMatrix3d result;
    result.identity();
    return (result);
}


//==============================================================================
/*!
    \brief  This function computes the addition between two matrices.

    \details
    The function computes the addition between two matrices \p a_matrix1 and 
    \p a_matrix2 that are passed by argument. \n

    \code
     result = a_matrix1 + a_matrix2
    \endcode

    \param  a_matrix1  First matrix.
    \param  a_matrix2  Second matrix.

    \return Result of operation.
*/
//==============================================================================
inline cMatrix3d cAdd(const cMatrix3d& a_matrix1,
                      const cMatrix3d& a_matrix2)
{
    cMatrix3d result;
    a_matrix1.addr(a_matrix2, result);
    return (result);
}


//==============================================================================
/*!
    \brief
    This function computes the subtraction between two matrices.

    \details
    This function computes the subtraction between two matrices \p a_matrix1 
    and \p a_matrix2 that are passed by argument. \n

    \code
     result = a_matrix1 - a_matrix2
    \endcode

    \param  a_matrix1  First matrix.
    \param  a_matrix2  Second matrixs.

    \return Result of operation.
*/
//==============================================================================
inline cMatrix3d cSub(const cMatrix3d& a_matrix1,
                      const cMatrix3d& a_matrix2)
{
    cMatrix3d result;
    a_matrix1.subr(a_matrix2, result);
    return (result);
}


//==============================================================================
/*!
    \brief
    This function builds a rotation matrix from a set of Euler angles defined 
    in radians.

    \details
    This function creates a rotation matrix from a set of Euler angles defined 
    in radians and co-moving axes of rotations. \n

    The angles are defined in radians and passed by argument as \p a_angleRad1,
    \p a_angleRad2, and \p a_angleRad3. \n

    The order of the rotations are defined by argument \p a_eulerOrder. 
    See \ref cEulerOrder for more information. \n

    The argument \p a_useIntrinsicEulerModel defines if the instrinsic (__true__) or 
    extrinsic (__false__) model is used.

    \param  a_angleRad1               Angle in radians of the first rotation in the sequence.
    \param  a_angleRad2               Angle in radians of the second rotation in the sequence.
    \param  a_angleRad3               Angle in radians of the third rotation in the sequence.
    \param  a_eulerOrder              The order of the axes about which the rotations are to be applied
    \param  a_useIntrinsicEulerModel  If __true__ use intrinsic Euler model, if __false__ use extrinsic Euler model.

    \return Rotation matrix.
*/
//==============================================================================
inline cMatrix3d cRotEulerRad(const double& a_angleRad1,
    const double& a_angleRad2,
    const double& a_angleRad3,
    const cEulerOrder a_eulerOrder,
    const bool a_useIntrinsicEulerModel = true)
{
    // create matrix
    cMatrix3d rot;
    if (a_useIntrinsicEulerModel)
        rot.setIntrinsicEulerRotationRad(a_angleRad1, a_angleRad2, a_angleRad3, a_eulerOrder);
    else
        rot.setExtrinsicEulerRotationRad(a_angleRad1, a_angleRad2, a_angleRad3, a_eulerOrder);

    // return result
    return (rot);
}


//==============================================================================
/*!
    \brief
    This function builds a rotation matrix from a set of Euler angles defined
    in degrees.

    \details
    This function creates a rotation matrix from a set of Euler angles defined 
    in degrees and co-moving axes of rotations. \n

    The angles are defined in radian and passed by argument as \p a_angleDeg1,
    \p a_angleDeg2, and \p a_angleDeg3. \n

    The order of the rotations are defined by argument \p a_eulerOrder. 
    See \ref cEulerOrder for more information. \n

    The argument \p a_useIntrinsicEulerModel defines if the instrinsic (__true__) or 
    extrinsic (__false__) model is used.

    \param  a_angleDeg1               Angle in degrees of the first rotation in the sequence.
    \param  a_angleDeg2               Angle in degrees of the second rotation in the sequence.
    \param  a_angleDeg3               Angle in degrees of the third rotation in the sequence.
    \param  a_eulerOrder              The order of the axes about which the rotations are to be applied
    \param  a_useIntrinsicEulerModel  If __true__ use intrinsic Euler model, if __false__ use extrinsic Euler model.

    \return Rotation matrix.
*/
//==============================================================================
inline cMatrix3d cRotEulerDeg(const double& a_angleDeg1,
    const double& a_angleDeg2,
    const double& a_angleDeg3,
    const cEulerOrder a_eulerOrder,
    const bool a_useIntrinsicEulerModel = true)
{
    // create matrix
    cMatrix3d rot;
    if (a_useIntrinsicEulerModel)
        rot.setExtrinsicEulerRotationDeg(a_angleDeg1, a_angleDeg2, a_angleDeg3, a_eulerOrder);
    else
        rot.setExtrinsicEulerRotationDeg(a_angleDeg1, a_angleDeg2, a_angleDeg3, a_eulerOrder);

    // return result
    return (rot);
}


//==============================================================================
/*!
    \brief
    This function builds a rotation matrix from an axis-angle representation.

    \details
    This function creates a rotation matrix from an axis-angle representation. \n

    The axis is defined by arguments \p a_axisX, \p a_axisY, and \p a_axisZ.
    The rotation angle is defined in radians by argument \p a_angleRad.

    \param  a_axisX     __x__ component of axis.
    \param  a_axisY     __y__ component of axis.
    \param  a_axisZ     __z__ component of axis.
    \param  a_angleRad  Angle of rotation in radians.

    \return Rotation matrix.
*/
//==============================================================================
inline cMatrix3d cRotAxisAngleRad(const double& a_axisX,
    const double& a_axisY,
    const double& a_axisZ,
    const double& a_angleRad)
{
    // create matrix
    cMatrix3d rot;
    rot.setAxisAngleRotationRad(a_axisX,
                                a_axisY,
                                a_axisZ,
                                a_angleRad);

    // return result
    return (rot);
}


//==============================================================================
/*!
    \brief
    This function builds a rotation matrix from an axis-angle representation.

    \details
    This function creates a rotation matrix from an axis-angle representation. \n

    The axis is defined by arguments \p a_axisX, \p a_axisY, and \p a_axisZ.
    The rotation angle is defined in degrees by argument \p a_angleDeg.

    \param  a_axisX     __x__ component of axis.
    \param  a_axisY     __y__ component of axis.
    \param  a_axisZ     __z__ component of axis.
    \param  a_angleDeg  Angle of roation in degrees.

    \return Rotation matrix.
*/
//==============================================================================
inline cMatrix3d cRotAxisAngleDeg(const double& a_axisX,
    const double& a_axisY,
    const double& a_axisZ,
    const double& a_angleDeg)
{
    // create matrix
    cMatrix3d rot;
    rot.setAxisAngleRotationDeg(a_axisX,
                                a_axisY,
                                a_axisZ,
                                a_angleDeg);

    // return result
    return (rot);
}


//==============================================================================
/*!
    \brief
    This function computes the multiplication of two matrices.

    \details
    This function computes the multiplication of two matrices \p a_matrix1 and 
    \p a_matrix2 passed by argument. \n

    \code
     result = a_matrix1 * a_matrix2
    \endcode

    \param  a_matrix1  First matrix.
    \param  a_matrix2  Second matrix.

    \return Result of operation.
*/
//==============================================================================
inline cMatrix3d cMul(const cMatrix3d& a_matrix1,
                      const cMatrix3d& a_matrix2)
{
    cMatrix3d result;
    a_matrix1.mulr(a_matrix2, result);
    return (result);
}


//==============================================================================
/*!
    \brief
    This function computes the multiplication of a matrix and a vector.

    \details
    This function computes the multiplication of a matrix \p a_matrix and 
    a vector \p a_vector passed by argument. \n

    \code
     result = a_matrix * a_vector
    \endcode

    \return Result of operation.
*/
//==============================================================================
inline cVector3d cMul(const cMatrix3d& a_matrix,
                      const cVector3d& a_vector)
{
    cVector3d result;
    a_matrix.mulr(a_vector, result);
    return(result);
}


//==============================================================================
/*!
    \brief
    This function computes the transpose of a matrix.

    \details
    This function computes the transpose of a matrix \p a_matrix passed 
    by argument.

    \param  a_matrix  Matrix to be transposed.

    \return Transpose of \p a_matrix.
*/
//==============================================================================
inline cMatrix3d cTranspose(const cMatrix3d& a_matrix)
{
    cMatrix3d result;
    a_matrix.transr(result);
    return(result);
}


//==============================================================================
/*!
    \brief
    This function computes the inverse of a matrix.

    \details
    This function computes the the inverse of a matrix \p a_matrix passed by
    argument.

    \param  a_matrix  Matrix to be inversed.

    \return The inverse of the matrix.
*/
//==============================================================================
inline cMatrix3d cInverse(const cMatrix3d& a_matrix)
{
    cMatrix3d result;
    a_matrix.invertr(result);
    return(result);
}


//==============================================================================
/*!
    \brief
    This function computes the angle in radians between two vectors.

    \details
    This function computes the angle in radians between two vectors \p a_vector1
    and \p a_vector2 passed by argument.

    \param  a_vector1  First vector.
    \param  a_vector2  Second vector.

    \return Angle in radians between both vectors.
*/
//==============================================================================
inline double cAngle(const cVector3d& a_vector1,
                     const cVector3d& a_vector2)
{
    // compute length of vectors
    double n1 = a_vector1.length();
    double n2 = a_vector2.length();
    double val = n1 * n2;

    // check if lengths of vectors are not zero
    if (fabs(val) < C_SMALL)
    {
        return (0.0);
    }

    // compute angle
    double result = a_vector1.dot(a_vector2)/(val);
    if (result > 1.0) { result = 1.0; }
    else if (result < -1.0) { result = -1.0; }

    return(acos(result));
}


//==============================================================================
/*!
    \brief
    This function computes the cosine of the angle between two vectors.

    \details
    This function computes the cosine of the angle between two vectors
    \p a_vector1 and \p a_vector2 passed by argument.

    \param  a_vector1  First vector.
    \param  a_vector2  Second vector.

    \return Cosine of the angle between both vectors.
*/
//==============================================================================
inline double cCosAngle(const cVector3d& a_vector1,
                        const cVector3d& a_vector2)
{
    // compute length of vectors
    double n1 = a_vector1.length();
    double n2 = a_vector2.length();
    double val = n1 * n2;

    // check if lengths of vectors are not zero
    if (fabs(val) < C_SMALL)
    {
        return (0.0);
    }

    // compute angle
    return(a_vector1.dot(a_vector2)/(val));
}

//@}

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif  // CMathsH
//------------------------------------------------------------------------------
