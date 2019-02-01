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
    \author    Dan Morris
    \version   3.2.0 $Rev: 1890 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CVector3dH
#define CVector3dH
//------------------------------------------------------------------------------
#include "system/CString.h"
#include "system/CGlobals.h"
#include "math/CConstants.h"
//------------------------------------------------------------------------------
#include <locale>
#include <ostream>
#include <cmath>
#include <string>
#include <sstream>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CVector3d.h
    \ingroup    math

    \brief
    Implements a 3D vector.
*/
//==============================================================================

//==============================================================================
/*!
    \struct     cVector3d
    \ingroup    math

    \brief
    This class implements a 3D vector.

    \details
    This class implement a 3D vector that provides storage for a 3 dimensional
    double precision floating point vector as well as basic floating point 
    arithmetic operations.
*/
//==============================================================================
struct cVector3d
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Constructor of cVector3d.
    */
    //--------------------------------------------------------------------------
    cVector3d() {}


    //--------------------------------------------------------------------------
    /*!
        \brief
        Constructor of cVector3d.

        \details
        This constructor initializes a vector by passing three doubles by
        argument.

        \param  a_x  X value.
        \param  a_y  Y value.
        \param  a_z  Z value.
    */
    //--------------------------------------------------------------------------
    cVector3d(const double a_x, const double a_y, const double a_z)
    { 
        (*this)(0) = a_x;
        (*this)(1) = a_y;
        (*this)(2) = a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Constructor of cVector3d.

        \details
        This constructor initializes a vector by passing a vector by
        argument.

        \param  a_vector  Vector.
    */
    //--------------------------------------------------------------------------
    cVector3d (const cVector3d &a_vector)
    {
        (*this)(0) = a_vector(0);
        (*this)(1) = a_vector(1);
        (*this)(2) = a_vector(2);
    }


#ifdef C_USE_EIGEN

    //--------------------------------------------------------------------------
    /*!
        \brief
        Constructor of cVector3d.

        \details
        This constructor initializes a vector by passing an Eigen vector by
        argument.

        \param  a_vector  Eigen vector.
    */
    //--------------------------------------------------------------------------
    cVector3d (const Eigen::Vector3d &a_vector)
    {
        (*this)(0) = a_vector(0);
        (*this)(1) = a_vector(1);
        (*this)(2) = a_vector(2);
    }

#endif


    //--------------------------------------------------------------------------
    /*!
        \brief
        Constructor of cVector3d.

        \details
        This constructor initializes a vector by passing an ANSI string by
        argument.

        \param  a_string  Vector in string format.
    */
    //--------------------------------------------------------------------------
    cVector3d(const char* a_string)
    {
        set(a_string); 
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Constructor of cVector3d.

        \details
        This constructor initializes a vector by passing a string by argument.

        \param  a_string  Vector in string format.
    */
    //--------------------------------------------------------------------------
    cVector3d(const std::string& a_string)
    {
        set(a_string);
    }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS
    //--------------------------------------------------------------------------

#ifdef C_USE_EIGEN

    //! This method converts this vector into an __Eigen__ Vector3d.
    Eigen::Vector3d eigen()
    {
        return (Eigen::Vector3d((*this)(0), (*this)(1), (*this)(2)));
    }

#endif

    //! This method returns vector component __x__.
    inline double x() const
    { 
        return((*this)(0)); 
    }
    
    //! This method returns vector component __y__.
    inline double y() const
    { 
        return((*this)(1)); 
    }
    
    //! This method returns vector component __z__.
    inline double z() const
    {
        return((*this)(2)); 
    }
    
    //! This method sets vector component __x__.
    inline void x(const double a_value)
    { 
        (*this)(0) = a_value; 
    }
    
    //! This method sets vector component __y__.
    inline void y(const double a_value)
    { 
        (*this)(1) = a_value; 
    }
    
    //! This method sets vector component __z__.
    inline void z(const double a_value)
    { 
        (*this)(2) = a_value; 
    }

    //! This method clears all vector components with zeros.
    inline void zero()
    {
        (*this)(0) = 0.0;
        (*this)(1) = 0.0;
        (*this)(2) = 0.0;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the _i_ th component of the vector.

        \details
        This method returns the _i_ th component of the vector.

        \param  a_component  Component index number (0,1, or 2).

        \return Selected vector component.
    */
    //--------------------------------------------------------------------------
    inline double get(const unsigned int& a_component) const
    {
        return ((const double*)(this))[a_component];
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes this vector with components __x__, __y__, and 
        __z__ passed as arguments.

        \details
        This method initializes this vector with arguments \p a_x, \p a_y, and
        \p a_z.

        \param  a_x  Component __x__ of vector.
        \param  a_y  Component __y__ of vector.
        \param  a_z  Component __z__ of vector.
    */
    //--------------------------------------------------------------------------
    inline void set(const double& a_x, const double& a_y, const double& a_z)
    {
        (*this)(0) = a_x;
        (*this)(1) = a_y;
        (*this)(2) = a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes this vector from an input __ANSI spring__.

        \details
        This method initializes this vector from an input string of the form 
        <b> (x,y,z) </b>. The following string formats are accepted:\n

        - (4.3,23,54) \n
        - 4.3 54 2.1  \n
        - 4.5,7.8,9.1 \n

        The methods expects three numbers, optionally preceded by '(' and 
        whitespace, and separated by commas or whitespace.

        \param  a_initStr  Vector in string format.

        \return __true__ if the conversion was successful, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool set(const char* a_initStr)
    {
        // sanity check
        if (a_initStr == 0) return (false);

        // look for a valid-format string. ignore leading whitespace and ('s
        const char* curpos = a_initStr;
        while( (*curpos != '\0') &&
               (*curpos == ' ' || *curpos == '\t' || *curpos == '('))
        {
            curpos++;
        }

        // parse data
        double ax, ay, az;
        std::string str = a_initStr;
		std::locale loc;
        for (int j=0; j<(int)(str.length()); j++) if (!std::isdigit(str[j], loc) && str[j] != '.') str[j] = ' ';
        std::istringstream is(str);
        if (!(is >> ax && !is.fail())) return (false);
        if (!(is >> ay && !is.fail())) return (false);
        if (!(is >> az && !is.fail())) return (false);

        // copy the values we found
        (*this)(0) = ax;
        (*this)(1) = ay;
        (*this)(2) = az;

        // return result
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes this vector from and input __string__.

        \details
        This method initializes this vector from an input string of the 
        form <b> (x,y,z) </b>. The following string formats are accepted:\n

        - (4.3,23,54) \n
        - 4.3 54 2.1  \n
        - 4.5,7.8,9.1 \n

        The methods expects three numbers, optionally preceded by '(' and 
        whitespace, and separated by commas or whitespace.

        \param  a_initStr  Vector in string format.

        \return __true__ if the conversion was successful, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool set(const std::string& a_initStr)
    {
        return ( set(a_initStr.c_str()) );
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method copies the content of this vector to another.

        \details
        This method copies components (__x__, __y__, __z__), of this vector 
        to a vector passed as argument.

        \code
        a_destination = this
        \endcode

        \param  a_destination  Destination vector where data is copied.
    */
    //--------------------------------------------------------------------------
    inline void copyto(cVector3d& a_destination) const
    {
        a_destination(0) = (*this)(0);
        a_destination(1) = (*this)(1);
        a_destination(2) = (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method copies the content of a vector to this one.

        \details
        This method copies components (__x__, __y__, __z__) of a vector passed 
        by argument to this one.

        \code
        this = a_source
        \endcode

        \param  a_source  Source vector from where data is copied.
    */
    //--------------------------------------------------------------------------
    inline void copyfrom(const cVector3d &a_source)
    {
        (*this)(0) = a_source(0);
        (*this)(1) = a_source(1);
        (*this)(2) = a_source(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the addition of this vector with another.

        \details
        This method computes the addition of this vector with a vector passed by
        argument. \n
        The result is stored in this vector. \n

        \code
        this = this + a_vector
        \endcode

        \param  a_vector  Vector to be added to current one.
    */
    //--------------------------------------------------------------------------
    inline void add(const cVector3d& a_vector)
    {
        (*this)(0) = (*this)(0) + a_vector(0);
        (*this)(1) = (*this)(1) + a_vector(1);
        (*this)(2) = (*this)(2) + a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the addition of this vector with another.

        \details
        This method computes the addition of this vector with a vector passed by
        argument. \n
        The result is stored in this vector. \n

        \code
        this = this + cVector3d(a_x, a_y, a_z)
        \endcode

        \param  a_x  Component __x__ of vector.
        \param  a_y  Component __y__ of vector.
        \param  a_z  Component __z__ of vector.
    */
    //--------------------------------------------------------------------------
    inline void add(const double& a_x,
                    const double& a_y,
                    const double& a_z)
    {
        (*this)(0) = (*this)(0) + a_x;
        (*this)(1) = (*this)(1) + a_y;
        (*this)(2) = (*this)(2) + a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the addition of this vector with another.

        \details
        This method computes the addition of this vector with a vector passed by
        argument. \n
        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \code
        a_result = this + a_vector
        \endcode

        \param  a_vector  Vector to be added to current one.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void addr(const cVector3d& a_vector,
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0) + a_vector(0);
        a_result(1)  = (*this)(1) + a_vector(1);
        a_result(2)  = (*this)(2) + a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the addition of this vector with another.

        \details
        This method computes the addition of this vector with a vector passed by
        argument. \n
        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \code
        a_result = this + cVector3d(a_x, a_y, a_z)
        \endcode

        \param  a_x       Component __x__ of vector.
        \param  a_y       Component __y__ of vector.
        \param  a_z       Component __z__ of vector.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void addr(const double& a_x,
                     const double& a_y,
                     const double& a_z,
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0) + a_x;
        a_result(1)  = (*this)(1) + a_y;
        a_result(2)  = (*this)(2) + a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the subtraction of this vector with another.

        \details
        This method computes the subtraction of this vector with a vector passed by
        argument. \n
        The result is stored in this vector. \n

        \code
        this = this - a_vector
        \endcode

        \param  a_vector  Vector to be subtracted from current one.
    */
    //--------------------------------------------------------------------------
    inline void sub(const cVector3d& a_vector)
    {
        (*this)(0) = (*this)(0) - a_vector(0);
        (*this)(1) = (*this)(1) - a_vector(1);
        (*this)(2) = (*this)(2) - a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the subtraction of this vector with another.

        \details
        This method computes the subtraction of this vector with a vector passed by
        argument. \n
        The result is stored in this vector. \n

        \code
        this = this - cVector3d(a_x, a_y, a_z)
        \endcode

        \param  a_x  __x__ component.
        \param  a_y  __y__ component.
        \param  a_z  __z__ component.
    */
    //--------------------------------------------------------------------------
    inline void sub(const double& a_x,
                    const double& a_y,
                    const double& a_z)
    {
        (*this)(0) = (*this)(0) - a_x;
        (*this)(1) = (*this)(1) - a_y;
        (*this)(2) = (*this)(2) - a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the subtraction of this vector with another.

        \details
        This method computes the subtraction of this vector with a vector passed by
        argument. \n
        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \code
        a_result = this - a_vector
        \endcode

        \param  a_vector  Vector to be subtracted from current one.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void subr(const cVector3d& a_vector,
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0) - a_vector(0);
        a_result(1)  = (*this)(1) - a_vector(1);
        a_result(2)  = (*this)(2) - a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the subtraction of this vector with another.

        \details
        This method computes the subtraction of this vector with a vector passed by
        argument. \n
        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \code
        a_result = this - cVector3d(a_x, a_y, a_z)
        \endcode

        \param  a_x       __x__ component.
        \param  a_y       __y__ component.
        \param  a_z       __z__ component.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void subr(const double& a_x,
                     const double& a_y,
                     const double& a_z,
                     cVector3d &a_result) const
    {
        a_result(0)  = (*this)(0) - a_x;
        a_result(1)  = (*this)(1) - a_y;
        a_result(2)  = (*this)(2) - a_z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the multiplication of this vector with a scalar.

        \details
        This method computes the multiplication of this vector by a scalar \p a_scalar 
        passed by argument. \n
        The result is stored in this vector. \n

        \code
        this = a_scalar * this
        \endcode

        \param  a_scalar  Scalar value.
    */
    //--------------------------------------------------------------------------
    inline void mul(const double &a_scalar)
    {
        (*this)(0) = a_scalar * (*this)(0);
        (*this)(1) = a_scalar * (*this)(1);
        (*this)(2) = a_scalar * (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the multiplication of each component of this vector
        by a set of scalars.

        \details
        This method computes the multiplication of each component of this vector by a 
        set of scalars \p a_scalar0, \p a_scalar1, and \p a_scalar2 passed by argument. \n
        The result is stored in this vector. \n

        \code
        this(0) = a_scalar0 * this(0)
        this(1) = a_scalar1 * this(1)
        this(2) = a_scalar2 * this(2)
        \endcode

        \param  a_scalar0  Scalar value for component 0.
        \param  a_scalar1  Scalar value for component 1.
        \param  a_scalar2  Scalar value for component 2.
    */
    //--------------------------------------------------------------------------
    inline void mul(const double &a_scalar0,
                    const double &a_scalar1,
                    const double &a_scalar2)
    {
        (*this)(0) = a_scalar0 * (*this)(0);
        (*this)(1) = a_scalar1 * (*this)(1);
        (*this)(2) = a_scalar2 * (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the multiplication of this vector with a scalar.

        \details
        This method computes the multiplication of this vector by a scalar \p a_scalar 
        passed by argument. \n
        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \code
        a_result = a_scalar * this
        \endcode

        \param  a_scalar  Scalar value.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void mulr(const double& a_scalar,
                     cVector3d& a_result) const
    {
        a_result(0)  = a_scalar * (*this)(0);
        a_result(1)  = a_scalar * (*this)(1);
        a_result(2)  = a_scalar * (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the multiplication of each component of this vector by a set of scalars.

        \details
        This method computes the multiplication of each component of this vector by a 
        set of scalars \p a_scalar0, \p a_scalar1, and \p a_scalar2 passed by argument. \n
        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \code
        a_result(0) = a_scalar0 * this(0)
        a_result(1) = a_scalar1 * this(1)
        a_result(2) = a_scalar2 * this(2)
        \endcode

        \param  a_scalar0  Scalar value for component 0.
        \param  a_scalar1  Scalar value for component 1.
        \param  a_scalar2  Scalar value for component 2.
        \param  a_result   Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void mulr(const double &a_scalar0,
                     const double &a_scalar1,
                     const double &a_scalar2,
                     cVector3d& a_result) const
    {
        a_result(0)  = a_scalar0 * (*this)(0);
        a_result(1)  = a_scalar1 * (*this)(1);
        a_result(2)  = a_scalar2 * (*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the element-by-element product between this vector 
        and another.

        \details
        This method computes the element-by-element product between this vector
        and a vector \p a_vector passed by argument. \n

        The result is stored in this vector.\n

        \param  a_vector  Input vector.
    */
    //--------------------------------------------------------------------------
    inline void mulElement(const cVector3d& a_vector)
    {
        (*this)(0)*=a_vector(0);
        (*this)(1)*=a_vector(1);
        (*this)(2)*=a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the element-by-element product between this vector 
        and another.

        \details
        This method computes the element-by-element product between this vector
        and a vector \p a_vector passed by argument. \n

        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \param  a_vector  Input vector.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void mulElementr(const cVector3d& a_vector, 
                            cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0)*a_vector(0);
        a_result(1)  = (*this)(1)*a_vector(1);
        a_result(2)  = (*this)(2)*a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the division of this vector with a scalar.

        \details
        This method computes the division of this vector by a scalar \p a_scalar 
        passed by argument. \n
        The result is stored in this vector. \n

        \code
        this = this / a_scalar
        \endcode

        \param  a_scalar  Scalar value.
    */
    //--------------------------------------------------------------------------
    inline void div(const double& a_scalar)
    {
        double factor = 1.0 / a_scalar;
        (*this)(0) = (*this)(0) * factor;
        (*this)(1) = (*this)(1) * factor;
        (*this)(2) = (*this)(2) * factor;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the division of this vector with a scalar.

        \details
        This method computes the division of this vector by a scalar \p a_scalar 
        passed by argument. \n
        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \code
        a_result = this / a_scalar
        \endcode

        \param  a_scalar  Scalar value.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void divr(const double& a_scalar, 
                     cVector3d& a_result) const
    {
        double factor = 1.0 / a_scalar;
        a_result(0)  = (*this)(0) * factor;
        a_result(1)  = (*this)(1) * factor;
        a_result(2)  = (*this)(2) * factor;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the negated vector.

        \details
        This method computes the negated vector of this vector. \n
        The result is stored in this vector. \n

        \code
        this = -this
        \endcode
    */
    //--------------------------------------------------------------------------
    inline void negate()
    {
        (*this)(0) = -(*this)(0);
        (*this)(1) = -(*this)(1);
        (*this)(2) = -(*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the negated vector.

        \details
        This method computes the negated vector of this vector. \n
        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \code
        a_result = -this
        \endcode

        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void negater(cVector3d& a_result) const
    {
        a_result(0)  = -(*this)(0);
        a_result(1)  = -(*this)(1);
        a_result(2)  = -(*this)(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the cross product.

        \details
        This method computes the cross product between this vector and 
        an vector \p a_vector passed by argument. \n
        The result is stored in the current vector.

        \param  a_vector  Input vector.
    */
    //--------------------------------------------------------------------------
    inline void cross(const cVector3d& a_vector)
    {
        // compute cross product
        double a =  ((*this)(1) * a_vector(2) ) - ((*this)(2) * a_vector(1) );
        double b = -((*this)(0) * a_vector(2) ) + ((*this)(2) * a_vector(0) );
        double c =  ((*this)(0) * a_vector(1) ) - ((*this)(1) * a_vector(0) );

        // store result in current vector
        (*this)(0) = a;
        (*this)(1) = b;
        (*this)(2) = c;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the cross product.

        \details
        This method computes the cross product between this vector and 
        an vector \p a_vector passed by argument. \n
        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \param  a_vector  Input vector.
        \param  a_result  Resulting cross product.
    */
    //--------------------------------------------------------------------------
    inline void crossr(const cVector3d& a_vector, 
                       cVector3d& a_result) const
    {
        a_result(0)  =  ((*this)(1) * a_vector(2) ) - ((*this)(2) * a_vector(1) );
        a_result(1)  = -((*this)(0) * a_vector(2) ) + ((*this)(2) * a_vector(0) );
        a_result(2)  =  ((*this)(0) * a_vector(1) ) - ((*this)(1) * a_vector(0) );
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the dot product between this vector and another.

        \details
        This method computes and returns the dot product between this vector and 
        an vector \p a_vector passed by argument. \n

        \param  a_vector  Input vector.

        \return Dot product between both vectors.
    */
    //--------------------------------------------------------------------------
    inline double dot(const cVector3d& a_vector) const
    {
        return(((*this)(0) * a_vector(0) ) + ((*this)(1) * a_vector(1) ) + ((*this)(2) * a_vector(2) ));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the Euclidean norm of this vector.

        \details
        This method computes and returns the Euclidean norm of this vector.

        \return Euclidean norm of this vector.
    */
    //--------------------------------------------------------------------------
    inline double length() const
    {
        return(sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2))));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the squared value of the Euclidean norm of this vector.

        \details
        This method computes and returns the squared value of the Euclidean norm of 
        this vector.

        \return Squared value of the Euclidean norm of this vector.
    */
    //--------------------------------------------------------------------------
    inline double lengthsq() const
    {
        return(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method normalizes this vector to length 1.

        \details
        This method normalizes this vector to length 1. \n\n

        __WARNING:__ \n
        The vector should not be equal to (0,0,0), otherwise a division by zero error 
        will occur. \n

        \n The result of this operation is stored in this vector.
    */
    //--------------------------------------------------------------------------
    inline void normalize()
    {
        // compute length of vector
        double len = sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
        if (len == 0.0) { return; }
        double factor = 1.0 / len;

        // divide current vector by its length
        (*this)(0) = (*this)(0) * factor;
        (*this)(1) = (*this)(1) * factor;
        (*this)(2) = (*this)(2) * factor;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method normalizes this vector to length 1.

        \details
        This method normalizes this vector to length 1. \n\n

        __WARNING:__ \n
        The vector should not be equal to (0,0,0), otherwise a division by zero error 
        will occur. \n

        The result is stored in the result vector \p a_result that is passed
        by argument. \n

        \param  a_result  Normalized vector.
    */
    //--------------------------------------------------------------------------
    inline void normalizer(cVector3d& a_result) const
    {
        // compute length of vector
        double len = sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
        double factor = 1.0 / len;

        // divide current vector by its length
        a_result(0)  = (*this)(0) * factor;
        a_result(1)  = (*this)(1) * factor;
        a_result(2)  = (*this)(2) * factor;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method clamps this vector to a maximum desired length.

        \details
        This method clamps this vector to a maximum desired length. Vectors that 
        are shorter argument \p than _a_maxLength_ are not affected.\n

        The result of this operation is stored in this vector.

        \param  a_maxLength  Maximum length value.
    */
    //--------------------------------------------------------------------------
    inline void clamp(const double& a_maxLength)
    {
        double len = sqrt(((*this)(0) * (*this)(0)) + ((*this)(1) * (*this)(1)) + ((*this)(2) * (*this)(2)));
        if (a_maxLength == 0)
        {
            (*this)(0) = 0.0;
            (*this)(1) = 0.0;
            (*this)(2) = 0.0;
        }
        else if (len > a_maxLength)
        {
            double factor = a_maxLength / len;
            (*this)(0) = (*this)(0) * factor;
            (*this)(1) = (*this)(1) * factor;
            (*this)(2) = (*this)(2) * factor;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the distance between two points.

        \details
        This method computes and returns the distance between this point and 
        an point \p a_vector passed by argument.

        \param  a_vector  Input point.

        \return Distance between both points.
    */
    //--------------------------------------------------------------------------
    inline double distance(const cVector3d& a_vector) const
    {
        // compute distance between both points
        double dx = (*this)(0) - a_vector(0) ;
        double dy = (*this)(1) - a_vector(1) ;
        double dz = (*this)(2) - a_vector(2) ;

        // return result
        return(sqrt(dx*dx + dy*dy + dz*dz));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the squared value distance between two points.

        \details
        This method computes and returns the squared value distance between this 
        point and an point \p a_vector passed by argument.

        \param  a_vector  Input point.

        \return Squared value distance between both points.
    */
    //--------------------------------------------------------------------------
    inline double distancesq(const cVector3d& a_vector) const
    {
        // compute distance for each element
        double dx = (*this)(0) - a_vector(0) ;
        double dy = (*this)(1) - a_vector(1) ;
        double dz = (*this)(2) - a_vector(2) ;

        // return squared distance
        return(dx*dx + dy*dy + dz*dz);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method performs an equality test between two vectors.

        \details
        This method tests whether the current vector and external vector \p a_vector
        passed as argument are both equal. Two vectors are considered equal if each
        of their components are distant within an distance _epsilon_ defined
        by the second argument \p a_epsilon. By default, _epsilon_ is set to zero.

        \param  a_vector Input vector.
        \param  a_epsilon  Tolerance error.

        \return __true__ if both vectors are equal, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool equals(const cVector3d& a_vector, 
                       const double a_epsilon = 0.0) const
    {
        // accelerated path for exact equality
        if (a_epsilon == 0.0)
        {
            if ( ((*this)(0) == a_vector(0) ) && ((*this)(1) == a_vector(1) ) && ((*this)(2) == a_vector(2) ) )
            {
                return (true);
            }
            else
            {
                return (false);
            }
        }

        if ((fabs(a_vector(0) - (*this)(0)) < a_epsilon) &&
            (fabs(a_vector(1) - (*this)(1)) < a_epsilon) &&
            (fabs(a_vector(2) - (*this)(2)) < a_epsilon))
        {
            return (true);
        }
        else
        {
            return (false);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method convert this vector into a string.

        \details
        This method convert this vector into a string. The number of digits after
        the decimal point are set by argument \p a_precision.

        \param  a_precision  Number of digits.

        \return Converted vector in string format.
    */
    //--------------------------------------------------------------------------
    inline std::string str(const unsigned int a_precision = 2) const
    {
        std::string result;
        result = (cStr((*this)(0), a_precision) + ", " +
                  cStr((*this)(1), a_precision) + ", " +
                  cStr((*this)(2), a_precision));
        return (result);
    }


    //--------------------------------------------------------------------------
    // OPERATORS:
    //--------------------------------------------------------------------------

public:

    //! An overloaded <b> /= </b> operator for vector/scalar division.
    inline void operator/= (const double& a_val)
    {
        double factor = 1.0 / a_val;
        (*this)(0) *= factor;
        (*this)(1) *= factor;
        (*this)(2) *= factor;
    }


    //! An overloaded <b> *= </b> operator for vector/scalar multiplication.
    inline void operator*= (const double& a_val)
    {
        (*this)(0) *= a_val;
        (*this)(1) *= a_val;
        (*this)(2) *= a_val;
    }


    //! An overloaded <b> += </b> operator for vector/vector addition.
    inline void operator+= (const cVector3d& a_input)
    {
        (*this)(0) += a_input(0);
        (*this)(1) += a_input(1);
        (*this)(2) += a_input(2);
    }


    //! An overloaded <b> -= </b> operator for vector/vector subtraction.
    inline void operator-= (const cVector3d& a_input)
    {
        (*this)(0) -= a_input(0);
        (*this)(1) -= a_input(1);
        (*this)(2) -= a_input(2);
    }


    //! An overloaded <b> () </b> operator.
    inline double& operator() (const int a_index)
    {
        return m_data[a_index];
    }


    //! An overloaded <b> () </b> operator.
    inline const double& operator() (const int a_index) const
    {
        return m_data[a_index];
    }


    //--------------------------------------------------------------------------
    // PRIVATE MEMBERS
    //--------------------------------------------------------------------------
    
private:

    //! Vector data.
    double m_data[3];
};


//==============================================================================
// OPERATORS:
//==============================================================================


//! An overloaded <b> * </b> operator for vector/scalar multiplication.
inline cVector3d operator*(const cVector3d& a_vector, const double a_scale)
{
    return (cVector3d(a_vector(0) * a_scale,
                      a_vector(1) * a_scale,
                      a_vector(2) * a_scale));
}


//! An overloaded <b> * </b> operator for scalar/vector multiplication.
inline cVector3d operator*(const double a_scale, const cVector3d& a_vector)
{
    return (cVector3d(a_vector(0) * a_scale,
                      a_vector(1) * a_scale,
                      a_vector(2) * a_scale));
}


//! An overloaded <b> / </b> operator for vector/scalar division.
inline cVector3d operator/(const cVector3d& a_vector, const double a_scale)
{
    return (cVector3d(a_vector(0) / a_scale,
                      a_vector(1) / a_scale,
                      a_vector(2) / a_scale));
}


//! An overloaded <b> + </b> operator for vector/vector addition.
inline cVector3d operator+(const cVector3d& a_vector0, const cVector3d& a_vector1)
{
    return cVector3d(a_vector0(0) + a_vector1(0), 
                     a_vector0(1) + a_vector1(1), 
                     a_vector0(2) + a_vector1(2));
}


//! An overloaded <b> - </b> operator for vector/vector subtraction.
inline cVector3d operator-(const cVector3d& a_vector0, const cVector3d& a_vector1)
{
    return cVector3d(a_vector0(0) - a_vector1(0), 
                     a_vector0(1) - a_vector1(1), 
                     a_vector0(2) - a_vector1(2));
}


//! An overloaded <b> - </b> operator for vector negation.
inline cVector3d operator-(const cVector3d& a_vector0)
{
    return cVector3d(-a_vector0(0), 
                     -a_vector0(1), 
                     -a_vector0(2));
}


//! An overloaded <b> * </b> operator for vector/vector dotting.
inline double operator*(const cVector3d& a_vector0, const cVector3d& a_vector1)
{
    return (a_vector0(0) * a_vector1(0) + 
            a_vector0(1) * a_vector1(1) + 
            a_vector0(2) * a_vector1(2));
}


//! <b> ostream <b> operator. Outputs the vector's components separated by commas.
static inline std::ostream &operator << (std::ostream &a_os, cVector3d const& a_vector)
{
    a_os << a_vector(0)  << ", " << a_vector(1)  << ", " << a_vector(2) ;
    return (a_os);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
