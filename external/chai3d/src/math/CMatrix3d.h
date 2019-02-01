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
#ifndef CMatrix3dH
#define CMatrix3dH
//------------------------------------------------------------------------------
#include "math/CConstants.h"
#include "math/CVector3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
enum cEulerOrder 
{
    C_EULER_ORDER_XYZ, 
    C_EULER_ORDER_XYX,
    C_EULER_ORDER_XZY, 
    C_EULER_ORDER_XZX, 
    C_EULER_ORDER_YZX,
    C_EULER_ORDER_YZY, 
    C_EULER_ORDER_YXZ,
    C_EULER_ORDER_YXY, 
    C_EULER_ORDER_ZXY, 
    C_EULER_ORDER_ZXZ, 
    C_EULER_ORDER_ZYX, 
    C_EULER_ORDER_ZYZ  
};
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMatrix3d.h
    \ingroup    math

    \brief
    Implements a 3D matrix.
*/
//==============================================================================

//==============================================================================
/*!
    \struct     cMatrix3d
    \ingroup    math

    \brief
    This class implements a 3D matrix.

    \details
    This matrix class provides storage for a 3x3 double precision 
    floating point matrix as well as basic matrix and vector operations.
*/
//==============================================================================
struct cMatrix3d
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Constructor of cMatrix3d.
    */
    //--------------------------------------------------------------------------
    cMatrix3d() {}


#ifdef C_USE_EIGEN
    //--------------------------------------------------------------------------
    /*!
        Constructor of cMatrix3d.

        \param  a_matrix  Eigen 3d Matrix.
    */
    //--------------------------------------------------------------------------
    cMatrix3d(const Eigen::Matrix3d& a_matrix)
    { 
        (*this)(0,0) = a_matrix(0,0);
        (*this)(0,1) = a_matrix(0,1);
        (*this)(0,2) = a_matrix(0,2);
        (*this)(1,0) = a_matrix(1,0);
        (*this)(1,1) = a_matrix(1,1);
        (*this)(1,2) = a_matrix(1,2);
        (*this)(2,0) = a_matrix(2,0);
        (*this)(2,1) = a_matrix(2,1);
        (*this)(2,2) = a_matrix(2,2);
    }
#endif

    //--------------------------------------------------------------------------
    /*!
        Constructor of cMatrix3d. \n
        Initializes matrix by passing as arguments values for each cell.

        \param  a_m00  Matrix Component (0,0)
        \param  a_m01  Matrix Component (0,1)
        \param  a_m02  Matrix Component (0,2)
        \param  a_m10  Matrix Component (1,0)
        \param  a_m11  Matrix Component (1,1)
        \param  a_m12  Matrix Component (1,2)
        \param  a_m20  Matrix Component (2,0)
        \param  a_m21  Matrix Component (2,1)
        \param  a_m22  Matrix Component (2,2)
    */
    //--------------------------------------------------------------------------
    cMatrix3d(const double& a_m00, const double& a_m01, const double& a_m02,
              const double& a_m10, const double& a_m11, const double& a_m12,
              const double& a_m20, const double& a_m21, const double& a_m22)
    {
        (*this)(0,0) = a_m00;  (*this)(0,1) = a_m01;  (*this)(0,2) = a_m02;
        (*this)(1,0) = a_m10;  (*this)(1,1) = a_m11;  (*this)(1,2) = a_m12;
        (*this)(2,0) = a_m20;  (*this)(2,1) = a_m21;  (*this)(2,2) = a_m22;
    }


    //--------------------------------------------------------------------------
    /*!
        Constructor of cMatrix3d. \n
        Initializes matrix from a set of column vectors passed as arguments.

        \param  a_colVector0  Column vector 0.
        \param  a_colVector1  Column vector 1.
        \param  a_colVector2  Column vector 2.
    */
    //--------------------------------------------------------------------------
    cMatrix3d(const cVector3d& a_colVector0, 
              const cVector3d& a_colVector1, 
              const cVector3d& a_colVector2)
    { 
        (*this).setCol0(a_colVector0);
        (*this).setCol1(a_colVector1);
        (*this).setCol2(a_colVector2);
    }


#ifdef C_USE_EIGEN
    //--------------------------------------------------------------------------
    /*!
        Constructor of cMatrix3d.
        Initializes matrix from a set of Eigen column vectors passed as arguments.

        \param  a_colVector0  Column vector 0.
        \param  a_colVector1  Column vector 1.
        \param  a_colVector2  Column vector 2.
    */
    //--------------------------------------------------------------------------
    cMatrix3d(const Eigen::Vector3d& a_colVector0,
              const Eigen::Vector3d& a_colVector1,
              const Eigen::Vector3d& a_colVector2)
    { 
        (*this)(0,0) = a_colVector0(0);  (*this)(0,1) = a_colVector1(0);  (*this)(0,2) = a_colVector2(0);
        (*this)(1,0) = a_colVector0(1);  (*this)(1,1) = a_colVector1(1);  (*this)(1,2) = a_colVector2(1);
        (*this)(2,0) = a_colVector0(2);  (*this)(2,1) = a_colVector1(2);  (*this)(2,2) = a_colVector2(2);
    }
#endif

    //--------------------------------------------------------------------------
    /*!
        Constructor of cMatrix3d. \n
        Builds a rotation matrix from a set of Euler angles and co-moving 
        axes of rotations.

        \param  a_angle1      Angle of the first rotation in the sequence.
        \param  a_angle2      Angle of the second rotation in the sequence.
        \param  a_angle3      Angle of the third rotation in the sequence.
        \param  a_eulerOrder  The order of the axes about which the rotations 
                              are to be applied

        \param  a_useIntrinsicEulerModel  If __true__ use _intrinsic_ Euler 
                              model, if __false__ use _extrinsic_ Euler model.

        \param  a_anglesDefinedInDegrees  If __true__ then the angles passed
                              by argument are defined in degrees, if __false__ then 
                              the angles are defined in radians.
    */
    //--------------------------------------------------------------------------
    cMatrix3d(const double& a_angle1,
              const double& a_angle2,
              const double& a_angle3,
              const cEulerOrder a_eulerOrder,
              const bool a_useIntrinsicEulerModel = true,
              const bool a_anglesDefinedInDegrees = false)
    {
        if (a_anglesDefinedInDegrees)
        {
            if (a_useIntrinsicEulerModel)
            {
                setIntrinsicEulerRotationDeg(a_angle1, a_angle2, a_angle3, a_eulerOrder);
            }
            else
            {
                setExtrinsicEulerRotationDeg(a_angle1, a_angle2, a_angle3, a_eulerOrder);
            }
        }
        else
        {
            if (a_useIntrinsicEulerModel)
            {
                setIntrinsicEulerRotationRad(a_angle1, a_angle2, a_angle3, a_eulerOrder);
            }
            else
            {
                setExtrinsicEulerRotationRad(a_angle1, a_angle2, a_angle3, a_eulerOrder);
            }
        }
    }


    //--------------------------------------------------------------------------
    /*!
        Constructor of cMatrix3d. \n
        Builds a rotation matrix from an __axis-angle__ representation passed
        as argument.

        \param  a_axis      Axis vector.
        \param  a_angleRad  Angle in radians.
    */
    //--------------------------------------------------------------------------
    cMatrix3d(const cVector3d& a_axis, 
              const double& a_angleRad)
    {
        setAxisAngleRotationRad(a_axis, a_angleRad);
    }


    //--------------------------------------------------------------------------
    /*!
        Constructor of cMatrix3d. \n
        Builds a rotation matrix from an __axis-angle__ representation.

        \param  a_axisX     __x__ component of axis.
        \param  a_axisY     __y__ component of axis.
        \param  a_axisZ     __z__ component of axis.
        \param  a_angleRad  Angle in radian.
    */
    //--------------------------------------------------------------------------
    cMatrix3d(const double& a_axisX,
              const double& a_axisY,
              const double& a_axisZ,
              const double& a_angleRad)
    {
        setAxisAngleRotationRad(a_axisX,
                                a_axisY,
                                a_axisZ,
                                a_angleRad);
    }


#ifdef C_USE_EIGEN

    //! This method converts this matrix to an Eigen Mector3d.
    Eigen::Matrix3d eigen()
    {
        Eigen::Matrix3d m;
        m(0,0) = (*this)(0,0);
        m(0,1) = (*this)(0,1);
        m(0,2) = (*this)(0,2);
        m(1,0) = (*this)(1,0);
        m(1,1) = (*this)(1,1);
        m(1,2) = (*this)(1,2);
        m(2,0) = (*this)(2,0);
        m(2,1) = (*this)(2,1);
        m(2,2) = (*this)(2,2);
        return (m);
    }

#endif


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds an identity matrix.

        \details
        This method builds an identity matrix with ones on the main diagonal
        and zeros elsewhere.
    */
    //--------------------------------------------------------------------------
    inline void identity()
    {
        (*this)(0,0) = 1.0;  (*this)(0,1) = 0.0;  (*this)(0,2) = 0.0;
        (*this)(1,0) = 0.0;  (*this)(1,1) = 1.0;  (*this)(1,2) = 0.0;
        (*this)(2,0) = 0.0;  (*this)(2,1) = 0.0;  (*this)(2,2) = 1.0;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes all elements of this matrix with an input value.

        \details
        This method initializes all elements of this matrix with an input scalar 
        value \p a_value passed as argument.

        \param  a_value  Value.
    */
    //--------------------------------------------------------------------------
    inline void set(const double& a_value)
    {
        (*this)(0,0) = a_value;  (*this)(0,1) = a_value;  (*this)(0,2) = a_value;
        (*this)(1,0) = a_value;  (*this)(1,1) = a_value;  (*this)(1,2) = a_value;
        (*this)(2,0) = a_value;  (*this)(2,1) = a_value;  (*this)(2,2) = a_value;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes this matrix from an array.

        \details
        This method initializes this matrix from a table of __doubles__ passed 
        as argument.

        \param  a_source  Table of __doubles__.
    */
    //--------------------------------------------------------------------------
    inline void set(const double a_source[3][3])
    {
        (*this)(0,0) = a_source[0][0];
        (*this)(0,1) = a_source[0][1];
        (*this)(0,2) = a_source[0][2];
        (*this)(1,0) = a_source[1][0];
        (*this)(1,1) = a_source[1][1];
        (*this)(1,2) = a_source[1][2];
        (*this)(2,0) = a_source[2][0];
        (*this)(2,1) = a_source[2][1];
        (*this)(2,2) = a_source[2][2];
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes this matrix with individual elements passed
        as arguments.

        \details
        This method initialize this matrix by passing as argument values for
        each cell.

        \param  a_m00  Matrix Component (0,0)
        \param  a_m01  Matrix Component (0,1)
        \param  a_m02  Matrix Component (0,2)
        \param  a_m10  Matrix Component (1,0)
        \param  a_m11  Matrix Component (1,1)
        \param  a_m12  Matrix Component (1,2)
        \param  a_m20  Matrix Component (2,0)
        \param  a_m21  Matrix Component (2,1)
        \param  a_m22  Matrix Component (2,2)
    */
    //--------------------------------------------------------------------------
    inline void set(const double& a_m00, const double& a_m01, const double& a_m02,
                    const double& a_m10, const double& a_m11, const double& a_m12,
                    const double& a_m20, const double& a_m21, const double& a_m22)
    {
        (*this)(0,0) = a_m00;  (*this)(0,1) = a_m01;  (*this)(0,2) = a_m02;
        (*this)(1,0) = a_m10;  (*this)(1,1) = a_m11;  (*this)(1,2) = a_m12;
        (*this)(2,0) = a_m20;  (*this)(2,1) = a_m21;  (*this)(2,2) = a_m22;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initialize this matrix with column vectors passed as
        arguments.

        \details
        This method initialize this matrix with column vectors passed as
        arguments \p a_vectCol0, \p a_vectCol1, and \p a_vectCol2. \n

        \code
         this = [a_vectCol0, a_vectCol1, a_vectCol2]
        \endcode

        \param  a_vectCol0  Vector Column 0.
        \param  a_vectCol1  Vector Column 1.
        \param  a_vectCol2  Vector Column 2.
    */
    //--------------------------------------------------------------------------
    inline void setCol(const cVector3d& a_vectCol0, 
                       const cVector3d& a_vectCol1,
                       const cVector3d& a_vectCol2)
    {
        (*this)(0,0) = a_vectCol0(0);  (*this)(0,1) = a_vectCol1(0);  (*this)(0,2) = a_vectCol2(0);
        (*this)(1,0) = a_vectCol0(1);  (*this)(1,1) = a_vectCol1(1);  (*this)(1,2) = a_vectCol2(1);
        (*this)(2,0) = a_vectCol0(2);  (*this)(2,1) = a_vectCol1(2);  (*this)(2,2) = a_vectCol2(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes column 0 of this matrix.

        \details
        This method sets column 0 of this matrix by passing an input vector 
        \p a_vectCol passed as argument.

        \param  a_vectCol  Vector Column 0.
    */
    //--------------------------------------------------------------------------
    inline void setCol0(const cVector3d& a_vectCol)
    {
        (*this)(0,0) = a_vectCol(0);
        (*this)(1,0) = a_vectCol(1);
        (*this)(2,0) = a_vectCol(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes column 1 of this matrix.

        \details
        This method sets column 1 of this matrix by passing an input vector 
        \p a_vectCol passed as argument.

        \param  a_vectCol  Vector Column 1.
    */
    //--------------------------------------------------------------------------
    inline void setCol1(const cVector3d& a_vectCol)
    {
        (*this)(0,1) = a_vectCol(0);
        (*this)(1,1) = a_vectCol(1);
        (*this)(2,1) = a_vectCol(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes column 2 of this matrix.

        \details
        This method sets column 2 of this matrix by passing an input vector 
        \p a_vectCol passed as argument.

        \param  a_vectCol  Vector Column 2.
    */
    //--------------------------------------------------------------------------
    inline void setCol2(const cVector3d& a_vectCol)
    {
        (*this)(0,2) = a_vectCol(0);
        (*this)(1,2) = a_vectCol(1);
        (*this)(2,2) = a_vectCol(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method copies matrix elements to a table of __doubles__

        \details
        This method copies matrix elements to a table \p a_destination 
        of __doubles__ passed as argument.

        \param  a_destination  Table of doubles.
    */
    //--------------------------------------------------------------------------
    inline void get(double* a_destination[])
    {
        *a_destination[0] = (*this)(0,0);
        *a_destination[1] = (*this)(0,1);
        *a_destination[2] = (*this)(0,2);
        *a_destination[3] = (*this)(1,0);
        *a_destination[4] = (*this)(1,1);
        *a_destination[5] = (*this)(1,2);
        *a_destination[6] = (*this)(2,0);
        *a_destination[7] = (*this)(2,1);
        *a_destination[8] = (*this)(2,2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns column vector 0 of matrix.

        \details
        This method returns column vector 0 of matrix.

        \return Column vector 0 of matrix.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getCol0() const
    {
        cVector3d result;
        result(0) = (*this)(0,0);
        result(1) = (*this)(1,0);
        result(2) = (*this)(2,0);
        return (result);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns column vector 1 of matrix.

        \details
        This method returns column vector 1 of matrix.

        \return Column vector 1 of matrix.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getCol1() const
    {
        cVector3d result;
        result(0) = (*this)(0,1);
        result(1) = (*this)(1,1);
        result(2) = (*this)(2,1);
        return (result);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns column vector 2 of matrix.

        \details
        This method returns column vector 2 of matrix.

        \return Column vector 2 of matrix.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getCol2() const
    {
        cVector3d result;
        result(0) = (*this)(0,2);
        result(1) = (*this)(1,2);
        result(2) = (*this)(2,2);
        return (result);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the _i_ th row of matrix.

        \details
        This method returns the _i_ th of matrix. The three components are returned 
        through a cVector3d data structure.

        \param  a_index  Index of row. (0,1 or 2)

        \return _i_ th row of matrix.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getRow(const unsigned int& a_index) const
    {
        cVector3d result;
        result(0) = (*this)(a_index, 0);
        result(1) = (*this)(a_index, 1);
        result(2) = (*this)(a_index, 2);
        return (result);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method copies all elements of this matrix to another matrix.

        \details
        This method copies all elements of this matrix to another matrix
        \p a_destination passed as argument.

        \code
        a_destination = this
        \endcode

        \param  a_destination  Destination matrix.
    */
    //--------------------------------------------------------------------------
    inline void copyto(cMatrix3d& a_destination) const
    {
        a_destination = (*this);
    }


#ifdef C_USE_EIGEN
    //--------------------------------------------------------------------------
    /*!
        \brief
        This method copies all elements of this matrix to an Eigen matrix.

        \details
        This method copies all elements of this matrix to an Eigen matrix
        \p a_destination passed as argument.

        \code
        a_destination = this
        \endcode

        \param  a_destination  Destination matrix.
    */
    //--------------------------------------------------------------------------
    inline void copyto(Eigen::Matrix3d& a_destination) const
    {
        a_destination(0,0) = (*this)(0,0);  a_destination(0,1) = (*this)(0,1);  a_destination(0,2) = (*this)(0,2);
        a_destination(1,0) = (*this)(1,0);  a_destination(1,1) = (*this)(1,1);  a_destination(1,2) = (*this)(1,2);
        a_destination(2,0) = (*this)(2,0);  a_destination(2,1) = (*this)(2,1);  a_destination(2,2) = (*this)(2,2);
    }
#endif

    //--------------------------------------------------------------------------
    /*!
        \brief
        This method copies all elements from another matrix to this one.

        \details
        This method copies all elements of a matrix \p a_source passed as 
        argument to this one.

        \code
        this = a_source
        \endcode

        \param  a_source  Source matrix.
    */
    //--------------------------------------------------------------------------
    inline void copyfrom(const cMatrix3d& a_source)
    {
        (*this) = a_source;
    }


#ifdef C_USE_EIGEN
    //--------------------------------------------------------------------------
    /*!
        \brief
        This method copies all elements from an Eigen matrix to this one.

        \details
        This method copies all elements of an Eigen matrix \p a_source passed as 
        argument to this one.

        \code
        this = a_source
        \endcode

        \param  a_source  Source matrix.
    */
    //--------------------------------------------------------------------------
    inline void copyfrom(const Eigen::Matrix3d& a_source)
    {
        (*this)(0,0) = a_source(0,0);   (*this)(0,1) = a_source(0,1);   (*this)(0,2) = a_source(0,2);
        (*this)(1,0) = a_source(1,0);   (*this)(1,1) = a_source(1,1);   (*this)(1,2) = a_source(1,2);
        (*this)(2,0) = a_source(2,0);   (*this)(2,1) = a_source(2,1);   (*this)(2,2) = a_source(2,2);
    }
#endif


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method compares if two matrices are equal.

        \details
        This methods compare two matrices and returns __true__ if both 
        matrices are equal, otherwise __false__.

        \param  a_matrix  Matrix to compare with.

        \return __true__ if elements of both matrices are equal, 
                otherwise __false__.
    */
    //--------------------------------------------------------------------------
    inline bool equals(cMatrix3d& a_matrix) const
    {
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
            {
                if (a_matrix(i,j) != (*this)(i,j)) return (false);
            }
        }
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the addition of this matrix with another.

        \details
        This method computes the addition of this matrix with a matrix passed as
        argument. \n
        The result is stored in this matrix. \n

        \code
        this = this + a_matrix
        \endcode

        \param  a_matrix  Input matrix.
    */
    //--------------------------------------------------------------------------
    inline void add(const cMatrix3d& a_matrix)
    {
        (*this)(0,0) = (*this)(0,0) + a_matrix(0,0);
        (*this)(0,1) = (*this)(0,1) + a_matrix(0,1);
        (*this)(0,2) = (*this)(0,2) + a_matrix(0,2);
        (*this)(1,0) = (*this)(1,0) + a_matrix(1,0);
        (*this)(1,1) = (*this)(1,1) + a_matrix(1,1);
        (*this)(1,2) = (*this)(1,2) + a_matrix(1,2);
        (*this)(2,0) = (*this)(2,0) + a_matrix(2,0);
        (*this)(2,1) = (*this)(2,1) + a_matrix(2,1);
        (*this)(2,2) = (*this)(2,2) + a_matrix(2,2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the addition of this matrix with another.

        \details
        This method computes the addition of this matrix with a matrix passed as
        argument. \n
        The result is stored is the result matrix \p a_result that is passed
        as argument. \n

        \code
        a_result = this + a_matrix
        \endcode

        \param  a_matrix  Input matrix.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void addr(const cMatrix3d& a_matrix, 
                     cMatrix3d& a_result) const
    {
        a_result(0,0) = (*this)(0,0) + a_matrix(0,0);
        a_result(0,1) = (*this)(0,1) + a_matrix(0,1);
        a_result(0,2) = (*this)(0,2) + a_matrix(0,2);
        a_result(1,0) = (*this)(1,0) + a_matrix(1,0);
        a_result(1,1) = (*this)(1,1) + a_matrix(1,1);
        a_result(1,2) = (*this)(1,2) + a_matrix(1,2);
        a_result(2,0) = (*this)(2,0) + a_matrix(2,0);
        a_result(2,1) = (*this)(2,1) + a_matrix(2,1);
        a_result(2,2) = (*this)(2,2) + a_matrix(2,2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the subtraction of this matrix with another.

        \details
        This method computes the subtraction of this matrix with a matrix passed as
        argument. \n
        The result is stored in this matrix. \n

        \code
        this = this - a_matrix
        \endcode

        \param  a_matrix  Input matrix.
    */
    //--------------------------------------------------------------------------
    inline void sub(const cMatrix3d& a_matrix)
    {
        (*this)(0,0) = (*this)(0,0) - a_matrix(0,0);
        (*this)(0,1) = (*this)(0,1) - a_matrix(0,1);
        (*this)(0,2) = (*this)(0,2) - a_matrix(0,2);
        (*this)(1,0) = (*this)(1,0) - a_matrix(1,0);
        (*this)(1,1) = (*this)(1,1) - a_matrix(1,1);
        (*this)(1,2) = (*this)(1,2) - a_matrix(1,2);
        (*this)(2,0) = (*this)(2,0) - a_matrix(2,0);
        (*this)(2,1) = (*this)(2,1) - a_matrix(2,1);
        (*this)(2,2) = (*this)(2,2) - a_matrix(2,2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the subtraction of this matrix with another.

        \details
        This method computes the subtraction of this matrix with a matrix passed as
        argument. \n
        The result is stored is the result matrix \p a_result that is passed
        as argument. \n

        \code
        a_result = this - a_matrix
        \endcode

        \param  a_matrix  Input matrix.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void subr(const cMatrix3d& a_matrix, 
                     cMatrix3d& a_result) const
    {
        a_result(0,0) = (*this)(0,0) - a_matrix(0,0);
        a_result(0,1) = (*this)(0,1) - a_matrix(0,1);
        a_result(0,2) = (*this)(0,2) - a_matrix(0,2);
        a_result(1,0) = (*this)(1,0) - a_matrix(1,0);
        a_result(1,1) = (*this)(1,1) - a_matrix(1,1);
        a_result(1,2) = (*this)(1,2) - a_matrix(1,2);
        a_result(2,0) = (*this)(2,0) - a_matrix(2,0);
        a_result(2,1) = (*this)(2,1) - a_matrix(2,1);
        a_result(2,2) = (*this)(2,2) - a_matrix(2,2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This function computes the multiplication of this matrix with another.

        \details
        This function computes the multiplication of this matrix with a matrix
        \p a_matrix passed as argument. \n
        The result of this operation is stored in this matrix overwriting 
        previous values.

        \code
        this = this * a_matrix
        \endcode

        \param  a_matrix  Input matrix.
    */
    //--------------------------------------------------------------------------
    inline void mul(const cMatrix3d& a_matrix)
    {
        // compute multiplication between both matrices
        double m00 = (*this)(0,0) * a_matrix(0,0) + (*this)(0,1) * a_matrix(1,0) + (*this)(0,2) * a_matrix(2,0);
        double m01 = (*this)(0,0) * a_matrix(0,1) + (*this)(0,1) * a_matrix(1,1) + (*this)(0,2) * a_matrix(2,1);
        double m02 = (*this)(0,0) * a_matrix(0,2) + (*this)(0,1) * a_matrix(1,2) + (*this)(0,2) * a_matrix(2,2);
        double m10 = (*this)(1,0) * a_matrix(0,0) + (*this)(1,1) * a_matrix(1,0) + (*this)(1,2) * a_matrix(2,0);
        double m11 = (*this)(1,0) * a_matrix(0,1) + (*this)(1,1) * a_matrix(1,1) + (*this)(1,2) * a_matrix(2,1);
        double m12 = (*this)(1,0) * a_matrix(0,2) + (*this)(1,1) * a_matrix(1,2) + (*this)(1,2) * a_matrix(2,2);
        double m20 = (*this)(2,0) * a_matrix(0,0) + (*this)(2,1) * a_matrix(1,0) + (*this)(2,2) * a_matrix(2,0);
        double m21 = (*this)(2,0) * a_matrix(0,1) + (*this)(2,1) * a_matrix(1,1) + (*this)(2,2) * a_matrix(2,1);
        double m22 = (*this)(2,0) * a_matrix(0,2) + (*this)(2,1) * a_matrix(1,2) + (*this)(2,2) * a_matrix(2,2);

        // return values to current matrix
        (*this)(0,0) = m00;  (*this)(0,1) = m01;  (*this)(0,2) = m02;
        (*this)(1,0) = m10;  (*this)(1,1) = m11;  (*this)(1,2) = m12;
        (*this)(2,0) = m20;  (*this)(2,1) = m21;  (*this)(2,2) = m22;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This function computes the multiplication of this matrix with another.

        \details
        This function computes the multiplication of this matrix with a matrix
        \p a_matrix passed as argument. \n
        The result of this operation is stored in the result matrix \p a_result 
        passed as argument.

        \code
        a_result = this * a_matrix
        \endcode

        \param  a_matrix  Input matrix.
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void mulr(const cMatrix3d& a_matrix, 
                     cMatrix3d& a_result) const
    {
        // compute multiplication between both matrices
        a_result(0,0) = (*this)(0,0) * a_matrix(0,0) + (*this)(0,1) * a_matrix(1,0) + (*this)(0,2) * a_matrix(2,0);
        a_result(0,1) = (*this)(0,0) * a_matrix(0,1) + (*this)(0,1) * a_matrix(1,1) + (*this)(0,2) * a_matrix(2,1);
        a_result(0,2) = (*this)(0,0) * a_matrix(0,2) + (*this)(0,1) * a_matrix(1,2) + (*this)(0,2) * a_matrix(2,2);
        a_result(1,0) = (*this)(1,0) * a_matrix(0,0) + (*this)(1,1) * a_matrix(1,0) + (*this)(1,2) * a_matrix(2,0);
        a_result(1,1) = (*this)(1,0) * a_matrix(0,1) + (*this)(1,1) * a_matrix(1,1) + (*this)(1,2) * a_matrix(2,1);
        a_result(1,2) = (*this)(1,0) * a_matrix(0,2) + (*this)(1,1) * a_matrix(1,2) + (*this)(1,2) * a_matrix(2,2);
        a_result(2,0) = (*this)(2,0) * a_matrix(0,0) + (*this)(2,1) * a_matrix(1,0) + (*this)(2,2) * a_matrix(2,0);
        a_result(2,1) = (*this)(2,0) * a_matrix(0,1) + (*this)(2,1) * a_matrix(1,1) + (*this)(2,2) * a_matrix(2,1);
        a_result(2,2) = (*this)(2,0) * a_matrix(0,2) + (*this)(2,1) * a_matrix(1,2) + (*this)(2,2) * a_matrix(2,2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This function computes the multiplication of this matrix with a vector.

        \details
        This function computes the multiplication of this matrix with a vector
        \p a_vector passed as argument. \n
        The result of this operation is stored in the same vector \p a_vector 
        overwriting previous values.

        \code
        a_vector = this * a_vector
        \endcode

        \param  a_vector  Input vector and return result.
    */
    //--------------------------------------------------------------------------
    inline void mul(cVector3d& a_vector) const
    {
        // compute multiplication
        double x = (*this)(0,0) * a_vector(0) + (*this)(0,1) * a_vector(1) + (*this)(0,2) * a_vector(2);
        double y = (*this)(1,0) * a_vector(0) + (*this)(1,1) * a_vector(1) + (*this)(1,2) * a_vector(2);
        double z = (*this)(2,0) * a_vector(0) + (*this)(2,1) * a_vector(1) + (*this)(2,2) * a_vector(2);

        // store result
        a_vector(0)  = x;
        a_vector(1)  = y;
        a_vector(2)  = z;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This function computes the multiplication of this matrix with a vector.

        \details
        This function computes the multiplication of this matrix with a vector
        \p a_vector passed as argument. \n
        The result of this operation is stored in the result vector \p a_result 
        passed as second argument.

        \param  a_vector  Input vector
        \param  a_result  Result of operation.
    */
    //--------------------------------------------------------------------------
    inline void mulr(const cVector3d& a_vector, 
                     cVector3d& a_result) const
    {
        a_result(0)  = (*this)(0,0) * a_vector(0) + (*this)(0,1) * a_vector(1) + (*this)(0,2) * a_vector(2);
        a_result(1)  = (*this)(1,0) * a_vector(0) + (*this)(1,1) * a_vector(1) + (*this)(1,2) * a_vector(2);
        a_result(2)  = (*this)(2,0) * a_vector(0) + (*this)(2,1) * a_vector(1) + (*this)(2,2) * a_vector(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes and returns the determinant of this matrix.

        \details
        This method computes and returns the determinant of this matrix.

        \return Determinant.of matrix.
    */
    //--------------------------------------------------------------------------
    inline double det() const
    {
        return (+ (*this)(0,0) * (*this)(1,1) * (*this)(2,2)
                + (*this)(0,1) * (*this)(1,2) * (*this)(2,0)
                + (*this)(0,2) * (*this)(1,0) * (*this)(2,1)
                - (*this)(2,0) * (*this)(1,1) * (*this)(0,2)
                - (*this)(2,1) * (*this)(1,2) * (*this)(0,0)
                - (*this)(2,2) * (*this)(1,0) * (*this)(0,1));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the transpose of this matrix.

        \details
        This method computes the transpose of this matrix. \n
        The result of this operation is stored in this matrix overwriting 
        previous values.
    */
    //--------------------------------------------------------------------------
    inline void trans()
    {
        double t;
        t = (*this)(0,1); (*this)(0,1) = (*this)(1,0); (*this)(1,0) = t;
        t = (*this)(0,2); (*this)(0,2) = (*this)(2,0); (*this)(2,0) = t;
        t = (*this)(1,2); (*this)(1,2) = (*this)(2,1); (*this)(2,1) = t;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the transpose of this matrix.

        \details
        This method computes the transpose of this matrix. \n
        The result of this operation is stored in the result matrix \p a_result 
        matrix passed as argument.

        \param  a_result  Output matrix.
    */
    //--------------------------------------------------------------------------
    inline void transr(cMatrix3d& a_result) const
    {
        a_result(0,0) = (*this)(0,0);
        a_result(0,1) = (*this)(1,0);
        a_result(0,2) = (*this)(2,0);

        a_result(1,0) = (*this)(0,1);
        a_result(1,1) = (*this)(1,1);
        a_result(1,2) = (*this)(2,1);

        a_result(2,0) = (*this)(0,2);
        a_result(2,1) = (*this)(1,2);
        a_result(2,2) = (*this)(2,2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the inverse of this matrix.

        \details
        This method computes the inverse of this matrix. \n
        If the operation succeeds, the result is stored in this matrix.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    bool invert()
    {
        // compute determinant
        double d = ( + (*this)(0,0) * (*this)(1,1) * (*this)(2,2)
                     + (*this)(0,1) * (*this)(1,2) * (*this)(2,0)
                     + (*this)(0,2) * (*this)(1,0) * (*this)(2,1)
                     - (*this)(2,0) * (*this)(1,1) * (*this)(0,2)
                     - (*this)(2,1) * (*this)(1,2) * (*this)(0,0)
                     - (*this)(2,2) * (*this)(1,0) * (*this)(0,1));

        // check if determinant null
        if ((d < C_TINY) && (d > -C_TINY))
        {
            // determinant null, matrix inversion could not be performed
            return (false);
        }
        else
        {
            // compute inverted matrix
            double m00 =  ((*this)(1,1) * (*this)(2,2) - (*this)(2,1)*(*this)(1,2)) / d;
            double m01 = -((*this)(0,1) * (*this)(2,2) - (*this)(2,1)*(*this)(0,2)) / d;
            double m02 =  ((*this)(0,1) * (*this)(1,2) - (*this)(1,1)*(*this)(0,2)) / d;

            double m10 = -((*this)(1,0) * (*this)(2,2) - (*this)(2,0)*(*this)(1,2)) / d;
            double m11 =  ((*this)(0,0) * (*this)(2,2) - (*this)(2,0)*(*this)(0,2)) / d;
            double m12 = -((*this)(0,0) * (*this)(1,2) - (*this)(1,0)*(*this)(0,2)) / d;

            double m20 =  ((*this)(1,0) * (*this)(2,1) - (*this)(2,0)*(*this)(1,1)) / d;
            double m21 = -((*this)(0,0) * (*this)(2,1) - (*this)(2,0)*(*this)(0,1)) / d;
            double m22 =  ((*this)(0,0) * (*this)(1,1) - (*this)(1,0)*(*this)(0,1)) / d;

            // return values to current matrix
            (*this)(0,0) = m00;  (*this)(0,1) = m01;  (*this)(0,2) = m02;
            (*this)(1,0) = m10;  (*this)(1,1) = m11;  (*this)(1,2) = m12;
            (*this)(2,0) = m20;  (*this)(2,1) = m21;  (*this)(2,2) = m22;

            // return success
            return (true);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method computes the inverse of this matrix.

        \details
        This method computes the inverse of this matrix. \n
        If the operation succeeds, the result is stored in the result matrix 
        \p a_result passed as argument.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    bool invertr(cMatrix3d& a_result) const
    {
        // compute determinant
        double d = ( + (*this)(0,0) * (*this)(1,1) * (*this)(2,2)
                     + (*this)(0,1) * (*this)(1,2) * (*this)(2,0)
                     + (*this)(0,2) * (*this)(1,0) * (*this)(2,1)
                     - (*this)(2,0) * (*this)(1,1) * (*this)(0,2)
                     - (*this)(2,1) * (*this)(1,2) * (*this)(0,0)
                     - (*this)(2,2) * (*this)(1,0) * (*this)(0,1));

        // check if determinant null.
        if ((d < C_TINY) && (d > -C_TINY))
        {
            // determinant null, matrix inversion can not be performed
            return (false);
        }
        else
        {
            // compute inverted matrix
            a_result(0,0) =  ((*this)(1,1) * (*this)(2,2) - (*this)(2,1)*(*this)(1,2)) / d;
            a_result(0,1) = -((*this)(0,1) * (*this)(2,2) - (*this)(2,1)*(*this)(0,2)) / d;
            a_result(0,2) =  ((*this)(0,1) * (*this)(1,2) - (*this)(1,1)*(*this)(0,2)) / d;

            a_result(1,0) = -((*this)(1,0) * (*this)(2,2) - (*this)(2,0)*(*this)(1,2)) / d;
            a_result(1,1) =  ((*this)(0,0) * (*this)(2,2) - (*this)(2,0)*(*this)(0,2)) / d;
            a_result(1,2) = -((*this)(0,0) * (*this)(1,2) - (*this)(1,0)*(*this)(0,2)) / d;

            a_result(2,0) =  ((*this)(1,0) * (*this)(2,1) - (*this)(2,0)*(*this)(1,1)) / d;
            a_result(2,1) = -((*this)(0,0) * (*this)(2,1) - (*this)(2,0)*(*this)(0,1)) / d;
            a_result(2,2) =  ((*this)(0,0) * (*this)(1,1) - (*this)(1,0)*(*this)(0,1)) / d;

            // return success
            return (true);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method orthogonalizes this matrix.

        \details
        This method orthogonalizes this matrix so that all columns vectors are 
        orthogonal to each other and of length 1. This method is useful if your 
        rotation matrix has accumulated small error. 
    */
    //--------------------------------------------------------------------------
    void orthogonalize()
    {
        cVector3d c0 = getCol0();
        cVector3d c1 = getCol1();
        cVector3d c2;

        c0.crossr(c1, c2);
        c2.crossr(c0, c1);

        c0.normalize();
        c1.normalize();
        c2.normalize();
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a rotation matrix from an __axis-angle__ representation.

        \details
        this method builds a rotation matrix from an input _axis_ and _angle_ defined in 
        radians.\n 
        The result is stored in this matrix overwriting previous values.

        \param  a_axis      Axis of rotation.
        \param  a_angleRad  Angle in radians.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool setAxisAngleRotationRad(const cVector3d& a_axis, 
                                        const double& a_angleRad)
    {
        // compute length of axis vector
        double length = a_axis.length();

        // check length of axis vector
        if (length < C_TINY)
        {
            // rotation matrix could not be computed because axis vector is not defined
            return (false);
        }

        // normalize axis vector
        double f = 1.0 / length;
        double x = f * a_axis(0);
        double y = f * a_axis(1);
        double z = f * a_axis(2);

        // compute rotation matrix
        double c = ::cos(a_angleRad);
        double s = ::sin(a_angleRad);
        double v = 1-c;

        (*this)(0,0) = x*x*v+c;     (*this)(0,1) = x*y*v-z*s;  (*this)(0,2) = x*z*v+y*s;
        (*this)(1,0) = x*y*v+z*s;   (*this)(1,1) = y*y*v+c;    (*this)(1,2) = y*z*v-x*s;
        (*this)(2,0) = x*z*v-y*s;   (*this)(2,1) = y*z*v+x*s;  (*this)(2,2) = z*z*v+c;

        // return success
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a rotation matrix from an __axis-angle__ representation.

        \details
        This method builds a rotation matrix from an input _axis_ and _angle_ defined 
        in degrees. \n 
        The result is stored in this matrix overwriting previous values.

        \param  a_axis      Axis of rotation.
        \param  a_angleDeg  Angle in degrees.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool setAxisAngleRotationDeg(const cVector3d& a_axis, 
                                        const double& a_angleDeg)
    {
        return (setAxisAngleRotationRad(a_axis, C_DEG2RAD * (a_angleDeg)));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a rotation matrix from an __axis-angle__ representation.

        \details
        This method builds a rotation matrix from an input _axis_ and _angle_ defined in 
        radians. \n 
        The result is stored in this matrix overwriting previous values.

        \param  a_axisX     Component __x__ of axis.
        \param  a_axisY     Component __y__ of axis.
        \param  a_axisZ     Component __z__ of axis.
        \param  a_angleRad  Angle in radians.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool setAxisAngleRotationRad(const double& a_axisX,
                                        const double& a_axisY,
                                        const double& a_axisZ,
                                        const double& a_angleRad)
    {
        return (setAxisAngleRotationRad(cVector3d(a_axisX, a_axisY, a_axisZ), a_angleRad));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a rotation matrix from __axis-angle__ representation.

        \details
        This method builds a rotation matrix from an _axis_ and _angle_ defined 
        in degrees. \n
        The result is stored in this matrix overwriting previous values.

        \param  a_axisX     Component __x__ of axis.
        \param  a_axisY     Component __y__ of axis.
        \param  a_axisZ     Component __z__ of axis.
        \param  a_angleDeg  Angle in degrees.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool setAxisAngleRotationDeg(const double& a_axisX,
                                        const double& a_axisY,
                                        const double& a_axisZ,
                                        const double& a_angleDeg)
    {
        return (setAxisAngleRotationRad(cVector3d(a_axisX, a_axisY, a_axisZ), C_DEG2RAD * (a_angleDeg)));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a rotation matrix from an Euler angle representation.

        \details
        This method builds a rotation matrix from a set of Euler angles and fixed
        axes of rotations.

        \param  a_angle1      Angle in radians of the first rotation in the sequence.
        \param  a_angle2      Angle in radians of the second rotation in the sequence.
        \param  a_angle3      Angle in radians of the third rotation in the sequence.
        \param  a_eulerOrder  The order of the axes about which the rotations 
                              are to be applied.
    */
    //--------------------------------------------------------------------------
    inline void setExtrinsicEulerRotationRad(const double& a_angle1,
                                             const double& a_angle2,
                                             const double& a_angle3,
                                             const cEulerOrder a_eulerOrder)
    {
       switch(a_eulerOrder)
        {
            case(C_EULER_ORDER_XYZ):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_ZYX);
                break;

            case(C_EULER_ORDER_XYX):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_XYX);
                break;

            case(C_EULER_ORDER_XZY):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_YZX);
                break;

            case(C_EULER_ORDER_XZX):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_XZX);
                break;

            case(C_EULER_ORDER_YZX):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_XZY);
                break;

            case(C_EULER_ORDER_YZY):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_YZY);
                break;

            case(C_EULER_ORDER_YXZ):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_ZXY);
                break;

            case(C_EULER_ORDER_YXY):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_YXY);
                break;

            case(C_EULER_ORDER_ZXY):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_YXZ);
                break; 

            case(C_EULER_ORDER_ZXZ):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_ZXZ);
                break; 

            case(C_EULER_ORDER_ZYX):
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_XYZ);
                break; 

            case(C_EULER_ORDER_ZYZ): 
                setIntrinsicEulerRotationRad(a_angle3, a_angle2, a_angle1, C_EULER_ORDER_ZYZ);
                break;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a rotation matrix from an Euler angle representation.

        \details
        This method builds a rotation matrix from a set of Euler angles and fixed 
        axes of rotations.

        \param  a_angle1      Angle in degrees of the first rotation in the sequence.
        \param  a_angle2      Angle in degrees of the second rotation in the sequence.
        \param  a_angle3      Angle in degrees of the third rotation in the sequence.
        \param  a_eulerOrder  The order of the axes about which the rotations 
                              are to be applied.
    */
    //--------------------------------------------------------------------------
    inline void setExtrinsicEulerRotationDeg(const double& a_angle1,
                                             const double& a_angle2,
                                             const double& a_angle3,
                                             const cEulerOrder a_eulerOrder)
    {
        setExtrinsicEulerRotationRad(C_DEG2RAD * (a_angle1),
                                     C_DEG2RAD * (a_angle2),
                                     C_DEG2RAD * (a_angle3),
                                     a_eulerOrder);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a rotation matrix from an Euler angle representation.

        \details
        This method builds a rotation matrix from a set of Euler angles and 
        co-moving axes of rotations.

        \param  a_angle1      Angle in radians of the first rotation in the sequence.
        \param  a_angle2      Angle in radians of the second rotation in the sequence.
        \param  a_angle3      Angle in radians of the third rotation in the sequence.
        \param  a_eulerOrder  The order of the axes about which the rotations 
                              are to be applied.
    */
    //--------------------------------------------------------------------------
    inline void setIntrinsicEulerRotationRad(const double& a_angle1,
                                             const double& a_angle2,
                                             const double& a_angle3,
                                             const cEulerOrder a_eulerOrder)
    {
        double c1 = ::cos(a_angle1);
        double s1 = ::sin(a_angle1);
        double c2 = ::cos(a_angle2);
        double s2 = ::sin(a_angle2);
        double c3 = ::cos(a_angle3);
        double s3 = ::sin(a_angle3);

        switch(a_eulerOrder)
        {
            case(C_EULER_ORDER_XYZ):
                (*this)(0,0) = c2*c3;               (*this)(0,1) =-c2*s3;               (*this)(0,2) = s2; 
                (*this)(1,0) = c1*s3+c3*s1*s2;      (*this)(1,1) = c1*c3-s1*s2*s3;      (*this)(1,2) =-c2*s1; 
                (*this)(2,0) = s1*s3-c1*c3*s2;      (*this)(2,1) = c3*s1+c1*s2*s3;      (*this)(2,2) = c1*c2; 
                break;

            case(C_EULER_ORDER_XYX):
                (*this)(0,0) = c2;                  (*this)(0,1) = s2*s3;               (*this)(0,2) = c3*s2; 
                (*this)(1,0) = s1*s2;               (*this)(1,1) = c1*c3-c2*s1*s3;      (*this)(1,2) =-c1*s3-c2*c3*s1; 
                (*this)(2,0) =-c1*s2;               (*this)(2,1) = c3*s1+c1*c2*s3;      (*this)(2,2) = c1*c2*c3-s1*s3; 
                break;

            case(C_EULER_ORDER_XZY):
                (*this)(0,0) = c2*c3;               (*this)(0,1) =-s2;                  (*this)(0,2) = c2*s3; 
                (*this)(1,0) = s1*s3+c1*c3*s2;      (*this)(1,1) = c1*c2;               (*this)(1,2) = c1*s2*s3-c3*s1; 
                (*this)(2,0) = c3*s1*s2-c1*s3;      (*this)(2,1) = c2*s1;               (*this)(2,2) = c1*c3+s1*s2*s3;
                break;

            case(C_EULER_ORDER_XZX):
                (*this)(0,0) = c2;                  (*this)(0,1) =-c3*s2;               (*this)(0,2) = s2*s3; 
                (*this)(1,0) = c1*s2;               (*this)(1,1) = c1*c2*c3-s1*s3;      (*this)(1,2) =-c3*s1-c1*c2*s3; 
                (*this)(2,0) = s1*s2;               (*this)(2,1) = c1*s3+c2*c3*s1;      (*this)(2,2) = c1*c3-c2*s1*s3; 
                break;

            case(C_EULER_ORDER_YZX):
                (*this)(0,0) = c1*c2;               (*this)(0,1) = s1*s3-c1*c3*s2;      (*this)(0,2) = c3*s1+c1*s2*s3; 
                (*this)(1,0) = s2;                  (*this)(1,1) = c2*c3;               (*this)(1,2) =-c2*s3; 
                (*this)(2,0) =-c2*s1;               (*this)(2,1) = c1*s3+c3*s1*s2;      (*this)(2,2) = c1*c3-s1*s2*s3;
                break;

            case(C_EULER_ORDER_YZY):
                (*this)(0,0) = c1*c2*c3-s1*s3;      (*this)(0,1) =-c1*s2;               (*this)(0,2) = c3*s1+c1*c2*s3; 
                (*this)(1,0) = c3*s2;               (*this)(1,1) = c2;                  (*this)(1,2) = s2*s3; 
                (*this)(2,0) =-c1*s3-c2*c3*s1;      (*this)(2,1) = s1*s2;               (*this)(2,2) = c1*c3-c2*s1*s3; 
                break;

            case(C_EULER_ORDER_YXZ):
                (*this)(0,0) = c1*c3+s1*s2*s3;      (*this)(0,1) = c3*s1*s2-c1*s3;       (*this)(0,2) = c2*s1; 
                (*this)(1,0) = c2*s3;               (*this)(1,1) = c2*c3;                (*this)(1,2) =-s2; 
                (*this)(2,0) = c1*s2*s3-c3*s1;      (*this)(2,1) = s1*s3+c1*c3*s2;       (*this)(2,2) = c1*c2;
                break;

            case(C_EULER_ORDER_YXY):
                (*this)(0,0) = c1*c3-c2*s1*s3;      (*this)(0,1) = s1*s2;               (*this)(0,2) = c1*s3+c2*c3*s1; 
                (*this)(1,0) = s2*s3;               (*this)(1,1) = c2;                  (*this)(1,2) =-c3*s2; 
                (*this)(2,0) =-c3*s1-c1*c2*s3;      (*this)(2,1) = c1*s2;               (*this)(2,2) = c1*c2*c3-s1*s3;                                                        
                break;

            case(C_EULER_ORDER_ZXY):
                (*this)(0,0) = c1*c3-s1*s2*s3;      (*this)(0,1) =-c2*s1;               (*this)(0,2) = c1*s3+c3*s1*s2; 
                (*this)(1,0) = c3*s1+c1*s2*s3;      (*this)(1,1) = c1*c2;               (*this)(1,2) = s1*s3-c1*c3*s2; 
                (*this)(2,0) =-c2*s3;               (*this)(2,1) = s2;                  (*this)(2,2) = c2*c3;
                break; 

            case(C_EULER_ORDER_ZXZ):
                (*this)(0,0) = c1*c3-c2*s1*s3;      (*this)(0,1) =-c1*s3-c2*c3*s1;      (*this)(0,2) = s1*s2; 
                (*this)(1,0) = c3*s1+c1*c2*s3;      (*this)(1,1) = c1*c2*c3-s1*s3;      (*this)(1,2) =-c1*s2; 
                (*this)(2,0) = s2*s3;               (*this)(2,1) = c3*s2;               (*this)(2,2) = c2;
                break; 

            case(C_EULER_ORDER_ZYX):
                (*this)(0,0) = c1*c2;               (*this)(0,1) = c1*s2*s3-c3*s1;      (*this)(0,2) = s1*s3+c1*c3*s2; 
                (*this)(1,0) = c2*s1;               (*this)(1,1) = c1*c3+s1*s2*s3;      (*this)(1,2) = c3*s1*s2-c1*s3; 
                (*this)(2,0) =-s2;                  (*this)(2,1) = c2*s3;               (*this)(2,2) = c2*c3;
                break; 

            case(C_EULER_ORDER_ZYZ): 
                (*this)(0,0) = c1*c2*c3-s1*s3;      (*this)(0,1) =-c3*s1-c1*c2*s3;      (*this)(0,2) = c1*s2; 
                (*this)(1,0) = c1*s3+c2*c3*s1;      (*this)(1,1) = c1*c3-c2*s1*s3;      (*this)(1,2) = s1*s2; 
                (*this)(2,0) =-c3*s2;               (*this)(2,1) = s2*s3;               (*this)(2,2) = c2;
                break;
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a rotation matrix from an Euler angle representation.

        \details
        This method builds a rotation matrix from a set of Euler angles and 
        co-moving axes of rotations.

        \param  a_angle1      Angle in degrees of the first rotation in the sequence.
        \param  a_angle2      Angle in degrees of the second rotation in the sequence.
        \param  a_angle3      Angle in degrees of the third rotation in the sequence.
        \param  a_eulerOrder  The order of the axes about which the rotations 
                              are to be applied.
    */
    //--------------------------------------------------------------------------
    inline void setIntrinsicEulerRotationDeg(const double& a_angle1,
                                             const double& a_angle2,
                                             const double& a_angle3,
                                             const cEulerOrder a_eulerOrder)
    {
        setIntrinsicEulerRotationRad(C_DEG2RAD * (a_angle1),
                                     C_DEG2RAD * (a_angle2),
                                     C_DEG2RAD * (a_angle3),
                                     a_eulerOrder);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method rotates this matrix around an axis described in global
        coordinates. 

        \details
        This method rotates this matrix around an axis described in global 
        coordinates.

        \param  a_axis      Axis of rotation.
        \param  a_angleRad  Angle in radians.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool rotateAboutGlobalAxisRad(const cVector3d& a_axis, 
                                         const double& a_angleRad)
    {
        // compute length of axis vector
        double length = a_axis.length();

        // check length of axis vector
        if (length < C_TINY)
        {
            // rotation matrix could not be computed because axis vector is not defined
            return (false);
        }

        // normalize axis vector
        double f = 1.0 / length;
        double x = f * a_axis(0);
        double y = f * a_axis(1);
        double z = f * a_axis(2);

        // compute rotation matrix
        double c = ::cos(a_angleRad);
        double s = ::sin(a_angleRad);
        double v = 1-c;

        cMatrix3d m;
        m(0,0) = x*x*v+c;     m(0,1) = x*y*v-z*s;  m(0,2) = x*z*v+y*s;
        m(1,0) = x*y*v+z*s;   m(1,1) = y*y*v+c;    m(1,2) = y*z*v-x*s;
        m(2,0) = x*z*v-y*s;   m(2,1) = y*z*v+x*s;  m(2,2) = z*z*v+c;

        (*this) = m*(*this);

        // return success
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method rotates this matrix around an axis described in global 
        coordinates.

        \details
        This method rotates this matrix around an axis described in global 
        coordinates.

        \param  a_axis      Axis of rotation.
        \param  a_angleDeg  Angle in degrees.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool rotateAboutGlobalAxisDeg(const cVector3d& a_axis, 
                                         const double& a_angleDeg)
    {
        return (rotateAboutGlobalAxisRad(a_axis, C_DEG2RAD * (a_angleDeg)));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method rotates this matrix around an axis described in global 
        coordinates.

        \details
        This method rotates this matrix around an axis described in global 
        coordinates.

        \param  a_axisX     __x__ component of axis.
        \param  a_axisY     __y__ component of axis.
        \param  a_axisZ     __z__ component of axis.
        \param  a_angleRad  Angle in radians.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool rotateAboutGlobalAxisRad(const double& a_axisX,
                                         const double& a_axisY,
                                         const double& a_axisZ,
                                         const double& a_angleRad)
    {
        return(rotateAboutGlobalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), a_angleRad));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method rotates this matrix around an axis described in global 
        coordinates.

        \details
        This method rotates current matrix around an axis described in global 
        coordinates.

        \param  a_axisX     __x__ component of axis.
        \param  a_axisY     __y__ component of axis.
        \param  a_axisZ     __z__ component of axis.
        \param  a_angleDeg  Angle in degrees.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool rotateAboutGlobalAxisDeg(const double& a_axisX,
                                         const double& a_axisY,
                                         const double& a_axisZ,
                                         const double& a_angleDeg)
    {
        return(rotateAboutGlobalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), C_DEG2RAD * (a_angleDeg)));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method rotates this matrix around an axis described in local 
        coordinates.

        \details
        This method rotates this current matrix around an axis described in 
        local coordinates.

        \param  a_axis      Axis of rotation.
        \param  a_angleRad  Angle in radians.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool rotateAboutLocalAxisRad(const cVector3d& a_axis, 
                                        const double& a_angleRad)
    {
        // compute length of axis vector
        double length = a_axis.length();

        // check length of axis vector
        if (length < C_TINY)
        {
            // rotation matrix could not be computed because axis vector is not defined
            return (false);
        }

        // normalize axis vector
        double f = 1.0 / length;
        double x = f * a_axis(0);
        double y = f * a_axis(1);
        double z = f * a_axis(2);

        // compute rotation matrix
        double c = ::cos(a_angleRad);
        double s = ::sin(a_angleRad);
        double v = 1-c;

        cMatrix3d m;
        m(0,0) = x*x*v+c;     m(0,1) = x*y*v-z*s;  m(0,2) = x*z*v+y*s;
        m(1,0) = x*y*v+z*s;   m(1,1) = y*y*v+c;    m(1,2) = y*z*v-x*s;
        m(2,0) = x*z*v-y*s;   m(2,1) = y*z*v+x*s;  m(2,2) = z*z*v+c;

        (*this) = (*this)*m;

        // return success
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method rotates this matrix around an axis described in local 
        coordinates.

        \details
        This method rotates this matrix around an axis described in local
        coordinates.

        \param  a_axis      Axis of rotation.
        \param  a_angleDeg  Angle in degrees.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool rotateAboutLocalAxisDeg(const cVector3d& a_axis, 
                                        const double& a_angleDeg)
    {
        return(rotateAboutLocalAxisRad(a_axis, C_DEG2RAD * (a_angleDeg)));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method rotates this matrix around an axis described in local 
        coordinates.

        \details
        This method rotates this matrix around an axis described in local
        coordinates.

        \param  a_axisX     __x__ component of axis.
        \param  a_axisY     __y__ component of axis.
        \param  a_axisZ     __z__ component of axis.
        \param  a_angleRad  Angle in radians.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool rotateAboutLocalAxisRad(const double& a_axisX,
                                        const double& a_axisY,
                                        const double& a_axisZ,
                                        const double& a_angleRad)
    {
        return(rotateAboutLocalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), a_angleRad));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method rotates this matrix around an axis described in local 
        coordinates.

        \details
        This method rotates this matrix around an axis described in local 
        coordinates.

        \param  a_axisX     __x__ component of axis.
        \param  a_axisY     __y__ component of axis.
        \param  a_axisZ     __z__ component of axis.
        \param  a_angleDeg  Angle in degrees.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool rotateAboutLocalAxisDeg(const double& a_axisX,
                                        const double& a_axisY,
                                        const double& a_axisZ,
                                        const double& a_angleDeg)
    {
        return(rotateAboutLocalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), C_DEG2RAD * (a_angleDeg)));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method converts this rotation matrix to an __axis-angle__ 
        representation.

        \details
        This method converts this rotation matrix to an __axis-angle__ 
        representation.

        \param  a_axis   Output axis.
        \param  a_angle  Output angle.

        \return __true__ if operation succeeds, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    bool toAxisAngle(cVector3d& a_axis, 
                     double& a_angle) const
    {
        double angle,x,y,z;     // variables for result
        double epsilon1 = 0.01; // margin to allow for rounding errors
        double epsilon2 = 0.1;  // margin to distinguish between 0 and 180 degrees

        if ((fabs((*this)(0,1) - (*this)(1,0)) < epsilon1) &&
            (fabs((*this)(0,2) - (*this)(2,0)) < epsilon1) &&
            (fabs((*this)(1,2) - (*this)(2,1)) < epsilon1))
        {
            // singularity found
            // first check for identity matrix which must have +1 for all terms
            // in leading diagonal and zero in other terms
            if ( (fabs((*this)(0,1) + (*this)(1,0)) < epsilon2) &&
                 (fabs((*this)(0,2) + (*this)(2,0)) < epsilon2) &&
                 (fabs((*this)(1,2) + (*this)(2,1)) < epsilon2) &&
                 (fabs((*this)(0,0) + (*this)(1,1) + (*this)(2,2)-3) < epsilon2))
            {
                // this singularity is identity matrix so angle = 0
                a_axis.set(1,0,0);
                a_angle = 0;
                return (true);
            }

            // otherwise this singularity is angle = 180
            angle = C_PI;
            double xx = ((*this)(0,0)+1)/2;
            double yy = ((*this)(1,1)+1)/2;
            double zz = ((*this)(2,2)+1)/2;
            double xy = ((*this)(0,1)+(*this)(1,0))/4;
            double xz = ((*this)(0,2)+(*this)(2,0))/4;
            double yz = ((*this)(1,2)+(*this)(2,1))/4;

            if ((xx > yy) && (xx > zz)) 
            { 
                // (*this)(0,0) is the largest diagonal term
                if (xx < epsilon1) 
                {
                    x = 0;
                    y = 0.7071;
                    z = 0.7071;
                } 
                else 
                {
                    x = sqrt(xx);
                    y = xy/x;
                    z = xz/x;
                }
            } 
            else if (yy > zz) 
            { 
                // (*this)(1,1) is the largest diagonal term
                if (yy< epsilon1) 
                {
                    x = 0.7071067811865475;
                    y = 0.0;
                    z = 0.7071067811865475;
                } 
                else 
                {
                    y = sqrt(yy);
                    x = xy/y;
                    z = yz/y;
                }	
            } 
            else 
            { 
                // (*this)(2,2) is the largest diagonal term so base result on this
                if (zz < epsilon1) 
                {
                    x = 0.7071067811865475;
                    y = 0.7071067811865475;
                    z = 0.0;
                } 
                else 
                {
                    z = sqrt(zz);
                    x = xz/z;
                    y = yz/z;
                }
            }

            a_axis.set(x,y,z);
            a_angle = angle;
            return (true);
        }

        // as we have reached here there are no singularities so we can handle normally
        double s = sqrt(((*this)(2,1) - (*this)(1,2))*((*this)(2,1) - (*this)(1,2)) +
                        ((*this)(0,2) - (*this)(2,0))*((*this)(0,2) - (*this)(2,0)) +
                        ((*this)(1,0) - (*this)(0,1))*((*this)(1,0) - (*this)(0,1)));


        // prevent divide by zero, should not happen if matrix is orthogonal and should be
        // caught by singularity test above, but I've left it in just in case
        if (fabs(s) < 0.001) s=1;

        angle = acos(( (*this)(0,0) + (*this)(1,1) + (*this)(2,2) - 1)/2);
        x = ((*this)(2,1) - (*this)(1,2))/s;
        y = ((*this)(0,2) - (*this)(2,0))/s;
        z = ((*this)(1,0) - (*this)(0,1))/s;

        a_axis.set(x,y,z);
        a_angle = angle;

        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method converts this matrix to a string representation.

        \details
        This method converts this matrix to a string representation. The output 
        springs displays the three column vectors of matrix. \n
        The number of digits after the decimal point are set by 
        argument \p a_precision.

        \param  a_precision  Number of digits.

        \return Converted matrix in string format.
    */
    //--------------------------------------------------------------------------
    inline std::string str(const unsigned int a_precision = 2) const
    {
        std::string result;
        result.append("[ ");

        for (int i=0; i<3; i++)
        {
            result.append("( ");
            for (int j=0; j<3; j++)
            {
                result.append(cStr((*this)(j,i), a_precision));
                if (j<2)
                {
                    result.append(", ");
                }
            }
            result.append(" ) ");
        }
        result.append("]");

        return (result);
    }


    //! An overloaded <b> *= </b> operator for matrix/scalar multiplication.
    inline void operator*= (const double& a_val)
    {
        (*this)(0,0) *= a_val; (*this)(0,1) *= a_val; (*this)(0,2) *= a_val;
        (*this)(1,0) *= a_val; (*this)(1,1) *= a_val; (*this)(1,2) *= a_val;
        (*this)(2,0) *= a_val; (*this)(2,1) *= a_val; (*this)(2,2) *= a_val;
    }


    //! An overloaded <b> * </b> operator for matrix/vector multiplication.
    inline cVector3d operator* (const cVector3d& a_val)
    {
        cVector3d result;
        mulr(a_val, result);
        return result;
    }


    //! An overloaded <b> * </b> operator for matrix/matrix multiplication.
    inline cMatrix3d operator* (const cMatrix3d& a_val)
    {
        cMatrix3d result;
        mulr(a_val,result);
        return result;
    }


    //! An overloaded <b> *= </b> operator for matrix/matrix multiplication.
    inline void operator*= (const cMatrix3d& a_val)
    {
        (*this).mul(a_val);
    }


    //! An overloaded <b> += </b> operator for matrix/matrix addition.
    inline void operator+= (const cMatrix3d& a_input)
    {
        (*this)(0,0) += a_input(0,0);
        (*this)(0,1) += a_input(0,1);
        (*this)(0,2) += a_input(0,2);

        (*this)(1,0) += a_input(1,0);
        (*this)(1,1) += a_input(1,1);
        (*this)(1,2) += a_input(1,2);

        (*this)(2,0) += a_input(2,0);
        (*this)(2,1) += a_input(2,1);
        (*this)(2,2) += a_input(2,2);
    }


    //! An overloaded <b> -= </b> operator for matrix/matrix subtraction.
    inline void operator-= (const cMatrix3d& a_input)
    {
        (*this)(0,0) -= a_input(0,0);
        (*this)(0,1) -= a_input(0,1);
        (*this)(0,2) -= a_input(0,2);

        (*this)(1,0) -= a_input(1,0);
        (*this)(1,1) -= a_input(1,1);
        (*this)(1,2) -= a_input(1,2);

        (*this)(2,0) -= a_input(2,0);
        (*this)(2,1) -= a_input(2,1);
        (*this)(2,2) -= a_input(2,2);
    }

   
    //! An overloaded () operator.
    inline double& operator() (const int a_index0, const int a_index1)
    {
        return m_data[a_index0][a_index1];
    }


    //! An overloaded () operator.
    inline const double& operator() (const int a_index0, const int a_index1) const
    {
        return m_data[a_index0][a_index1];
    }


    //--------------------------------------------------------------------------
    // PRIVATE MEMBERS
    //--------------------------------------------------------------------------
    
private:

    //! Matrix data.
    double m_data[3][3];
};


//==============================================================================
// OPERATORS
//==============================================================================

//! An overloaded <b> * </b> operator for matrix/vector multiplication.
inline cVector3d operator*(const cMatrix3d& a_matrix, 
                           const cVector3d& a_vector)
{
    cVector3d result;
    a_matrix.mulr(a_vector, result);
    return (result);
}


//! An overloaded <b> * </b> operator for matrix/matrix multiplication.
inline cMatrix3d operator*(const cMatrix3d& a_matrix1, 
                           const cMatrix3d& a_matrix2)
{
    cMatrix3d result;
    a_matrix1.mulr(a_matrix2, result);
    return (result);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
