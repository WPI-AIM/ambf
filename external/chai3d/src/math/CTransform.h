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
#ifndef CTransformH
#define CTransformH
//------------------------------------------------------------------------------
#include "math/CMatrix3d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CTransform.h
    \ingroup    math

    \brief
    Implements a transformation matrix.
*/
//==============================================================================


//==============================================================================
/*!
    \struct   cTransform
    \ingroup  math

    \brief
    This class implements a 4D transformation matrix encoded as column-major.
*/
//==============================================================================
struct cTransform
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        Default constructor of cTransform.
    */
    //--------------------------------------------------------------------------
    cTransform()
    {
        identity();
        m_flagTransform = true;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Constructor of cTransform.

        \details
        Builds a transformation matrix from a position vector and rotation matrix
        passed as arguments.

        \param  a_pos  Position vector.
        \param  a_rot  Rotation matrix.
    */
    //--------------------------------------------------------------------------
    cTransform(const cVector3d& a_pos, const cMatrix3d& a_rot)
    {
        identity();
        set(a_pos, a_rot);
        m_flagTransform = true;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Constructor of cTransform.

        \details
        Builds a transformation matrix from a position vector passed as argument.

        \param  a_pos  Position vector.
    */
    //--------------------------------------------------------------------------
    cTransform(const cVector3d& a_pos)
    {
        identity();
        setLocalPos(a_pos);
        m_flagTransform = true;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        Constructor of cTransform.

        \details
        Build transformation matrix from a rotation matrix passed as argument.

        \param  a_rot  Rotation matrix.
    */
    //--------------------------------------------------------------------------
    cTransform(const cMatrix3d& a_rot)
    {
        identity();
        setLocalRot(a_rot);
        m_flagTransform = true;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a transformation matrix from a position vector.

        \details
        This method builds a transformation matrix from a position vector
        \p a_pos passed as argument.

        \param  a_pos  Position vector.
    */
    //--------------------------------------------------------------------------
    inline void setLocalPos(const cVector3d& a_pos)
    {
        m[3][0] = a_pos(0); m[3][1] = a_pos(1);  m[3][2] = a_pos(2);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method Builds a transformation matrix from a rotation matrix.

        \details
        This method Builds a transformation matrix from a rotation matrix 
        \p a_rot passed as argument.

        \param  a_rot  Rotation matrix.
    */
    //--------------------------------------------------------------------------
    void setLocalRot(const cMatrix3d& a_rot)
    {
        m[0][0] = a_rot(0,0);  m[0][1] = a_rot(1,0);  m[0][2] = a_rot(2,0);
        m[1][0] = a_rot(0,1);  m[1][1] = a_rot(1,1);  m[1][2] = a_rot(2,1);
        m[2][0] = a_rot(0,2);  m[2][1] = a_rot(1,2);  m[2][2] = a_rot(2,2);

        m_flagTransform = true;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method Builds a transformation matrix from a position vector
        and a rotation matrix.

        \details
        This method Builds a transformation matrix from a position vector
        \p a_pos and a rotation matrix \p a_rot passed as arguments.

        \param  a_pos  Position vector.
        \param  a_rot  Rotation matrix.
    */
    //--------------------------------------------------------------------------
    void set(const cVector3d& a_pos,
             const cMatrix3d& a_rot)
    {
        m[0][0] = a_rot(0,0);  m[0][1] = a_rot(1,0);  m[0][2] = a_rot(2,0);
        m[1][0] = a_rot(0,1);  m[1][1] = a_rot(1,1);  m[1][2] = a_rot(2,1);
        m[2][0] = a_rot(0,2);  m[2][1] = a_rot(1,2);  m[2][2] = a_rot(2,2);
        m[3][0] = a_pos(0);    m[3][1] = a_pos(1);    m[3][2] = a_pos(2);

        m_flagTransform = true;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method sets values to all elements of this transformation matrix.

        \details
        This method sets values passed as arguments to all elements of this 
        transformation matrix.

        \param  a_00  Matrix element (0,0).
        \param  a_01  Matrix element (0,1).
        \param  a_02  Matrix element (0,2).
        \param  a_03  Matrix element (0,3).
        \param  a_10  Matrix element (1,0).
        \param  a_11  Matrix element (1,1).
        \param  a_12  Matrix element (1,2).
        \param  a_13  Matrix element (1,3).
        \param  a_20  Matrix element (2,0).
        \param  a_21  Matrix element (2,1).
        \param  a_22  Matrix element (2,2).
        \param  a_23  Matrix element (2,3).
        \param  a_30  Matrix element (3,0).
        \param  a_31  Matrix element (3,1).
        \param  a_32  Matrix element (3,2).
        \param  a_33  Matrix element (3,3).
    */
    //--------------------------------------------------------------------------
    inline void set(const double& a_00, const double& a_01, const double& a_02, const double& a_03,
                    const double& a_10, const double& a_11, const double& a_12, const double& a_13,
                    const double& a_20, const double& a_21, const double& a_22, const double& a_23,
                    const double& a_30, const double& a_31, const double& a_32, const double& a_33)
    {
        m[0][0] = a_00;      m[0][1] = a_01;       m[0][2] = a_02;       m[0][3] = a_03;
        m[1][0] = a_10;      m[1][1] = a_11;       m[1][2] = a_12;       m[1][3] = a_13;
        m[2][0] = a_20;      m[2][1] = a_21;       m[2][2] = a_22;       m[2][3] = a_23;
        m[3][0] = a_30;      m[3][1] = a_31;       m[3][2] = a_32;       m[3][3] = a_33;

        m_flagTransform = false;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method creates a frustum matrix.

        \details
        This method creates a frustum matrix, as defined by the OpenGL 
        glFrustum() function.

        \param  a_l  Specifies the coordinate for the left clipping plane.
        \param  a_r  Specifies the coordinate for the right plane.
        \param  a_b  Specifies the coordinate for the bottom and top horizontal clipping plane.
        \param  a_t  Specifies the coordinate for the top horizontal clipping plane.
        \param  a_n  Specifies the distance to the near depth clipping plane. Distance must be positive.
        \param  a_f  Specifies the distance to the far depth clipping plane. Distance must be positive.
    */
    //--------------------------------------------------------------------------
    inline void setFrustumMatrix(double a_l,
                                 double a_r,
                                 double a_b,
                                 double a_t,
                                 double a_n,
                                 double a_f)
    {
        m[0][0] = (2.0*a_n) / (a_r-a_l);
        m[0][1] = 0.0;
        m[0][2] = 0.0;
        m[0][3] = 0.0;

        m[1][0] = 0.0;
        m[1][1] = (2.0*a_n) / (a_t-a_b);
        m[1][2] = 0.0;
        m[1][3] = 0.0;

        m[2][0] = (a_r+a_l) / (a_r-a_l);
        m[2][1] = (a_t+a_b) / (a_t-a_b);
        m[2][2] = -(a_f+a_n) / (a_f-a_n);
        m[2][3] = -1.0;

        m[3][0] = 0.0;
        m[3][1] = 0.0;
        m[3][2] = -(2.0*a_f*a_n) / (a_f-a_n);
        m[3][3] = 0.0;

        m_flagTransform = false;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method build a transformation matrix.

        \details
        This method build a transformation matrix, according to the OpenGL
        gluLookAt() function.

        \param  a_eyeX     Specifies the position of the eye point.
        \param  a_eyeY     Specifies the position of the eye point.
        \param  a_eyeZ     Specifies the position of the eye point.
        \param  a_centerX  Specifies the position of the reference point.
        \param  a_centerY  Specifies the position of the reference point.
        \param  a_centerZ  Specifies the position of the reference point.
        \param  a_upX      Specifies the direction of the up vector.
        \param  a_upY      Specifies the direction of the up vector.
        \param  a_upZ      Specifies the direction of the up vector.
    */
    //--------------------------------------------------------------------------
    inline void setLookAtMatrix(const double a_eyeX,
                                const double a_eyeY,
                                const double a_eyeZ,
                                const double a_centerX,
                                const double a_centerY,
                                const double a_centerZ,
                                const double a_upX,
                                const double a_upY,
                                const double a_upZ)
    {
        double x[3], y[3], z[3];
        double mag;

        // create rotation matrix

        // Z vector
        z[0] = a_eyeX - a_centerX;
        z[1] = a_eyeY - a_centerY;
        z[2] = a_eyeZ - a_centerZ;

        mag = sqrt(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
        if (mag > 0.0) 
        {  
          z[0] /= mag;
          z[1] /= mag;
          z[2] /= mag;
        }

        // Y vector
        y[0] = a_upX;
        y[1] = a_upY;
        y[2] = a_upZ;

        // X vector = Y cross Z
        x[0] =  y[1]*z[2] - y[2]*z[1];
        x[1] = -y[0]*z[2] + y[2]*z[0];
        x[2] =  y[0]*z[1] - y[1]*z[0];

        // recompute Y = Z cross X
        y[0] =  z[1]*x[2] - z[2]*x[1];
        y[1] = -z[0]*x[2] + z[2]*x[0];
        y[2] =  z[0]*x[1] - z[1]*x[0];

        // normalize
        mag = sqrt( x[0]*x[0] + x[1]*x[1] + x[2]*x[2] );
        if (mag) {
          x[0] /= mag;
          x[1] /= mag;
          x[2] /= mag;
        }

        mag = sqrt( y[0]*y[0] + y[1]*y[1] + y[2]*y[2] );
        if (mag) {
          y[0] /= mag;
          y[1] /= mag;
          y[2] /= mag;
        }

        m[0][0] = x[0];  m[1][0] = x[1];  m[2][0] = x[2];  m[3][0] = -x[0]*a_eyeX + -x[1]*a_eyeY + -x[2]*a_eyeZ;
        m[0][1] = y[0];  m[1][1] = y[1];  m[2][1] = y[2];  m[3][1] = -y[0]*a_eyeX + -y[1]*a_eyeY + -y[2]*a_eyeZ;
        m[0][2] = z[0];  m[1][2] = z[1];  m[2][2] = z[2];  m[3][2] = -z[0]*a_eyeX + -z[1]*a_eyeY + -z[2]*a_eyeZ;
        m[0][3] = 0.0;   m[1][3] = 0.0;   m[2][3] = 0.0;   m[3][3] = 1.0;

        m_flagTransform = true;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method build a transformation matrix.

        \details
        This method build a transformation matrix, according to the OpenGL
        gluLookAt() function.

        \param  a_eye     Specifies the position of the eye point.
        \param  a_lookAt  Specifies the position of the reference point.
        \param  a_up      Specifies the direction of the up vector.
    */
    //--------------------------------------------------------------------------
    inline void setLookAtMatrix(const cVector3d& a_eye,
                                const cVector3d& a_lookAt,
                                const cVector3d& a_up)
    {
        setLookAtMatrix(a_eye(0), a_eye(1), a_eye(2),
                        a_lookAt(0), a_lookAt(1), a_lookAt(2),
                        a_up(0), a_up(1), a_up(2));

        m_flagTransform = true;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method builds a perspective matrix.

        \details
        This method builds a perspective matrix, according to the OpenGL
        gluPerspective() function.

        \param  a_fovy    Specifies the field of view angle, in degrees, in the y direction.
        \param  a_aspect  Specifies the aspect ratio that determines the field of view in 
                          the x direction. The aspect ratio is the ratio of x (width) to y (height).
        \param  a_zNear   Specifies the distance from the viewer to the near clipping plane (always positive).
        \param  a_zFar    Specifies the distance from the viewer to the far clipping plane (always positive).
    */
    //--------------------------------------------------------------------------
    inline void setPerspectiveMatrix(const double a_fovy,
                                     const double a_aspect,
                                     const double a_zNear,
                                     const double a_zFar)
    {
        double xMin, xMax, yMin, yMax;

        yMax = a_zNear * tan(a_fovy * C_PI / 360.0);
        yMin = -yMax;

        xMin = yMin * a_aspect;
        xMax = yMax * a_aspect;

        setFrustumMatrix(xMin, xMax, yMin, yMax, a_zNear, a_zFar);

        m_flagTransform = false;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns a pointer to the matrix array in memory.

        \details
        This method returns a pointer to the matrix array in memory.

        \return  Pointer to data array.
    */
    //--------------------------------------------------------------------------
    inline double* getData()
    {
        return (&(m[0][0]));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the translational component of this matrix.

        \details
        This method returns the translational component of this matrix.

        \return Position vector.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getLocalPos() const
    {
        return cVector3d(m[3][0],m[3][1],m[3][2]);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the rotation component of this matrix.

        \details
        This method returns the rotation component of this matrix.

        \return Rotation matrix.
    */
    //--------------------------------------------------------------------------
    inline cMatrix3d getLocalRot() const
    {
        cMatrix3d mat;
        mat.set(m[0][0],m[1][0],m[2][0],
                m[0][1],m[1][1],m[2][1],
                m[0][2],m[1][2],m[2][2]);
        return (mat);
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
    inline void copyto(cTransform& a_destination) const
    {
        a_destination.m[0][0] = m[0][0];  a_destination.m[0][1] = m[0][1];
        a_destination.m[0][2] = m[0][2];  a_destination.m[0][3] = m[0][3];
        a_destination.m[1][0] = m[1][0];  a_destination.m[1][1] = m[1][1];
        a_destination.m[1][2] = m[1][2];  a_destination.m[1][3] = m[1][3];
        a_destination.m[2][0] = m[2][0];  a_destination.m[2][1] = m[2][1];
        a_destination.m[2][2] = m[2][2];  a_destination.m[2][3] = m[2][3];
        a_destination.m[3][0] = m[3][0];  a_destination.m[3][1] = m[3][1];
        a_destination.m[3][2] = m[3][2];  a_destination.m[3][3] = m[3][3];

        a_destination.m_flagTransform = m_flagTransform;
    }


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
    inline void copyfrom(const cTransform& a_source)
    {
        m[0][0] = a_source.m[0][0];  m[0][1] = a_source.m[0][1];
        m[0][2] = a_source.m[0][2];  m[0][3] = a_source.m[0][3];
        m[1][0] = a_source.m[1][0];  m[1][1] = a_source.m[1][1];
        m[1][2] = a_source.m[1][2];  m[1][3] = a_source.m[1][3];
        m[2][0] = a_source.m[2][0];  m[2][1] = a_source.m[2][1];
        m[2][2] = a_source.m[2][2];  m[2][3] = a_source.m[2][3];
        m[3][0] = a_source.m[3][0];  m[3][1] = a_source.m[3][1];
        m[3][2] = a_source.m[3][2];  m[3][3] = a_source.m[3][3];

        m_flagTransform = a_source.m_flagTransform;
    }


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
        m[0][0] = 1.0;  m[0][1] = 0.0;  m[0][2] = 0.0;  m[0][3] = 0.0;
        m[1][0] = 0.0;  m[1][1] = 1.0;  m[1][2] = 0.0;  m[1][3] = 0.0;
        m[2][0] = 0.0;  m[2][1] = 0.0;  m[2][2] = 1.0;  m[2][3] = 0.0;
        m[3][0] = 0.0;  m[3][1] = 0.0;  m[3][2] = 0.0;  m[3][3] = 1.0;

        m_flagTransform = true;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method left-multiplies this matrix with a transformation matrix 
        passed as argument.

        \details
        This method left-multiplies this matrix with a transformation matrix 
        \p a_matrix passed as argument. \n
        The result of this operation is stored in this matrix overwriting 
        previous values.

        \code
        this = this * a_matrix
        \endcode

        \param  a_matrix  Input matrix.
    */
    //--------------------------------------------------------------------------
    inline void mul(const cTransform& a_matrix)
    {
        double m00 = m[0][0] * a_matrix.m[0][0] + m[1][0] * a_matrix.m[0][1] +
                     m[2][0] * a_matrix.m[0][2] + m[3][0] * a_matrix.m[0][3];
        double m01 = m[0][0] * a_matrix.m[1][0] + m[1][0] * a_matrix.m[1][1] +
                     m[2][0] * a_matrix.m[1][2] + m[3][0] * a_matrix.m[1][3];
        double m02 = m[0][0] * a_matrix.m[2][0] + m[1][0] * a_matrix.m[2][1] +
                     m[2][0] * a_matrix.m[2][2] + m[3][0] * a_matrix.m[2][3];
        double m03 = m[0][0] * a_matrix.m[3][0] + m[1][0] * a_matrix.m[3][1] +
                     m[2][0] * a_matrix.m[3][2] + m[3][0] * a_matrix.m[3][3];

        double m10 = m[0][1] * a_matrix.m[0][0] + m[1][1] * a_matrix.m[0][1] +
                     m[2][1] * a_matrix.m[0][2] + m[3][1] * a_matrix.m[0][3];
        double m11 = m[0][1] * a_matrix.m[1][0] + m[1][1] * a_matrix.m[1][1] +
                     m[2][1] * a_matrix.m[1][2] + m[3][1] * a_matrix.m[1][3];
        double m12 = m[0][1] * a_matrix.m[2][0] + m[1][1] * a_matrix.m[2][1] +
                     m[2][1] * a_matrix.m[2][2] + m[3][1] * a_matrix.m[2][3];
        double m13 = m[0][1] * a_matrix.m[3][0] + m[1][1] * a_matrix.m[3][1] +
                     m[2][1] * a_matrix.m[3][2] + m[3][1] * a_matrix.m[3][3];

        double m20 = m[0][2] * a_matrix.m[0][0] + m[1][2] * a_matrix.m[0][1] +
                     m[2][2] * a_matrix.m[0][2] + m[3][2] * a_matrix.m[0][3];
        double m21 = m[0][2] * a_matrix.m[1][0] + m[1][2] * a_matrix.m[1][1] +
                     m[2][2] * a_matrix.m[1][2] + m[3][2] * a_matrix.m[1][3];
        double m22 = m[0][2] * a_matrix.m[2][0] + m[1][2] * a_matrix.m[2][1] +
                     m[2][2] * a_matrix.m[2][2] + m[3][2] * a_matrix.m[2][3];
        double m23 = m[0][2] * a_matrix.m[3][0] + m[1][2] * a_matrix.m[3][1] +
                     m[2][2] * a_matrix.m[3][2] + m[3][2] * a_matrix.m[3][3];

        double m30 = m[0][3] * a_matrix.m[0][0] + m[1][3] * a_matrix.m[0][1] +
                     m[2][3] * a_matrix.m[0][2] + m[3][3] * a_matrix.m[0][3];
        double m31 = m[0][3] * a_matrix.m[1][0] + m[1][3] * a_matrix.m[1][1] +
                     m[2][3] * a_matrix.m[1][2] + m[3][3] * a_matrix.m[1][3];
        double m32 = m[0][3] * a_matrix.m[2][0] + m[1][3] * a_matrix.m[2][1] +
                     m[2][3] * a_matrix.m[2][2] + m[3][3] * a_matrix.m[2][3];
        double m33 = m[0][3] * a_matrix.m[3][0] + m[1][3] * a_matrix.m[3][1] +
                     m[2][3] * a_matrix.m[3][2] + m[3][3] * a_matrix.m[3][3];

        // return values to current matrix
        m[0][0] = m00;  m[0][1] = m10;  m[0][2] = m20;  m[0][3] = m30;
        m[1][0] = m01;  m[1][1] = m11;  m[1][2] = m21;  m[1][3] = m31;
        m[2][0] = m02;  m[2][1] = m12;  m[2][2] = m22;  m[2][3] = m32;
        m[3][0] = m03;  m[3][1] = m13;  m[3][2] = m23;  m[3][3] = m33;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method left-multiplies this matrix with a transformation matrix 
        passed as argument.

        \details
        This method left-multiplies this matrix with a transformation matrix 
        \p a_matrix passed as argument. \n
        The result of this operation is stored in matrix \p a_result passed
        as second argument.

        \code
        a_result = this * a_matrix
        \endcode

        \param  a_matrix  Input matrix.
        \param  a_result  Result matrix.
    */
    //--------------------------------------------------------------------------
    inline void mulr(const cTransform& a_matrix,
                     cTransform& a_result) const
    {
        double m00 = m[0][0] * a_matrix.m[0][0] + m[1][0] * a_matrix.m[0][1] +
                     m[2][0] * a_matrix.m[0][2] + m[3][0] * a_matrix.m[0][3];
        double m01 = m[0][0] * a_matrix.m[1][0] + m[1][0] * a_matrix.m[1][1] +
                     m[2][0] * a_matrix.m[1][2] + m[3][0] * a_matrix.m[1][3];
        double m02 = m[0][0] * a_matrix.m[2][0] + m[1][0] * a_matrix.m[2][1] +
                     m[2][0] * a_matrix.m[2][2] + m[3][0] * a_matrix.m[2][3];
        double m03 = m[0][0] * a_matrix.m[3][0] + m[1][0] * a_matrix.m[3][1] +
                     m[2][0] * a_matrix.m[3][2] + m[3][0] * a_matrix.m[3][3];

        double m10 = m[0][1] * a_matrix.m[0][0] + m[1][1] * a_matrix.m[0][1] +
                     m[2][1] * a_matrix.m[0][2] + m[3][1] * a_matrix.m[0][3];
        double m11 = m[0][1] * a_matrix.m[1][0] + m[1][1] * a_matrix.m[1][1] +
                     m[2][1] * a_matrix.m[1][2] + m[3][1] * a_matrix.m[1][3];
        double m12 = m[0][1] * a_matrix.m[2][0] + m[1][1] * a_matrix.m[2][1] +
                     m[2][1] * a_matrix.m[2][2] + m[3][1] * a_matrix.m[2][3];
        double m13 = m[0][1] * a_matrix.m[3][0] + m[1][1] * a_matrix.m[3][1] +
                     m[2][1] * a_matrix.m[3][2] + m[3][1] * a_matrix.m[3][3];

        double m20 = m[0][2] * a_matrix.m[0][0] + m[1][2] * a_matrix.m[0][1] +
                     m[2][2] * a_matrix.m[0][2] + m[3][2] * a_matrix.m[0][3];
        double m21 = m[0][2] * a_matrix.m[1][0] + m[1][2] * a_matrix.m[1][1] +
                     m[2][2] * a_matrix.m[1][2] + m[3][2] * a_matrix.m[1][3];
        double m22 = m[0][2] * a_matrix.m[2][0] + m[1][2] * a_matrix.m[2][1] +
                     m[2][2] * a_matrix.m[2][2] + m[3][2] * a_matrix.m[2][3];
        double m23 = m[0][2] * a_matrix.m[3][0] + m[1][2] * a_matrix.m[3][1] +
                     m[2][2] * a_matrix.m[3][2] + m[3][2] * a_matrix.m[3][3];

        double m30 = m[0][3] * a_matrix.m[0][0] + m[1][3] * a_matrix.m[0][1] +
                     m[2][3] * a_matrix.m[0][2] + m[3][3] * a_matrix.m[0][3];
        double m31 = m[0][3] * a_matrix.m[1][0] + m[1][3] * a_matrix.m[1][1] +
                     m[2][3] * a_matrix.m[1][2] + m[3][3] * a_matrix.m[1][3];
        double m32 = m[0][3] * a_matrix.m[2][0] + m[1][3] * a_matrix.m[2][1] +
                     m[2][3] * a_matrix.m[2][2] + m[3][3] * a_matrix.m[2][3];
        double m33 = m[0][3] * a_matrix.m[3][0] + m[1][3] * a_matrix.m[3][1] +
                     m[2][3] * a_matrix.m[3][2] + m[3][3] * a_matrix.m[3][3];

        // return values to current matrix
        a_result.m[0][0] = m00;  a_result.m[0][1] = m10;  a_result.m[0][2] = m20;  a_result.m[0][3] = m30;
        a_result.m[1][0] = m01;  a_result.m[1][1] = m11;  a_result.m[1][2] = m21;  a_result.m[1][3] = m31;
        a_result.m[2][0] = m02;  a_result.m[2][1] = m12;  a_result.m[2][2] = m22;  a_result.m[2][3] = m32;
        a_result.m[3][0] = m03;  a_result.m[3][1] = m13;  a_result.m[3][2] = m23;  a_result.m[3][3] = m33;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method multiplies this  matrix by a vector passed as argument.

        \details
        This method multiplies this  matrix by a vector \p a_vector passed as 
        argument. \n
        The result of this operation is stored in vector \p a_result passed
        as second argument.

        \code
        a_result = this * a_matrix
        \endcode

        \param  a_vector  Input vector.
        \param  a_result  Matrix where the result is stored.
    */
    //--------------------------------------------------------------------------
    inline void mulr(const cVector3d& a_vector,
                     cVector3d& a_result) const
    {
        a_result(0) = m[0][0] * a_vector(0) + m[1][0] * a_vector(1) + m[2][0] * a_vector(2) + m[3][0] * 1.0;
        a_result(1) = m[0][1] * a_vector(0) + m[1][1] * a_vector(1) + m[2][1] * a_vector(2) + m[3][1] * 1.0;
        a_result(2) = m[0][2] * a_vector(0) + m[1][2] * a_vector(1) + m[2][2] * a_vector(2) + m[3][2] * 1.0;
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

        t = m[0][1]; m[0][1] = m[1][0]; m[1][0] = t;
        t = m[0][2]; m[0][2] = m[2][0]; m[2][0] = t;
        t = m[0][3]; m[0][3] = m[3][0]; m[3][0] = t;
        t = m[1][2]; m[1][2] = m[2][1]; m[2][1] = t;
        t = m[1][3]; m[1][3] = m[3][1]; m[3][1] = t;
        t = m[2][3]; m[2][3] = m[3][2]; m[3][2] = t;
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
    inline void transr(cTransform& a_result) const
    {
        a_result.m[0][0] = m[0][0];
        a_result.m[0][1] = m[1][0];
        a_result.m[0][2] = m[2][0];
        a_result.m[0][3] = m[3][0];

        a_result.m[1][0] = m[0][1];
        a_result.m[1][1] = m[1][1];
        a_result.m[1][2] = m[2][1];
        a_result.m[1][3] = m[3][1];

        a_result.m[2][0] = m[0][2];
        a_result.m[2][1] = m[1][2];
        a_result.m[2][2] = m[2][2];
        a_result.m[2][3] = m[3][2];

        a_result.m[3][0] = m[0][3];
        a_result.m[3][1] = m[1][3];
        a_result.m[3][2] = m[2][3];
        a_result.m[3][3] = m[3][3];
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
    bool inline invert()
    {
        if (m_flagTransform)
        {
            // transpose 3x3 rotation matrix R
            double t;
            t = m[0][1]; m[0][1] = m[1][0]; m[1][0] = t;
            t = m[0][2]; m[0][2] = m[2][0]; m[2][0] = t;
            t = m[1][2]; m[1][2] = m[2][1]; m[2][1] = t;

            // compute -R{t} * p
            double x = - m[0][0] * m[3][0] - m[1][0] * m[3][1] - m[2][0] * m[3][2];
            double y = - m[0][1] * m[3][0] - m[1][1] * m[3][1] - m[2][1] * m[3][2];
            double z = - m[0][2] * m[3][0] - m[1][2] * m[3][1] - m[2][2] * m[3][2];

            m[3][0] = x;
            m[3][1] = y;
            m[3][2] = z;

            return (true);
        }
        else
        {
            // Macros used during inversion
            #ifndef DOXYGEN_SHOULD_SKIP_THIS
            #define SWAP_ROWS(a, b) { double *_tmp = a; (a)=(b); (b)=_tmp; }
            #define MAT(m,r,c) (m)[(c)*4+(r)]
            #endif
            double *mat = m[0];

            double wtmp[4][8];
            double m0, m1, m2, m3, s;
            double *r0, *r1, *r2, *r3;

            r0 = wtmp[0], r1 = wtmp[1], r2 = wtmp[2], r3 = wtmp[3];

            r0[0] = MAT(mat,0,0), r0[1] = MAT(mat,0,1),
            r0[2] = MAT(mat,0,2), r0[3] = MAT(mat,0,3),
            r0[4] = 1.0, r0[5] = r0[6] = r0[7] = 0.0,

            r1[0] = MAT(mat,1,0), r1[1] = MAT(mat,1,1),
            r1[2] = MAT(mat,1,2), r1[3] = MAT(mat,1,3),
            r1[5] = 1.0, r1[4] = r1[6] = r1[7] = 0.0,

            r2[0] = MAT(mat,2,0), r2[1] = MAT(mat,2,1),
            r2[2] = MAT(mat,2,2), r2[3] = MAT(mat,2,3),
            r2[6] = 1.0, r2[4] = r2[5] = r2[7] = 0.0,

            r3[0] = MAT(mat,3,0), r3[1] = MAT(mat,3,1),
            r3[2] = MAT(mat,3,2), r3[3] = MAT(mat,3,3),
            r3[7] = 1.0, r3[4] = r3[5] = r3[6] = 0.0;

            // choose pivot
            if (fabs(r3[0])>fabs(r2[0])) SWAP_ROWS(r3, r2);
            if (fabs(r2[0])>fabs(r1[0])) SWAP_ROWS(r2, r1);
            if (fabs(r1[0])>fabs(r0[0])) SWAP_ROWS(r1, r0);
            if (0.0 == r0[0])
            {
                return (false);
            }

            // eliminate first variable
            m1 = r1[0]/r0[0]; m2 = r2[0]/r0[0]; m3 = r3[0]/r0[0];
            s = r0[1]; r1[1] -= m1 * s; r2[1] -= m2 * s; r3[1] -= m3 * s;
            s = r0[2]; r1[2] -= m1 * s; r2[2] -= m2 * s; r3[2] -= m3 * s;
            s = r0[3]; r1[3] -= m1 * s; r2[3] -= m2 * s; r3[3] -= m3 * s;
            s = r0[4];
            if (s != 0.0) { r1[4] -= m1 * s; r2[4] -= m2 * s; r3[4] -= m3 * s; }
            s = r0[5];
            if (s != 0.0) { r1[5] -= m1 * s; r2[5] -= m2 * s; r3[5] -= m3 * s; }
            s = r0[6];
            if (s != 0.0) { r1[6] -= m1 * s; r2[6] -= m2 * s; r3[6] -= m3 * s; }
            s = r0[7];
            if (s != 0.0) { r1[7] -= m1 * s; r2[7] -= m2 * s; r3[7] -= m3 * s; }

            // choose pivot
            if (fabs(r3[1])>fabs(r2[1])) SWAP_ROWS(r3, r2);
            if (fabs(r2[1])>fabs(r1[1])) SWAP_ROWS(r2, r1);
            if (0.0 == r1[1])
            {
                return (false);
            }

            // eliminate second variable
            m2 = r2[1]/r1[1]; m3 = r3[1]/r1[1];
            r2[2] -= m2 * r1[2]; r3[2] -= m3 * r1[2];
            r2[3] -= m2 * r1[3]; r3[3] -= m3 * r1[3];
            s = r1[4]; if (0.0 != s) { r2[4] -= m2 * s; r3[4] -= m3 * s; }
            s = r1[5]; if (0.0 != s) { r2[5] -= m2 * s; r3[5] -= m3 * s; }
            s = r1[6]; if (0.0 != s) { r2[6] -= m2 * s; r3[6] -= m3 * s; }
            s = r1[7]; if (0.0 != s) { r2[7] -= m2 * s; r3[7] -= m3 * s; }

            // choose pivot
            if (fabs(r3[2])>fabs(r2[2])) SWAP_ROWS(r3, r2);
            if (0.0 == r2[2])
            {
                return (false);
            }

            // eliminate third variable
            m3 = r3[2]/r2[2];
            r3[3] -= m3 * r2[3], r3[4] -= m3 * r2[4],
            r3[5] -= m3 * r2[5], r3[6] -= m3 * r2[6],
            r3[7] -= m3 * r2[7];

            // last check
            if (0.0 == r3[3])
            {
                return (false);
            }

            s = 1.0/r3[3];
            r3[4] *= s; r3[5] *= s; r3[6] *= s; r3[7] *= s;

            m2 = r2[3];
            s  = 1.0/r2[2];
            r2[4] = s * (r2[4] - r3[4] * m2), r2[5] = s * (r2[5] - r3[5] * m2),
            r2[6] = s * (r2[6] - r3[6] * m2), r2[7] = s * (r2[7] - r3[7] * m2);
            m1 = r1[3];
            r1[4] -= r3[4] * m1, r1[5] -= r3[5] * m1,
            r1[6] -= r3[6] * m1, r1[7] -= r3[7] * m1;
            m0 = r0[3];
            r0[4] -= r3[4] * m0, r0[5] -= r3[5] * m0,
            r0[6] -= r3[6] * m0, r0[7] -= r3[7] * m0;

            m1 = r1[2];
            s  = 1.0/r1[1];
            r1[4] = s * (r1[4] - r2[4] * m1), r1[5] = s * (r1[5] - r2[5] * m1),
            r1[6] = s * (r1[6] - r2[6] * m1), r1[7] = s * (r1[7] - r2[7] * m1);
            m0 = r0[2];
            r0[4] -= r2[4] * m0, r0[5] -= r2[5] * m0,
            r0[6] -= r2[6] * m0, r0[7] -= r2[7] * m0;

            m0 = r0[1];
            s  = 1.0/r0[0];
            r0[4] = s * (r0[4] - r1[4] * m0), r0[5] = s * (r0[5] - r1[5] * m0),
            r0[6] = s * (r0[6] - r1[6] * m0), r0[7] = s * (r0[7] - r1[7] * m0);

            MAT(mat,0,0) = r0[4]; MAT(mat,0,1) = r0[5],
            MAT(mat,0,2) = r0[6]; MAT(mat,0,3) = r0[7],
            MAT(mat,1,0) = r1[4]; MAT(mat,1,1) = r1[5],
            MAT(mat,1,2) = r1[6]; MAT(mat,1,3) = r1[7],
            MAT(mat,2,0) = r2[4]; MAT(mat,2,1) = r2[5],
            MAT(mat,2,2) = r2[6]; MAT(mat,2,3) = r2[7],
            MAT(mat,3,0) = r3[4]; MAT(mat,3,1) = r3[5],
            MAT(mat,3,2) = r3[6]; MAT(mat,3,3) = r3[7];

            return (true);

            // Macros used during inversion
            #undef MAT
            #undef SWAP_ROWS
        }
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
    inline std::string str(const int a_precision)
    {
        std::string result;
        result.append("[ ");
        for (int i=0; i<4; i++)
        {
            result.append("( ");
            for (int j=0; j<4; j++)
            {
                result.append(cStr(m[j][i], a_precision));
                if (j<3)
                {
                    result.append(", ");
                }
            }
            result.append(" ) ");
        }
        result.append("]");

        return (result);
    }


    //! An overloaded () operator.
    inline double& operator() (const int a_index0, const int a_index1)
    {
        return m[a_index1][a_index0];
    }


    //! An overloaded () operator.
    inline const double& operator() (const int a_index0, const int a_index1) const
    {
        return m[a_index1][a_index0];
    }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Transformation matrix data.
    double  m[4][4];

    //! If __true__ then matrix encodes a translation and rotation matrix. If __false__ matrix is represents a general 4x4.
    bool m_flagTransform;
};


//==============================================================================
// Operators on cTransform
//==============================================================================

//------------------------------------------------------------------------------
/*!
    An overloaded * operator for matrix/matrix multiplication.
*/
//------------------------------------------------------------------------------
inline cTransform operator*(const cTransform& a_matrix1, 
                            const cTransform& a_matrix2)
{
    cTransform result;
    a_matrix1.mulr(a_matrix2, result);
    return (result);
}

//------------------------------------------------------------------------------
/*!
    An overloaded * operator for matrix/vector multiplication.
*/
//------------------------------------------------------------------------------
inline cVector3d operator*(const cTransform& a_matrix, 
                           const cVector3d& a_vector)
{
    cVector3d result;
    a_matrix.mulr(a_vector, result);
    return (result);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
