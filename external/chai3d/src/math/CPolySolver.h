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
    \version   3.2.0 $Rev: 2015 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CPolySolverH
#define CPolySolverH
//------------------------------------------------------------------------------
#include "math/CMaths.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CPolySolver.h
    \ingroup    math

    \brief
    Implements polynomial solvers.
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
    This function computes the solution of a linear equation.

    \details
    This function determines the solution of a linear equation. \n
    It takes as parameters a pointer to the two coefficient of the linear equation 
    (the c[1] is the coefficient of x and so on) and a pointer to the two element 
    array in which the solution is to be placed. It outputs is a solution has been 
    found.

    \param  a_coefficient  Input coefficient values.
    \param  a_solution     Output roots found.

    \return The number of solutions found.
*/
//==============================================================================
inline int cSolveLinear(double a_coefficient[2], double a_solution[1])
{
    if (cZero(a_coefficient[1]))
    {
        return (0);
    }

    a_solution[0] = - a_coefficient[0] / a_coefficient[1];
    
    return (1);
}


//==============================================================================
/*!
    \brief
    This function computes the solution of a quadric equation.

    \details
    This function determines the roots of a quadric equation \n
    It takes as parameters a pointer to the three coefficient of the quadric equation 
    (the c[2] is the coefficient of x2 and so on) and a pointer to the two element 
    array in which the roots are to be placed. It outputs the number of roots found.

    \param  a_coefficient  Input coefficient values.
    \param  a_solution     Output roots found.

    \return The number of roots found.
*/
//==============================================================================
inline int cSolveQuadric(double a_coefficient[3], double a_solution[2])
{
    double p, q, D;

    // make sure we have a d2 equation
    if (cZero(a_coefficient[2]))
    {
        return (cSolveLinear(a_coefficient, a_solution));
    }

    // normal for: x^2 + px + q
    p = a_coefficient[1] / (2.0 * a_coefficient[2]);
    q = a_coefficient[0] / a_coefficient[2];
    D = p * p - q;

    if (cZero(D))
    {
        // one double root
        a_solution[0] = a_solution[1] = -p;
        return (1);
    }

    if (D < 0.0)
    {
        // no real root
        return (0);
    }
    else
    {
        // two real roots
        double sqrt_D = sqrt(D);
        a_solution[0] = sqrt_D - p;
        a_solution[1] = -sqrt_D - p;
        return (2);
    }
}


//==============================================================================
/*!
    \brief
    This function computes the solution of a cubic equation.

    \details
    This function determines the roots of a cubic equation. \n
    It takes as parameters a pointer to the four coefficient of the cubic equation 
    (the c[3] is the coefficient of x3 and so on) and a pointer to the three element 
    array in which the roots are to be placed. It outputs the number of roots found.

    \param  a_coefficient  Input coefficient values.
    \param  a_solution     Output roots found.

    \return The number of roots found.
*/
//==============================================================================
inline int cSolveCubic(double a_coefficient[4], double a_solution[3])
{
    int i, num;
    double sub,
           A, B, C,
           sq_A, p, q,
           cb_p, D;

    // normalize the equation:x ^ 3 + Ax ^ 2 + Bx  + C = 0
    A = a_coefficient[2] / a_coefficient[3];
    B = a_coefficient[1] / a_coefficient[3];
    C = a_coefficient[0] / a_coefficient[3];

    // substitute x = y - A / 3 to eliminate the quadric term: x^3 + px + q = 0
    sq_A = A * A;
    p = 1.0/3.0 * (-1.0/3.0 * sq_A + B);
    q = 1.0/2.0 * (2.0/27.0 * A *sq_A - 1.0/3.0 * A * B + C);

    // use Cardano's formula
    cb_p = p * p * p;
    D = q * q + cb_p;

    if (cZero(D))
    {
        if (cZero(q))
        {
            // one triple solution
            a_solution[0] = 0.0;
            num = 1;
        }
        else
        {
            // one single and one double solution
            double u = cCbrt(-q);
            a_solution[0] = 2.0 * u;
            a_solution[1] = - u;
            num = 2;
        }
    }
    else
    {
        if (D < 0.0)
        {
            // casus irreductibilis: three real solutions
            double phi = 1.0/3.0 * acos(-q / sqrt(-cb_p));
            double t = 2.0 * sqrt(-p);
            a_solution[0] = t * cos(phi);
            a_solution[1] = -t * cos(phi + M_PI / 3.0);
            a_solution[2] = -t * cos(phi - M_PI / 3.0);
            num = 3;
        }
        else
        {
            // one real solution
            double sqrt_D = sqrt(D);
            double u = cCbrt(sqrt_D + fabs(q));
            if (q > 0.0)
            {
                a_solution[0] = - u + p / u ;
            }
            else
            {
                a_solution[0] = u - p / u;
            }
            num = 1;
        }
    }

    // resubstitute
    sub = 1.0 / 3.0 * A;
    for (i = 0; i < num; i++)
    {
        a_solution[i] -= sub;
    }

    return (num);
}


//==============================================================================
/*!
    \brief
    This function computes the solution of a quartic equation.

    \details
    This function determines the roots of a quartic equation. \n
    It takes as parameters a pointer to the five coefficient of the quartic equation 
    (the c[4] is the coefficient of x^4 and so on) and a pointer to the four element 
    array in which the roots are to be placed. It outputs the number of roots found.

    \param  a_coefficient  Input coefficient values.
    \param  a_solution     Output roots found.

    \return The number of roots found.
*/
//==============================================================================
inline int cSolveQuartic(double a_coefficient[5], double a_solution[4])
{
    double  coeffs[4],
            z, u, v, sub,
            A, B, C, D,
            sq_A, p, q, r;
    int i, num;

    // normalize the equation:x ^ 4 + Ax ^ 3 + Bx ^ 2 + Cx + D = 0
    A = a_coefficient[3] / a_coefficient[4];
    B = a_coefficient[2] / a_coefficient[4];
    C = a_coefficient[1] / a_coefficient[4];
    D = a_coefficient[0] / a_coefficient[4];

    // subsitute x = y - A / 4 to eliminate the cubic term: x^4 + px^2 + qx + r = 0
    sq_A = A * A;
    p = -3.0 / 8.0 * sq_A + B;
    q = 1.0 / 8.0 * sq_A * A - 1.0 / 2.0 * A * B + C;
    r = -3.0 / 256.0 * sq_A * sq_A + 1.0 / 16.0 * sq_A * B - 1.0 / 4.0 * A * C + D;

    if (cZero(r))
    {
        // no absolute term:y(y ^ 3 + py + q) = 0
        coeffs[0] = q;
        coeffs[1] = p;
        coeffs[2] = 0.0;
        coeffs[3] = 1.0;

        num = cSolveCubic(coeffs, a_solution);
        a_solution[num++] = 0;
    }
    else
    {
        // solve the resolvent cubic...
        coeffs[0] = 1.0 / 2.0 * r * p - 1.0 / 8.0 * q * q;
        coeffs[1] = -r;
        coeffs[2] = -1.0 / 2.0 * p;
        coeffs[3] = 1.0;
        (void) cSolveCubic(coeffs, a_solution);

        // ...and take the one real solution...
        z = a_solution[0];

        // ...to build two quadratic equations
        u = z * z - r;
        v = 2.0 * z - p;

        if (cZero(u))
        {
            u = 0.0;
        }
        else if (u > 0.0)
        {
            u = sqrt(u);
        }
        else
        {
            return (0);
        }
        
        if (cZero(v))
        {
            v = 0;
        }
        else if (v > 0.0)
        {
            v = sqrt(v);
        }
        else
        {
            return (0);
        }

        coeffs[0] = z - u;
        coeffs[1] = q < 0 ? -v : v;
        coeffs[2] = 1.0;

        num = cSolveQuadric(coeffs, a_solution);

        coeffs[0] = z + u;
        coeffs[1] = q < 0 ? v : -v;
        coeffs[2] = 1.0;

        num += cSolveQuadric(coeffs, a_solution + num);
    }

    // resubstitute
    sub = 1.0 / 4 * A;
    for (i = 0; i < num; i++)
    {
        a_solution[i] -= sub;
    }

    return (num);
}

//@}

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif  // CPolySolverH
//------------------------------------------------------------------------------


