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
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "graphics/CDraw3D.h"
#include "materials/CMaterial.h"
//------------------------------------------------------------------------------
#ifdef C_USE_OPENGL
#ifdef MACOSX
#include "OpenGL/glu.h"
#else
#include "GL/glu.h"
#endif
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This function aligns the current z-axis with a reference frame in the same
    way than the OpenGL gluLookAt() function does.

    \param  a_eye  Position of eye.
    \param  a_at   Look at position.
    \param  a_up   Up direction.
*/
//==============================================================================
void cLookAt(const cVector3d& a_eye, const cVector3d& a_at, const cVector3d& a_up)
{
#ifdef C_USE_OPENGL

    // Define our look vector (z axis)
    cVector3d look = a_at - a_eye;
    look.normalize();

    // Define our new x axis
    cVector3d xaxis;
    xaxis = cCross(look,a_up);
    xaxis.normalize();

    // Define our new y axis as the cross of the x and z axes
    cVector3d upv = cCross(xaxis,look);

    // Turn around the z axis
    look.mul(-1.0);
  
    // Put it all into a GL-friendly matrix
    double matrix[16];
    matrix[0] = xaxis(0);
    matrix[1] = xaxis(1);
    matrix[2] = xaxis(2);
    matrix[3] = 0.0;
    matrix[4] = upv(0);
    matrix[5] = upv(1);
    matrix[6] = upv(2);
    matrix[7] = 0.0;
    matrix[8] = look(0);
    matrix[9] = look(1);
    matrix[10] = look(2);
    matrix[11] = 0.0;
    matrix[12] = a_eye(0);
    matrix[13] = a_eye(1);
    matrix[14] = a_eye(2);
    matrix[15] = 1.0;

    // Push it onto the matrix stack
    glMultMatrixd(matrix);

#endif
}


//==============================================================================
/*!
    This function draws an X-Y-Z Frame. The red arrow corresponds to the X-Axis,
    green to the Y-Axis, and blue to the Z-Axis. \n

    The scale parameter determines the size of the arrows.

    \param  a_scale  Length of each arrow
*/
//==============================================================================
void cDrawFrame(const double& a_scale)
{
    cDrawFrame(a_scale, a_scale);
}


//==============================================================================
/*!
    This function draws an X-Y-Z Frame. The red arrow corresponds to the X-Axis,
    green to the Y-Axis, and blue to the Z-Axis. \n

    The scale parameter determines the size of the arrows.

    \param  a_axisLengthScale     Length of each arrow
    \param  a_axisThicknessScale  Thickness factor of frame.
*/
//==============================================================================
void cDrawFrame(const double& a_axisLengthScale, 
                const double& a_axisThicknessScale)
{
#ifdef C_USE_OPENGL

    // Triangle vertices:
    static int nTriangles = 8;

    static float triangle_vertices[72] = 
    {
        0.000000f, 0.040000f,-0.800000f, 0.028284f, 0.028284f,-0.800000f,
        0.000000f, 0.000000f,-1.000000f, 0.028284f, 0.028284f,-0.800000f,
        0.040000f, 0.000000f,-0.800000f, 0.000000f, 0.000000f,-1.000000f,
        0.040000f, 0.000000f,-0.800000f, 0.028284f,-0.028284f,-0.800000f,
        0.000000f, 0.000000f,-1.000000f, 0.028284f,-0.028284f,-0.800000f,
        0.000000f,-0.040000f,-0.800000f, 0.000000f, 0.000000f,-1.000000f,
        0.000000f,-0.040000f,-0.800000f,-0.028284f,-0.028284f,-0.800000f,
        0.000000f, 0.000000f,-1.000000f,-0.028284f,-0.028284f,-0.800000f,
       -0.040000f, 0.000000f,-0.800000f, 0.000000f, 0.000000f,-1.000000f,
       -0.040000f, 0.000000f,-0.800000f,-0.028284f, 0.028284f,-0.800000f,
        0.000000f, 0.000000f,-1.000000f,-0.028284f, 0.028284f,-0.800000f,
        0.000000f, 0.040000f,-0.800000f, 0.000000f, 0.000000f,-1.000000f
    };

    // Triangle normals:
    static float triangle_normals[72] = {
         0.000000f, 0.980581f,-0.196116f, 0.693375f, 0.693375f,-0.196116f,
         0.357407f, 0.862856f,-0.357407f, 0.693375f, 0.693375f,-0.196116f,
         0.980581f, 0.000000f,-0.196116f, 0.862856f, 0.357407f,-0.357407f,
         0.980581f, 0.000000f,-0.196116f, 0.693375f,-0.693375f,-0.196116f,
         0.862856f,-0.357407f,-0.357407f, 0.693375f,-0.693375f,-0.196116f,
         0.000000f,-0.980581f,-0.196116f, 0.357407f,-0.862856f,-0.357407f,
         0.000000f,-0.980581f,-0.196116f,-0.693375f,-0.693375f,-0.196116f,
        -0.357407f,-0.862856f,-0.357407f,-0.693375f,-0.693375f,-0.196116f,
        -0.980581f, 0.000000f,-0.196116f,-0.862856f,-0.357407f,-0.357407f,
        -0.980581f, 0.000000f,-0.196116f,-0.693375f, 0.693375f,-0.196116f,
        -0.862856f, 0.357407f,-0.357407f,-0.693375f, 0.693375f,-0.196116f,
         0.000000f, 0.980581f,-0.196116f,-0.357407f, 0.862856f,-0.357407f
    };

    // Quad vertices:
    static int nQuads = 16;

    static float quad_vertices[192] = 
    {
        0.000000f, 0.010000f, 0.000000f, 0.007000f, 0.007000f, 0.000000f,
        0.007000f, 0.007000f,-0.800000f, 0.000000f, 0.010000f,-0.800000f,
        0.000000f,-0.010000f, 0.000000f,-0.007000f,-0.007000f, 0.000000f,
       -0.007000f,-0.007000f,-0.800000f, 0.000000f,-0.010000f,-0.800000f,
       -0.007000f,-0.007000f, 0.000000f,-0.010000f, 0.000000f, 0.000000f,
       -0.010000f, 0.000000f,-0.800000f,-0.007000f,-0.007000f,-0.800000f,
       -0.010000f, 0.000000f, 0.000000f,-0.007000f, 0.007000f, 0.000000f,
       -0.007000f, 0.007000f,-0.800000f,-0.010000f, 0.000000f,-0.800000f,
       -0.007000f, 0.007000f, 0.000000f, 0.000000f, 0.010000f, 0.000000f,
        0.000000f, 0.010000f,-0.800000f,-0.007000f, 0.007000f,-0.800000f,
        0.007000f, 0.007000f, 0.000000f, 0.010000f, 0.000000f, 0.000000f,
        0.010000f, 0.000000f,-0.800000f, 0.007000f, 0.007000f,-0.800000f,
        0.010000f, 0.000000f, 0.000000f, 0.007000f,-0.007000f, 0.000000f,
        0.007000f,-0.007000f,-0.800000f, 0.010000f, 0.000000f,-0.800000f,
        0.007000f,-0.007000f, 0.000000f, 0.000000f,-0.010000f, 0.000000f,
        0.000000f,-0.010000f,-0.800000f, 0.007000f,-0.007000f,-0.800000f,
       -0.007000f, 0.007000f,-0.800000f,-0.028284f, 0.028284f,-0.800000f,
       -0.040000f, 0.000000f,-0.800000f,-0.010000f, 0.000000f,-0.800000f,
       -0.010000f, 0.000000f,-0.800000f,-0.040000f, 0.000000f,-0.800000f,
       -0.028284f,-0.028284f,-0.800000f,-0.007000f,-0.007000f,-0.800000f,
       -0.007000f,-0.007000f,-0.800000f,-0.028284f,-0.028284f,-0.800000f,
        0.000000f,-0.040000f,-0.800000f, 0.000000f,-0.010000f,-0.800000f,
        0.000000f,-0.010000f,-0.800000f, 0.000000f,-0.040000f,-0.800000f,
        0.028284f,-0.028284f,-0.800000f, 0.007000f,-0.007000f,-0.800000f,
        0.028284f,-0.028284f,-0.800000f, 0.040000f, 0.000000f,-0.800000f,
        0.010000f, 0.000000f,-0.800000f, 0.007000f,-0.007000f,-0.800000f,
        0.040000f, 0.000000f,-0.800000f, 0.028284f, 0.028284f,-0.800000f,
        0.007000f, 0.007000f,-0.800000f, 0.010000f, 0.000000f,-0.800000f,
        0.007000f, 0.007000f,-0.800000f, 0.028284f, 0.028284f,-0.800000f,
        0.000000f, 0.040000f,-0.800000f, 0.000000f, 0.010000f,-0.800000f,
        0.000000f, 0.010000f,-0.800000f, 0.000000f, 0.040000f,-0.800000f,
       -0.028284f, 0.028284f,-0.800000f,-0.007000f, 0.007000f,-0.800000f
    };

    // Quad normals:
    static float quad_normals[192] = 
    {
         0.000000f, 1.000000f, 0.000000f, 0.707107f, 0.707107f, 0.000000f,
         0.707107f, 0.707107f, 0.000000f, 0.000000f, 1.000000f, 0.000000f,
         0.000000f,-1.000000f, 0.000000f,-0.707107f,-0.707107f, 0.000000f,
        -0.707107f,-0.707107f, 0.000000f, 0.000000f,-1.000000f, 0.000000f,
        -0.707107f,-0.707107f, 0.000000f,-1.000000f, 0.000000f, 0.000000f,
        -1.000000f, 0.000000f, 0.000000f,-0.707107f,-0.707107f, 0.000000f,
        -1.000000f, 0.000000f, 0.000000f,-0.707107f, 0.707107f, 0.000000f,
        -0.707107f, 0.707107f, 0.000000f,-1.000000f, 0.000000f, 0.000000f,
        -0.707107f, 0.707107f, 0.000000f, 0.000000f, 1.000000f, 0.000000f,
         0.000000f, 1.000000f, 0.000000f,-0.707107f, 0.707107f, 0.000000f,
         0.707107f, 0.707107f, 0.000000f, 1.000000f, 0.000000f, 0.000000f,
         1.000000f, 0.000000f, 0.000000f, 0.707107f, 0.707107f, 0.000000f,
         1.000000f, 0.000000f, 0.000000f, 0.707107f,-0.707107f, 0.000000f,
         0.707107f,-0.707107f, 0.000000f, 1.000000f, 0.000000f, 0.000000f,
         0.707107f,-0.707107f, 0.000000f, 0.000000f,-1.000000f, 0.000000f,
         0.000000f,-1.000000f, 0.000000f, 0.707107f,-0.707107f, 0.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f,
         0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 1.000000f
    };

    // set material properties
    float fnull[4] = {0,0,0,0};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, (const float *)&fnull);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, (const float *)&fnull);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 0);
    glEnable(GL_COLOR_MATERIAL);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // enable vertex and normal arrays
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);

    for(int k=0; k<3; k++)
    {
        glPushMatrix();

        // Rotate to the appropriate axis
        if (k==0) {
          glRotatef(-90.0,0,1,0);
          glColor3f(1.0f,0.0f,0.0f);
        }
        else if (k==1) {
          glRotatef(90.0,1,0,0);
          glColor3f(0.0f,1.0f,0.0f);
        }
        else {
          glRotatef(180.0,1,0,0);
          glColor3f(0.0f,0.0f,1.0f);
        }

        // scaling
        glScaled(a_axisThicknessScale, a_axisThicknessScale, a_axisLengthScale);

        // render frame object
        glVertexPointer(3, GL_FLOAT, 0, triangle_vertices);
        glNormalPointer(GL_FLOAT, 0, triangle_normals);
        glDrawArrays(GL_TRIANGLES, 0, nTriangles*3);

        glVertexPointer(3, GL_FLOAT, 0, quad_vertices);
        glNormalPointer(GL_FLOAT, 0, quad_normals);
        glDrawArrays(GL_QUADS, 0, nQuads*4);

        glPopMatrix();
    }

    // disable vertex and normal arrays
    glDisable(GL_COLOR_MATERIAL);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

#endif
}


//==============================================================================
/*!
    This function draws a line-based box with sides parallel to the x-y-z axes.

    \param  a_xMin  Box side x min position.
    \param  a_xMax  Box side x max position.
    \param  a_yMin  Box side y min position.
    \param  a_yMax  Box side y max position.
    \param  a_zMin  Box side z min position.
    \param  a_zMax  Box side z max position.
*/
//==============================================================================
void cDrawWireBox(const double& a_xMin, const double& a_xMax,
                  const double& a_yMin, const double& a_yMax,
                  const double& a_zMin, const double& a_zMax)
{
#ifdef C_USE_OPENGL

    // render lines for each edge of the box
    glBegin(GL_LINES);
        glVertex3d(a_xMin, a_yMin, a_zMin);
        glVertex3d(a_xMax, a_yMin, a_zMin);
        glVertex3d(a_xMin, a_yMax, a_zMin);
        glVertex3d(a_xMax, a_yMax, a_zMin);
        glVertex3d(a_xMin, a_yMin, a_zMax);
        glVertex3d(a_xMax, a_yMin, a_zMax);
        glVertex3d(a_xMin, a_yMax, a_zMax);
        glVertex3d(a_xMax, a_yMax, a_zMax);

        glVertex3d(a_xMin, a_yMin, a_zMin);
        glVertex3d(a_xMin, a_yMax, a_zMin);
        glVertex3d(a_xMax, a_yMin, a_zMin);
        glVertex3d(a_xMax, a_yMax, a_zMin);
        glVertex3d(a_xMin, a_yMin, a_zMax);
        glVertex3d(a_xMin, a_yMax, a_zMax);
        glVertex3d(a_xMax, a_yMin, a_zMax);
        glVertex3d(a_xMax, a_yMax, a_zMax);

        glVertex3d(a_xMin, a_yMin, a_zMin);
        glVertex3d(a_xMin, a_yMin, a_zMax);
        glVertex3d(a_xMax, a_yMin, a_zMin);
        glVertex3d(a_xMax, a_yMin, a_zMax);
        glVertex3d(a_xMin, a_yMax, a_zMin);
        glVertex3d(a_xMin, a_yMax, a_zMax);
        glVertex3d(a_xMax, a_yMax, a_zMin);
        glVertex3d(a_xMax, a_yMax, a_zMax);
    glEnd();

#endif
}


//==============================================================================
/*!
    This function draws a sphere.

    \param  a_radius     Radius of the sphere
    \param  a_numSlices  Specifies the number of subdivisions around the
                         z axis (similar to lines of longitude)
    \param  a_numStacks  Specifies the number of subdivisions along the
                         x/y axes (similar to lines of latitude)
*/
//==============================================================================
void cDrawSphere(const double& a_radius,
                 const unsigned int a_numSlices, 
                 const unsigned int a_numStacks)
{
#ifdef C_USE_OPENGL

    // allocate a new OpenGL quadric object for rendering a sphere
    GLUquadricObj *quadObj;
    quadObj = gluNewQuadric ();

    // set rendering style
    gluQuadricDrawStyle (quadObj, GLU_FILL);

    // set normal-rendering mode
    gluQuadricNormals (quadObj, GLU_SMOOTH);

    // render a sphere
    gluSphere(quadObj, a_radius, a_numSlices, a_numStacks);

    // delete our quadric object
    gluDeleteQuadric(quadObj);

#endif
}


//==============================================================================
/*!
    This function draws an arrow along the z-axis using a cone and a cylinder.

    \param  a_arrowStart  The location of the back of the arrow.
    \param  a_arrowTip    The location of the tip of the arrow.
    \param  a_width       The width of the arrow shaft.
*/
//==============================================================================
void cDrawArrow(const cVector3d& a_arrowStart, 
                const cVector3d& a_arrowTip, 
                const double a_width)
{
#ifdef C_USE_OPENGL

    glPushMatrix();

    // we don't really care about the up vector, but it can't
    // be parallel to the arrow...
    cVector3d up = cVector3d(0,1,0);
    cVector3d arrow = a_arrowTip-a_arrowStart;
    arrow.normalize();
    double d = fabs(cDot(up,arrow));
    if (d > .9)
    {
        up = cVector3d(1,0,0);
    }

    cLookAt(a_arrowStart, a_arrowTip, up);
    double distance = cDistance(a_arrowTip,a_arrowStart);

    // This flips the z axis around
    glRotatef(180,1,0,0);

    // create a new OpenGL quadratic object 
    GLUquadricObj *quadObj;
    quadObj = gluNewQuadric();

    #define ARROW_CYLINDER_PORTION 0.75
    #define ARRROW_CONE_PORTION (1.0 - 0.75)

    // set rendering style
    gluQuadricDrawStyle(quadObj, GLU_FILL);

    // set normal-rendering mode
    gluQuadricNormals(quadObj, GLU_SMOOTH);

    // render a cylinder and a cone
    glRotatef(180,1,0,0);
    gluDisk(quadObj,0,a_width,10,10);
    glRotatef(180,1,0,0);

    gluCylinder(quadObj, a_width, a_width, distance*ARROW_CYLINDER_PORTION, 10, 10);
    glTranslated(0, 0, ARROW_CYLINDER_PORTION*distance);

    glRotatef(180, 1, 0, 0);
    gluDisk(quadObj, 0, a_width*2.0, 10, 10);
    glRotatef(180,1,0,0);

    gluCylinder(quadObj, a_width*2.0, 0.0, distance*ARRROW_CONE_PORTION, 10, 10);

    // delete our quadric object
    gluDeleteQuadric(quadObj);

    glPopMatrix();

#endif
}


//==============================================================================
/*!
    This function draws a torus.

    \param  a_innerRadius  Inner radius of the torus.
    \param  a_outerRadius  Outer radius of the torus.
    \param  a_sides        Number of sides for each radial section.
    \param  a_rings        Number of radial divisions for the torus.
*/
//==============================================================================
void cDrawSolidTorus(const double& a_innerRadius, 
                     const double& a_outerRadius, 
                     const int a_sides, 
                     const int a_rings)
{
#ifdef C_USE_OPENGL

    double  iradius = a_innerRadius, oradius = a_outerRadius, phi, psi, dpsi, dphi;
    double *vertex, *normal;
    int    i, j;
    double spsi, cpsi, sphi, cphi ;

    int nSides = a_sides;
    int nRings = a_rings;

    if ( nSides < 1 ) nSides = 1;
    if ( nRings < 1 ) nRings = 1;

    // increment the number of sides and rings to allow for one more point than surface
    nSides ++ ;
    nRings ++ ;

    // allocate the vertices array
    vertex = (double *)calloc( sizeof(double), 3 * nSides * nRings );
    normal = (double *)calloc( sizeof(double), 3 * nSides * nRings );

    glPushMatrix();

    dpsi =  2.0 * C_PI / (double)(nRings - 1) ;
    dphi = -2.0 * C_PI / (double)(nSides - 1) ;
    psi  = 0.0;

    for( j=0; j<nRings; j++ )
    {
        cpsi = cos ( psi ) ;
        spsi = sin ( psi ) ;
        phi = 0.0;

        for( i=0; i<nSides; i++ )
        {
            int offset = 3 * ( j * nSides + i ) ;
            cphi = cos ( phi ) ;
            sphi = sin ( phi ) ;
            *(vertex + offset + 0) = cpsi * ( oradius + cphi * iradius ) ;
            *(vertex + offset + 1) = spsi * ( oradius + cphi * iradius ) ;
            *(vertex + offset + 2) =                    sphi * iradius  ;
            *(normal + offset + 0) = cpsi * cphi ;
            *(normal + offset + 1) = spsi * cphi ;
            *(normal + offset + 2) =        sphi ;
            phi += dphi;
        }

        psi += dpsi;
    }

    glBegin( GL_QUADS );
    for( i=0; i<nSides-1; i++ )
    {
        for( j=0; j<nRings-1; j++ )
        {
            int offset = 3 * ( j * nSides + i ) ;
            glNormal3dv( normal + offset );
            glVertex3dv( vertex + offset );
            glNormal3dv( normal + offset + 3 );
            glVertex3dv( vertex + offset + 3 );
            glNormal3dv( normal + offset + 3 * nSides + 3 );
            glVertex3dv( vertex + offset + 3 * nSides + 3 );
            glNormal3dv( normal + offset + 3 * nSides );
            glVertex3dv( vertex + offset + 3 * nSides );
        }
    }

    glEnd();

    free ( vertex ) ;
    free ( normal ) ;
    glPopMatrix();

#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
