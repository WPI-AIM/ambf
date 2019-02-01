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
#ifndef CShapeTorusH
#define CShapeTorusH
//------------------------------------------------------------------------------
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
#include "world/CGenericObject.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CShapeTorus.h

    \brief 
    Implementation of a 3D shape torus object.
*/
//==============================================================================

//==============================================================================
/*!    
    \class      cShapeTorus
    \ingroup    world

    \brief
    This class implements a 3D shape torus.

    \details
    This class implements a 3D shape torus.
*/
//==============================================================================
class cShapeTorus : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cShapeTorus.
    cShapeTorus(const double& a_innerRadius,
                const double& a_outerRadius,
                cMaterialPtr a_material = cMaterialPtr());

    //! Destructor of cShapeTorus.
    virtual ~cShapeTorus() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    cShapeTorus* copy(const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false, 
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = false);

    //! This method sets the inner and outer radii of the torus.
    void setSize(const double& a_innerRadius, 
        const double& a_outerRadius);

    //! This method returns the inner radius of the torus.
    inline double getInnerRadius() const { return (m_innerRadius); }

    //! This method returns the outer radius of the torus.
    inline double getOuterRadius() const { return (m_outerRadius); }


    //--------------------------------------------------------------------------
    // PROTECTED VIRTUAL METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method renders this object graphically using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method updates the boundary box of this object.
    virtual void updateBoundaryBox();

    //! This method scales the size of this object with given scale factor.
    virtual void scaleObject(const double& a_scaleFactor);

    //! This method updates the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int a_IDN);

    //! This method computes collisions between a segment and this object.
    virtual bool computeOtherCollisionDetection(cVector3d& a_segmentPointA,
        cVector3d& a_segmentPointB,
        cCollisionRecorder& a_recorder,
        cCollisionSettings& a_settings);


    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:

    //! This method copies all properties of this object to another.
    void copyShapeTorusProperties(cShapeTorus* a_obj,
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Inside radius of the torus.
    double m_innerRadius;

    //! Outside radius of the torus.
    double m_outerRadius;

    //! Resolution of the graphical model.
    unsigned int m_resolution;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
