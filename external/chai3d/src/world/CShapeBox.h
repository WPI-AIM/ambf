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
#ifndef CShapeBoxH
#define CShapeBoxH
//------------------------------------------------------------------------------
#include "world/CGenericObject.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CShapeBox.h

    \brief 
    Implementation of a 3D shape box object.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cShapeBox
    \ingroup    world

    \brief
    This class implements a 3D shape box.

    \details
    This class implements a 3D shape box.
*/
//==============================================================================
class cShapeBox : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cShapeBox.
    cShapeBox(const double& a_sizeX, 
              const double& a_sizeY, 
              const double& a_sizeZ,
              cMaterialPtr a_material = cMaterialPtr());
    
    //! Destructor of cShapeBox.
    virtual ~cShapeBox() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    cShapeBox* copy(const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false, 
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = true);

    //! This method sets the dimensions of the box along each axis.
    void setSize(const double& a_sizeX, 
        const double& a_sizeY, 
        const double& a_sizeZ);

    //! This method returns the box length along axis X.
    inline double getSizeX() const { return (2.0 * m_hSizeX); }

    //! This method returns the box length along axis Y.
    inline double getSizeY() const { return (2.0 * m_hSizeY); }

    //! This method returns the box length along axis Z.
    inline double getSizeZ() const { return (2.0 * m_hSizeZ); }


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
    void copyShapeBoxProperties(cShapeBox* a_obj,
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS: (cShapeBox)
    //--------------------------------------------------------------------------

  protected:

    //! half length of box along axis Z.
    double m_hSizeX;

    //! half length of box along axis Y.
    double m_hSizeY;

    //! half length of box along axis Z.
    double m_hSizeZ;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
