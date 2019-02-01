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
    \author    Sonny Chan
    \author    Francois Conti
    \version   3.2.0 $Rev: 2171 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CVoxelObjectH
#define CVoxelObjectH
//------------------------------------------------------------------------------
#include "world/CMesh.h"
#include "world/CMultiMesh.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------
const int C_NUM_VOXEL_RENDERING_MODES = 9;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CVoxelObject.h

    \brief 
    Implementation of a 3D volumetric object.
*/
//==============================================================================


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//! Describes a relative voxel coordinate.
struct cVoxelCoord
{
    int m_x;
    int m_y;
    int m_z;
};

//! Describes a list of voxel coordinates.
struct cVoxelCoordList
{
    int m_radiusX;
    int m_radiusY;
    int m_radiusZ;
    std::vector<cVoxelCoord> m_coords;
};

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//==============================================================================
/*!
    \class      cVoxelObject
    \ingroup    world

    \brief
    This class implements a 3D volumetric object composed of voxels.

    \details
    This class implements a 3D volumetric object composed of voxels.
*/
//==============================================================================
class cVoxelObject : public cMesh
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cVoxelObject.
    cVoxelObject();

    //! Destructor of cVoxelObject.
    virtual ~cVoxelObject();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - RENDERING MODES:
    //--------------------------------------------------------------------------

public:

    //! This method sets the basic rendering mode without shader.
    void setRenderingModeBasic();

    //! This method sets a basic voxel renderer with shader.
    void setRenderingModeVoxelColors();

    //! This method sets a basic voxel renderer with shader and colormap.
    void setRenderingModeVoxelColorMap();

    //! This method sets an isosurface renderer using shader and material color.
    void setRenderingModeIsosurfaceMaterial();

    //! This method sets an isosurface renderer using shader and individual voxel colors.
    void setRenderingModeIsosurfaceColors();

    //! This method sets an isosurface renderer using shader and colormap.
    void setRenderingModeIsosurfaceColorMap();

    //! This method sets a direct volume renderer.
    void setRenderingModeDVRColorMap();

    //! This method sets a custom shader renderer.
    void setRenderingModeCustom();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - SETTINGS:
    //--------------------------------------------------------------------------

public:

    //! This method sets the quality level of the graphic rendering. This number ranges from 0.1 (poor) to 1.0 (high).
    void setQuality(const double a_quality) { m_quality = cClamp(a_quality, 0.1, 10.0); }

    //! This method returns the quality level of the graphic rendering.
    double getQuality() { return (m_quality); }

    //! This method sets the isosurface value. Used when isometric rendering is enabled.
    void setIsosurfaceValue(const float a_isosurfaceValue) { m_isosurfaceValue = cClamp(a_isosurfaceValue, 0.0f, 1.0f); }

    //! This method returns the isosurface value. This setting is used when isometric rendering is enabled.
    float getIsosurfaceValue() { return (m_isosurfaceValue); }

    //! This method sets the voxel opacity value. This setting is used with basic voxel rendering mode.
    void setVoxelOpacity(const float a_voxelOpacity) { m_voxelOpacity = cClamp(a_voxelOpacity, 0.0f, 1.0f); }

    //! This method returns the voxel opacity value.
    float getVoxelOpacity() { return(m_voxelOpacity); }

    //! This method sets the opacity threshold. This value is the opacity accumulated along the ray before the shader decides to stop casting.
    void setOpacityThreshold(const float a_opacityThreshold) { m_opacityThreshold = cClamp(a_opacityThreshold, 0.0f, 1.0f); }

    //! This method returns the opacity threshold.
    float getOpacityThreshold() { return(m_opacityThreshold); }

    //! This method sets the optical density factor. This setting is used when the direct volume rendering mode is enabled.
    void setOpticalDensity(const float a_opticalDensity) { m_opticalDensity = fabs(a_opticalDensity); }

    //! This method returns the optical density factor.
    float getOpticalDensity() { return(m_opacityThreshold); }

    //! This method enables or disables linear interpolation.
    void setUseLinearInterpolation(const bool a_useLinearInterpolation) { m_useLinearInterpolation = a_useLinearInterpolation; }

    //! This method returns __true__ if linear interpolation is enabled, __false__ otherwise.
    bool getUseLinearInterpolation() { return (m_useLinearInterpolation); }

    //! This method enables or disables the use of the color look-up table.
    void setUseColorMap(const bool a_useColorMap) { m_useColorMap = a_useColorMap; }

    //! This method returns __true__ if the color look-up table is enabled, __false__ otherwise.
    bool getUseColorMap() const { return m_useColorMap; }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - POLYGONIZATION:
    //--------------------------------------------------------------------------

public:

    //! This method converts this voxel object into a triangle mesh.
    bool polygonize(cMesh* a_mesh, double a_gridSizeX = -1.0, double a_gridSizeY = -1.0, double a_gridSizeZ = -1.0);

    //! This method converts this voxel object into a triangle multi-mesh.
    bool polygonize(cMultiMesh* a_multiMesh, double a_gridSizeX = -1.0, double a_gridSizeY = -1.0, double a_gridSizeZ = -1.0);


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Corner with minimum position coordinate.
    cVector3d m_minCorner;

    //! Corner with maximum position coordinate.
    cVector3d m_maxCorner;

    //! Texture coordinate at point v000 (minimum corner).
    cVector3d m_minTextureCoord;

    //! Texture coordinate at point v111 (maximum corner).
    cVector3d m_maxTextureCoord;

    //! Color Map. (Lookup table)
    cTexture1dPtr m_colorMap;


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method loads all rendering shaders.
    void loadRenderingShaders();

    //! This method updates the mesh model.
    void update(cRenderOptions& a_options);

    //! This method renders the object graphically using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method computes any collision between a segment and this object.
    virtual bool computeOtherCollisionDetection(cVector3d& a_segmentPointA,
        cVector3d& a_segmentPointB,
        cCollisionRecorder& a_recorder,
        cCollisionSettings& a_settings);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! List of edges.
    cVector3d m_edges[12][2];

    //! Quality of the displayed graphical model.
    double m_quality;

    //! Isosurface value.
    float m_isosurfaceValue;

    //! Voxel opacity value.
    float m_voxelOpacity;

    //! Opacity threshold. This value is the opacity accumulated along the ray before the shader decides to stop casting.
    float  m_opacityThreshold;

    //! Optical density factor.
    float m_opticalDensity;

    //! Rendering mode.
    int m_renderingMode;

    //! If __true__ use linear interpolation, otherwise used nearest voxel.
    bool m_useLinearInterpolation;

    //! If __true__ use color mapping.
    bool m_useColorMap;

    //! List of points.
    std::vector<cVoxelCoordList> m_voxelCoordList;


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS - SHADERS:
    //--------------------------------------------------------------------------

protected:

    //! Flag that indicates if shaders have been initialized.
    bool m_flagShadersInitialized;

    //! Vertex shaders.
    cShaderPtr m_vertexShaders[C_NUM_VOXEL_RENDERING_MODES];

    //! Fragment shaders.
    cShaderPtr m_fragmentShaders[C_NUM_VOXEL_RENDERING_MODES];

    //! Program shaders.
    cShaderProgramPtr m_programShaders[C_NUM_VOXEL_RENDERING_MODES];
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
