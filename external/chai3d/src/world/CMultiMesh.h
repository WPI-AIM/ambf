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
    \version   3.2.0 $Rev: 2161 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CMultiMeshH
#define CMultiMeshH
//------------------------------------------------------------------------------
#include "world/CMesh.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMultiMesh.h

    \brief 
    Implements a 3D multi-mesh object.
*/
//==============================================================================

//==============================================================================
/*!    
    \class      cMultiMesh
    \ingroup    world

    \brief
    This class implements a 3D multi-mesh object.

    \details
    This class implements a collection of cMesh objects. Each cMesh object
    includes one material and texture properties with a set of vertices
    and triangles. cMultiMesh allows the user to build more complicated 
    polygonal objects composed of sets of triangles that share digfferent
    materials.
*/
//==============================================================================
class cMultiMesh : public cGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cMultiMesh.
    cMultiMesh();

    //! Destructor of cMultiMesh.
    virtual ~cMultiMesh();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL:
    //--------------------------------------------------------------------------

public:

    //! This method enables or disables this object. When an object is disabled, both haptic and graphic rendering no longer occur.
    virtual void setEnabled(bool a_enabled,
                            const bool a_affectChildren = false);

    //! This method creates a copy of itself.
    virtual cMultiMesh* copy(const bool a_duplicateMaterialData = false,
                             const bool a_duplicateTextureData = false, 
                             const bool a_duplicateMeshData = false,
                             const bool a_buildCollisionDetector = true);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables haptic perception of this object, optionally propagating the change to children.
    virtual void setHapticEnabled(const bool a_hapticEnabled, 
                                  const bool a_affectChildren = false);

    //! This method sets the haptic stiffness of the object, optionally recursively affecting children.
    virtual void setStiffness(const double a_stiffness, 
                              const bool a_affectChildren = false);

    //! This method sets the static and dynamic friction properties (polygonal models only), optionally recursively affecting children.
    virtual void setFriction(double a_staticFriction, 
                             double a_dynamicFriction, 
                             const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - GRAPHIC PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables the graphic display of this object, optionally propagating the change to children.
    virtual void setShowEnabled(const bool a_show, const bool a_affectChildren = false);

    //! This method returns whether wireframe rendering is enabled.
    virtual void setWireMode(const bool a_showWireMode, 
        const bool a_affectChildren = true);

    //! Enable or disabling face-culling, optionally propagating the operation to my children.
    virtual void setUseCulling(const bool a_useCulling, 
        const bool a_affectChildren=true);

    //! This method enables or disables transparency.
    virtual void setUseTransparency(const bool a_useTransparency, const bool a_affectChildren = false);

    //! This method sets the transparency level of the object.
    virtual void setTransparencyLevel(const float a_level,
        const bool a_applyToVertices = false,
        const bool a_applyToTextures = false,
        const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - DISPLAY LISTS:
    //-----------------------------------------------------------------------

public:

    //! This method enabled or disables the use of a display list for rendering, optionally propagating the operation to its children.
    virtual void setUseDisplayList(const bool a_useDisplayList, const bool a_affectChildren = false);

    //! This method invalidates any existing display lists, optionally propagating the operation to its children.
    virtual void markForUpdate(const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - MATERIAL PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables the use of material properties, optionally propagating the operation to its children.
    virtual void setUseMaterial(const bool a_useMaterial, const bool a_affectChildren = true);

    //! This method sets the material properties of this object, optionally propagating the operation to its children.
    virtual void setMaterial(cMaterialPtr a_material, const bool a_affectChildren = false);

    //! This method setd the material properties of this object, optionally propagating the operation to its children.
    virtual void setMaterial(cMaterial& a_material, const bool a_affectChildren = false);

    //! This method creates a backup of the material colors of this object, optionally propagating the operation to its children.
    virtual void backupMaterialColors(const bool a_affectChildren = false);

    //! This method restores the material color properties of this object from a previous backup, optionally propagating the operation to its children.
    virtual void restoreMaterialColors(const bool a_affectChildren = false);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TEXTURE PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disables the use of texture-mapping, optionally propagating the operation to its children.
    virtual void setUseTexture(const bool a_useTexture, const bool a_affectChildren = true);

    //! This method sets a texture to this object, optionally propagating the operation to its children.
    virtual void setTexture(cTexture1dPtr, const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - SHADERS:
    //-----------------------------------------------------------------------

public:

    //! Set shader program.
    virtual void setShaderProgram(cShaderProgramPtr a_shaderProgram, const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - BOUNDARY BOX:
    //-----------------------------------------------------------------------

public:

    //! This method enables or disabled the graphic display of the boundary box for this object, optionally propagating the change to its children.
    virtual void setShowBoundaryBox(const bool a_showBoundaryBox, const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION DETECTION:
    //-----------------------------------------------------------------------

public:

    //! This method deletes any existing collision detector.
    virtual void deleteCollisionDetector(const bool a_affectChildren = false);

    //! This method computes any collision between a segment and this object.
    virtual bool computeCollisionDetection(const cVector3d& a_segmentPointA,
                                           const cVector3d& a_segmentPointB,
                                           cCollisionRecorder& a_recorder,
                                           cCollisionSettings& a_settings);

    //! This method enables or disables the display of the collision detector, optionally propagating the change to its children.
    virtual void setShowCollisionDetector(const bool a_showCollisionDetector, 
                                          const bool a_affectChildren = false);

    //! This method sets the collision detector graphic display properties.
    virtual void setCollisionDetectorProperties(unsigned int a_displayDepth, 
                                                cColorf& a_color, 
                                                const bool a_affectChildren = false);

    //! Set up a brute force collision detector for this mesh and (optionally) for its children.
    virtual void createBruteForceCollisionDetector();

    //! Set up an AABB collision detector for this mesh.
    virtual void createAABBCollisionDetector(const double a_radius);


    //-----------------------------------------------------------------------
    // PUBLIC VIRTUAL METHODS - INTERACTIONS
    //-----------------------------------------------------------------------

public:

    //! Computer haptic interaction.
    virtual cVector3d computeInteractions(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int a_IDN,
        cInteractionRecorder& a_interactions);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MESH PRIMITIVES:
    //--------------------------------------------------------------------------

public:

    //! This method creates a new mesh primitive.
    cMesh* newMesh();

    //! This method adds an existing mesh primitives to list of meshes
    bool addMesh(cMesh* a_mesh);

    //! This method removes a mesh primitive from the list of meshes.
    bool removeMesh(cMesh* a_mesh);

    //! This method removes all mesh primitives.
    bool removeAllMesh();

    //! This method deletes a mesh primitive from the list of meshes.
    bool deleteMesh(cMesh* a_mesh);

    //! This method deletes all meshes.
    bool deleteAllMeshes();

    //! This method retrieves the number of meshes that compose this multi-mesh object.
    int getNumMeshes();

    //! This method returns a pointer to a mesh primitive by passing its index number.
    cMesh* getMesh(unsigned int a_index);

    //! This method converts this multimesh into a single mesh object.
    void convertToSingleMesh(cMesh* a_mesh);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - VERTICES
    //--------------------------------------------------------------------------
    
public:

    //! This method returns the index number and mesh of a specific vertex that is part of this multi-mesh.
    bool getVertex(const unsigned int a_index, cMesh*& a_mesh, unsigned int& a_vertexIndex);

    //! This method returns the position data of specific vertex.
    cVector3d getVertexPos(unsigned int a_index);

    //! This method returns the the number of stored vertices.
    unsigned int getNumVertices() const;

    //! This method enables or disables the use of per-vertex colors, optionally propagating the operation to its children.
    virtual void setUseVertexColors(const bool a_useColors, const bool a_affectChildren=true);

    //! This method sets the color of each vertex.
    void setVertexColor(const cColorf& a_color);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TRIANGLES
    //--------------------------------------------------------------------------

public:

    //! This method returns the index number and mesh of a specific triangle that is part of this multi-mesh.
    bool getTriangle(const unsigned int a_index, cMesh*& a_mesh, unsigned int& a_triangleIndex);

    //! This method returns the the number of stored triangles.
    unsigned int getNumTriangles() const;

    //! This method clears all triangles and vertices of multi-mesh.
    void clear();

    //! This method enables or disables the rendering of triangles.
    void setShowTriangles(const bool a_showTriangles);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - EDGES
    //--------------------------------------------------------------------------

public:

    //! This method creates a list of edges by providing a threshold angle in degrees.
    void computeAllEdges(double a_angleThresholdDeg = 40.0);

    //! This method clears all edges.
    void clearAllEdges();

    //! This method enables or disables the rendering of edges.
    void setShowEdges(const bool a_showEdges);

    //! This method sets the graphic properties for edge-rendering.
    void setEdgeProperties(const double a_width, 
                           const cColorf& a_color);

    //! This method sets the line width of all edges.
    void setEdgeLineWidth(const double a_width);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - SURFACE NORMALS
    //--------------------------------------------------------------------------

public:

    //! This method enables or disables the rendering of vertex normals.
    void setShowNormals(const bool& a_showNormals);

    //! This method sets the graphic properties for normal-rendering.
    void setNormalsProperties(const double a_length, const cColorf& a_color);

    //! This method set the length of normals for display purposes.
    void setNormalsLength(const double a_length);

    //! This method computes all triangle normals.
    void computeAllNormals();

    //! This method reverses all normals on this model.
    void reverseAllNormals();

    //! This method computes the normal matrix vectors for all triangles.
    void computeBTN();

    //! This method enables or disables the rendering of tangents and bi-tangents.
    void setShowTangents(const bool a_showTangents);


    //--------------------------------------------------------------------------
    // PUBLIC VIRTUAL METHODS - FILES:
    //--------------------------------------------------------------------------

public:

    //! This method loads a 3D object from a file.
    virtual bool loadFromFile(std::string a_filename);

    //! This method saves 3D object to a file.
    virtual bool saveToFile(std::string a_filename);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - SCALING:
    //--------------------------------------------------------------------------

public:

    //! This method scales this object by a_scaleFactor (uniform scale).
    virtual void scale(const double& a_scaleFactor, const bool a_affectChildren = true);

    //! This method scales this object by using different factors along X,Y and Z axes.
    void scaleXYZ(const double a_scaleX, const double a_scaleY, const double a_scaleZ);


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method renders this object graphically using OpenGL.
    virtual void render(cRenderOptions& a_options);

    //! This method update the global position information about this object.
    virtual void updateGlobalPositions(const bool a_frameOnly);

    //! This method updates the boundary box of this object.
    virtual void updateBoundaryBox();

    //! This method copies all properties of this multi-mesh object to another.
    void copyMultiMeshProperties(cMultiMesh* a_obj,
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS
    //-----------------------------------------------------------------------

public:

    //! Array of meshes.
    std::vector<cMesh*> *m_meshes;

};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

