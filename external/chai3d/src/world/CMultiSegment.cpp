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
    \version   3.2.0 $Rev: 2174 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "world/CMultiSegment.h"
//------------------------------------------------------------------------------
#include "collisions/CGenericCollision.h"
#include "collisions/CCollisionBrute.h"
#include "collisions/CCollisionAABB.h"
#include "shaders/CShaderProgram.h"
//------------------------------------------------------------------------------
#include <algorithm>
#include <vector>
#include <list>
#include <utility>
#include <set>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cMultiSegment.
*/
//==============================================================================
cMultiSegment::cMultiSegment()
{
    // create vertex array
    m_vertices = cVertexArray::create(false, false, true, false, false, false);

    // create triangle array
    m_segments = cSegmentArray::create(m_vertices);

    // should the frame (X-Y-Z) be displayed?
    m_showFrame = false;

    // set default collision detector
    m_collisionDetector = NULL;

    // display lists disabled by default
    m_useDisplayList   = false;

    // default line width
    m_lineWidth = 1.0;

    // show segments
    m_showSegments = true;

    // initialize stippling settings
    m_stippleFactor = 1;
    m_stipplePattern = 0xFFFF;
}


//==============================================================================
/*!
    Destructor of cMultiSegment.
*/
//==============================================================================
cMultiSegment::~cMultiSegment()
{
    // delete any allocated display lists
    m_displayList.invalidate();
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.

    \return Pointer to new object.
*/
//==============================================================================
cMultiSegment* cMultiSegment::copy(const bool a_duplicateMaterialData,
                                   const bool a_duplicateTextureData, 
                                   const bool a_duplicateMeshData,
                                   const bool a_buildCollisionDetector)
{
    // create new instance
    cMultiSegment* obj = new cMultiSegment();

    // copy properties of cMultiSegment
    copyMultiSegmentProperties(obj,
                               a_duplicateMeshData,
                               a_buildCollisionDetector,
                               a_duplicateMeshData,
                               a_buildCollisionDetector);

    // return new object
    return (obj);
}


//==============================================================================
/*!
    This method copies all properties of this point cloud to another.

    \param  a_obj                     Destination object where properties are copied to.
    \param  a_duplicateMaterialData   If __true__, material (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateTextureData    If __true__, texture data (if available) is duplicated, otherwise it is shared.
    \param  a_duplicateMeshData       If __true__, mesh data (if available) is duplicated, otherwise it is shared.
    \param  a_buildCollisionDetector  If __true__, collision detector (if available) is duplicated, otherwise it is shared.
*/
//==============================================================================
void cMultiSegment::copyMultiSegmentProperties(cMultiSegment* a_obj,
                                               const bool a_duplicateMaterialData,
                                               const bool a_duplicateTextureData, 
                                               const bool a_duplicateMeshData,
                                               const bool a_buildCollisionDetector)
{
    // copy properties of cGenericObject
    copyGenericObjectProperties(a_obj, 
        a_duplicateMaterialData, 
        a_duplicateTextureData,
        a_duplicateMeshData,
        a_buildCollisionDetector);

    // copy properties of cMultiSegment
    if (a_duplicateMeshData)
    {
        // duplicate vertex data
        a_obj->m_vertices = m_vertices->copy();

        // duplicate segment data
        a_obj->m_segments = m_segments->copy();

        // build collision detector
        if (a_buildCollisionDetector)
        {
            double radius = 0.0;
            if (m_collisionDetector)
            {
                radius = m_collisionDetector->getBoundaryRadius();
            }
            a_obj->createAABBCollisionDetector(radius);
        }
    }
    else
    {
        // share mesh data
        a_obj->m_vertices = m_vertices;
        a_obj->m_segments = m_segments;
        if (a_buildCollisionDetector)
        {   
            if (m_collisionDetector)
            {
                a_obj->m_collisionDetector = m_collisionDetector;
            }
            else
            {
                a_obj->createAABBCollisionDetector(0.0);
            }
        }
    }
}


//==============================================================================
/*!
    This method computes the center of mass of this point cloud, based on vertex 
    positions.

    \return Center of mass.
*/
//==============================================================================
cVector3d cMultiSegment::getCenterOfMass()
{
    cVector3d centerOfMass(0,0,0);
    int numVertices = m_vertices->getNumElements();

    if (numVertices > 0) 
    {
        for(int i=0; i<numVertices; i++)
        {
            cVector3d pos = m_vertices->getLocalPos(i);
            centerOfMass += pos;
        }
        centerOfMass.mul(1.0 / numVertices);
    }

    return (centerOfMass);
}


//==============================================================================
/*!
    This method returns the number of stored segments.

    \return Number of segments.
*/
//==============================================================================
unsigned int cMultiSegment::getNumSegments()
{ 
    return (unsigned int)(m_segments->getNumElements()); 
}


//==============================================================================
/*!
    This method creates a new vertex and adds it to the vertex list.

    \param  a_x  X coordinate of vertex.
    \param  a_y  Y coordinate of vertex.
    \param  a_z  Z coordinate of vertex.

    \return Index number of new vertex.
*/
//==============================================================================
unsigned int cMultiSegment::newVertex(const double a_x, 
                                      const double a_y, 
                                      const double a_z)
{
    unsigned int index = m_vertices->newVertex();

    // set data
    m_vertices->setLocalPos(index, a_x, a_y, a_z);

    // return vertex index
    return (index);
}


//==============================================================================
/*!
    This method creates a new vertex and adds it to the vertex list.

    \param  a_pos    Position of new vertex.
    \param  a_color  Color.

    \return Index number of new vertex.
*/
//==============================================================================
unsigned int cMultiSegment::newVertex(const cVector3d& a_pos, 
                                      const cColorf& a_color)
{
    unsigned int index = m_vertices->newVertex();

    // set data
    m_vertices->setLocalPos(index, a_pos);
    m_vertices->setColor(index, a_color);

    // return vertex index
    return (index);
}


//==============================================================================
/*!
    This method creates a new segment by passing vertex indices.

    \param  a_indexVertex0  Index number of vertex 0.
    \param  a_indexVertex1  Index number of vertex 1.

    \return Index number of the new segment.
*/
//==============================================================================
unsigned int cMultiSegment::newSegment(const unsigned int a_indexVertex0, 
                                       const unsigned int a_indexVertex1)
{
    int index = m_segments->newSegment(a_indexVertex0, a_indexVertex1);

    // mark object for update
    markForUpdate(false);

    // return the index at which I inserted this triangle in my triangle array
    return (index);
}


//==============================================================================
/*!
    This method creates a new segment and two new vertices by passing its vertex 
    positions and color properties.

    \param  a_vertex0       Position of vertex 0.
    \param  a_vertex1       Position of vertex 1.
    \param  a_colorVertex0  Color at vertex 0.
    \param  a_colorVertex1  Color at vertex 1.

    \return Index number of new segment.
*/
//==============================================================================
unsigned int cMultiSegment::newSegment(const cVector3d& a_vertex0, 
                                const cVector3d& a_vertex1,
                                const cColorf& a_colorVertex0,
                                const cColorf& a_colorVertex1)
{

    unsigned int indexVertex0 = m_vertices->newVertex();
    unsigned int indexVertex1 = m_vertices->newVertex();

    int index = m_segments->newSegment(indexVertex0, indexVertex1);

    m_vertices->setLocalPos(indexVertex0, a_vertex0);
    m_vertices->setLocalPos(indexVertex1, a_vertex1);

    m_vertices->setColor(indexVertex0, a_colorVertex0);
    m_vertices->setColor(indexVertex1, a_colorVertex1);

    // mark object for update
    markForUpdate(false);

    // return result
    return (index);
}


//==============================================================================
/*!
    This method removes a segment by passing its index number.

    \param  a_index  Index number of segment.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiSegment::removeSegment(const unsigned int a_index)
{
    m_segments->removeSegment(a_index);

    // mark mesh for update
    markForUpdate(false);

    // return success
    return (true);
}


//==============================================================================
/*!
    This method clears all segments and vertices.
*/
//==============================================================================
void cMultiSegment::clear()
{
    // clear all triangles
    m_segments->clear();

    // clear all vertices
    m_vertices->clear();
}


//==============================================================================
/*!
    This method scales this object by using different scale factors along X,Y and 
    Z axes.

    \param  a_scaleX  Scale factor along X axis.
    \param  a_scaleY  Scale factor along Y axis.
    \param  a_scaleZ  Scale factor along Z axis.
*/
//==============================================================================
void cMultiSegment::scaleXYZ(const double a_scaleX, const double a_scaleY, const double a_scaleZ)
{
    int numVertices = m_vertices->getNumElements();

    for (int i=0; i<numVertices; i++)
    {
        m_vertices->m_localPos[i].mul(a_scaleX, a_scaleY, a_scaleZ);
    }

    m_boundaryBoxMax.mul(a_scaleX, a_scaleY, a_scaleZ);
    m_boundaryBoxMin.mul(a_scaleX, a_scaleY, a_scaleZ);
}


//==============================================================================
/*!
    This method computes the global position of all vertices.

    \param  a_frameOnly  If __false__, then the global position of all vertices is computed.
*/
//==============================================================================
void cMultiSegment::updateGlobalPositions(const bool a_frameOnly)
{
    if (a_frameOnly) return;

    int numVertices = m_vertices->getNumElements();

    for (int i=0; i<numVertices; i++)
    {
        m_vertices->computeGlobalPosition(i, m_globalPos, m_globalRot);
    }
}


//==============================================================================
/*!
    This method invalidates any existing display lists. You should call this on
    if you are using display lists and you modify mesh options, vertex 
    positions, etc.

    \param  a_affectChildren  If __true__, then children are updated too.
*/
//==============================================================================
void cMultiSegment::markForUpdate(const bool a_affectChildren)
{
    // mark segments for update
    m_segments->m_flagMarkForUpdate = true;

    // update display list of cGenericObject and children
    cGenericObject::markForUpdate(a_affectChildren);
}


//==============================================================================
/*!
    This method sets the alpha value at each vertex, in all of its material 
    colors, optionally propagating the operation to my children.

    \param  a_level            Level of transparency ranging from 0.0 to 1.0.
    \param  a_applyToVertices  If __true__, then apply changes to vertex colors.
    \param  a_applyToTextures  If __true__, then apply changes to texture.
    \param  a_affectChildren   If __true__, then children are updated too.
*/
//==============================================================================
void cMultiSegment::setTransparencyLevel(const float a_level,
    const bool a_applyToVertices,
    const bool a_applyToTextures,
    const bool a_affectChildren)
{
    cGenericObject::setTransparencyLevel(a_level, a_applyToVertices, a_applyToTextures, a_affectChildren);

    // apply the new value to all vertex colors
    if (a_applyToVertices)
    {
        int numVertices = m_vertices->getNumElements();
        for (int i = 0; i < numVertices; i++)
        {
            m_vertices->m_color[i].setA(a_level);
        }

        // mark for update
        markForUpdate(true);
    }
}


//==============================================================================
/*!
    This method sets the color of each segment.

    \param  a_color  New color to be applied to each segment.
*/
//==============================================================================
void cMultiSegment::setLineColor(const cColorf& a_color)
{
    // apply color to all vertex colors
    int numVertices = m_vertices->getNumElements();

    for (int i=0; i<numVertices; i++)
    {
        m_vertices->setColor(i, a_color);
    }
}


//==============================================================================
/*!
    This method shifts all vertex positions by the specified amount.

    \param  a_offset                   Translation to apply to each vertex.
    \param  a_updateCollisionDetector  If __true__, then update collision detector.
*/
//==============================================================================
void cMultiSegment::offsetVertices(const cVector3d& a_offset,
                                   const bool a_updateCollisionDetector)
{
    // offset all vertices
    int numVertices = m_vertices->getNumElements();

    for (int i=0; i<numVertices; i++)
    {
        m_vertices->m_localPos[i].add(a_offset);
    }

    // update boundary box
    m_boundaryBoxMin+=a_offset;
    m_boundaryBoxMax+=a_offset;

    // update collision detector if requested
    if (a_updateCollisionDetector && m_collisionDetector)
    {
        m_collisionDetector->update();
    }
}


//==============================================================================
/*!
    This method compute the axis-aligned boundary box that encloses all
    triangles in this multi-segment object.
*/
//==============================================================================
void cMultiSegment::updateBoundaryBox()
{
    unsigned int numSegments = m_segments->getNumElements();

    if (numSegments == 0)
    {
        m_boundaryBoxMin.zero();
        m_boundaryBoxMax.zero();
        m_boundaryBoxEmpty = true;
        return;
    }

    double xMin = C_LARGE;
    double yMin = C_LARGE;
    double zMin = C_LARGE;
    double xMax = -C_LARGE;
    double yMax = -C_LARGE;
    double zMax = -C_LARGE;
    bool flag = false;

    // loop over all segments
    for(unsigned int i=0; i<numSegments; i++)
    {
        if (m_segments->m_allocated[i])
        {
            cVector3d tVertex0 = m_vertices->getLocalPos(m_segments->getVertexIndex0(i));
            xMin = cMin(tVertex0(0) , xMin);
            yMin = cMin(tVertex0(1) , yMin);
            zMin = cMin(tVertex0(2) , zMin);
            xMax = cMax(tVertex0(0) , xMax);
            yMax = cMax(tVertex0(1) , yMax);
            zMax = cMax(tVertex0(2) , zMax);

            cVector3d tVertex1 = m_vertices->getLocalPos(m_segments->getVertexIndex1(i));
            xMin = cMin(tVertex1(0) , xMin);
            yMin = cMin(tVertex1(1) , yMin);
            zMin = cMin(tVertex1(2) , zMin);
            xMax = cMax(tVertex1(0) , xMax);
            yMax = cMax(tVertex1(1) , yMax);
            zMax = cMax(tVertex1(2) , zMax);

            flag = true;
        }
    }

    if (flag)
    {
        m_boundaryBoxMin.set(xMin, yMin, zMin);
        m_boundaryBoxMax.set(xMax, yMax, zMax);
        m_boundaryBoxEmpty = false;
    }
    else
    {
        m_boundaryBoxMin.zero();
        m_boundaryBoxMax.zero();
        m_boundaryBoxEmpty = true;
    }
}


//==============================================================================
/*!
     This method builds a brute Force collision detector for this multi-segment 
     object.
*/
//==============================================================================
void cMultiSegment::createBruteForceCollisionDetector()
{
    // delete previous collision detector
    if (m_collisionDetector != NULL)
    {
        delete m_collisionDetector;
        m_collisionDetector = NULL;
    }

    // create brute collision detector
    m_collisionDetector = new cCollisionBrute(m_segments);
}


//==============================================================================
/*!
    This method builds an AABB collision detector for this multi-segment 
    object.

    \param  a_radius  Bounding radius.
*/
//==============================================================================
void cMultiSegment::createAABBCollisionDetector(const double a_radius)
{
    // delete previous collision detector
    if (m_collisionDetector != NULL)
    {
        delete m_collisionDetector;
        m_collisionDetector = NULL;
    }

    // create AABB collision detector
    cCollisionAABB* collisionDetector = new cCollisionAABB();
    collisionDetector->initialize(m_segments, a_radius);

    // assign new collision detector
    m_collisionDetector = collisionDetector;
}


//==============================================================================
/*!
    This method renders this multi-segment object using OpenGL.

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMultiSegment::render(cRenderOptions& a_options)
{
    /////////////////////////////////////////////////////////////////////////
    // Render parts that are always opaque
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        // render segments
        if (m_showSegments) 
        {
            renderSegments(a_options);
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // Render parts that use material properties
    /////////////////////////////////////////////////////////////////////////
    if (SECTION_RENDER_PARTS_WITH_MATERIALS(a_options, m_useTransparency))
    {
    }
}


//==============================================================================
/*!
    This method renders all segments.  This function is declared public to allow
    sharing of data among other objects, which is not possible given most
    implementations of 'protected'.  But it should only be accessed
    from within render() or derived versions of render().

    \param  a_options  Rendering options.
*/
//==============================================================================
void cMultiSegment::renderSegments(cRenderOptions& a_options)
{
#ifdef C_USE_OPENGL

    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    unsigned int numVertices = m_vertices->getNumElements();
    unsigned int numSegments = m_segments->getNumElements();

    // check if object contains any segments or vertices
    if ((numVertices == 0) || (numSegments == 0))
    {
        return;
    }

    // initialize some OpenGL flags
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_EDGE_FLAG_ARRAY);
    glDisable(GL_COLOR_MATERIAL);


    //--------------------------------------------------------------------------
    // RENDER MATERIAL
    //--------------------------------------------------------------------------

    // render material properties if enabled
    if (m_useMaterialProperty && a_options.m_render_materials)
    {
        m_material->render(a_options);
    }


    //--------------------------------------------------------------------------
    // RENDER TEXTURE
    //--------------------------------------------------------------------------

    // check if texture is available
    if (m_texture == nullptr)
    {
        m_useTextureMapping = false;
    }

    // render texture if enabled
    if ((m_texture != nullptr) && (m_useTextureMapping) && (a_options.m_render_materials))
    {
        m_texture->renderInitialize(a_options);
    }


    //--------------------------------------------------------------------------
    // SETTINGS
    //--------------------------------------------------------------------------

    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_LIGHTING);

    //--------------------------------------------------------------------------
    // RENDER OBJECT (OBJECT BUFFER + SHADERS)
    //--------------------------------------------------------------------------
    if ((m_shaderProgram != nullptr) && (!a_options.m_creating_shadow_map))
    {
        // enable shader
        m_shaderProgram->use(this, a_options);

        // render normal texture if enabled
        if (m_normalMap != nullptr)
        {
            if (m_shaderProgram->isUsed())
            {
                m_normalMap->renderInitialize(a_options);
            }
        }

        // render VBO
        m_segments->render();

        // disable normal map is available
        if (m_normalMap != nullptr)
        {
            m_normalMap->renderFinalize(a_options);
        }

        // disable shader
        m_shaderProgram->disable();
    }

    //--------------------------------------------------------------------------
    // RENDER OBJECT (OLD METHOD)
    //--------------------------------------------------------------------------
    else if (!m_displayList.render(m_useDisplayList))
    {
        // get texture unit
        GLenum textureUnit = GL_TEXTURE1_ARB;
        if (m_texture != nullptr)
        {
            textureUnit = m_texture->getTextureUnit();
        }

        // if requested, begin creating display list
        m_displayList.begin(m_useDisplayList);

        //-------------------------------------------------------------------
        // RENDER ALL SEGMENTS
        //-------------------------------------------------------------------

        // set line width
        glLineWidth((GLfloat)m_lineWidth);

        // set stipple settings
        if (m_stipplePattern == 0xFFFF)
        {
            glDisable(GL_LINE_STIPPLE);
        }
        else
        {
            glLineStipple(m_stippleFactor, m_stipplePattern);
            glEnable(GL_LINE_STIPPLE);
        }


        /////////////////////////////////////////////////////////////////////
        // RENDER SEGMENTS USING CLASSIC OPENGL COMMANDS
        /////////////////////////////////////////////////////////////////////
        {
            // begin rendering
            glBegin(GL_LINES);

            for (unsigned int i=0; i<numSegments; i++)
            {
                if (m_segments->m_allocated[i])
                {
                    unsigned int index0 = m_segments->getVertexIndex0(i);
                    unsigned int index1 = m_segments->getVertexIndex1(i);

                    // render vertex 0
                    glColor4fv(m_vertices->m_color[index0].getData());
                    glVertex3dv(&m_vertices->m_localPos[index0](0));

                    // render vertex 1
                    glColor4fv(m_vertices->m_color[index1].getData());
                    glVertex3dv(&m_vertices->m_localPos[index1](0));
                }
            }

            // finalize rendering 
            glEnd();
        }

        //-------------------------------------------------------------------
        // FINALIZE DISPLAY LIST
        //-------------------------------------------------------------------

        // if being created, finalize display list
        m_displayList.end(true);
    }


    //--------------------------------------------------------------------------
    // RESTORE OPENGL
    //--------------------------------------------------------------------------

    // turn off texture rendering
    if ((m_texture != nullptr) && (m_useTextureMapping))
    {
        m_texture->renderFinalize(a_options);
    }

    // restore OpenGL variables
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_LINE_STIPPLE);


#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
