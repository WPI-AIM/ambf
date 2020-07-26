//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
    (https://github.com/WPI-AIM/ambf)

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

    * Neither the name of authors nor the names of its contributors may
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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   1.0$
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef AF_SOFT_MULTIMESH_H
#define AF_SOFT_MULTIMESH_H
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#include "CBulletMultiMesh.h"
#include "chai3d.h"
#include "CGELMesh.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace ambf {
using namespace chai3d;
//------------------------------------------------------------------------------

///
/// \brief The VertexTree struct
///
struct VertexTree{
    std::vector<int> triangleIdx;
    std::vector<int> vertexIdx;
};

// A struct to store relevant data collection for a softbody node
struct afSoftNode{
  cGELSkeletonNode* m_gelNode;
  btSoftBody::Node* m_btNode;
  std::vector<btSoftBody::Link*> m_btLinks;
  std::vector<cGELSkeletonLink*> m_gelLinks;

  // Get number of links attached to this node
  int getNumLinks(){
      if (!_link_size_computed){
          m_link_size = m_btLinks.size();
          _link_size_computed = true;
      }
      return m_link_size;
  }

private:
  // Boolean to compute the size of links only on the first call
  bool _link_size_computed = false;
  int m_link_size = 0;
};

// A struct to store relevant data collection for a softbody node
struct afSoftLink{
    cGELSkeletonLink* m_gelLink;
    btSoftBody::Link* m_btLink;
};

///
/// \brief The afSoftMultiMesh class
///
class afSoftMultiMesh : public cBulletMultiMesh
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of afSoftMultiMesh.
    afSoftMultiMesh(cBulletWorld* a_world) : cBulletMultiMesh(a_world) {
    }

    //! Destructor of afSoftMultiMesh.
    virtual ~afSoftMultiMesh() {}


    //--------------------------------------------------------------------------
    // IMPORT:
    //--------------------------------------------------------------------------

public:

    //! import base class overloaded virtual and non-virtual methods
    using cGenericObject::setLocalPos;
    using cGenericObject::setLocalRot;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TRANSLATION AND ORIENTATION:
    //--------------------------------------------------------------------------

public:

    //! This method sets the local position of object.
    virtual void setLocalPos(const cVector3d& a_position);

    //! This method sets the orientation of this object.
    virtual void setLocalRot(const cMatrix3d& a_rotation);

    //! This method update the CHAI3D position representation from the Bullet dynamics engine.
    virtual void updatePositionFromDynamics();

    //! Override method
    virtual void buildDynamicModel();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - CONTACT MODEL:
    //--------------------------------------------------------------------------

public:
    //! This method creates a Bullet collision model for this object.
    virtual void buildContactConvexTriangles(const double a_margin = 0.01);

    //! This method creates a Bullet collision model for this object.
    virtual void buildContactTriangles(const double a_margin = 0.01, cMultiMesh *lowResMesh = NULL) ;

    //! This method creates a Bullet collision model for this object.
    virtual void buildContactHull(const double a_margin = 0.01);

    //! Create CGEL Bodys and Nodes based on bulletSoftBody
    virtual void createGELSkeleton();

    //! Helper Function to Create Links from Lines
    bool createLinksFromLines(btSoftBody* a_sb, std::vector< std::vector<int>>* a_lines, cMesh* a_mesh);

    //! Overrride the loadFromFile Method.
    virtual bool loadFromFile(std::string a_filename);

    //! Overrride the render method
    virtual void render(cRenderOptions &a_options);

    //! This method updates the skeletal model of GEL from Bullet's softBody.
    virtual void updateGELSkeletonFrombtSoftBody();

    //! This method returns the Bullet Soft Body's Ptr
    inline btSoftBody* getSoftBody(){return m_bulletSoftBody;}

    //! This method set the Bullet SoftBody Ptr
    inline void setSoftBody(btSoftBody* a_softBody){m_bulletSoftBody = a_softBody;}

    //! Override the setMaterial method
    virtual void setMaterial(cMaterial& a_material, const bool a_affectChildren = false){m_gelMesh.setMaterial(a_material);}

    //! This method scales this object by a_scaleFactor (uniform scale).
    virtual void scale(const double& a_scaleFactor, const bool a_affectChildren = true);

    //! This method toggles the drawing of skeletal model.
    inline void toggleSkeletalModelVisibility(){m_gelMesh.m_showSkeletonModel = !m_gelMesh.m_showSkeletonModel;}


private:
    //! Ptr to scalar vertex arrays of the sofy body
    std::vector<btScalar> m_verticesPtr;
    //! Ptr to Triangles arrays referring to vertices by indices
    std::vector<int> m_trianglesPtr;
    //! Ptr to vector vertex arrays of the sofy body
    std::vector<btVector3> m_verticesVecPtr;
    //! Vertex Tree containing vtx idx's that are repeated for a given vtx
    std::vector<VertexTree> m_vertexTree;
    //! Function to detect, index and store repeat vertices
    void computeUniqueVerticesandTriangles(cMesh* mesh, std::vector<btScalar>* outputVertices, std::vector<int>* outputTriangles, std::vector< std::vector<int> >* outputLines = NULL, bool print_debug_info=false);
    //! Function to detect, index and store repeat vertices
    void computeUniqueVerticesandTrianglesSequential(cMesh* mesh, std::vector<btScalar>* outputVertices, std::vector<int>* outputTriangles, std::vector< std::vector<int> >* outputLines = NULL, bool print_debug_info=false);

    unsigned int m_counter = 0;

public:

    cGELMesh m_gelMesh;

    std::vector<afSoftNode> m_afSoftNodes;
    std::vector<afSoftLink> m_afSoftLinks;

};

//------------------------------------------------------------------------------
} // namespace ambf
#endif
//------------------------------------------------------------------------------
