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
    \author    Tim Schroder
    \author    Francois Conti
    \version   3.2.0 $Rev: 2146 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CFileModelOBJH
#define CFileModelOBJH
//------------------------------------------------------------------------------
#include "world/CMultiMesh.h"
//------------------------------------------------------------------------------
#include <map>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CFileModelOBJ.h
    \ingroup    files

    \brief
    Implements OBJ model file support.
*/
//==============================================================================

//------------------------------------------------------------------------------
/*!
    \addtogroup files
*/
//------------------------------------------------------------------------------

//@{

//! This function loads an OBJ model file.
bool cLoadFileOBJ(cMultiMesh* a_object, const std::string& a_filename);

//! This function saves an OBJ model file.
bool cSaveFileOBJ(cMultiMesh* a_object, const std::string& a_filename);

//@}


//------------------------------------------------------------------------------
/*!
    Clients can use this to tell the OBJ file loader how to behave in terms
    of vertex merging. \n
    If __true__ (default), loaded OBJ files will have three _distinct_ vertices
    per triangle, with no vertex re-use.
*/
//------------------------------------------------------------------------------
extern bool g_objLoaderShouldGenerateExtraVertices;


//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//==============================================================================
// INTERNAL DEFINITIONS FOR OBJ FILE LOADER:
//============================================================================== 

//! A face vertex, as defined in an .obj file (a vertex/normal/texture set)
struct vertexIndexSet 
{
    int vIndex;
    int nIndex;
    int tIndex;

    vertexIndexSet() 
    {
        vIndex = nIndex = tIndex = 0;
    }
    vertexIndexSet(int vIndex, int nIndex, int tIndex) 
    {
        this->vIndex = vIndex;
        this->nIndex = nIndex;
        this->tIndex = tIndex;
    }
    vertexIndexSet(int vIndex) 
    {
        this->vIndex = vIndex;
        this->nIndex = 0;
        this->tIndex = 0;
    }
};

//------------------------------------------------------------------------------

struct ltVertexIndexSet
{
    bool operator()(vertexIndexSet v1, vertexIndexSet v2) const
    {
        if (v1.vIndex < v2.vIndex) return 1;
        if (v2.vIndex < v1.vIndex) return 0;
        if (v1.nIndex < v2.nIndex) return 1;
        if (v2.nIndex < v1.nIndex) return 0;
        if (v1.tIndex < v2.tIndex) return 1;
        return 0;
    }
};

//------------------------------------------------------------------------------

typedef std::map<vertexIndexSet,unsigned int,ltVertexIndexSet> vertexIndexSet_uint_map;

//------------------------------------------------------------------------------

//==============================================================================
//  INTERNAL IMPLEMENTATION
//==============================================================================

// OBJ maximum length of a path
#define C_OBJ_SIZE_PATH		255

// OBJ File string identifiers
#define C_OBJ_VERTEX_ID    "v"
#define C_OBJ_TEXCOORD_ID  "vt"
#define C_OBJ_NORMAL_ID    "vn"
#define C_OBJ_FACE_ID      "f"
#define C_OBJ_COMMENT_ID   "#"
#define C_OBJ_MTL_LIB_ID   "mtllib"
#define C_OBJ_USE_MTL_ID   "usemtl"
#define C_OBJ_NAME_ID      "g"

// MTL File string identifiers
#define C_OBJ_NEW_MTL_ID       "newmtl"
#define C_OBJ_MTL_TEXTURE_ID   "map_Kd"
#define C_OBJ_MTL_AMBIENT_ID   "Ka"
#define C_OBJ_MTL_DIFFUSE_ID   "Kd"
#define C_OBJ_MTL_SPECULAR_ID  "Ks"
#define C_OBJ_MTL_SHININESS_ID "Ns"
#define C_OBJ_MTL_ALPHA_ID     "Tr"
#define C_OBJ_MTL_ALPHA_ID_ALT "d"

// Maximum size of a string that could be read out of the OBJ file
#define C_OBJ_MAX_STR_SIZE 1024

// Maximum number of vertices a that a single face can have
#define C_OBJ_MAX_VERTICES 256

// Image File information.
struct cOBJFileInfo
{
    cOBJFileInfo()
    {
        init();
    }

    void init()
    {
        m_vertexCount = 0;
        m_texCoordCount = 0;
        m_normalCount = 0;
        m_faceCount = 0;
        m_materialCount = 0;
    }

    unsigned int m_vertexCount;
    unsigned int m_texCoordCount;
    unsigned int m_normalCount;
    unsigned int m_faceCount;
    unsigned int m_materialCount;
};

// Information about a surface face.
struct cFace
{
    unsigned int m_numVertices;
    unsigned int m_materialIndex;
    int          m_groupIndex;
    int*         m_pVertexIndices;
    cVector3d*   m_pVertices;
    cColorf*     m_pColors;
    int*         m_pNormalIndices;
    cVector3d*   m_pNormals;
    int*         m_pTextureIndices;
    cVector3d*   m_pTexCoords;

    cFace()
    {
        init();
    }

    void init()
    {
        m_numVertices = 0;
        m_materialIndex = -1;
        m_groupIndex = -1;
        m_pVertexIndices = NULL;
        m_pVertices = NULL;
        m_pColors = NULL;
        m_pNormalIndices = NULL;
        m_pNormals = NULL;
        m_pTextureIndices = NULL;
        m_pTexCoords = NULL;
    }
};

// Information about a material property
struct cMaterialInfo
{
    char m_name[1024];
    char m_texture[C_OBJ_SIZE_PATH];
    int	m_textureID;
    float m_diffuse[3];
    float m_ambient[3];
    float m_specular[3];
    float m_emmissive[3];
    float m_alpha;
    float m_shininess;

    cMaterialInfo() 
    {
        init();
    }

    void init()
    {
        m_name[0] = '\0';
        m_texture[0] = '\0';
        m_textureID = 0;
        m_diffuse[0] = m_diffuse[1] = m_diffuse[2] = 0.8f;
        m_ambient[0] = m_ambient[1] = m_ambient[2] = 0.5f;
        m_specular[0] = m_specular[1] = m_specular[2] = 0.3f;
        m_emmissive[0] = m_emmissive[1] = m_emmissive[2] = 0.0f;
        m_shininess = 0;
        m_alpha = 1.0f;
    }
};


//==============================================================================
/*!
    \class      cOBJModel
    \ingroup    files

    \brief
    Implementation of an OBJ file loader.
*/
//==============================================================================
class cOBJModel
{    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    public:

    //! Constructor of cOBJModel.
    cOBJModel();
    
    //! Destructor of cOBJModel.
    ~cOBJModel();

    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    //! Load model file.
    bool LoadModel(const char szFileName[]);


    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------
    
    //! List of vertices.
    cVector3d* m_pVertices;

    //! List of colors.
    cColorf* m_pColors;

    //! List of faces.
    cFace* m_pFaces;
    
    //! List of normals.
    cVector3d* m_pNormals;
    
    //! List of texture coordinates.
    cVector3d* m_pTexCoords;
    
    //! List of material and texture properties.
    cMaterialInfo* m_pMaterials;
    
    //! Information about image file.
    cOBJFileInfo m_OBJInfo;

    //! List of names obtained from 'g' commands, with the most recent at the back...
    std::vector<char*> m_groupNames;


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

    private:

    //! Read next string of file.
    void readNextString(char *a_str, int a_size, FILE *a_hStream);
    
    //! Get next token from file.
    void getTokenParameter(char a_str[], const unsigned int a_strSize, FILE *a_hFile);

    //! File path.
    void makePath(char a_fileAndPath[]);
    
    //! Load material file [mtl].
    bool loadMaterialLib(const char a_fFileName[], cMaterialInfo *a_pMaterials,
        unsigned int *a_curMaterialIndex, char a_basePath[]);

    //! Parse information about face.
    void  parseFaceString(char a_faceString[], cFace *a_faceOut, const cVector3d *a_pVertices,
        const cVector3d *a_pNormals, const cVector3d *a_pTexCoords, const unsigned int a_materialIndex);

    //! Read information about file.
    void  getFileInfo(FILE *a_hStream, cOBJFileInfo *a_stat, const char a_constBasePath[]);
};

//! Internal: get a (possibly new) vertex index for a vertex.
unsigned int getVertexIndex(cMesh* a_Mesh, 
                            cOBJModel* a_model, 
                            vertexIndexSet_uint_map* a_VertexMap, 
                            vertexIndexSet& vis);

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
