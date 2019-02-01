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
    \author    Sebastien Grange
    \version   3.2.0 $Rev: 2147 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CMultiImageH
#define CMultiImageH
//------------------------------------------------------------------------------
#include "graphics/CImage.h"
//------------------------------------------------------------------------------
#include <string>
#include <vector>
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMultiImage.h

    \brief
    Implements an array of 2D image structures.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cMultiImage;
typedef std::shared_ptr<cMultiImage> cMultiImagePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cMultiImage
    \ingroup    graphics

    \brief
    This class implements an array of 2D image structures.

    \details
    cMultiImage provides a child class to \ref cImage that handles multiple
    images with similar properties (size and format). Each image can be used
    either as a slice of a volume representation, or a frame of a time lapse.
    The images are treated as a set of pixel arrays, all sharing common
    properties and geometry. All pixel arrays are guaranteed to be allocated
    contiguously in a single memory array.

*/
//==============================================================================
class cMultiImage : public cImage
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Default constructor of cMultiImage.
    cMultiImage();

    //! Destructor of cMultiImage.
    virtual ~cMultiImage();

    //! Shared cMultiImage allocator.
    static cMultiImagePtr create() { return (std::make_shared<cMultiImage>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL COMMANDS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy itself.
    cMultiImagePtr copy();

    //! This method allocates a new multi image array by defining its size, pixel format and pixel type.
    bool allocate(const unsigned int a_width,
                  const unsigned int a_height,
                  const unsigned int a_slices,
                  const GLenum a_format = GL_RGB,
                  const GLenum a_type = GL_UNSIGNED_BYTE);

    //! This method deletes all image data from memory.
    void erase() { cleanup(); }

    //! This method returns the number of images stored.
    virtual unsigned int getImageCount() const { return (unsigned int)(m_imageCount);  }

    //! This method converts the image to a new format passed as argument.
    bool convert(const unsigned int a_newFormat);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MULTIMAGE SELECTION:
    //--------------------------------------------------------------------------

public:

    //! This method returns the index number of the current image.
    virtual unsigned long getCurrentIndex() { return m_currentIndex; }

    //! This method sets the current image.
    virtual bool selectImage(unsigned long a_index);

    //! This method adds an image to the set if size and format are compatible.
    bool addImage(cImage &a_image, unsigned long a_index=-1);

    //! This method removes an image from the set.
    bool removeImage(unsigned long a_index);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MANIPULATING PIXELS:
    //--------------------------------------------------------------------------

public:

    //! This method defines a pixel color for all images to be transparent.
    virtual void setTransparentColor(const cColorb &a_color,
        const unsigned char a_transparencyLevel);

    //! This method defines a pixel color to be transparent for all images.
    virtual void setTransparentColor(const unsigned char a_r,
        const unsigned char a_g,
        const unsigned char a_b,
        const unsigned char a_transparencyLevel)  { cColorb color(a_r, a_g, a_b); setTransparentColor(color, a_transparencyLevel); }

    //! This method defines a transparent level to all pixels of image set.
    virtual void setTransparency(const unsigned char a_transparencyLevel);

    //! This method flips all images horizontally.
    virtual void flipHorizontal();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MANIPULATING VOXELS:
    //--------------------------------------------------------------------------

public:

    //! This method retrieves the nearest voxel location from a texture coordinate.
    virtual void getVoxelLocation(const cVector3d& a_texCoord, int& a_voxelX, int& a_voxelY, int& a_voxelZ, bool a_clampToImageSize = true) const;

    //! This method retrieves the voxel location from a texture coordinate.
    virtual void getVoxelLocationInterpolated(const cVector3d& a_texCoord, double& a_voxelX, double& a_voxelY, double& a_voxelZ, bool a_clampToImageSize = true) const;

    //! This method returns a pointer to voxel memory data.
    virtual unsigned char* getVoxelData(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z) const;

    //! This method returns the color of an image voxel at location (x,y,z).
    virtual bool getVoxelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z,
        cColorb& a_color) const;

    //! This method returns the color of an image voxel at location (x,y,z).
    virtual bool getVoxelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z,
        cColorf& a_color) const;

    //! This method returns the interpolated color of an image voxel at location (x,y,z).
    virtual bool getVoxelColorInterpolated(const double a_x,
        const double a_y,
        const double a_z,
        cColorb& a_color) const;

    //! This method returns the interpolated color of an image voxel at location (x,y,z).
    virtual bool getVoxelColorInterpolated(const double a_x,
        const double a_y,
        const double a_z,
        cColorf& a_color) const;

    //! This method sets the color of an image voxel at location (x,y,z).
    virtual void setVoxelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z,
        const cColorb& a_color);

    //! This method sets the color of an image voxel at location (x,y,z).
    virtual void setVoxelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z,
        const unsigned char a_r,
        const unsigned char a_g,
        const unsigned char a_b) { cColorb color(a_r, a_g, a_b); setVoxelColor(a_x, a_y, a_z, color); }

    //! This method sets the gray level of an image voxel at location (x,y,z).
    virtual void setVoxelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z,
        const unsigned char a_grayLevel);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MEMORY DATA:
    //--------------------------------------------------------------------------

public:

    //! This method returns a pointer to the actual image data array. Use with care!
    inline unsigned char* getArray() { return (m_array); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - FILES:
    //--------------------------------------------------------------------------

public:

    //! This method loads an image file.
    virtual bool loadFromFile(const std::string& a_filename);
    
    //! This method loads an image set from a set of files.
    virtual int loadFromFiles(const std::vector<std::string>& a_filename);

    //! This method loads an image set from a set of similarly named images.
    virtual int loadFromFiles(const std::string& a_basename, const std::string& a_extension, unsigned long a_max = 9999);

    //! This method adds an image file to the set if size and format are compatible.
    virtual bool addFromFile(const std::string& a_filename, unsigned long a_index=-1);

    //! This method saves all images to a set of files.
    virtual bool saveToFiles(const std::string& a_basename, const std::string& a_extension);


    //--------------------------------------------------------------------------
    // PROTECTED  METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method initializes all member variables.
    void defaults();

    //! This method deletes memory and rid ourselves of any image previously stored.
    void cleanup();

    //! Add an image file to a preallocated set if size and format are compatible.
    virtual bool addFromFilePrealloc(const std::string& a_filename, unsigned long a_index);

    //! Add an image to a preallocated set if size and format are compatible.
    bool addImagePrealloc(cImage &a_image, unsigned long a_index);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! The image array data that holds all images contiguously.
    unsigned char* m_array;

    //! Number of images contained in the structure.
    unsigned long m_imageCount;

    //! Index of the currently selected image.
    unsigned long m_currentIndex;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
