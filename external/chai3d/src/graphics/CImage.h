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
    \version   3.2.0 $Rev: 2149 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CImageH
#define CImageH
//------------------------------------------------------------------------------
#include "graphics/CColor.h"
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CImage.h

    \brief
    Implements a 2D image data structure.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cImage;
typedef std::shared_ptr<cImage> cImagePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cImage
    \ingroup    graphics

    \brief
    This class implements a 2D image data structure.

    \details
    This class provides support for 2D images of the following formats: 
    GL_LUMINANCE, GL_RGB, and GL_RGBA. \n
    Several file formats are also supported for loading and saving images 
    to disk. These include __bmp__, __gif__, __jpg__, __png__, __ppm__, 
    and __raw__.
*/
//==============================================================================
class cImage
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Default constructor of cImage.
    cImage();

    //! Constructor of cImage. Initializes an image by passing its width, height, pixel format and pixel type.
    cImage(const unsigned int a_width,
           const unsigned int a_height,
           const GLenum a_format = GL_RGB,
           const GLenum a_type = GL_UNSIGNED_BYTE);

    //! Destructor of cImage.
    virtual ~cImage();

    //! Shared cImage allocator.
    static cImagePtr create() { return (std::make_shared<cImage>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL COMMANDS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy itself.
    cImagePtr copy();

    //! This method allocates a new image by defining its size, pixel format and pixel type.
    bool allocate(const unsigned int a_width,
                  const unsigned int a_height,
                  const GLenum a_format = GL_RGB,
                  const GLenum a_type = GL_UNSIGNED_BYTE);

    //! This method deletes all image data from memory.
    void erase() { cleanup(); }

    //! This method returns the number of images stored. (1 only for class \ref cImage).
    virtual unsigned int getImageCount() const { return (1); }

    //! This method returns __true__ if the image has been allocated in memory, __false__ otherwise.
    inline bool isInitialized() const { return (m_allocated); }

    //! This method sets and allocates the size of the image by defining its width and height.
    void setSize(const unsigned int a_width, const unsigned int a_height);

    //! This method returns the width of the image.
    inline unsigned int getWidth() const { return (m_width); }

    //! This method returns the height of the image.
    inline unsigned int getHeight() const { return (m_height); }

    //! This method returns the pixel format of the image (GL_RGB, GL_RGBA, GL_LUMINANCE for instance).
    inline GLenum getFormat() const { return (m_format); }

    //! This method returns the pixel data type. (GL_UNSIGNED_BYTE, GL_UNSIGNED_INT for instance).
    inline GLenum getType() const { return (m_type); }

    //! This method returns the number of bits per pixel used to store this image.
    inline unsigned int getBitsPerPixel() const { return (8 * m_bytesPerPixel); }

    //! This method returns the number of bytes per pixel used to store this image.
    inline unsigned int getBytesPerPixel() const { return (m_bytesPerPixel); }

    //! This method returns the size in bytes of the current image.
    inline unsigned int getSizeInBytes() const { return (m_bytesPerPixel * m_width * m_height); }

    //! This method converts the image to a new format passed as argument.
    bool convert(const unsigned int a_newFormat);

    //! This method queries the number of bytes per pixel for a given format.
    static int queryBytesPerPixel(const GLenum a_format,
        const GLenum a_type);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MEMORY DATA:
    //--------------------------------------------------------------------------

public:

    //! This method returns a pointer to this cImage object.
    cImage* getImage() { return (this); }

    //! This method returns a pointer to the actual image data. Use with care!
    virtual unsigned char* getData() { return (m_data); }

    //! This method modifies the pointer to the actual image data. Use with care!
    virtual void setData(unsigned char* a_data,
        const unsigned int a_dataSizeInBytes,
        const bool a_dealloc = false);

    //! This method overrides the properties of the image. Use with care!
    bool setProperties(const unsigned int a_width,
        const unsigned int a_height,
        const GLenum a_format,
        const GLenum a_type);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MANIPULATING PIXELS:
    //--------------------------------------------------------------------------

public:

    //! This method clears all image pixels with a black color.
    virtual void clear();

    //! This method clears all image pixels with a color passed as argument.
    virtual void clear(const cColorb& a_color);

    //! This method clears all image pixels with a color passed as argument.
    virtual void clear(const unsigned char a_r,
        const unsigned char a_g,
        const unsigned char a_b,
        const unsigned char a_a = 0xff) { clear(cColorb(a_r, a_g, a_b, a_a)); }

    //! This method clears all image pixels with a level of gray passed as argument.
    virtual void clear(const unsigned char a_grayLevel) { clear(cColorb(a_grayLevel, a_grayLevel, a_grayLevel)); }

    //! This method retrieves the nearest pixel location from a texture coordinate.
    virtual void getPixelLocation(const cVector3d& a_texCoord, int& a_pixelX, int& a_pixelY, bool a_clampToImageSize = true) const;

    //! This method retrieves a pixel location from a texture coordinate.
    virtual void getPixelLocationInterpolated(const cVector3d& a_texCoord, double& a_pixelX, double& a_pixelY, bool a_clampToImageSize = true) const;

    //! This method returns the color of a pixel at location (x,y).
    virtual bool getPixelColor(const unsigned int a_x,
        const unsigned int a_y,
        cColorb& a_color) const;

    //! This method returns the color of a pixel at location (x,y).
    virtual bool getPixelColor(const unsigned int a_x,
        const unsigned int a_y,
        cColorf& a_color) const;

    //! This method returns the interpolated color of an image pixel at location (x,y).
    virtual bool getPixelColorInterpolated(const double a_x,
        const double a_y,
        cColorb& a_color) const;

    //! This method returns the interpolated color of an image pixel at location (x,y).
    virtual bool getPixelColorInterpolated(const double a_x,
        const double a_y,
        cColorf& a_color) const;

    //! This method sets the color of a pixel at location (x,y).
    virtual void setPixelColor(const unsigned int a_x,
        const unsigned int a_y, 
        const cColorb& a_color);

    //! This method sets the color of a pixel at location (x,y).
    virtual void setPixelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned char a_r,
        const unsigned char a_g,
        const unsigned char a_b) { cColorb color(a_r, a_g, a_b); setPixelColor(a_x, a_y, color); }

    //! This method sets the gray scale of a pixel at location (x,y).
    virtual void setPixelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned char a_grayLevel);

    //! This method defines a specific pixel color to be transparent.
    virtual  void setTransparentColor(const cColorb &a_color,
        const unsigned char a_transparencyLevel);

    //! This method defines a specific pixel color to be transparent.
    virtual void setTransparentColor(const unsigned char a_r,
        const unsigned char a_g,
        const unsigned char a_b,
        const unsigned char a_transparencyLevel);

    //! This method defines a transparency value to be applied to all image pixels.
    virtual void setTransparency(const unsigned char a_transparencyLevel);

    //! This method flips this image horizontally.
    virtual void flipHorizontal();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MANIPULATING VOXELS:
    //--------------------------------------------------------------------------

public:

    //! This method retrieves the nearest voxel location from a texture coordinate.
    virtual void getVoxelLocation(const cVector3d& a_texCoord, int& a_voxelX, int& a_voxelY, int& a_voxelZ, bool a_clampToImageSize = true) const
    { 
        getPixelLocation(a_texCoord, a_voxelX, a_voxelY, a_clampToImageSize);
        a_voxelZ = 0;
    }

    //! This method retrieves the voxel location from a texture coordinate.
    virtual void getVoxelLocationInterpolated(const cVector3d& a_texCoord, double& a_voxelX, double& a_voxelY, double& a_voxelZ, bool a_clampToImageSize = true) const
    {
        getPixelLocationInterpolated(a_texCoord, a_voxelX, a_voxelY, a_clampToImageSize);
        a_voxelZ = 0.0;
    }

    //! This method returns the color of an image voxel at location (x,y,z).
    virtual bool getVoxelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z,
        cColorb& a_color) const 
    {
        return (getPixelColor(a_x, a_y, a_color));
    }

    //! This method returns the interpolated color of an image voxel at location (x,y,z).
    virtual bool getVoxelColorInterpolated(const double a_x,
        const double a_y,
        const double a_z,
        cColorb& a_color) const
    {
        return (getPixelColorInterpolated(a_x, a_y, a_color));
    }

    //! This method returns the interpolated color of an image voxel at location (x,y,z).
    virtual bool getVoxelColorInterpolated(const double a_x,
        const double a_y,
        const double a_z,
        cColorf& a_color) const
    {
        return (getPixelColorInterpolated(a_x, a_y, a_color));
    }

    //! This method sets the color of an image voxel at location (x,y,z).
    virtual void setVoxelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z,
        const cColorb& a_color)
    { 
        setPixelColor(a_x, a_y, a_color);
    }

    //! This method sets the color of an image voxel at location (x,y,z).
    virtual void setVoxelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z,
        const unsigned char a_r,
        const unsigned char a_g,
        const unsigned char a_b)
    { 
        cColorb color(a_r, a_g, a_b);
        setVoxelColor(a_x, a_y, a_z, color);
    }

    //! This method sets the gray level of an image voxel at location (x,y,z).
    virtual void setVoxelColor(const unsigned int a_x,
        const unsigned int a_y,
        const unsigned int a_z,
        const unsigned char a_grayLevel)
    {
        setPixelColor(a_x, a_y, a_grayLevel);
    }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - COPYING DATA:
    //--------------------------------------------------------------------------

public:

    //! This method copies a section of this current image to a destination image.
    void copyTo(const unsigned int a_sourcePosX,
        const unsigned int a_sourcePosY,
        const unsigned int a_sourceSizeX,
        const unsigned int a_sourceSizeY,
        cImagePtr a_destImage,
        const unsigned int a_destPosX = 0,
        const unsigned int a_destPosY = 0);

    //! This method copies the entire image to a destination image.
    void copyTo(cImagePtr a_destImage,
        const unsigned int a_destPosX = 0,
        const unsigned int a_destPosY = 0);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MULTIMAGE SELECTION:
    //--------------------------------------------------------------------------

public:

    //! This method returns the index number of the current image. For cImage objects, the value is always 0.
    virtual unsigned long getCurrentIndex() 
    { 
        return (0);
    }

    //! This method sets the current image. For cImage objects, this value is always 0.
    virtual bool selectImage(unsigned long a_index)
    { 
        if (a_index == 0)
        {
            return(C_SUCCESS);
        }
        else
        {
            return (C_ERROR);
        }
    }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - FILES:
    //--------------------------------------------------------------------------

public:

    //! This method loads an image file by passing a filename as argument.
    virtual bool loadFromFile(const std::string& a_filename);

    //! This method saves an image file by passing a filename as argument.
    virtual bool saveToFile(const std::string& a_filename);

    //! This method returns the filename from which this image was last loaded or saved.
    std::string getFilename() const { return (m_filename); }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Returned color when accessing pixels located outside of the image.
    cColorb m_borderColor;


    //--------------------------------------------------------------------------
    // PROTECTED  METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method initializes all member variables.
    void defaults();

    //! This method deletes memory and rid ourselves of any image previously stored.
    void cleanup();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------


protected:

    //! Image filename.
    std::string m_filename;

    //! Width in pixels of the current image.
    unsigned int m_width;

    //! Height in pixels of the current image.
    unsigned int m_height;

    //! Pixel format of the image (GL_RGB, GL_RGBA, GL_LUMINANCE).
    GLenum m_format;

    //! Pixel data type. (GL_UNSIGNED_BYTE, GL_UNSIGNED_INT).
    GLenum m_type;

    //! Number of bytes per pixel.
    unsigned int m_bytesPerPixel;

    //! The image data itself.
    unsigned char* m_data;

    //! Size of current image in bytes.
    unsigned int m_memorySize;

    //! If __true__, then the image has been allocated in memory, __false__ otherwise.
    bool m_allocated;

    //! If __true__, then this object actually performed the memory allocation for this object.
    bool m_responsibleForMemoryAllocation;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
