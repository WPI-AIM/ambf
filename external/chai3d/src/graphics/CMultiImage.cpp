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
    \version   3.2.0 $Rev: 2149 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "CMultiImage.h"
//------------------------------------------------------------------------------
#include <iomanip>
#include <iostream>
#include <sstream>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Default constructor of cMultiImage.
*/
//==============================================================================
cMultiImage::cMultiImage() : cImage()
{
    // init internal variables
    defaults();
}


//==============================================================================
/*!
    Destructor of cMultiImage.
*/
//==============================================================================
cMultiImage::~cMultiImage()
{
    // clean up memory
    cleanup();
}


//==============================================================================
/*!
    This method initializes internal variables.
*/
//==============================================================================
void cMultiImage::defaults()
{
    m_imageCount   = 0;
    m_currentIndex = 0;
    m_array        = NULL;

    cImage::defaults();
}


//==============================================================================
/*!
    This method frees memory that was used for image data, and re-initialize
    internal variables.
*/
//==============================================================================
void cMultiImage::cleanup()
{
    // delete data
    if (m_responsibleForMemoryAllocation)
    {
        delete [] m_array;
        m_array = NULL;
        m_data  = NULL;
    }

    // delete parent class data
    cImage::cleanup();

    // reset to default values
    defaults();
}


//==============================================================================
/*!
    Allocate a new image by defining its width, height and pixel format.

    \param  a_width   Width of images.
    \param  a_height  Height of images.
    \param  a_slices  Number of images in the set.
    \param  a_format  Pixel format. Accepted values are: GL_LUMINANCE, GL_RGB, GL_RGBA
    \param  a_type    Pixel type. Accepted values are: GL_UNSIGNED_BYTE, GL_UNSIGNED_INT
*/
//==============================================================================
bool cMultiImage::allocate(const unsigned int a_width,
                           const unsigned int a_height,
                           const unsigned int a_slices,
                           const GLenum a_format,
                           const GLenum a_type)
{
    // verify size makes sense
    if (a_width <= 0 || a_height <= 0 || a_slices <= 0)
    {
        // invalid image set geometry
        return (false);
    }

    // verify the memory requirements for given format
    int bytesPerPixel = queryBytesPerPixel(a_format, a_type);
    if (bytesPerPixel < 1)
    {
        // format not valid
        return (false);
    }

    // allocate memory
    m_width         = a_width;
    m_height        = a_height;
    m_bytesPerPixel = bytesPerPixel;
    m_format        = a_format;
    m_type          = a_type;
    m_memorySize    = m_width * m_height * m_bytesPerPixel;
    m_imageCount    = a_slices;

    // delete current image data
    delete [] m_array;

    // allocated new image data
    m_array = new unsigned char[m_imageCount * m_memorySize];

    // check if memory has been allocated, otherwise cleanup
    if (m_array == NULL)
    {
        // allocation failed
        cleanup();
        return (false);
    }
    else
    {
        // image data set has been allocated
        m_allocated = true;
        m_responsibleForMemoryAllocation = true;
    }

    // clear all images
    for (unsigned int s=0; s<m_imageCount; s++)
    {
        selectImage(s);
        clear();
    }

    // point current image to first frame
    m_currentIndex = 0;
    m_data         = m_array;

    // success
    return (true);
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \return Pointer to new object.
*/
//==============================================================================
cMultiImagePtr cMultiImage::copy()
{
    // allocate new image
    cMultiImagePtr multiImage = cMultiImage::create();
    multiImage->allocate(m_width, m_height, (unsigned int)m_imageCount, m_format, m_type);

    // copy image data
    unsigned char* dstData = multiImage->getArray();
    unsigned char* srcData = getArray();
    unsigned int size = (unsigned int)(m_memorySize * m_imageCount);
    memcpy(dstData, srcData, size);

    // return new image
    return (multiImage);
}


//==============================================================================
/*!
    This method converts all images in the set into a new pixel format.

    \param  a_newFormat  New desired pixel format: GL_LUMINANCE, GL_RGB, GL_RGBA.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::convert(const unsigned int a_newFormat)
{
    // sanity check
    if (m_imageCount == 0)
    {
        return (false);
    }

    // check if object needs conversion
    if (a_newFormat == m_format)
    {
        return (true);
    }

    // check new data format
    int bytesPerPixel = queryBytesPerPixel(a_newFormat, m_type);
    if (bytesPerPixel < 1)
    {
        // format not valid
        return (false);
    }

    // allocate memory for new image
    unsigned int memorySize = m_width * m_height * bytesPerPixel;
    unsigned char* array = new unsigned char[m_imageCount * memorySize];

    // sanity check
    if (array == NULL)
    {
        return (false);
    }

    // for all image, convert and copy
    for (unsigned long i=0; i<m_imageCount; i++)
    {
        // create temporary image
        cImagePtr image = cImage::create();
        image->allocate(m_width, m_height, m_format, m_type);

        // select image
        selectImage(i);

        // copy image to temporary image
        copyTo(image);

        // convert temporary image
        if (image->convert(a_newFormat) == false)
        {
            // if error occurs, cleanup and exit
            delete [] array;
            return (false);
        }

        // copy data to new array
        unsigned char* dataSrc = image->getData();
        unsigned char* dataDst = array + i * memorySize;
        memcpy(dataDst, dataSrc, memorySize);
    }

    // release old data and update with new image settings
    delete [] m_array;
    m_array = array;
    m_format = a_newFormat;
    m_data = m_array;
    m_memorySize = memorySize;
    m_currentIndex = 0;

    // return success
    return (true);
}


//==============================================================================
/*!
    This method select the image from the set that will be displayed by the
    \ref cImage parent object.

    \return __true__ if operation succeeds, __false_otherwise.
*/
//==============================================================================
bool cMultiImage::selectImage(unsigned long a_index)
{
    if (a_index >= m_imageCount)
    {
        return (false);
    }
    else
    {
        m_data         = m_array + a_index*m_memorySize;
        m_currentIndex = a_index;

        return (true);
    }
}


//==============================================================================
/*!
    This method adds an image to the image set. If it is the first image, it
    will define the properties of the whole set. Otherwise, this method checks
    that the new image matches the properties of the images already in the set.
    If the image properties do not match, the image is ignored.

    \param  a_image  The image to be added to the image set.
    \param  a_index  The index in the set where the image should be added.
                     By default, the image is added at the end of the set.

    \return  __true__ if insertion is successful, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::addImage(cImage &a_image,
                           unsigned long a_index)
{
    // check index validity
    if (a_index != (unsigned long)-1 &&
        a_index >= m_imageCount &&
        m_imageCount > 0)
    {
        return (false);
    }

    // check image quality
    if (!a_image.isInitialized())
    {
        return (false);
    }

    // special case: first image in set
    if (m_imageCount == 0)
    {
        // update image properties
        m_bytesPerPixel = a_image.getBytesPerPixel();
        m_width         = a_image.getWidth();
        m_height        = a_image.getHeight();
        m_format        = a_image.getFormat();
        m_type          = a_image.getType();
        m_memorySize    = m_width * m_height * m_bytesPerPixel;
    }

    // check image state, geometry and format
    else if (a_image.getWidth()  != m_width  ||
             a_image.getHeight() != m_height ||
             a_image.getFormat() != m_format ||
             a_image.getType()   != m_type)
    {
        return (false);
    }

    // reallocate array
    unsigned char* array = new unsigned char[(m_imageCount+1) * m_memorySize];

    // default case: add image at the end
    if (a_index == (unsigned long)-1)
        a_index = m_imageCount;

    // copy first half of existing data set
    std::copy(m_array, m_array+a_index*m_memorySize, array);

    // copy second half of existing data set
    std::copy(m_array+a_index*m_memorySize, m_array+(m_imageCount * m_memorySize), array+(a_index+1)*m_memorySize);

    // copy new image data to data set
    unsigned char* img = a_image.getData();
    std::copy(img, img+m_memorySize, array+a_index*m_memorySize);

    // delete old array and reassign
    unsigned char *tmp = m_array;
    m_array = array;
    m_data  = m_array + m_currentIndex * m_memorySize;
    delete [] tmp;

    // image data set has been allocated
    m_allocated = true;
    m_responsibleForMemoryAllocation = true;

    // adjust image count
    m_imageCount += 1;

    return (true);
}


//==============================================================================
/*!
    This method adds an image to a preallocated image set. The set must be
    preallocated by calling \ref loadFromFiles(). This method checks that the
    new image matches the properties of the images already in the set. If the
    image properties do not match, the image is not loaded and an error is returned.

    \param  a_image  The image to be added to the image set.
    \param  a_index  The index in the set where the image should be added.
                     By default, the image is added at the end of the set.

    \return __true__ if the insertion is successful, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::addImagePrealloc(cImage &a_image,
                                   unsigned long a_index)
{
    // check index validity
    if (a_index != (unsigned long)-1 &&
        a_index >= m_imageCount &&
        m_imageCount > 0)
    {
        return (false);
    }

    // check image quality
    if (!a_image.isInitialized())
    {
        return (false);
    }

    // check image state, geometry and format
    if (a_image.getWidth()  != m_width  ||
        a_image.getHeight() != m_height ||
        a_image.getFormat() != m_format ||
        a_image.getType()   != m_type)
    {
        return (false);
    }

    // copy new image data to data set
    unsigned char* img = a_image.getData();
    std::copy(img, img+m_memorySize, m_array+a_index*m_memorySize);

    return (true);
}


//==============================================================================
/*!
    This method removes an image from the image set.

    \param  a_index  Index of the image to remove from the set.

    \return __true__ if the removal succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::removeImage(unsigned long a_index)
{
    // check index validity
    if (a_index >= m_imageCount)
    {
        return (false);
    }

    // reallocate array
    unsigned char* array = new unsigned char[(m_imageCount-1) * m_memorySize];

    // copy first half of existing data set
    std::copy(m_array, m_array+(a_index)*m_memorySize, array);

    // copy second half of existing data set
    std::copy(m_array+(a_index+1)*m_memorySize, m_array+(m_imageCount * m_memorySize), array+a_index*m_memorySize);

    // delete old array and reassign
    unsigned char *tmp = m_array;
    m_array = array;
    m_data  = m_array + m_currentIndex * m_memorySize;
    delete [] tmp;

    // adjust image count
    m_imageCount -= 1;

    // check is selected image has not been deleted
    if (m_currentIndex >= m_imageCount)
    {
        m_currentIndex = 0;
    }

    return (true);
}


//==============================================================================
/*!
    This method returns the nearest voxel from a texture coordinate.

    \param  a_texCoord  Texture coordinate.
    \param  a_voxelX    Return value for voxel coordinate X.
    \param  a_voxelY    Return value for voxel coordinate Y.
    \param  a_voxelZ    Return value for voxel coordinate Z.
    \param  a_clampToImageSize  If __true__ then pixel value is clamped to image size.
*/
//==============================================================================
void cMultiImage::getVoxelLocation(const cVector3d& a_texCoord,
                                   int& a_voxelX,
                                   int& a_voxelY,
                                   int& a_voxelZ,
                                   bool a_clampToImageSize) const
{
    double maxZ = (double)(m_imageCount - 1);
    double pz = (double)(m_imageCount) * a_texCoord.z();

    if (a_clampToImageSize)
    {
        a_voxelZ = (int)(cClamp(pz, 0.0, maxZ));
    }
    else
    {
        a_voxelZ = (int)(floor(pz));
    }

    return (getPixelLocation(a_texCoord, a_voxelX, a_voxelY, a_clampToImageSize));
}


//==============================================================================
/*!
    This method retrieves the voxel location from a texture coordinate.

    \param  a_texCoord  Texture coordinate.
    \param  a_voxelX    Return value for voxel coordinate X.
    \param  a_voxelY    Return value for voxel coordinate Y.
    \param  a_voxelZ    Return value for voxel coordinate Z.
    \param  a_clampToImageSize  If __true__ then pixel value is clamped to image size.
*/
//==============================================================================
void cMultiImage::getVoxelLocationInterpolated(const cVector3d& a_texCoord,
                                               double& a_voxelX,
                                               double& a_voxelY,
                                               double& a_voxelZ, 
                                               bool a_clampToImageSize) const
{
    double maxZ = (double)(m_imageCount - 1);
    double pz = (double)(m_imageCount) * a_texCoord.z();

    if (a_clampToImageSize)
    {
        a_voxelZ = cClamp(pz, 0.0, maxZ);
    }
    else
    {
        a_voxelZ = pz;
    }
    return (getPixelLocationInterpolated(a_texCoord, a_voxelX, a_voxelY, a_clampToImageSize));
}


//==============================================================================
/*!
    This method returns the color of a voxel by passing its x, y and z coordinates.

    \param  a_x      X coordinate of the voxel.
    \param  a_y      Y coordinate of the voxel.
    \param  a_z      Z coordinate of the voxel.
    \param  a_color  Return color of the voxel.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::getVoxelColor(const unsigned int a_x,
                                const unsigned int a_y,
                                const unsigned int a_z,
                                cColorb& a_color) const
{
    if ((a_x < ((unsigned int)(m_width))) && (a_y < ((unsigned int)(m_height))) && (a_z < ((unsigned int)(m_imageCount))))
    {
        // format: RGB
        if (m_format == GL_RGB)
        {
            unsigned int index = 3 * (a_x + a_y * m_width + a_z * m_width * m_height);
            a_color.set(m_array[index],
                        m_array[index + 1],
                        m_array[index + 2]);
            return (true);
        }

        // format: RGBA
        else if (m_format == GL_RGBA)
        {
            unsigned int index = 4 * (a_x + a_y * m_width + a_z * m_width * m_height);
            int* color = (int*)a_color.getData();
            int* data = (int*)&(m_array[index]);
            *color = *data;
            return (true);
        }

        // format: LUMINANCE
        else if (m_format == GL_LUMINANCE)
        {
            unsigned int index = a_x + a_y * m_width + a_z * m_width * m_height;
            unsigned char l = m_array[index];
            a_color.set(l, l, l, l);
            return (true);
        }

        // format not defined
        else
        {
            a_color = m_borderColor;
            return (false);
        }
    }
    else
    {
        a_color = m_borderColor;
        return (false);
    }
}


//==============================================================================
/*!
    This method returns the color of a voxel by passing its x, y and z coordinates.

    \param  a_x      X coordinate of the voxel.
    \param  a_y      Y coordinate of the voxel.
    \param  a_z      Z coordinate of the voxel.
    \param  a_color  Return color of the voxel.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::getVoxelColor(const unsigned int a_x,
                                const unsigned int a_y,
                                const unsigned int a_z,
                                cColorf& a_color) const
{
    cColorb color;
    bool result = getVoxelColor(a_x, a_y, a_z, color);
    a_color = color.getColorf();
    return (result);
}


//==============================================================================
/*!
    This method returns the interpolated color of an image voxel at
    location (x,y,z).

    \param  a_x      X coordinate of the voxel.
    \param  a_y      Y coordinate of the voxel.
    \param  a_z      Z coordinate of the voxel.
    \param  a_color  Return color of the voxel.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::getVoxelColorInterpolated(const double a_x,
                                            const double a_y,
                                            const double a_z,
                                            cColorb& a_color) const
{
    // compute fractional and integral parts of pixel position
    double dpx, dpy, dpz;
    double tx = modf (a_x , &dpx);
    double ty = modf (a_y , &dpy);
    double tz = modf (a_z , &dpz);

    int px = (int)dpx;
    int py = (int)dpy;
    int pz = (int)dpz;

    // retrieve color voxels
    cColorb c000;
    cColorb c100;
    cColorb c010;
    cColorb c110;
    cColorb c001;
    cColorb c101;
    cColorb c011;
    cColorb c111;

    bool result000 = getVoxelColor(px,   py,   pz,   c000);
    bool result100 = getVoxelColor(px+1, py,   pz,   c100);
    bool result010 = getVoxelColor(px,   py+1, pz,   c010);
    bool result110 = getVoxelColor(px+1, py+1, pz,   c110);

    bool result001 = getVoxelColor(px,   py,   pz+1, c001);
    bool result101 = getVoxelColor(px+1, py,   pz+1, c101);
    bool result011 = getVoxelColor(px,   py+1, pz+1, c011);
    bool result111 = getVoxelColor(px+1, py+1, pz+1, c111);

    // interpolate color
    double a00 = c000[0] * (1.0 - tx) + c100[0] * tx;
    double a10 = c000[1] * (1.0 - tx) + c100[1] * tx;
    double a20 = c000[2] * (1.0 - tx) + c100[2] * tx;
    double a30 = c000[3] * (1.0 - tx) + c100[3] * tx;

    double b00 = c010[0] * (1.0 - tx) + c110[0] * tx;
    double b10 = c010[1] * (1.0 - tx) + c110[1] * tx;
    double b20 = c010[2] * (1.0 - tx) + c110[2] * tx;
    double b30 = c010[3] * (1.0 - tx) + c110[3] * tx;

    double e0 = a00 * (1.0 - ty) + b00 * ty;
    double e1 = a10 * (1.0 - ty) + b10 * ty;
    double e2 = a20 * (1.0 - ty) + b20 * ty;
    double e3 = a30 * (1.0 - ty) + b30 * ty;

    double f0 = a00 * (1.0 - ty) + b00 * ty;
    double f1 = a10 * (1.0 - ty) + b10 * ty;
    double f2 = a20 * (1.0 - ty) + b20 * ty;
    double f3 = a30 * (1.0 - ty) + b30 * ty;

    a_color[0] = (int)(e0 * (1.0 - tz) + f0 * tz);
    a_color[1] = (int)(e1 * (1.0 - tz) + f1 * tz);
    a_color[2] = (int)(e2 * (1.0 - tz) + f2 * tz);
    a_color[3] = (int)(e3 * (1.0 - tz) + f3 * tz);

    // return result
    return (result000 || result100 || result010 || result110 ||
            result001 || result101 || result011 || result111);
}


//==============================================================================
/*!
    This method returns the interpolated color of an image voxel at
    location (x,y,z).

    \param  a_x      X coordinate of the voxel.
    \param  a_y      Y coordinate of the voxel.
    \param  a_z      Z coordinate of the voxel.
    \param  a_color  Return color of the voxel.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::getVoxelColorInterpolated(const double a_x,
                                            const double a_y,
                                            const double a_z,
                                            cColorf& a_color) const
{
    // compute fractional and integral parts of pixel position
    double dpx, dpy, dpz;
    double tx = modf (a_x , &dpx);
    double ty = modf (a_y , &dpy);
    double tz = modf (a_z , &dpz);

    int px = (int)dpx;
    int py = (int)dpy;
    int pz = (int)dpz;

    // retrieve color voxels
    cColorf c000;
    cColorf c100;
    cColorf c010;
    cColorf c110;
    cColorf c001;
    cColorf c101;
    cColorf c011;
    cColorf c111;

    bool result000 = getVoxelColor(px,   py,   pz,   c000);
    bool result100 = getVoxelColor(px+1, py,   pz,   c100);
    bool result010 = getVoxelColor(px,   py+1, pz,   c010);
    bool result110 = getVoxelColor(px+1, py+1, pz,   c110);

    bool result001 = getVoxelColor(px,   py,   pz+1, c001);
    bool result101 = getVoxelColor(px+1, py,   pz+1, c101);
    bool result011 = getVoxelColor(px,   py+1, pz+1, c011);
    bool result111 = getVoxelColor(px+1, py+1, pz+1, c111);

    // interpolate color
    double a00 = c000[0] * (1.0 - tx) + c100[0] * tx;
    double a10 = c000[1] * (1.0 - tx) + c100[1] * tx;
    double a20 = c000[2] * (1.0 - tx) + c100[2] * tx;
    double a30 = c000[3] * (1.0 - tx) + c100[3] * tx;

    double b00 = c010[0] * (1.0 - tx) + c110[0] * tx;
    double b10 = c010[1] * (1.0 - tx) + c110[1] * tx;
    double b20 = c010[2] * (1.0 - tx) + c110[2] * tx;
    double b30 = c010[3] * (1.0 - tx) + c110[3] * tx;

    double e0 = a00 * (1.0 - ty) + b00 * ty;
    double e1 = a10 * (1.0 - ty) + b10 * ty;
    double e2 = a20 * (1.0 - ty) + b20 * ty;
    double e3 = a30 * (1.0 - ty) + b30 * ty;

    double f0 = a00 * (1.0 - ty) + b00 * ty;
    double f1 = a10 * (1.0 - ty) + b10 * ty;
    double f2 = a20 * (1.0 - ty) + b20 * ty;
    double f3 = a30 * (1.0 - ty) + b30 * ty;

    a_color[0] = (GLfloat)(e0 * (1.0 - tz) + f0 * tz);
    a_color[1] = (GLfloat)(e1 * (1.0 - tz) + f1 * tz);
    a_color[2] = (GLfloat)(e2 * (1.0 - tz) + f2 * tz);
    a_color[3] = (GLfloat)(e3 * (1.0 - tz) + f3 * tz);

    // return result
    return (result000 || result100 || result010 || result110 ||
            result001 || result101 || result011 || result111);
}


//==============================================================================
/*!
    This method sets the color of a voxel.

    \param  a_x      X coordinate of the voxel.
    \param  a_y      Y coordinate of the voxel.
    \param  a_z      Z coordinate of the voxel.
    \param  a_color  New color of the voxel.
*/
//==============================================================================
void cMultiImage::setVoxelColor(const unsigned int a_x,
                                const unsigned int a_y,
                                const unsigned int a_z,
                                const cColorb& a_color)
{
    // check if image exists
    if (!m_allocated) { return; }

    if ((a_x < ((unsigned int)(m_width))) && (a_y < ((unsigned int)(m_height))) && (a_z < ((unsigned int)(m_imageCount))))
    {
        // format: RGB
        if (m_format == GL_RGB)
        {
            unsigned int index = 3 * (a_x + a_y * m_width + a_z * m_width * m_height);
            unsigned char* data = (unsigned char*)m_array;
            data[index]   = a_color.getR();
            data[index+1] = a_color.getG();
            data[index+2] = a_color.getB();
        }

        // format: RGBA
        else if (m_format == GL_RGBA)
        {
            unsigned int index = (a_x + a_y * m_width + a_z * m_width * m_height);
            int* data = (int*)m_array;
            int* color  = (int*)a_color.getData();
            data[index] = *color;
        }

        // format: LUMINANCE
        else if (m_format == GL_LUMINANCE)
        {
            unsigned int index  = (a_x + a_y * m_width + a_z * m_width * m_height);
            unsigned char* data = (unsigned char*)m_array;
            data[index] = a_color.getA();
        }
    }
}


//==============================================================================
/*!
    This method sets the color of a voxel.

    \param  a_x          X coordinate of the voxel.
    \param  a_y          Y coordinate of the voxel.
    \param  a_z          Z coordinate of the voxel.
    \param  a_grayLevel  New luminance value of the voxel.
*/
//==============================================================================
void cMultiImage::setVoxelColor(const unsigned int a_x,
                                const unsigned int a_y,
                                const unsigned int a_z,
                                const unsigned char a_grayLevel)
{
    // check if image exists
    if (!m_allocated) { return; }

    if ((a_x < ((unsigned int)(m_width))) && (a_y < ((unsigned int)(m_height))) && (a_z < ((unsigned int)(m_imageCount))))
    {
        // format: RGB
        if (m_format == GL_RGB)
        {
            unsigned int index = 3 * (a_x + a_y * m_width + a_z * m_width * m_height);
            unsigned char* data = (unsigned char*)m_array;
            data[index]   = a_grayLevel;
            data[index+1] = a_grayLevel;
            data[index+2] = a_grayLevel;
        }

        // format: RGBA
        else if (m_format == GL_RGBA)
        {
            unsigned int index = 4 * (a_x + a_y * m_width + a_z * m_width * m_height);
            unsigned char* data = (unsigned char*)m_array;
            data[index]   = a_grayLevel;
            data[index+1] = a_grayLevel;
            data[index+2] = a_grayLevel;
            data[index+3] = a_grayLevel;
        }

        // format: LUMINANCE
        else if (m_format == GL_LUMINANCE)
        {
            unsigned int index  = (a_x + a_y * m_width + a_z * m_width * m_height);
            unsigned char* data = (unsigned char*)m_array;
            data[index] = a_grayLevel;
        }
    }
}


//==============================================================================
/*!
    This method defines a voxel color to be transparent for all images in the
    set. If the images are not in GL_RGBA format, they are first converted in
    order to enable pixel transparency capabilities.

    \param  a_color              Selected pixel color.
    \param  a_transparencyLevel  Transparency level.
*/
//==============================================================================
void cMultiImage::setTransparentColor(const cColorb &a_color,
                                      const unsigned char a_transparencyLevel)
{
    // verify format, convert otherwise
    if (m_format != GL_RGBA)
    {
        convert(GL_RGBA);
    }

    // process all images one-by-one
    for (unsigned long i=0; i<m_imageCount; i++)
    {
        cImage image;
        image.setData(m_array + i*m_memorySize, m_memorySize, false);
        image.setProperties(m_width, m_height, m_format, m_type);
        image.setTransparentColor(a_color, a_transparencyLevel);
    }
}


//==============================================================================
/*!
    This method assigns a transparent value to all pixels of this image set.
    If the images are not in GL_RGBA format, they are first converted in order
    to enable pixel transparency capabilities.

    \param  a_transparencyLevel  Transparency level.
*/
//==============================================================================
void cMultiImage::setTransparency(const unsigned char a_transparencyLevel)
{
    // verify format, convert otherwise
    if (m_format != GL_RGBA)
    {
        convert(GL_RGBA);
    }

    // process all images one-by-one
    for (unsigned long i=0; i<m_imageCount; i++)
    {
        cImage image;
        image.setData(m_array + i*m_memorySize, m_memorySize, false);
        image.setProperties(m_width, m_height, m_format, m_type);
        image.setTransparency(a_transparencyLevel);
    }
}


//==============================================================================
/*!
    This method returns a pointer to voxel memory data.

    \param  a_x  X coordinate of the voxel.
    \param  a_y  Y coordinate of the voxel.
    \param  a_z  Z coordinate of the voxel.

    \return Pointer to voxel in memory.
*/
//==============================================================================
unsigned char* cMultiImage::getVoxelData(const unsigned int a_x,
    const unsigned int a_y,
    const unsigned int a_z) const
{
    // check if image exists
    if (!m_allocated) { return (NULL); }

    if ((a_x < ((unsigned int)(m_width))) && (a_y < ((unsigned int)(m_height))) && (a_z < ((unsigned int)(m_imageCount))))
    {
        // format: RGB
        if (m_format == GL_RGB)
        {
            unsigned int index = 3 * (a_x + a_y * m_width + a_z * m_width * m_height);
            unsigned char* data = &m_array[index];
            return (data);
        }

        // format: RGBA
        else if (m_format == GL_RGBA)
        {
            unsigned int index = 4 * (a_x + a_y * m_width + a_z * m_width * m_height);
            unsigned char* data = &m_array[index];
            return (data);
        }

        // format: LUMINANCE
        else if (m_format == GL_LUMINANCE)
        {
            unsigned int index  = (a_x + a_y * m_width + a_z * m_width * m_height);
            unsigned char* data = &m_array[index];
            return (data);
        }
    }

    return (NULL);
}


//==============================================================================
/*!
    This method flips all images horizontally.
*/
//==============================================================================
void cMultiImage::flipHorizontal()
{
    // process all images one-by-one
    for (unsigned long i=0; i<m_imageCount; i++)
    {
        cImage image;
        image.setData(m_array + i*m_memorySize, m_memorySize, false);
        image.setProperties(m_width, m_height, m_format, m_type);
        image.flipHorizontal();
    }
}


//==============================================================================
/*!
    This method loads an image from the specified file. The method returns
    __true__ if all goes well. Note that regardless of whether it succeeds,
    this over-writes any image that had previously been loaded by this object.

    \param  a_filename  Image filename.
    \return __true__ if file loaded successfully, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::loadFromFile(const string& a_filename)
{
    // cleanup previous set
    cleanup();

    // add image
    return (addFromFile(a_filename));
}


//==============================================================================
/*!
    This method loads a set of images from a set of files. The filenames are
    defined by the basename, a numeric index and the extension. The files are
    loaded chronologically. The numeric index in the file names match the number
    of digits of the a_max argument. The first image defines the properties of
    the whole set. Each subsequent image must match the properties of the first
    image, otherwise it will be ignored. This routine erases and replaces any
    previous content. \n\n

    E.g. \ref loadFromFiles("img", "png", 18) will attempt to load:
    "img0.png",
    "img00.png",
    "img1.png",
    "img01.png",
    ...,
    "img10.png",
    "img11.png",
    ...

    \param  a_basename   The common path and filename component of the files.
    \param  a_extension  The images file extension.
    \param  a_max        The maximum number of files to search for.

    \return  The number of images loaded into the set.
*/
//==============================================================================
int cMultiImage::loadFromFiles(const std::string& a_basename,
                               const std::string& a_extension,
                               unsigned long a_max)
{
    vector<string> filename;
    int nd = cNumDigits((int)a_max);

    // generate vector of possible filenames
    bool flagFilesFound = false;
    for (unsigned int i=0; i<=a_max; i++)
    {
        for (int d=min(cNumDigits(i),nd); d<=nd; d++)
        {
            ostringstream name;
            name << a_basename << setw(d) << setfill('0') << i << "." << a_extension;

            // test if file actually exists
            if (FILE *file = fopen(name.str().c_str(), "r"))
            {
                // if file exists, add it to vector
                filename.push_back(name.str());
                fclose(file);
                flagFilesFound = true;
            }
        }
    }

    if (flagFilesFound)
    {
        return (loadFromFiles(filename));
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    This method loads a set of images from a set of files. The filenames are
    contained in a vector, and the files are loaded in the order they are stored
    in the vector.

    The first image defines the properties of the whole set. Each subsequent image
    must match the properties of the first image, otherwise it will be ignored.
    This routine erases and replaces any previous content.

    \param  a_filename  The vector containing the filenames.

    \return The number of images actually loaded in the set.
*/
//==============================================================================
int cMultiImage::loadFromFiles(const std::vector<std::string>& a_filename)
{
    int loaded = 0;

    // sanity check
    if (a_filename.size() == 0) return (0);

    // cleanup previous set
    cleanup();

    // load first image to get parameters
    if (!addFromFile(a_filename[0]))
        return false;

    // set total image count
    m_imageCount = (unsigned long)(a_filename.size());

    // pre-allocate array for all images
    unsigned char* array = new unsigned char[m_imageCount * m_memorySize];

    // copy existing first image into reallocated array
    std::copy(m_array, m_array+m_memorySize, array);

    // delete old array and reassign
    unsigned char *tmp = m_array;
    m_array = array;
    m_data  = m_array + m_currentIndex * m_memorySize;
    delete [] tmp;

    // load each following file, count those that fit
    for (unsigned int i=0; i<m_imageCount; i++)
    {
        if (addFromFilePrealloc(a_filename[i], i))
        {
            loaded++;
        }
    }

    // return number of files actually loaded
    return (loaded);
}


//==============================================================================
/*!
    This method adds an image to the image set. If it is the first image, it
    will define the properties of the whole set. Otherwise, this method checks
    that the new image matches the properties of the images already in the set.
    If the image properties do not match, the image is ignored.

    \param  a_filename  The path and filename of the image to be added to the set.
    \param  a_index     The index in the set where the image should be added.
                        By default, the image is added at the end of the set.

    \return _true__ if insertion is successful, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::addFromFile(const string& a_filename,
                              unsigned long a_index)
{
    cImage image;

    // load image from file
    if (!image.loadFromFile(a_filename))
    {
        return (false);
    }

    // add new image to data set
    return (addImage(image, a_index));
}


//==============================================================================
/*!
    This method adds an image file to a preallocated image set. The set must be
    preallocated by calling \ref loadFromFiles(). This method checks that the
    new image matches the properties of the images already in the set. If the
    image properties do not match, the image is not loaded and an error is returned.

    \param  a_filename  The path and filename of the image to be added to the set.
    \param  a_index     The index in the set where the image should be added.
                        By default, the image is added at the end of the set.

    \return __true__ if insertion is successful, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::addFromFilePrealloc(const string& a_filename,
                                      unsigned long a_index)
{
    cImage image;

    // load image from file
    if (!image.loadFromFile(a_filename))
    {
        return (false);
    }

    // add new image to data set
    return (addImagePrealloc(image, a_index));
}


//==============================================================================
/*!
    This method saves all 3D imaging data as a set of 2D images. The filenames
    are define by the basename, a numeric index and the extension. The files are
    saved chronologically and the number index is generated automatically for
    each file.

    E.g. \ref saveToFiles("img", "png")

    \param  a_basename   The common path and filename component of the files.
    \param  a_extension  The images file extension.

    \return __true__ if operation succeeded, __false__ otherwise.
*/
//==============================================================================
bool cMultiImage::saveToFiles(const std::string& a_basename, const std::string& a_extension)
{
    // sanity check
    if (a_basename == "")
    {
        return (false);
    }

    // get number of images
    unsigned int numImages = getImageCount();
    int numDigits = cNumDigits(numImages);

    // get current selected image
    int currentIndex = (int)m_currentIndex;

    // save all images
    bool success = true;
    int i=0;
    while ((i<(int)numImages) && (success))
    {
        // create image number
        string number;
        int nDigits = cNumDigits(i);
        int zeros = numDigits - nDigits;
        for (int j=0; j<zeros; j++)
        {
            number.append("0");
        }
        number.append(cStr(i));

        // create filename
        string filename = a_basename + number + "." + a_extension;

        // save file
        success = selectImage(i);
        if (success)
        {
            success = saveToFile(filename);
        }

        // next image
        i++;
    }

    // restore selected image
    selectImage(currentIndex);

    return (success);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
