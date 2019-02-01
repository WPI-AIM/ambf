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
#include "graphics/CImage.h"
#include "files/CFileImageBMP.h"
#include "files/CFileImageGIF.h"
#include "files/CFileImageJPG.h"
#include "files/CFileImagePNG.h"
#include "files/CFileImagePPM.h"
#include "files/CFileImageRAW.h"
#include "math/CMaths.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Default Constructor of cImage.
*/
//==============================================================================
cImage::cImage()
{
    // init internal variables
    defaults();
}


//==============================================================================
/*!
    Constructor of cImage. Allocates the image by passing width, height and
    pixel format.

    \param  a_width   Width of image.
    \param  a_height  Height of image.
    \param  a_format  Pixel format. Accepted values are: GL_LUMINANCE, GL_RGB, GL_RGBA.
    \param  a_type    Pixel type. Accepted values are: GL_UNSIGNED_BYTE, GL_UNSIGNED_INT.
*/
//==============================================================================
cImage::cImage(const unsigned int a_width,
               const unsigned int a_height,
               const GLenum a_format,
               const GLenum a_type)
{
    // init internal variables
    defaults();

    // allocate image
    allocate(a_width, a_height, a_format, a_type);
}


//==============================================================================
/*!
    Destructor of cImage.
*/
//==============================================================================
cImage::~cImage()
{
    // clean up memory
    cleanup();
}


//==============================================================================
/*!
    This method initializes internal variables.
*/
//==============================================================================
void cImage::defaults()
{
    // no filename defined
    m_filename          = "";

    // size of image is zero since not yet defined and allocated
    m_width             = 0;
    m_height            = 0;

    // default format is RGB
    m_format            = GL_RGB;
    m_type              = GL_UNSIGNED_BYTE;
    m_bytesPerPixel     = queryBytesPerPixel(m_format, m_type);

    // image has not yet been allocated
    m_allocated         = false;
    m_data              = NULL;
    m_memorySize        = 0;

    // this value is set to 'true' when the user calls the allocate() function.
    // however, if the user uses setData() instead to define the memory where the image
    // data is located, then its value is set to 'false', at the current object
    // will not take care of freeing the image data.
    m_responsibleForMemoryAllocation = false;

    // default border color is black
    m_borderColor.set(0x00, 0x00, 0x00, 0x00);
}


//==============================================================================
/*!
    This method frees memory that was used for image data, and re-initialize
    internal variables.
*/
//==============================================================================
void cImage::cleanup()
{
    // delete image data
    if ((m_data != NULL) && (m_responsibleForMemoryAllocation))
    {
        delete [] m_data;
    }

    // reset to default values
    defaults();
}


//==============================================================================
/*!
    This method allocates a new image by defining its width, height and pixel
    format.

    \param  a_width   Width of image
    \param  a_height  Height of image
    \param  a_format  Pixel format. Accepted values are: GL_LUMINANCE, GL_RGB, GL_RGBA
    \param  a_type    Pixel type. Accepted values are: GL_UNSIGNED_BYTE, GL_UNSIGNED_INT
*/
//==============================================================================
bool cImage::allocate(const unsigned int a_width,
                      const unsigned int a_height,
                      const GLenum a_format,
                      const GLenum a_type)
{
    // verify the memory requirements for given format
    int bytesPerPixel = queryBytesPerPixel(a_format, a_type);
    if (bytesPerPixel < 1)
    {
        // format not valid
        return (false);
    }

    // allocate memory
    m_width             = a_width;
    m_height            = a_height;
    m_bytesPerPixel     = bytesPerPixel;
    m_format            = a_format;
    m_type              = a_type;
    m_memorySize        = m_width * m_height * m_bytesPerPixel;

    // delete current image data
    if (m_data && m_responsibleForMemoryAllocation)
    {
        delete [] m_data;
    }

    // allocated new image data
    m_data = new unsigned char[m_memorySize];

    // check if memory has been allocated, otherwise cleanup
    if (m_data == NULL)
    {
        // allocation failed
        cleanup();
        return (false);
    }
    else
    {
        // image data has been allocated
        m_allocated = true;
        m_responsibleForMemoryAllocation = true;
    }

    // clear image
    clear();

    // success
    return (true);
}


//==============================================================================
/*!
    This method sets the size of image by defining the width and height.

    \param  a_width   Width of image.
    \param  a_height  Height of image.
*/
//==============================================================================
void cImage::setSize(const unsigned int a_width, const unsigned int a_height)
{
    allocate(a_width,
             a_height,
             m_format,
             m_type);
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \return Pointer to new object.
*/
//==============================================================================
cImagePtr cImage::copy()
{
    // allocate new image
    cImagePtr image = cImage::create();
    image->allocate(m_width, m_height, m_format);

    // copy image data
    copyTo(image);

    // return new image
    return (image);
}


//==============================================================================
/*!
    This method queries the number of bytes per pixel for a given format.

    \param  a_format  Pixel format of image.
    \param  a_type    Pixel data type of image.

    \return Number of bytes used for storing a single pixel.
*/
//==============================================================================
int cImage::queryBytesPerPixel(const GLenum a_format,
                               const GLenum a_type)
{
    int bytesPerPixel = -1;
    int bytesPerPixelComponent = -1;

    // verify if requested type is supported
    switch (a_type)
    {
        // type: BYTE (1 bytes)
        case GL_BYTE:
        case GL_UNSIGNED_BYTE:
        {
            bytesPerPixelComponent = 1;
        }
        break;

        // type: BYTE (1 bytes)
        case GL_SHORT:
        case GL_UNSIGNED_SHORT:
        {
            bytesPerPixelComponent = 2;
        }
        break;

        // type: BYTE (4 bytes)
        case GL_UNSIGNED_INT:
        {
            bytesPerPixelComponent = 4;
        }
        break;
    }

    // if the number of bytes per pixel is unknown, then exit.
    if (bytesPerPixelComponent == -1) { return (-1); }

    // verify if requested format is supported
    switch (a_format)
    {
        // 1 component per pixel:
        case GL_LUMINANCE:
        case GL_DEPTH_COMPONENT:
            {
                bytesPerPixel = 1 * bytesPerPixelComponent;
            }
            break;

        // 2 components per pixel:
        case GL_LUMINANCE_ALPHA:
            {
                bytesPerPixel = 2 * bytesPerPixelComponent;
            }
            break;

        // 3 components per pixel:
        case GL_RGB:
            {
                bytesPerPixel = 3 * bytesPerPixelComponent;
            }
            break;

        // 4 components per pixel:
        case GL_RGBA:
            {
                bytesPerPixel = 4 * bytesPerPixelComponent;
            }
            break;
    }

    // return result
    return (bytesPerPixel);
}


//==============================================================================
/*!
    This method converts this image into a new pixel format.

    \param  a_newFormat  New desired pixel format: GL_LUMINANCE, GL_RGB, GL_RGBA.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cImage::convert(unsigned int a_newFormat)
{
    // verify if new format is different
    if (a_newFormat == m_format) { return (true); }

    // allocate memory for converted image
    cImagePtr image = cImage::create();
    image->allocate(m_width, m_height, a_newFormat);

    // convert image by using the copy function
    copyTo(0, 0, m_width, m_height, image, 0, 0);

    // free current image
    cleanup();

    // retrieve converted image
    unsigned char* data = image->getData();
    unsigned int size   = image->getSizeInBytes();

    // assign new values to current image
    setData(data, size);
    setProperties(image->getWidth(), image->getHeight(), image->getFormat(), image->getType());

    // current object will be responsible for freeing memory
    m_responsibleForMemoryAllocation = true;

    // set data property of temp image to NULL so that we do not delete
    // allocated data memory when deleting object image
    image->setData(NULL, 0);

    // success
    return (true);
}


//==============================================================================
/*!
    This method assigns new properties including width, height and pixel format
    to the image. If the requested image properties do not match the exact amount
    of allocated data image, then the operation fails and the desired properties
    are ignored.

    \param  a_width   Width of new image.
    \param  a_height  Height of new image.
    \param  a_format  Pixel format. Accepted values are (GL_LUMINANCE, GL_RGB, GL_RGBA).
    \param  a_type    Pixel data type. (GL_UNSIGNED_BYTE, GL_UNSIGNED_INT for instance).

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cImage::setProperties(const unsigned int a_width,
                           const unsigned int a_height,
                           const GLenum a_format,
                           const GLenum a_type)
{
    // sanity check on image format
    int bytesPerPixel = queryBytesPerPixel(a_format, a_type);
    if (bytesPerPixel < 1) { return (false); }

    // compute image size
    unsigned int size = a_width * a_height * bytesPerPixel;

    // verify that image properties matches allocated memory
    if (size != m_memorySize)
    {
        return (false);
    }

    // all looks good, we can update image properties
    m_width         = a_width;
    m_height        = a_height;
    m_format        = a_format;
    m_bytesPerPixel = bytesPerPixel;

    // success
    return (true);
}


//==============================================================================
/*!
    This method copies a section of this current image to a different location
    or onto a different destination image.

    \param  a_sourcePosX   X coordinate of top left pixel to copy.
    \param  a_sourcePosY   Y coordinate of top left pixel to copy.
    \param  a_sourceSizeX  Width of area to copy.
    \param  a_sourceSizeY  Height of area to copy.
    \param  a_destImage    Destination image when area will be pasted.
    \param  a_destPosX     X coordinate of top left pixel on destination image where area is pasted.
    \param  a_destPosY     Y coordinate of top left pixel on destination image where area is pasted.
*/
//==============================================================================
void cImage::copyTo(const unsigned int a_sourcePosX,
                    const unsigned int a_sourcePosY,
                    const unsigned int a_sourceSizeX,
                    const unsigned int a_sourceSizeY,
                    cImagePtr a_destImage,
                    const unsigned int a_destPosX,
                    const unsigned int a_destPosY)
{
    // temp variables
    unsigned int i,j;

    // sanity check
    if ((!m_allocated) || (a_destImage == nullptr))
        { return; }

    if (!a_destImage->isInitialized())
        { return; }

    // if source and destination images are the same object, then we need
    // to create a copied image of the source. (overwriting problem)
    cImagePtr temp_image = cImagePtr();
    if (a_destImage->getData() == getData())
    {
        temp_image = copy();
    }

    // store values
    unsigned int src_x = a_sourcePosX;
    unsigned int src_y = a_sourcePosY;
    unsigned int src_dx = a_sourceSizeX;
    unsigned int src_dy = a_sourceSizeY;
    unsigned int dst_x = a_destPosX;
    unsigned int dst_y = a_destPosY;
    unsigned char* src_img_data = m_data;
    if (temp_image)
    {
        src_img_data = temp_image->m_data;
    }
    unsigned char* dst_img_data = a_destImage->m_data;
    unsigned int src_w = m_width;
    unsigned int src_h = m_height;
    unsigned int dst_w = a_destImage->m_width;
    unsigned int dst_h = a_destImage->m_height;

    // verifications and area clamping related to source image
    if ((src_x >= src_w) || (src_y >= src_h))
    {
        return;
    }

    unsigned int origin = 0;
    unsigned int max_src_x = src_w - src_x;
    unsigned int max_src_y = src_h - src_y;
    src_dx = cClamp(src_dx, origin, max_src_x);
    src_dy = cClamp(src_dy, origin, max_src_y);

    // verifications and area clamping related to destination image
    if ((dst_x >= dst_w) || (dst_y >= dst_h))
    {
        return;
    }

    unsigned int max_dst_x = dst_w - dst_x;
    unsigned int max_dst_y = dst_h - dst_y;
    src_dx = cClamp(src_dx, origin, max_dst_x);
    src_dy = cClamp(src_dy, origin, max_dst_y);

    // the area has now been clamped and we can safely proceed with the data copying
    unsigned int src_format = m_format;
    unsigned int dst_format = a_destImage->getFormat();

    // if source and destination carry the same format and size, and if the
    // entire image is being copied, then perform a fast copy
    if ((src_format == dst_format) &&
        (src_x == 0) && (src_y == 0) &&
        (dst_x == 0) && (dst_y == 0) &&
        (src_dx == src_w) && (src_dy == src_h))
    {
        memcpy(dst_img_data, src_img_data, m_memorySize);
    }

    // src format: RGB
    else if (src_format == GL_RGB)
    {
        // information about src image
        int src_bpp = 3;
        int src_segment_size = src_dx * src_bpp;
        int src_offset = (src_w - src_dx) * src_bpp;
        unsigned int src_index = src_bpp * (src_x + src_y * src_w);
        unsigned char* src_data = &(src_img_data[src_index]);

        // dst format: RGB
        if (dst_format == GL_RGB)
        {
            int dst_bpp = 3;
            int dst_offset = dst_w * dst_bpp;
            src_offset = src_w * src_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                memcpy(dst_data, src_data, src_segment_size);
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }

        // dst format: RGBA
        if (dst_format == GL_RGBA)
        {
            int dst_bpp = 4;
            int dst_offset = (dst_w - src_dx) * dst_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                for (j=0; j<src_dx; j++)
                {
                    *dst_data = *src_data; dst_data++; src_data++;
                    *dst_data = *src_data; dst_data++; src_data++;
                    *dst_data = *src_data; dst_data++; src_data++;
                    *dst_data = 0xff; dst_data++;
                }
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }

        // dst format: LUMINANCE
        else if (dst_format == GL_LUMINANCE)
        {
            int dst_bpp = 1;
            int dst_offset = (dst_w - src_dx) * dst_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                for (j=0; j<src_dx; j++)
                {
                    *dst_data = (unsigned char)(((unsigned short)(src_data[0]) +
                                                 (unsigned short)(src_data[1]) +
                                                 (unsigned short)(src_data[2]))  / 3);
                    dst_data++; src_data+=3;
                }
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }

        // dst format: LUMINANCE_ALPHA
        else if (dst_format == GL_LUMINANCE_ALPHA)
        {
            int dst_bpp = 2;
            int dst_offset = (dst_w - src_dx) * dst_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                for (j=0; j<src_dx; j++)
                {
                    *dst_data = (unsigned char)(((unsigned short)(src_data[0]) +
                                                 (unsigned short)(src_data[1]) +
                                                 (unsigned short)(src_data[2]))  / 3);
                    dst_data++;
                    *dst_data = 0xff;
                    dst_data++;
                    src_data+=3;
                }
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }
    }

    // src format: RGBA
    else if (src_format == GL_RGBA)
    {
        // information about src image
        int src_bpp = 4;
        int src_segment_size = src_dx * src_bpp;
        int src_offset = (src_w - src_dx) * src_bpp;
        unsigned int src_index = src_bpp * (src_x + src_y * src_w);
        unsigned char* src_data = &(src_img_data[src_index]);

        // dst format: RGB
        if (dst_format == GL_RGB)
        {
            int dst_bpp = 3;
            int dst_offset = (dst_w - src_dx) * dst_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                for (j=0; j<src_dx; j++)
                {
                    *dst_data = *src_data; dst_data++; src_data++;
                    *dst_data = *src_data; dst_data++; src_data++;
                    *dst_data = *src_data; dst_data++; src_data+=2;
                }
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }

        // dst format: RGBA
        if (dst_format == GL_RGBA)
        {
            int dst_bpp = 4;
            int dst_offset = dst_w * dst_bpp;
            src_offset = src_w * src_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                memcpy(dst_data, src_data, src_segment_size);
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }

        // dst format: LUMINANCE
        if (dst_format == GL_LUMINANCE)
        {
            int dst_bpp = 1;
            int dst_offset = (dst_w - src_dx) * dst_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                for (j=0; j<src_dx; j++)
                {
                    *dst_data = (unsigned char)(((unsigned short)(src_data[0]) +
                                                 (unsigned short)(src_data[1]) +
                                                 (unsigned short)(src_data[2]))  / 3);
                    dst_data++; src_data+=4;
                }
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }

        // dst format: LUMINANCE_ALPHA
        else if (dst_format == GL_LUMINANCE_ALPHA)
        {
            int dst_bpp = 2;
            int dst_offset = (dst_w - src_dx) * dst_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                for (j=0; j<src_dx; j++)
                {
                    *dst_data = (unsigned char)(((unsigned short)(src_data[0]) +
                                                 (unsigned short)(src_data[1]) +
                                                 (unsigned short)(src_data[2]))  / 3);
                    dst_data++;
                    *dst_data = src_data[3];
                    dst_data++;
                    src_data+=4;
                }
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }
    }

    // src format: LUMINANCE
    else if (src_format == GL_LUMINANCE)
    {
        // information about src image
        int src_bpp = 1;
        int src_segment_size = src_dx * src_bpp;
        int src_offset = (src_w - src_dx) * src_bpp;
        unsigned int src_index = src_bpp * (src_x + src_y * src_w);
        unsigned char* src_data = &(src_img_data[src_index]);

        // dst format: RGB
        if (dst_format == GL_RGB)
        {
            int dst_bpp = 3;
            int dst_offset = (dst_w - src_dx) * dst_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                for (j=0; j<src_dx; j++)
                {
                    *dst_data = *src_data; dst_data++;
                    *dst_data = *src_data; dst_data++;
                    *dst_data = *src_data; dst_data++;
                    src_data++;
                }
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }

        // dst format: RGBA
        if (dst_format == GL_RGBA)
        {
            int dst_bpp = 4;
            int dst_offset = (dst_w - src_dx) * dst_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                for (j=0; j<src_dx; j++)
                {
                    *dst_data = *src_data; dst_data++;
                    *dst_data = *src_data; dst_data++;
                    *dst_data = *src_data; dst_data++;
                    *dst_data = 0xff; dst_data++;
                    src_data++;
                }
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }

        // dst format: LUMINANCE
        if (dst_format == GL_LUMINANCE)
        {
            int dst_bpp = 1;
            int dst_offset = dst_w * dst_bpp;
            src_offset = src_w * src_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                memcpy(dst_data, src_data, src_segment_size);
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }

        // dst format: LUMINANCE_ALPHA
        if (dst_format == GL_LUMINANCE_ALPHA)
        {
            int dst_bpp = 2;
            int dst_offset = (dst_w - src_dx) * dst_bpp;
            unsigned int dst_index = dst_bpp * (dst_x + dst_y * dst_w);
            unsigned char* dst_data = &(dst_img_data[dst_index]);
            for (i=0; i<src_dy; i++)
            {
                for (j=0; j<src_dx; j++)
                {
                    *dst_data = *src_data; dst_data++;
                    *dst_data = 0xff; dst_data++;
                    src_data++;
                }
                src_data += src_offset;
                dst_data += dst_offset;
            }
        }
    }
}


//==============================================================================
/*!
    This method copies this entire image to another destination image. If the
    destination images is smaller than the source, the copied area is cropped
    before being copied.

    \param  a_destImage  Destination image when area will be pasted.
    \param  a_destPosX   X coordinate of top left pixel on destination image where area is pasted.
    \param  a_destPosY   Y coordinate of top left pixel on destination image where area is pasted.
*/
//==============================================================================
void cImage::copyTo(cImagePtr a_destImage,
                    const unsigned int a_destPosX,
                    const unsigned int a_destPosY)
{
    copyTo(0, 0, m_width, m_height, a_destImage, 0, 0);
}


//==============================================================================
/*!
    This method assigns a different memory location for the image data.
    Use with care! \n
    Make sure to call function \ref setProperties() afterwards in order to correctly
    set the dimension and pixel format of the image described by the new data.

    \param  a_data             Pointer to new image data.
    \param  a_dataSizeInBytes  Size in byte of the image data.
    \param  a_dealloc          If __true__ then this class is responsible for deallocating
                               image data when the object is deleted.
*/
//==============================================================================
void cImage::setData(unsigned char* a_data,
                     const unsigned int a_dataSizeInBytes,
                     const bool a_dealloc)
{
    // update data information
    m_data            = a_data;
    m_memorySize      = a_dataSizeInBytes;
    m_allocated       = true;
    m_responsibleForMemoryAllocation = a_dealloc;

    // verify if image dimension match, otherwise set dimensions to zero
    unsigned int size = m_width * m_height * m_bytesPerPixel;
    if (size != m_memorySize)
    {
        m_width  = 0;
        m_height = 0;
    }
}


//==============================================================================
/*!
    This method clears an image with defaults data.
*/
//==============================================================================
void cImage::clear()
{
    clear(m_borderColor);
}


//==============================================================================
/*!
    This method clears an image with a defined color passed as argument.

    \param  a_color  New color of the image.
*/
//==============================================================================
void cImage::clear(const cColorb& a_color)
{
    // check if image exists
    if (!m_allocated) { return; }

    // image size
    unsigned int size = m_width * m_height;

    // format: RGB
    if (m_format == GL_RGB)
    {
        unsigned char r = a_color.getR();
        unsigned char g = a_color.getG();
        unsigned char b = a_color.getB();
        unsigned char* data = (unsigned char*)m_data;
        unsigned int i;
        for (i=0; i<size; i++)
        {
            *data = r; data++;
            *data = g; data++;
            *data = b; data++;
        }
    }

    // format: RGBA
    else if (m_format == GL_RGBA)
    {
        int* data = (int*)m_data;
        int* color = (int*)a_color.getData();
        unsigned int i;
        for (i=0; i<size; i++)
        {
            *data = *color; data++;
        }
    }

    // format: LUMINANCE
    else if (m_format == GL_LUMINANCE)
    {
        unsigned char l = a_color.getLuminance();
        unsigned char* data = (unsigned char*)m_data;
        unsigned int i;
        for (i=0; i<size; i++)
        {
            *data = l; data++;
        }
    }
}


//==============================================================================
/*!
    This method retrieves the nearest pixel location from a texture coordinate. \n
    Each texture coordinate is a value defined between 0.0 and 1.0.

    \param  a_texCoord  Texture coordinate.
    \param  a_pixelX    Return value for pixel coordinate X.
    \param  a_pixelY    Return value for pixel coordinate Y.
    \param  a_clampToImageSize  If __true__ then pixel value is clamped to image size.
*/
//==============================================================================
void cImage::getPixelLocation(const cVector3d& a_texCoord, int& a_pixelX, int& a_pixelY, bool a_clampToImageSize) const
{
    double maxX = (double)(m_width - 1);
    double maxY = (double)(m_height - 1);

    double px = m_width * a_texCoord.x();
    double py = m_height * a_texCoord.y();

    if (a_clampToImageSize)
    {
        a_pixelX = (int)(cClamp(px, 0.0, maxX));
        a_pixelY = (int)(cClamp(py, 0.0, maxY));
    }
    else
    {
        a_pixelX = (int)(floor(px));
        a_pixelY = (int)(floor(py));
    }
}


//==============================================================================
/*!
    This method retrieves a pixel location from a texture coordinate. \n
    Each texture coordinate is a value defined between 0.0 and 1.0.

    \param  a_texCoord  Texture coordinate.
    \param  a_pixelX    Return value for pixel coordinate X.
    \param  a_pixelY    Return value for pixel coordinate Y.
    \param  a_clampToImageSize  If __true__ then pixel value is clamped to image size.
*/
//==============================================================================
void cImage::getPixelLocationInterpolated(const cVector3d& a_texCoord, double& a_pixelX, double& a_pixelY, bool a_clampToImageSize) const
{
    double maxX = (double)(m_width - 1);
    double maxY = (double)(m_height - 1);

    double px = m_width * a_texCoord.x();
    double py = m_height * a_texCoord.y();

    if (a_clampToImageSize)
    {
        a_pixelX = cClamp(px, 0.0, maxX);
        a_pixelY = cClamp(py, 0.0, maxY);
    }
    else
    {
        a_pixelX = px;
        a_pixelY = py;
    }
}


//==============================================================================
/*!
    This method returns the color of a pixel by passing its __x__ and __y__
    image coordinates.

    \param  a_x      X coordinate of the pixel.
    \param  a_y      Y coordinate of the pixel.
    \param  a_color  Return color of the pixel.
    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cImage::getPixelColor(const unsigned int a_x,
                           const unsigned int a_y,
                           cColorb& a_color) const
{
    if ((m_allocated) && (a_x < ((unsigned int)(m_width))) && (a_y < ((unsigned int)(m_height))))
    {
        // format: RGB
        if (m_format == GL_RGB)
        {
            unsigned int index = 3 * (a_x + a_y * m_width);
            a_color.set(m_data[index],
                        m_data[index+1],
                        m_data[index+2]);
            return (true);
        }

        // format: RGBA
        else if (m_format == GL_RGBA)
        {
            unsigned int index = 4 * (a_x + a_y * m_width);
            int* color = (int*)a_color.getData();
            int* data = (int*)&(m_data[index]);
            *color = *data;
            return (true);
        }

        // format: LUMINANCE
        else if (m_format == GL_LUMINANCE)
        {
            unsigned int index = a_x + a_y * m_width;
            unsigned char l = m_data[index];
            a_color.set(l, l, l);
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
    This method returns the color of a pixel by passing its __x__ and __y__
    image coordinates.

    \param  a_x      X coordinate of the pixel.
    \param  a_y      Y coordinate of the pixel.
    \param  a_color  Return color of the pixel.
    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cImage::getPixelColor(const unsigned int a_x,
                           const unsigned int a_y,
                           cColorf& a_color) const
{
    cColorb color;
    bool result = getPixelColor(a_x, a_y, color);
    a_color = color.getColorf();
    return (result);
}


//==============================================================================
/*!
    This method returns the interpolated color of an image pixel at location (x,y).

    \param  a_x      X coordinate of the pixel.
    \param  a_y      Y coordinate of the pixel.
    \param  a_color  Return color of the pixel.
    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cImage::getPixelColorInterpolated(const double a_x,
                                       const double a_y,
                                       cColorb& a_color) const
{
    // compute fractional and integral parts of pixel position
    double dpx, dpy;
    double tx = modf (a_x , &dpx);
    double ty = modf (a_y , &dpy);

    int px = (int)dpx;
    int py = (int)dpy;

    // retrieve color pixels
    cColorb c00;
    cColorb c10;
    cColorb c01;
    cColorb c11;

    bool result00 = getPixelColor((int)px,   (int)py,   c00);
    bool result10 = getPixelColor((int)px+1, (int)py,   c10);
    bool result01 = getPixelColor((int)px,   (int)py+1, c01);
    bool result11 = getPixelColor((int)px+1, (int)py+1, c11);

    // interpolate color
    double a0 = c00[0] * (1.0 - tx) + c10[0] * tx;
    double a1 = c00[1] * (1.0 - tx) + c10[1] * tx;
    double a2 = c00[2] * (1.0 - tx) + c10[2] * tx;
    double a3 = c00[3] * (1.0 - tx) + c10[3] * tx;

    double b0 = c01[0] * (1.0 - tx) + c11[0] * tx;
    double b1 = c01[1] * (1.0 - tx) + c11[1] * tx;
    double b2 = c01[2] * (1.0 - tx) + c11[2] * tx;
    double b3 = c01[3] * (1.0 - tx) + c11[3] * tx;

    a_color[0] = (int)(a0 * (1.0 - ty) + b0 * ty);
    a_color[1] = (int)(a1 * (1.0 - ty) + b1 * ty);
    a_color[2] = (int)(a2 * (1.0 - ty) + b2 * ty);
    a_color[3] = (int)(a3 * (1.0 - ty) + b3 * ty);

    // return result
    return (result00 || result10 || result01 || result11);
}


//==============================================================================
/*!
    This method returns the interpolated color of an image pixel at location (x,y).

    \param  a_x      X coordinate of the pixel.
    \param  a_y      Y coordinate of the pixel.
    \param  a_color  Return color of the pixel.
    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cImage::getPixelColorInterpolated(const double a_x,
                                       const double a_y,
                                       cColorf& a_color) const
{
    // compute fractional and integral parts of pixel position
    double dpx, dpy;
    double tx = modf (a_x , &dpx);
    double ty = modf (a_y , &dpy);

    int px = (int)dpx;
    int py = (int)dpy;

    // retrieve color pixels
    cColorf c00;
    cColorf c10;
    cColorf c01;
    cColorf c11;

    bool result00 = getPixelColor((int)px,   (int)py,   c00);
    bool result10 = getPixelColor((int)px+1, (int)py,   c10);
    bool result01 = getPixelColor((int)px,   (int)py+1, c01);
    bool result11 = getPixelColor((int)px+1, (int)py+1, c11);

    // interpolate color
    double a0 = c00[0] * (1.0 - tx) + c10[0] * tx;
    double a1 = c00[1] * (1.0 - tx) + c10[1] * tx;
    double a2 = c00[2] * (1.0 - tx) + c10[2] * tx;
    double a3 = c00[3] * (1.0 - tx) + c10[3] * tx;

    double b0 = c01[0] * (1.0 - tx) + c11[0] * tx;
    double b1 = c01[1] * (1.0 - tx) + c11[1] * tx;
    double b2 = c01[2] * (1.0 - tx) + c11[2] * tx;
    double b3 = c01[3] * (1.0 - tx) + c11[3] * tx;

    a_color[0] = (GLfloat)(a0 * (1.0 - ty) + b0 * ty);
    a_color[1] = (GLfloat)(a1 * (1.0 - ty) + b1 * ty);
    a_color[2] = (GLfloat)(a2 * (1.0 - ty) + b2 * ty);
    a_color[3] = (GLfloat)(a3 * (1.0 - ty) + b3 * ty);

    // return result
    return (result00 || result10 || result01 || result11);
}


//==============================================================================
/*!
    This method sets the color of a pixel at a desired location (x,y).

    \param  a_x      X coordinate of the image pixel.
    \param  a_y      Y coordinate of the image pixel.
    \param  a_color  New pixel color.
*/
//==============================================================================
void cImage::setPixelColor(const unsigned int a_x,
                           const unsigned int a_y,
                           const cColorb& a_color)
{
    // check if image exists
    if (!m_allocated) { return; }

    if ((a_x < ((unsigned int)(m_width))) && (a_y < ((unsigned int)(m_height))))
    {
        // format: RGB
        if (m_format == GL_RGB)
        {
            unsigned int index = 3 * (a_x + a_y * m_width);
            unsigned char* data = (unsigned char*)m_data;
            data[index]   = a_color.getR();
            data[index+1] = a_color.getG();
            data[index+2] = a_color.getB();
        }

        // format: RGBA
        else if (m_format == GL_RGBA)
        {
            unsigned int index = (a_x + a_y * m_width);
            int* data   = (int*)m_data;
            int* color  = (int*)a_color.getData();
            data[index] = *color;
        }

        // format: LUMINANCE
        else if (m_format == GL_LUMINANCE)
        {
            unsigned int index  = (a_x + a_y * m_width);
            unsigned char* data = (unsigned char*)m_data;
            data[index] = a_color.getLuminance();
        }
    }
}


//==============================================================================
/*!
    This method sets the gray level of a pixel at a desired location (x,y).

    \param  a_x          X coordinate of the pixel.
    \param  a_y          Y coordinate of the pixel.
    \param  a_grayLevel  New luminance value of the pixel.
*/
//==============================================================================
void cImage::setPixelColor(const unsigned int a_x,
                           const unsigned int a_y,
                           const unsigned char a_grayLevel)
{
    // check if image exists
    if (!m_allocated) { return; }

    if ((a_x < ((unsigned int)(m_width))) && (a_y < ((unsigned int)(m_height))))
    {
        // format: RGB
        if (m_format == GL_RGB)
        {
            unsigned int index = 3 * (a_x + a_y * m_width);
            unsigned char* data = (unsigned char*)m_data;
            data[index]   = a_grayLevel;
            data[index+1] = a_grayLevel;
            data[index+2] = a_grayLevel;
        }

        // format: RGBA
        else if (m_format == GL_RGBA)
        {
            unsigned int index = 4 * (a_x + a_y * m_width);
            unsigned char* data = (unsigned char*)m_data;
            data[index]   = a_grayLevel;
            data[index+1] = a_grayLevel;
            data[index+2] = a_grayLevel;
            data[index+3] = a_grayLevel;
        }

        // format: LUMINANCE
        else if (m_format == GL_LUMINANCE)
        {
            unsigned int index  = (a_x + a_y * m_width);
            unsigned char* data = (unsigned char*)m_data;
            data[index] = a_grayLevel;
        }
    }
}


//==============================================================================
/*!
    This method defines a color to be transparent. If the image is not in GL_RGBA
    format, it is first converted in order to enable pixel transparency
    capabilities.

    \param  a_color              Selected pixel color.
    \param  a_transparencyLevel  Transparency level.
*/
//==============================================================================
void cImage::setTransparentColor(const cColorb &a_color,
                                 const unsigned char a_transparencyLevel)
{
    // check if image exists
    if (!m_allocated) { return; }

    // image size
    unsigned int size = m_width * m_height;

    // verify format, convert otherwise
    if (m_format != GL_RGBA)
    {
        convert(GL_RGBA);
    }

    // format: RGBA
    if (m_format == GL_RGBA)
    {
        unsigned char r = a_color.getR();
        unsigned char g = a_color.getG();
        unsigned char b = a_color.getB();
        unsigned char* data = (unsigned char*)m_data;
        unsigned int i;
        for (i=0; i<size; i++)
        {
            if ((data[0] == r) && (data[1] == g) && (data[2] == b))
            {
                data[3] = a_transparencyLevel;
            }

            data+=4;
        }
    }
}


//==============================================================================
/*!
    his method assigns a transparency level to all image pixels.
    If the image is not in GL_RGBA format, it is first converted in order to
    enable pixel transparency capabilities.

    \param  a_transparencyLevel  Transparency level.
*/
//==============================================================================
void cImage::setTransparency(const unsigned char a_transparencyLevel)
{
    // check if image exists
    if (!m_allocated) { return; }

    // image size
    unsigned int size = m_width * m_height;

    // verify format, convert otherwise
    if (m_format != GL_RGBA)
    {
        convert(GL_RGBA);
    }

    // format: RGBA
    if (m_format == GL_RGBA)
    {
        unsigned char* data = &m_data[3];
        unsigned int i;
        for (i=0; i<size; i++)
        {
            *data = a_transparencyLevel;
            data+=4;
        }
    }
}


//==============================================================================
/*!
    This method flips the image horizontally.
*/
//==============================================================================
void cImage::flipHorizontal()
{
    // check if image exists
    if (!m_allocated) { return; }

    // image line size
    unsigned int lineWidth = m_bytesPerPixel*m_width;
    unsigned char *data = new unsigned char[lineWidth];

    // flip
    for(unsigned int i=0; i<m_height/2; i++)
    {
        unsigned char *botLine = m_data+i*lineWidth;
        unsigned char *topLine = m_data+(m_height-i-1)*lineWidth;
        memcpy(data,    topLine, lineWidth);
        memcpy(topLine, botLine, lineWidth);
        memcpy(botLine, data,    lineWidth);
    }

    delete [] data;
}


//==============================================================================
/*!
    Define a pixel color to be transparent. If the image is not in GL_RGBA
    format, it is first converted in order to enable pixel transparency
    capabilities.

    \param  a_r                  Red component of selected pixel color.
    \param  a_g                  Green component of selected pixel color.
    \param  a_b                  Blue component of selected pixel color.
    \param  a_transparencyLevel  Transparency level.
*/
//==============================================================================
void cImage::setTransparentColor(const unsigned char a_r,
    const unsigned char a_g,
    const unsigned char a_b,
    const unsigned char a_transparencyLevel)
{
    cColorb color(a_r, a_g, a_b);
    setTransparentColor(color, a_transparencyLevel);
}


//==============================================================================
/*!
    Loads this image from the specified file. Returns true if all
    goes well.  Note that regardless of whether it succeeds,
    this over-writes any image that had previously been loaded by this object.

    \param  a_filename  Image filename
    \return __true__ if file loaded successfully, __false__ otherwise.
*/
//==============================================================================
bool cImage::loadFromFile(const string& a_filename)
{
    // cleanup previous image
    cleanup();

    // find extension
    string extension = cGetFileExtension(a_filename);

    // we need a file extension to figure out file type
    if (extension.length() == 0)
    {
        return (false);
    }

    // convert string to lower extension
    string fileType = cStrToLower(extension);

    // result for loading file
    bool result = false;

    //--------------------------------------------------------------------
    // .BMP FORMAT
    //--------------------------------------------------------------------
    if (fileType == "bmp")
    {
        result = cLoadFileBMP(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .GIF FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "gif")
    {
        result = cLoadFileGIF(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .JPG FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "jpg")
    {
        result = cLoadFileJPG(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .PNG FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "png")
    {
        result = cLoadFilePNG(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .PPM FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "ppm")
    {
        result = cLoadFilePPM(this, a_filename);
    }

    //--------------------------------------------------------------------
    // .RAW FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "raw")
    {
        result = cLoadFileRAW(this, a_filename);
    }

    return (result);
}


//==============================================================================
/*!
    Saves this image from the specified file. Returns __true__ if all
    goes well. All images are saved in RGB format.

    \param  a_filename  Image filename.
*/
//==============================================================================
bool cImage::saveToFile(const string& a_filename)
{
    // find extension
    string extension = cGetFileExtension(a_filename);

    // we need a file extension to figure out file type
    if (extension.length() == 0)
    {
        return (false);
    }

    // convert string to lower extension
    string fileType = cStrToLower(extension);

    // image
    cImage* image = this;

    // result for saving file
    bool result = false;

    //--------------------------------------------------------------------
    // .BMP FORMAT
    //--------------------------------------------------------------------
    if (fileType == "bmp")
    {
        result = cSaveFileBMP(image, a_filename);
    }

    //--------------------------------------------------------------------
    // .GIF FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "gif")
    {
        result = cSaveFileGIF(image, a_filename);
    }

    //--------------------------------------------------------------------
    // .JPG FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "jpg")
    {
        result = cSaveFileJPG(image, a_filename);
    }

    //--------------------------------------------------------------------
    // .PNG FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "png")
    {
        result = cSaveFilePNG(image, a_filename);
    }

    //--------------------------------------------------------------------
    // .PPM FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "ppm")
    {
        result = cSaveFilePPM(image, a_filename);
    }

    //--------------------------------------------------------------------
    // .RAW FORMAT
    //--------------------------------------------------------------------
    else if (fileType == "raw")
    {
        result = cSaveFileRAW(image, a_filename);
    }

    return (result);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
