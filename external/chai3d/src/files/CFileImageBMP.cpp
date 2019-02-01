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
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "files/CFileImageBMP.h"
//------------------------------------------------------------------------------
#include <cstring>
#include <fstream>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------
#if defined (WIN32) | defined (WIN64)

#include <windows.h>

#else

#include <sys/types.h>

typedef u_int8_t  BYTE, *LPBYTE;    // unsigned char
typedef u_int16_t WORD;             // unsigned short
typedef u_int32_t DWORD;            // unsigned long
typedef int32_t   LONG;             // long

// tell GCC to byte align the following structures
#pragma pack(1)

typedef struct tagBITMAPFILEHEADER
{
    WORD    bfType;
    DWORD   bfSize;
    WORD    bfReserved1;
    WORD    bfReserved2;
    DWORD   bfOffBits;
} BITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER
{
    DWORD  biSize;
    LONG   biWidth;
    LONG   biHeight;
    WORD   biPlanes;
    WORD   biBitCount;
    DWORD  biCompression;
    DWORD  biSizeImage;
    LONG   biXPelsPerMeter;
    LONG   biYPelsPerMeter;
    DWORD  biClrUsed;
    DWORD  biClrImportant;
} BITMAPINFOHEADER;

typedef struct tagRGBQUAD
{
    BYTE    rgbBlue;
    BYTE    rgbGreen;
    BYTE    rgbRed;
    BYTE    rgbReserved;
} RGBQUAD;

typedef enum
{
    BI_RGB = 0,
    BI_RLE8,
    BI_RLE4,
    BI_BITFIELDS,
    BI_JPEG,
    BI_PNG,
} bmp_compression_method_t;

// resume normal packing...
#pragma pack()

#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
using namespace chai3d;
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This function loads a BMP image from a file into a cImage structure. 
    If the operation succeeds, then the functions returns __true__ and the 
    image data is loaded into the image structure a_image. 
    If the operation fails, then the function returns __false__..
    In both cases, previous image information stored in a_image is erased.

    \param  a_image  Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadFileBMP(cImage* a_image, const std::string& a_filename)
{
    BITMAPINFOHEADER  bmih;
    BITMAPFILEHEADER  bmfh;
    int               i, j;
    unsigned char     r, g, b, a;
    unsigned char    *buffer;

    // sanity check
    if (a_image == NULL)
        return false;

    // open file
    std::ifstream imgfile(a_filename.c_str(), ios::binary);
    if (!imgfile)
        return false;

    // read header
    imgfile.read((char *) &bmfh.bfType,      sizeof(WORD));
    imgfile.read((char *) &bmfh.bfSize,      sizeof(DWORD));
    imgfile.read((char *) &bmfh.bfReserved1, sizeof(WORD));
    imgfile.read((char *) &bmfh.bfReserved2, sizeof(WORD));
    imgfile.read((char *) &bmfh.bfOffBits,   sizeof(DWORD));
    imgfile.read((char *) &bmih,             sizeof(BITMAPINFOHEADER));

    // update width of the height of image
    int width  = bmih.biWidth;
    int height = bmih.biHeight;

    // pixels are stored consecutively as RGB|RGB|RGB|RGB...
    if (bmih.biClrUsed == BI_RGB)
    {
        switch(bmih.biBitCount)
        {
            // 8-bit per pixel with color palette
            case 8:
            {
                // allocate memory for GL_RGB image
                if (!a_image->allocate(width, height, GL_RGB))
                    return false;

                // retrieve pointer to image data
                unsigned char* data = a_image->getData();

                // allocate color palette
                LPBYTE palette = new BYTE[1024];

                // read-in palette
                imgfile.read((char *) palette, 1024);

                // allocate line buffer
                if (NULL == (buffer = (unsigned char *)malloc(width)))
                {
                    imgfile.close();
                    delete [] palette;
                    return false;
                }

                // read data
                imgfile.seekg(bmfh.bfOffBits, ios::beg);
                for (j=0; j<height; j++)
                {
                    unsigned char *p = buffer;
                    imgfile.read((char *) buffer, width);
                    for (i=0; i<width; i++)
                    {
                        r = *p++;
                        *data++ = (unsigned char) palette[4 * r + 2];
                        *data++ = (unsigned char) palette[4 * r + 1];
                        *data++ = (unsigned char) palette[4 * r];
                    }
                }

                // cleanup
                free(buffer);
                delete [] palette;
            }
            break;

            // 24-bit per pixel (no palette)
            case 24:
            {
                // allocate memory for GL_RGB image
                if (!a_image->allocate(width, height, GL_RGB))
                    return false;

                // retrieve pointer to image data
                unsigned char* data = a_image->getData();

                // a bit tricky: align within 32 bits
                int          align     = (4 - (width%4)) & 3;
                unsigned int bytesLine = (width + align) * 3;

                // allocate line buffer
                if (NULL == (buffer = (unsigned char *)malloc(bytesLine)))
                {
                    imgfile.close();
                    return false;
                }

                // read data
                imgfile.seekg(bmfh.bfOffBits, ios::beg);
                for (j=0; j<height; j++)
                {
                    unsigned char *p = buffer;
                    imgfile.read((char *) buffer, bytesLine);
                    for (i=0; i <width; i++)
                    {
                        b = *p++;
                        g = *p++;
                        r = *p++;
                        *data++ = (unsigned char) r;
                        *data++ = (unsigned char) g;
                        *data++ = (unsigned char) b;
                    }
                }

                // cleanup
                free(buffer);
            }
            break;

            // 32-bit per pixel
            case 32:
            {
                // allocate memory for GL_RGBA image
                if (!a_image->allocate(width, height, GL_RGBA))
                    return false;

                // retrieve pointer to image data
                unsigned char* data = a_image->getData();

                // line stride
                unsigned int bytesLine = width * 4;

                // allocate line buffer
                if (NULL == (buffer = (unsigned char *)malloc(bytesLine)))
                {
                    imgfile.close();
                    return false;
                }

                // read data
                imgfile.seekg(bmfh.bfOffBits, ios::beg);
                for (j=0; j<height; j++)
                {
                    unsigned char *p = buffer;
                    imgfile.read((char *) buffer, bytesLine);
                    for (i=0; i<width; i++)
                    {
                        b = *p++;
                        g = *p++;
                        r = *p++;
                        a = *p++;
                        *data++ = (unsigned char) r;
                        *data++ = (unsigned char) g;
                        *data++ = (unsigned char) b;
                        *data++ = (unsigned char) a;
                    }
                }

                // cleanup
                free(buffer);
            }
            break;

            // monochrome and 4bits (8 colors) unsupported yet
            default:
            {
                imgfile.close();
                return false;
            }
        }
    }

    // indexed color
    else
    {
        // allocate memory for GL_RGB image
        if (!a_image->allocate(width, height, GL_RGB))
            return false;

        // retrieve pointer to image data
        unsigned char* data = a_image->getData();

        // allocate palette
        LPBYTE palette = new BYTE[4 * bmih.biClrUsed];

        // read-in palette
        imgfile.read((char *) palette, 4 * bmih.biClrUsed);

        // allocate line buffer
        if (NULL == (buffer = (unsigned char *)malloc(width)))
        {
            imgfile.close();
            delete [] palette;
            return false;
        }

        // read data
        imgfile.seekg(bmfh.bfOffBits, ios::beg);
        for (j=0; j<height; j++)
        {
            unsigned char *p = buffer;
            imgfile.read((char *) buffer, width);
            for (i=0; i<width; i++)
            {
                r = *p++;
                *data++ = (unsigned char) palette[4 * r + 2];
                *data++ = (unsigned char) palette[4 * r + 1];
                *data++ = (unsigned char) palette[4 * r];
            }
        }

        // cleanup
        free(buffer);
        delete [] palette;
    }

    imgfile.close();

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This function saves a BMP image from a cImage structure to a file. \n
    If the operation succeeds, then the functions returns __true__ and the
    image data is saved to a file. \n
    If the operation fails, then the function returns __false__.

    \param  a_image     Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSaveFileBMP(cImage* a_image, const std::string& a_filename)
{
    BITMAPINFOHEADER   bmih;
    BITMAPFILEHEADER   bmfh;
    WORD               bitCount;
    WORD               bpp;
    WORD               paletteSize;
    int                i, j;
    unsigned char     *buffer;

    // sanity check
    if (a_image == NULL)
        return false;

    // retrieve image size
    int width  = a_image->getWidth();
    int height = a_image->getHeight();
    if (!((width > 0) && (height > 0)))
        return (false);

    // adjust to image format
    switch(a_image->getFormat())
    {
        case GL_LUMINANCE:
            bitCount    = 8;
            bpp         = 1;
            paletteSize = 1024;
            break;

        case GL_RGB:
            bitCount    = 24;
            bpp         = sizeof(RGBQUAD);
            paletteSize = 0;
            break;

        case GL_RGBA:
            bitCount    = 32;
            bpp         = sizeof(RGBQUAD);
            paletteSize = 0;
            break;

        // unsupported
        case GL_LUMINANCE_ALPHA:
        default:
            return (C_ERROR);
    }

    // retrieve pointer to data
    unsigned char* data = a_image->getData();

    // set info header
    memset(&bmih, 0, sizeof(BITMAPINFOHEADER));
    bmih.biSize         = sizeof(BITMAPINFOHEADER);
    bmih.biWidth        = width;
    bmih.biHeight       = height;
    bmih.biPlanes       = 1;
    bmih.biBitCount     = bitCount;
    bmih.biCompression  = 0L;
    bmih.biClrUsed      = 0;
    bmih.biSizeImage    = width * height;

    // set file header
    memset(&bmfh, 0, sizeof(BITMAPFILEHEADER));
    bmfh.bfType         = (WORD) 0x4D42;
    bmfh.bfSize         = (DWORD)(sizeof(BITMAPFILEHEADER) + bmih.biSize + bmih.biClrUsed * bpp + bmih.biSizeImage + paletteSize);
    bmfh.bfOffBits      = (DWORD)(sizeof(BITMAPFILEHEADER) + bmih.biSize + bmih.biClrUsed * bpp + paletteSize);

    // open file
    std::ofstream imgfile(a_filename.c_str(), ios::binary);
    if (!imgfile)
        return (C_ERROR);

    // write header
    imgfile.write(reinterpret_cast<char*>(&bmfh), sizeof(BITMAPFILEHEADER));
    imgfile.write(reinterpret_cast<char*>(&bmih), sizeof(BITMAPINFOHEADER));

    // write payload
    switch(bitCount)
    {
        // gray scale (palette)
        case 8:
        {
            // allocate line buffer
            if (NULL == (buffer = (unsigned char *)malloc(width)))
                return false;

            // allocate color palette
            LPBYTE palette = new BYTE[1024];

            // create palette
            int pindex = 0;
            for (i=0; i<256; i++)
            {
                palette[pindex++] = i;
                palette[pindex++] = i;
                palette[pindex++] = i;
                palette[pindex++] = 0;
            }

            // write palette
            imgfile.write((char *) palette, 1024);

            // write indexed data
            imgfile.seekp(bmfh.bfOffBits, ios::beg);
            for (j=0; j<height; j++)
            {
                unsigned char *p = buffer;
                for (i=0; i<width; i++)
                {
                    *p++ = palette[4*(*data++)];
                }
                imgfile.write((char*)buffer, sizeof(unsigned char) * width);
            }

            // cleanup
            free(buffer);
        }
        break;

        // RGB
        case 24:
        {
            // allocate line buffer
            int          align     = (4 - ((width * 3) % 4)) & 3;
            unsigned int bytesLine = width * 3 + align;
            if (NULL == (buffer = (unsigned char *)malloc(bytesLine)))
                return false;

            // pour it in
            for (j=0; j<height; j++)
            {
                unsigned char *p = buffer;
                unsigned char  r, g, b;
                for (i=0; i<width; i++)
                {
                    r = (unsigned char) *data++;
                    g = (unsigned char) *data++;
                    b = (unsigned char) *data++;
                    *p++ = b;
                    *p++ = g;
                    *p++ = r;
                }
                imgfile.write((char*)buffer, sizeof(unsigned char) * bytesLine);
            }

            // cleanup
            free(buffer);
        }
        break;

        // RGBA
        case 32:
        {
            // allocate line buffer
            unsigned int bytesLine = 4*width;
            if (NULL == (buffer = (unsigned char *) malloc (bytesLine)))
                return false;

            // pour it in
            for (j=0; j<height; j++)
            {
                unsigned char *p = buffer;
                unsigned char  r, g, b, a;
                for (i=0; i<width; i++)
                {
                    r = (unsigned char) *data++;
                    g = (unsigned char) *data++;
                    b = (unsigned char) *data++;
                    a = (unsigned char) *data++;
                    *p++ = b;
                    *p++ = g;
                    *p++ = r;
                    *p++ = a;
                }
                imgfile.write((char*)buffer, sizeof(unsigned char) * bytesLine);
            }

            // cleanup
            free(buffer);
        }
    }

    // close file
    imgfile.close();

    // return success
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
