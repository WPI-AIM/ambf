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
#include "files/CFileImageGIF.h"
//------------------------------------------------------------------------------
#ifdef C_USE_FILE_GIF
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------
#include "gif_lib.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
using namespace chai3d;
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This function loads a GIF image from a file into a cImage structure. \n
    If the operation succeeds, then the functions returns __true__ and the
    image data is loaded into image structure a_image. \n
    If the operation fails, then the function returns __false__. \n
    In both cases, previous image information stored in a_image is erased.

    \param  a_image  Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadFileGIF(cImage* a_image, const std::string& a_filename)
{
    // sanity check
    if (a_image == NULL)
        return (C_ERROR);

    // open image file
    GifFileType *gft = DGifOpenFileName(a_filename.c_str());
    if (!gft)
        return (C_ERROR);

    // read in image data
    if (DGifSlurp(gft) == GIF_ERROR)
    {
        DGifCloseFile(gft);
        return (C_ERROR);
    }

    // update width and height of image
    int width  = gft->Image.Width;
    int height = gft->Image.Height;

    // we allocate memory for image. By default we shall use OpenGL's RGB mode.
    if (!a_image->allocate(width, height, GL_RGB))
    {
        DGifCloseFile(gft);
        return (C_ERROR);
    }

    // retrieve pointer to image data
    unsigned char* data = a_image->getData();

    // retrieve palette
    GifColorType *colors;
    if (gft->SColorMap)
        colors = gft->SColorMap->Colors;
    else if (gft->Image.ColorMap)
        colors = gft->Image.ColorMap->Colors;
    else if ((gft->ImageCount > 0) && gft->SavedImages->ImageDesc.ColorMap)
        colors = gft->SavedImages->ImageDesc.ColorMap->Colors;
    else
        // failed to find a palette
        return (C_ERROR);

    // copy pixels
    unsigned char *ptr = gft->SavedImages->RasterBits;
    for (int j=height-1; j>=0; j--)
    {
        int index = 3*j*width;
        for (int i=0; i<width; i++)
        {
            GifColorType *pixel = &(colors[*ptr++]);
            data[index++] = pixel->Red;
            data[index++] = pixel->Green;
            data[index++] = pixel->Blue;
        }
    }

    // cleanup
    DGifCloseFile(gft);

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This function saves a GIF image from a cImage structure to a file. \n
    If the operation succeeds, then the functions returns __true__ and the
    image data is saved to a file. \n
    If the operation fails, then the function returns __false__. \n

    \param  a_image     Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSaveFileGIF(cImage* a_image, const std::string& a_filename)
{
    // sanity check
    if (a_image == NULL) 
        return (C_ERROR);

    // retrieve image size
    int width  = a_image->getWidth();
    int height = a_image->getHeight();
    if (!((width > 0) && (height > 0)))
        return (C_ERROR);

    // retrieve pointer to data
    unsigned char* data = a_image->getData();

    // count colors
    int            colorCount = 0;
    GifColorType   color[256];
    unsigned char *src = data;
    for (int j=0; j<height; j++)
    {
        for (int i=0; i<width; i++)
        {
            unsigned char r      = *src++;
            unsigned char g      = *src++;
            unsigned char b      = *src++;
            bool          exists = false;

            for (int k=0; k<colorCount; k++)
            {
                if ((r == color[k].Red) && (g == color[k].Green) && (b == color[k].Blue))
                {
                    exists = true;
                    break;
                }
            }

            if (!exists)
            {
                color[colorCount].Red   = r;
                color[colorCount].Green = g;
                color[colorCount].Blue  = b;
                colorCount++;
            }

            // there are more than 256 colors, we must give up
            // (en.wikipedia.org/wiki/Graphics_Interchange_Format)
            if (colorCount == 256)
            {
                return (C_ERROR);
            }
        }
    }

    // open image file
    GifFileType *gft = EGifOpenFileName(a_filename.c_str(), FALSE);
    if (!gft)
        return (C_ERROR);

    // build palette (force count to 256 in order to satisfy power-of-two requirement)
    ColorMapObject *map = MakeMapObject(256, color);
    if (!map)
    {
        EGifCloseFile(gft);
        FreeMapObject(map);
        return (C_ERROR);
    }

    // configure GIF
    gft->SBackGroundColor = 0;
    gft->SColorMap        = map;
    gft->SColorResolution = 256;
    gft->SWidth           = width;
    gft->SHeight          = height;

    // configure image
    SavedImage *img = MakeSavedImage(gft, NULL);
    img->ImageDesc.ColorMap  = map;
    img->ImageDesc.Height    = height;
    img->ImageDesc.Interlace = 0;
    img->ImageDesc.Left      = 0;
    img->ImageDesc.Top       = 0;
    img->ImageDesc.Width     = width;
    img->RasterBits          = (unsigned char *)malloc(width*height*sizeof(GifPixelType));

    // put all pixels in the file
    unsigned char *ptr = img->RasterBits;
    for (int j=height-1; j>=0; j--)
    {
        int index = 3*j*width;
        for (int i=0; i<width; i++)
        {
            unsigned char r = data[index++];
            unsigned char g = data[index++];
            unsigned char b = data[index++];

            for (int k=0; k<colorCount; k++)
            {
                if ((r == color[k].Red) && (g == color[k].Green) && (b == color[k].Blue))
                {
                    *ptr++ = k;
                    break;
                }
            }
        }
    }

    // actually write
    if (EGifSpew(gft) == GIF_ERROR)
    {
        EGifCloseFile(gft);
        FreeMapObject(map);
        return (C_ERROR);
    }

    // cleanup
    EGifCloseFile(gft);
    FreeMapObject(map);

    // return success.
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
#endif // C_USE_FILE_GIF
//------------------------------------------------------------------------------
