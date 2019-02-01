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
#include "files/CFileImagePPM.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
using namespace chai3d;
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This function loads a PPM image from a file into a cImage structure. 
    If the operation succeeds, then the functions returns __true__ and the 
    image data is loaded into image structure a_image. 
    If the operation fails, then the function returns __false__. 
    In both cases, previous image information stored in a_image is erased.

    \param  a_image     Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadFilePPM(cImage* a_image, const std::string& a_filename)
{
    int            i, j;
    int            width;
    int            height;
    int            maxval;
    char           header[3];
    FILE          *pFp;
    unsigned char *buffer;
    unsigned int   bytesLine;
    bool           p3 = false;
    bool           p6 = false;

    // sanity check
    if (a_image == NULL)
        return (C_ERROR);

    // open file
    if (NULL == (pFp = fopen(a_filename.c_str(), "rb")))
        return (C_ERROR);

    // read PPM header
    if (1 != fscanf(pFp, "%2s", &header[0]))
    {
      fclose(pFp);
      return (C_ERROR);
    }

    // identify format
    if      (strcmp(header, "P3") == 0) p3 = true;
    else if (strcmp(header, "P6") == 0) p6 = true;
    else
    {
        fclose(pFp);
        return (C_ERROR);
      }

    // skip any lines not whitespace & comments
    while (1)
    {
        do
        {
            i = fgetc(pFp);
        }
        while (isspace(i));

        if (isdigit(i)) 
        {
            ungetc(i, pFp);
            break;
        }
        else
        {
            do
            {
                i = fgetc(pFp);
            }
            while (i != '\n');
        }
    }

    // read image parameters
    if (3 != fscanf(pFp, "%d %d %d", &width, &height, &maxval))
    {
        fclose(pFp);
        return (C_ERROR);
    }

    // check ceiling value
    if (maxval != 255)
    {
        fclose(pFp);
        return (C_ERROR);
    }

    // we allocate memory for image. By default we shall use OpenGL's RGB mode.
    if (!a_image->allocate(width, height, GL_RGB))
    {
        return (C_ERROR);
    }

    // retrieve pointer to image data
    unsigned char* data = a_image->getData();

    // read the whitespace character
    i = fgetc(pFp);
    if (!isspace(i))
    {
        fclose(pFp);
        return (C_ERROR);
    }

    // allocate line buffer
    bytesLine = 3*width;
    if (NULL == (buffer = (unsigned char *)malloc(bytesLine)))
    {
        fclose(pFp);
        return (C_ERROR);
    }

    for (j=height-1; j>=0; j--)
    {
        // RAW format, line by line
        if (p6)
        {
            if (bytesLine != fread((void *) buffer, sizeof(unsigned char), bytesLine, pFp))
            {
                fclose(pFp);
                free(buffer);
                return (C_ERROR);
            }
        }

        // PLAIN format, line by line
        if (p3)
        {
            for (i=0; i<(int)bytesLine; i++)
            {
                int tmp;
                if (1 != fscanf(pFp, "%d", &tmp))
                {
                    fclose(pFp);
                    free(buffer);
                    return (C_ERROR);
                }
                buffer[i] = (unsigned char)(tmp);
            }
        }

        int index = 3*j*width;
        unsigned char *p = buffer;
        for (i=0; i<width; i++)
        {
            data[index++] = (unsigned char) *p++;
            data[index++] = (unsigned char) *p++;
            data[index++] = (unsigned char) *p++;
        }
    }

    // cleanup
    free(buffer);
    fclose(pFp);

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This function saves a BMP image from a cImage structure to a file.
    If the operation succeeds, then the functions returns __true__ and the
    image data is saved to a file.
    If the operation fails, then the function returns __false__.

    \param  a_image     Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSaveFilePPM(cImage* a_image, const std::string& a_filename)
{
    FILE          *pFp;
    int            i, j;
    unsigned char *buffer;
    unsigned int   bytesLine;

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

    // open file for binary write
    if (NULL == (pFp = fopen(a_filename.c_str(), "wb")))
        return (C_ERROR);

    // allocate line buffer
    bytesLine = width * 3;
    if (NULL == (buffer = (unsigned char *)malloc(bytesLine)))
    {
        fclose(pFp);
        return (C_ERROR);
    }

    // write PPM header (P6 format)
    fprintf (pFp, "P6\n%d %d\n255\n", width, height);

    // write PPM data one line at a time (this is MUCH faster than one RGB
    // pixel at a time and slightly faster than the whole image at once)
    for (i=height-1; i>=0; i--)
    {
        int            index = 3*i*width;
        unsigned char *p     = buffer;
        for (j=0; j<width; j++)
        {
            *p++ = (unsigned char) data[index++];
            *p++ = (unsigned char) data[index++];
            *p++ = (unsigned char) data[index++];
        }

        if (bytesLine != fwrite((void *) buffer, sizeof(unsigned char), bytesLine, pFp))
        {
            fclose(pFp);
            free(buffer);
            return (C_ERROR);
        }
    }

    // cleanup
    fclose(pFp);
    free(buffer);

    // return success
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
