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
#include "files/CFileImageRAW.h"
//------------------------------------------------------------------------------
#include <fstream>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
using namespace chai3d;
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    This function loads a RAW image from a file into a cImage structure.
    If the operation succeeds, then the functions returns __true__ and the
    image data is loaded into image structure a_image.
    If the operation fails, then the function returns __false__.
    In both cases, previous image information stored in a_image is erased.

    \param  a_image     Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadFileRAW(cImage* a_image, const std::string& a_filename)
{
    unsigned int format;
    unsigned int bpp;
    unsigned int width;
    unsigned int height;

    // sanity check
    if (a_image == NULL) 
        return (C_ERROR);

    // open file
    ifstream file(a_filename.c_str(), ios::binary);
    if (!file)
        return (C_ERROR);

    // read header
    file.read((char*)(&format), sizeof(unsigned int));
    file.read((char*)(&bpp),    sizeof(unsigned int));
    file.read((char*)(&width),  sizeof(unsigned int));
    file.read((char*)(&height), sizeof(unsigned int));

    // we allocate memory for image
    if (!a_image->allocate(width, height, format))
    {
        file.close();
        return false;
    }

    // read payload
    file.read((char*)(a_image->getData()), bpp*width*height);

    // close
    file.close();

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This function saves a RAW image from a cImage structure to a file.
    If the operation succeeds, then the functions returns __true__ and the
    image data is saved to a file.
    If the operation fails, then the function returns __false__.

    \param  a_image     Image structure.
    \param  a_filename  Filename.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cSaveFileRAW(cImage* a_image, const std::string& a_filename)
{
    // sanity check
    if (a_image == NULL) 
        return (C_ERROR);

    // retrieve image size
    unsigned int format = a_image->getFormat();
    unsigned int bpp    = a_image->getBytesPerPixel();
    unsigned int width  = a_image->getWidth();
    unsigned int height = a_image->getHeight();

    // store image to file
    ofstream file(a_filename.c_str(), ios::binary);
    if (!file)
        return (C_ERROR);

    // write header
    file.write((char*)(&format), sizeof (unsigned int));
    file.write((char*)(&bpp),    sizeof (unsigned int));
    file.write((char*)(&width),  sizeof (unsigned int));
    file.write((char*)(&height), sizeof (unsigned int));

    // write payload
    file.write((char*)(a_image->getData()), bpp*width*height);

    // close
    file.close();

    // return success
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
