//===========================================================================
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
    \version   3.2.0 $Rev: 2177 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <iostream>
#include <iomanip>
#include <ostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <cstring>
using namespace std;
//---------------------------------------------------------------------------
#include "chai3d.h"
using namespace chai3d;
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED TYPES
//---------------------------------------------------------------------------

enum CmpFormat { NONE, PNG, JPG };


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// header file printer
int writeHeader(cImage &img, string  imagename, string  filename)
{
    unsigned int   width  = img.getWidth();
    unsigned int   height = img.getHeight();
    unsigned int   bpp    = img.getBytesPerPixel();
    unsigned int   format = img.getFormat();
    int            bcount = width*height*bpp;
    int            ptr    = 0;
    unsigned char *data   = img.getData();

    // convert imagename to root name
    imagename = imagename.substr(imagename.find_last_of('/')+1, imagename.length());

    ofstream out(filename.c_str());

    out << "//===========================================================================" << endl;
    out << "/*" << endl;
    out << "    Header file containing the \"" << imagename << "\" image." << endl;
    out << endl;
    out << "    Automatically generated using the CHAI3D visualization and haptics library." << endl;
    out << "    http://www.chai3d.org" << endl;
    out << endl;
    out << "*/" << endl;
    out << "//===========================================================================" << endl;

    // convert filename to root name
    filename = filename.substr(filename.find_last_of('/')+1, filename.length());
    filename = filename.substr(0, filename.find_first_of('.'));
    replace(filename.begin(), filename.end(), '.', '_');
    replace(filename.begin(), filename.end(), ' ', '_');
    replace(filename.begin(), filename.end(), '\'', '_');
    replace(filename.begin(), filename.end(), '-', '_');

    // convert image name to root name
    transform(imagename.begin(), imagename.end(),imagename.begin(), ::toupper);
    imagename = imagename.substr(0, imagename.find_first_of('.'));
    replace(imagename.begin(), imagename.end(), '.', '_');
    replace(imagename.begin(), imagename.end(), ' ', '_');
    replace(imagename.begin(), imagename.end(), '\'', '_');
    replace(imagename.begin(), imagename.end(), '-', '_');
    imagename = "CIMAGE_" + imagename;

    // avoid conflicts
    out << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#ifndef " << filename << "H" << endl;
    out << "#define " << filename << "H" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // include headers (required by convenience functions)
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#include \"graphics/CImage.h\"" << endl;
    out << "//---------------------------------------------------------------------------" << endl;

    // namespace
    out << "namespace chai3d {" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // print image info
    out << "const unsigned int " << imagename << "_SIZE   = " << bcount << ";" << endl;
    out << "const unsigned int " << imagename << "_BPP    = " << bpp    << ";" << endl;
    out << "const unsigned int " << imagename << "_WIDTH  = " << width  << ";" << endl;
    out << "const unsigned int " << imagename << "_HEIGHT = " << height << ";" << endl;
    out << "const unsigned int " << imagename << "_FORMAT = " << format << ";" << "    // ";
    switch (format)
    {
        case GL_RGB:  out << "GL_RGB";  break;
        case GL_RGBA: out << "GL_RGBA"; break;
    }
    out << endl << endl;

    // print image payload
    out << "const unsigned char " << imagename << "_BYTEARRAY[] =" << endl;
    out << "{";
    for (unsigned int j=0; j<height; j++)
    {
        out << endl << "    ";
        for (unsigned int i=0; i<width; i++)
        {
            for (unsigned int h=0; h<bpp; h++)
            {
                out << "0x" << hex << setw(2) << setfill('0') << (int)(data[ptr++]);
                if (ptr < bcount) out << ", ";
            }
        }
    }
    out << endl << "};" << endl << endl << endl;

    // convenience function: allocate image
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;
    out << endl;
    out << "inline chai3d::cImagePtr NEW_" << imagename << "()" << endl;
    out << "{" << endl;
    out << "    chai3d::cImagePtr img = chai3d::cImage::create();" << endl;
    out << "    img->allocate(" << imagename << "_WIDTH, " << imagename << "_HEIGHT, " << imagename << "_FORMAT);" << endl;
    out << "    unsigned char  *bytearray = new unsigned char[" << imagename << "_SIZE];" << endl;
    out << endl;
    out << "    memcpy(bytearray, " << imagename << "_BYTEARRAY, " << imagename << "_SIZE);" << endl;
    out << "    img->setData(bytearray, " << imagename << "_SIZE, true);" << endl;
    out << endl;
    out << "    return img;" << endl;
    out << "}" << endl << endl << endl;

    // close namespace
    out << "//---------------------------------------------------------------------------" << endl;
    out << "} // namespace chai3d" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // tidy up
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#endif" << endl;
    out << "//---------------------------------------------------------------------------" << endl;

    out.close();

    return 0;
}


// PNG header file printer
int writeHeaderCompressed(cImage &img, string  imagename, string  filename, CmpFormat format)
{
    const int lineWidth = 256;

    // check that format is supported
    switch (format)
    {
    case PNG:
    case JPG:
        break;
    default:
        return -1;
    }

    unsigned int ptr = 0;

    // convert imagename to root name
    imagename = imagename.substr(imagename.find_last_of('/')+1, imagename.length());

    ofstream out(filename.c_str());

    out << "//===========================================================================" << endl;
    out << "/*" << endl;
    out << "    Header file containing the \"" << imagename << "\" image in ";
    switch (format)
    {
    case PNG:
        out << "PNG";
        break;
    case JPG:
        out << "JPG";
        break;
    default:
        out << "unknown";
        break;
    }
    out << " format." << endl << endl;
    out << endl;
    out << "    Automatically generated using the CHAI3D visualization and haptics library." << endl;
    out << "    http://www.chai3d.org" << endl;
    out << endl;
    out << "*/" << endl;
    out << "//===========================================================================" << endl;

    // convert filename to root name
    filename = filename.substr(filename.find_last_of('/')+1, filename.length());
    filename = filename.substr(0, filename.find_first_of('.'));
    replace(filename.begin(), filename.end(), '.', '_');
    replace(filename.begin(), filename.end(), ' ', '_');
    replace(filename.begin(), filename.end(), '\'', '_');
    replace(filename.begin(), filename.end(), '-', '_');

    // convert image name to root name
    transform(imagename.begin(), imagename.end(),imagename.begin(), ::toupper);
    imagename = imagename.substr(0, imagename.find_first_of('.'));
    replace(imagename.begin(), imagename.end(), '.', '_');
    replace(imagename.begin(), imagename.end(), ' ', '_');
    replace(imagename.begin(), imagename.end(), '\'', '_');
    replace(imagename.begin(), imagename.end(), '-', '_');
    imagename = "CIMAGE_" + imagename;

    // convert image to compressed format
    unsigned char *buffer;
    unsigned int   len;
    switch (format)
    {
    case PNG:
        if (!cSavePNG(&img, &buffer, &len))
        {
            cout << "PNG compression failed" << endl;
            return -1;
        }
        break;
    case JPG:
        if (!cSaveJPG(&img, &buffer, &len))
        {
            cout << "JPG compression failed" << endl;
            return -1;
        }
        break;
    default:
        cout << "unknown compression format" << endl;
        return false;
    }

    // avoid conflicts
    out << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#ifndef " << filename << "H" << endl;
    out << "#define " << filename << "H" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // include headers (required by convenience functions)
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#include \"graphics/CImage.h\"" << endl;
    switch (format)
    {
    case PNG:
        out << "#include \"files/CFileImagePNG.h\"" << endl << endl;
        break;
    case JPG:
        out << "#include \"files/CFileImageJPG.h\"" << endl << endl;
        break;
    default:
        break;
    }
    out << "//---------------------------------------------------------------------------" << endl;

    // namespace
    out << "namespace chai3d {" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // print image payload
    out << "const unsigned char " << imagename << "_BYTEARRAY[] =" << endl;
    out << "{";
    for (unsigned int s=0; s<len; s++)
    {
        if (s%lineWidth == 0) out << endl << "\t";
        out << "0x" << hex << setw(2) << setfill('0') << (int)(((unsigned char*)buffer)[ptr++]);
        if (ptr < len) out << ", ";
    }
    out << endl << "};" << endl << endl << endl;

    // convenience function: allocate image
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;
    out << endl;
    out << "inline chai3d::cImagePtr NEW_" << imagename << "()" << endl;
    out << "{" << endl;
    out << "    chai3d::cImagePtr img = chai3d::cImage::create();" << endl;
    switch (format)
    {
    case PNG:
        out << "    cLoadPNG(img->getImage(), " << imagename << "_BYTEARRAY, sizeof(" << imagename << "_BYTEARRAY));" << endl;
        break;
    case JPG:
        out << "    cLoadJPG(img->getImage(), " << imagename << "_BYTEARRAY, sizeof(" << imagename << "_BYTEARRAY));" << endl;
        break;
    default:
        break;
    }
    out << "    return (img);" << endl;
    out << "}" << endl << endl << endl;

    // close namespace
    out << "//---------------------------------------------------------------------------" << endl;
    out << "} // namespace chai3d" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // tidy up
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#endif" << endl;
    out << "//---------------------------------------------------------------------------" << endl;

    out.close();

    return 0;
}


// simple usage printer
int usage()
{
    cout << endl << "cimage [-p] [-j] image.{bmp|gif|jpg|png|raw} [-o header.h]" << endl;
    cout << "\t-p\tuse PNG compression to store image in header" << endl;
    cout << "\t-j\tuse JPG compression to store image in header" << endl;
    cout << "\t-o\tspecify header filename" << endl;
    cout << "\t-h\tdisplay this message" << endl << endl;

    return -1;
}


//===========================================================================
/*
    UTILITY:    cimage.cpp

    This utility takes an image in any CHAI3D supported format and produces
    a C/C++ compatible header containing the image data and geometry. This
    allows programmers to easily embed images into their executables.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    cImage img;
    string imagename;
    string filename;
    string altfilename;
    bool   compressPNG = false;
    bool   compressJPG = false;

    // process arguments
    if (argc < 2) return usage();
    for (int i=1; i<argc; i++)
    {
        if (argv[i][0] != '-') {
            if (imagename.length() > 0) return usage();
            imagename = string(argv[i]);
            filename  = imagename.substr(imagename.find_last_of('/')+1, imagename.length());
            filename  = filename.substr(0, filename.find_last_of('.'));
            filename[0] = toupper(filename[0]);
            filename = "CImage" + filename + ".h";
        }
        else switch (argv[i][1]) {
            case 'h':
                return usage ();
            case 'o':
                if ((i < argc) && (argv[i+1][0] != '-')) {
                    i++;
                    filename = string(argv[i]);
                }
                else return usage ();
                break;
            case 'p':
                compressPNG = true;
                break;
            case 'j':
                compressJPG = true;
                break;
            default:
                return usage ();
        }
    }

    // figure out output filename
    if (altfilename != "")
    {
        filename = altfilename;
    }

    // pretty message
    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Image Converter" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl;
    cout << endl;

    // report action
    cout << "converting " << imagename << " to " << filename << "..." << endl;

    // read image
    if (!img.loadFromFile (imagename))
    {
        cout << "error: cannot load image file " << imagename << endl;
    }
    else {
        cout << "image load succeeded" << endl;
    }
    cout << endl;

    // export image
    if (!compressPNG && !compressJPG && writeHeader (img, imagename, filename) < 0)
    {
        cout << "error: conversion failed" << endl;
    }
    else if (compressPNG && writeHeaderCompressed (img, imagename, filename, PNG) < 0)
    {
        cout << "error: conversion failed" << endl;
    }
    else if (compressJPG && writeHeaderCompressed (img, imagename, filename, JPG) < 0)
    {
        cout << "error: conversion failed" << endl;
    }
    else
    {
        cout << "conversion succeeded" << endl;
    }

    return 0;
}

//---------------------------------------------------------------------------
