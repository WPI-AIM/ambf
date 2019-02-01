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
    \author    Dan Morris
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "system/CString.h"
#include "math/CMaths.h"
//------------------------------------------------------------------------------
#include <algorithm>
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
    This function computes the length of an __ANSI string__ of up to 255 characters.
    If the end of the string cannot be found, then -1 is returned as a result.

    \param  a_input  Input __ANSI string__. (Pointer to first character).

    \return Length of input string, otherwise -1.
*/
//==============================================================================
int cStrLength(const char* a_input)
{
    return (int)(strlen(a_input));
}


//==============================================================================
/*!
    This function discards the path component of a filename and returns the 
    filename itself, optionally including the file extension.

    \param  a_input                 Input string containing path and filename.
    \param  a_includeFileExtension  If __true__, then file extension is included.

    \return Filename with or without file extension.
*/
//==============================================================================
std::string cGetFilename(const std::string& a_input, const bool a_includeFileExtension)
{
    int pos;
    string result = a_input;
    
    pos= (int)(result.find_last_of("\\"));
    if (pos > -1) result = result.substr(pos+1, result.length()-pos-1);
    
    pos = (int)(result.find_last_of("/"));
    if (pos > -1) result = result.substr(pos+1, result.length()-pos-1);
    
    if (!a_includeFileExtension)
    {
        pos = (int)(result.find_last_of("."));
        result = result.substr(0, pos);
    }

    return (result);
}


//==============================================================================
/*!
    This function extracts the file extension of a file, optionally include the
    dot at the beginning of the extension. \n
    Example: .jpg \n

    \param  a_input       Input filename string with or without path.
    \param  a_includeDot  If __true__, then include the dot at the beginning of 
                          the extension.

    \return File extension with or without leading dot.
*/
//==============================================================================
std::string cGetFileExtension(const std::string& a_input, const bool a_includeDot)
{
    int pos = (int)(a_input.find_last_of("."));
    if (pos < 0) return "";
    if (a_includeDot)
    {
        return (a_input.substr(pos, a_input.length()));
    }
    else
    {
        return (a_input.substr(pos+1, a_input.length()));
    }
}


//==============================================================================
/*!
    This function extracts the directory path portion of the source, and include
    trailing '/'.  If there are no '/'s found, then the function 
    returns an empty string.

    \param  a_input  Input string including directory path and filename.

    \return Directory path.
*/
//==============================================================================
std::string cGetDirectory(const std::string& a_input)
{
    return (a_input.substr(0, a_input.length() - cGetFilename(a_input, true).length()));
}


//==============================================================================
/*!
    This function replaces the file extension of a filename with a new extension
    name.

    \param  a_input      Input string including path and filename.
    \param  a_extension  New file extension.

    \return New path and filename.
*/
//==============================================================================
std::string cReplaceFileExtension(const std::string& a_input, const std::string& a_extension)
{
    string result = a_input;
    string extension = cGetFileExtension(a_extension);
    int pos = (int)(result.find_last_of("."));
    if (pos < 0)
    {
        return (a_input);
    }

    result.replace(pos+1, result.length(), extension);
    return (result);
}


//=================================================================================
/*!
    This function converts a __boolean__ into a __string__.

    \param  a_value  Input value of type __boolean__.

    \return Converted value into __string__ format.
*/
//=================================================================================
std::string cStr(const bool a_value)
{
    string result;
    if (a_value) 
    {
        result = "true";
    }
    else 
    {
        result = "false";
    }
    return (result);
}


//=================================================================================
/*!
    This function converts an __integer__ into a __string__.

    \param  a_value  Input value of type __integer__.

    \return Converted value into __string__ format.
*/
//==============================================================================
std::string cStr(const int a_value)
{
    ostringstream result;
    result << a_value;
    return (result.str());
}


//==============================================================================
/*!
    This function converts an __unsigned integer__ into a __string__.

    \param  a_value  Input value of type __unsigned integer__.

    \return Converted value into __string__ format.
*/
//==============================================================================
std::string cStr(const unsigned int a_value)
{
    ostringstream result;
    result << a_value;
    return (result.str());
}


//==============================================================================
/*!
    This function converts a __float__ into a __string__.

    \param  a_value      Input value of type __float__.
    \param  a_precision  Number of digits displayed after the decimal point.

    \return Converted value into __string__ format.
*/
//==============================================================================
std::string cStr(const float a_value, const unsigned int a_precision)
{
    ostringstream result;
    result << fixed << setprecision(a_precision) << a_value;
    return (result.str());
}


//==============================================================================
/*!
    This function converts a __double__ into a __string__.

    \param  a_value      Input value of type __double__.
    \param  a_precision  Number of digits displayed after the decimal point.

    \return Converted value into __string__ format.
*/
//==============================================================================
std::string cStr(const double a_value, const unsigned int a_precision)
{
    ostringstream result;
    result << fixed << setprecision(a_precision) << a_value;
    return (result.str());
}


//==============================================================================
/*!
    This function converts a __string__ into __lowercase__.

    \param  a_input  Input string to be converted.

    \return Converted __string__.
*/
//==============================================================================
std::string cStrToLower(const std::string& a_input)
{
    string result = a_input;
    transform(result.begin(), result.end(), result.begin(), ::tolower);
    return (result);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
