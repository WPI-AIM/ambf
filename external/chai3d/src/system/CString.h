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
#ifndef CStringH
#define CStringH
//------------------------------------------------------------------------------
#include <string>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CString.h
    \ingroup    system

    \brief
    Implements utility functions for manipulating strings.
*/
//==============================================================================

//------------------------------------------------------------------------------
/*!
    \addtogroup system
*/
//------------------------------------------------------------------------------

//@{

//------------------------------------------------------------------------------
// ANSI STRINGS:
//------------------------------------------------------------------------------

//! This function computes the length of an __ANSI string__.
int cStrLength(const char* a_input);


//------------------------------------------------------------------------------
// FILENAME TOOLS:
//------------------------------------------------------------------------------

//! This function extracts the __filename__ with or without its extension.
std::string cGetFilename(const std::string& a_input, const bool a_includeFileExtension = true);

//! This function extracts the __file extension__ of a file.
std::string cGetFileExtension(const std::string& a_input, const bool a_includeDot = false);

//! This function extracts the __directory path__ from the full file path.
std::string cGetDirectory(const std::string& a_input);

//! This function replaces the __file extension__ of a filename with a new extension name.
std::string cReplaceFileExtension(const std::string& a_input, const std::string& a_extension);


//------------------------------------------------------------------------------
// CONVERTING VALUES TO STRINGS:
//------------------------------------------------------------------------------

//! This function converts a __boolean__ into a __string__.
std::string cStr(const bool a_value);

//! This function converts an __integer__ into a __string__.
std::string cStr(const int a_value);

//! This function converts an __unsigned integer__ into a __string__.
std::string cStr(const unsigned int a_value);

//! This function converts a __float__ into a __string__.
std::string cStr(const float a_value, const unsigned int a_precision = 2);

//! This function converts a __double__ into a __string__.
std::string cStr(const double a_value, const unsigned int a_precision = 2);

//! This function converts a __string__ to __lower case__.
std::string cStrToLower(const std::string& a_input);

//@}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
