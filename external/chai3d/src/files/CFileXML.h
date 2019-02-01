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
    \version   3.2.0 $Rev: 2016 $
 */
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CFileXML
#define CFileXML
//------------------------------------------------------------------------------
#include <string>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CFileXML.h
    \ingroup    files

    \brief
    Implements XML file support.
 */
//==============================================================================

//------------------------------------------------------------------------------
/*!
    \addtogroup files
 */
//------------------------------------------------------------------------------

//@{

class cFileXML
{
    //! Pointer to internal XML structure.
    void *m_xml;

public:

    //! Constructor.
    cFileXML();

    //! Constructor.
    cFileXML(const std::string& a_filename);

    //! Destructor.
    virtual ~cFileXML();

    //! Open an XML file and load its XML data internally.
    bool loadFromFile(const std::string& a_filename);

    //! Save the current internal XML data to the same file that was loaded with \ref loadFromFile().
    bool saveToFile();

    //! Save the current internal XML data to a file.
    bool saveToFile(const std::string& a_filename);

    //! Clear internal XML data.
    void clear();

    //! Reset current node to the root node.
    void gotoRoot();

    //! Set current node to the first child of the current node.
    bool gotoFirstChild();

    //! Set current node to the next sibling of the current node.
    bool gotoNextSibling();

    //! Set current node to a specific child of the current node.
    int gotoChild(const std::string& a_name, int a_index = 0, bool a_create = false);

    //! Remove a specific child of the current node and all its children.
    bool removeChild(const std::string& a_name, int a_index = 0);

    //! Set current node to the parent of the current node.
    bool gotoParent();

    //! Set the name of the current node.
    bool setName(const std::string& a_name);

    //! Set the value of the current node.
    bool setValue(const bool a_val);

    //! Set the value of the current node.
    bool setValue(const long int a_val);

    //! Set the value of the current node.
    bool setValue(const int a_val) { long int tmp = a_val; return setValue(tmp); }

    //! Set the value of the current node.
    bool setValue(const unsigned int a_val) { long int tmp = a_val; return setValue(tmp); }

    //! Set the value of the current node.
    bool setValue(const short a_val) { long int tmp = a_val; return setValue(tmp); }

    //! Set the value of the current node.
    bool setValue(const unsigned short a_val) { long int tmp = a_val; return setValue(tmp); }

    //! Set the value of the current node.
    bool setValue(const char a_val) { long int tmp = a_val; return setValue(tmp); }

    //! Set the value of the current node.
    bool setValue(const unsigned char a_val) { long int tmp = a_val; return setValue(tmp); }

    //! Set the value of the current node.
    bool setValue(const float a_val) { double tmp = a_val; return setValue(tmp); }

    //! Set the value of the current node.
    bool setValue(const double a_val);

    //! Set the value of the current node.
    bool setValue(const std::string a_val);

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const bool a_val);

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const long int a_val);

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const int a_val) { return setAttribute(a_attribute, (long int)a_val); }

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const unsigned int a_val) { return setAttribute(a_attribute, (long int)a_val); }

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const short a_val) { return setAttribute(a_attribute, (long int)a_val); }

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const unsigned short a_val) { return setAttribute(a_attribute, (long int)a_val); }

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const char a_val) { return setAttribute(a_attribute, (long int)a_val); }

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const unsigned char a_val) { return setAttribute(a_attribute, (long int)a_val); }

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const double a_val);

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const float a_val) { return setAttribute(a_attribute, (double)a_val); }

    //! Set an attribute of the current node.
    bool setAttribute(const std::string& a_attribute, const std::string a_val);

    //! Get the name of the current node.
    bool getName(std::string& a_name) const;

    //! Get the value of the current node.
    bool getValue(bool& a_val) const;

    //! Get the value of the current node.
    bool getValue(long int& a_val) const;

    //! Get the value of the current node.
    bool getValue(int& a_val) const { long int tmp; bool res = getValue(tmp); a_val = (int)tmp; return res; }

    //! Get the value of the current node.
    bool getValue(unsigned int& a_val) const { long int tmp; bool res = getValue(tmp); a_val = (unsigned int)tmp; return res; }

    //! Get the value of the current node.
    bool getValue(short& a_val) const { long int tmp; bool res = getValue(tmp); a_val = (short)tmp; return res; }

    //! Get the value of the current node.
    bool getValue(unsigned short& a_val) const { long int tmp; bool res = getValue(tmp); a_val = (unsigned short)tmp; return res; }

    //! Get the value of the current node.
    bool getValue(char& a_val) const { long int tmp; bool res = getValue(tmp); a_val = (char)tmp; return res; }

    //! Get the value of the current node.
    bool getValue(unsigned char& a_val) const { long int tmp; bool res = getValue(tmp); a_val = (unsigned char)tmp; return res; }

    //! Get the value of the current node.
    bool getValue(float& a_val) const { double tmp; bool res = getValue(tmp); a_val = (float)tmp; return res; }

    //! Get the value of the current node.
    bool getValue(double& a_val) const;

    //! Get the value of the current node.
    bool getValue(std::string& a_val) const;

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, bool& a_val) const;

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, long int& a_val) const;

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, int& a_val) const { long int tmp; bool res = getAttribute (a_attribute, tmp); a_val = (int)tmp; return res; }

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, unsigned int& a_val) const { long int tmp; bool res = getAttribute (a_attribute, tmp); a_val = (unsigned int)tmp; return res; }

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, short& a_val) const { long int tmp; bool res = getAttribute (a_attribute, tmp); a_val = (short)tmp; return res; }

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, unsigned short& a_val) const { long int tmp; bool res = getAttribute (a_attribute, tmp); a_val = (unsigned short)tmp; return res; }

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, char& a_val) const { long int tmp; bool res = getAttribute (a_attribute, tmp); a_val = (char)tmp; return res; }

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, unsigned char& a_val) const { long int tmp; bool res = getAttribute (a_attribute, tmp); a_val = (unsigned char)tmp; return res; }

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, float& a_val) const { double   tmp; bool res = getAttribute (a_attribute, tmp); a_val = (float)tmp; return res; }

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, double& a_val) const;

    //! Get the value of a specific attribute of the current node.
    bool getAttribute(const std::string& a_attribute, std::string& a_val) const;

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const bool a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const long int a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const int a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const unsigned int a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const short a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const unsigned short a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const char a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const unsigned char a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const double a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a child node of the current node with a specific name and value.
    bool setValue(const std::string& a_name, const std::string& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setValue(a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, bool a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, long int a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, int a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, unsigned int a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, short a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, unsigned short a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, char a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, unsigned char a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, double a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, float a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Create or set a specific attribute and its value for the current node .
    bool setAttribute(const std::string& a_name, const int a_index, const std::string& a_attribute, std::string a_val) { if (gotoChild(a_name, a_index, true) < 0) return false; bool res = setAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, bool& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, long int& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, int& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, unsigned int& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, short& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, unsigned short& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, char& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, unsigned char& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, float& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, double& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific child node of the current node.
    bool getValue(const std::string& a_name, std::string& a_val, int a_index = 0) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getValue(a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, bool& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, long int& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, int& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, unsigned int& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, short& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, unsigned short& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, char& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, unsigned char& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, float& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, double& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }

    //! Get the value of a specific attribute of a specific child node of the current node.
    bool getAttribute(const std::string& a_name, int a_index, const std::string& a_attribute, std::string& a_val) { if (gotoChild(a_name, a_index, false) < 0) return false; bool res = getAttribute(a_attribute, a_val); gotoParent(); return res; }
};

//@}

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

