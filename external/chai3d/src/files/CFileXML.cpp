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
    \version   3.2.0 $Rev: 2173 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "files/CFileXML.h"
//------------------------------------------------------------------------------
#include "pugixml.hpp"
using namespace pugi;
//------------------------------------------------------------------------------
#include <string>
#include <sstream>
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

struct XML
{
    xml_node      m_rootNode;
    xml_node      m_currentNode;
    xml_document  m_document;
    std::string   m_filename;

    XML()
    {
        m_filename = "";
        m_currentNode = m_document;
    }
};

//------------------------------------------------------------------------------
#endif // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//==============================================================================
/*!
    This function converts a __string__ into a __long int__.

    \param  a_str   Input __string__ to convert.

    \return The converted __long int__ value if the string is valid, 0 otherwise.
*/
//==============================================================================
static inline long int strToInt(const std::string& a_str)
{
  std::istringstream i(a_str);
  int x = 0;
  if (!(i >> x)) return 0;
  else return x;
}


//==============================================================================
/*!
    This function converts a __string__ into a __double__.

    \param  a_str   Input __string__ to convert.

    \return The converted __double__ value if the string is valid, 0.0 otherwise.
*/
//==============================================================================
static inline double strToDouble(const std::string& a_str)
{
  std::istringstream i(a_str);
  double x = 0.0;
  if (!(i >> x)) return 0.0;
  else return x;
}


//==============================================================================
/*!
    Constructor of cFileXML.
*/
//==============================================================================
cFileXML::cFileXML()
{
    m_xml = (void*)(new XML);
}


//==============================================================================
/*!
    Constructor of cFileXML for loading a specific file.

    \param  a_filename  XML file to load.
*/
//==============================================================================
cFileXML::cFileXML(const string& a_filename)
{
    m_xml = (void*)(new XML);

    loadFromFile(a_filename);
}


//==============================================================================
/*!
    cFileXML destructor.

    \note The destructor does not save the XML data back into the file loaded
          using \ref loadFromFile(). Remember to call \ref saveToFile() as required.
*/
//==============================================================================
cFileXML::~cFileXML()
{
  delete (XML*)(m_xml);
}


//==============================================================================
/*!
    Load XML data from a given file. The XML data is stored internally, and a
    pointer to the current XML node is kept internally. The XML data can be
    navigated using \ref gotoChild(), \ref gotoParent() and other related methods.
    If the file does not exist, it will be created and written to disk when calling
    \ref saveToFile().

    \param  a_filename  Name of the XML file to load.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::loadFromFile(const string& a_filename)
{
    // store filename for use by the save() method
    ((XML*)(m_xml))->m_filename = a_filename;

    // load XML content (even if file is empty with no document elements)
    xml_parse_result res = ((XML*)(m_xml))->m_document.load_file(a_filename.c_str());
    if (res.status == status_ok || res.status == status_no_document_element)
    {
        // make sure the current node pointer is set to the root of the XML data
        gotoRoot();

        // success
        return true;
    }

    // if file loading failed, return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Save the current XML data using the filename saved from the call
    to \ref loadFromFile(). If the XML data was not loaded from a file, the method
    fails.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::saveToFile()
{
    // save XML data using the current filename
    return saveToFile(((XML*)(m_xml))->m_filename);
}


//==============================================================================
/*!
    Save the current XML data to a given file. If the XML data is empty, or the
    filename invalid, the method fails.

    \param  a_filename  The name of the file to save the XML data to.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::saveToFile(const string& a_filename)
{
    // check that filename is valid
    if (((XML*)(m_xml))->m_filename == "")
    {
        return false;
    }

    // save the file
    if (((XML*)(m_xml))->m_document.save_file(a_filename.c_str()))
    {
        return  true;
    }

    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Remove all XML data.
*/
//==============================================================================
void cFileXML::clear()
{
    // remove all XML data
    ((XML*)(m_xml))->m_document.reset();

    return gotoRoot();
}


//==============================================================================
/*!
    Set the current node pointer to the XML data root.
*/
//==============================================================================
void cFileXML::gotoRoot()
{
    // point to the document root
    ((XML*)(m_xml))->m_currentNode = ((XML*)(m_xml))->m_document;
}


//==============================================================================
/*!
    Set the current node pointer to a given child of the current node. Optionally,
    create the child node if it does not exist.

    \param  a_name      Name of the child node to navigate to.
    \param  a_index     Index of the child, used when several children have
                        the same name.
    \param  a_create    Node creation flag: if the specified child node does not
                        exist and __a_create__ is set to __true__, the node will
                        be created.

    \return Return 0 if child node existed and operation succeeded,
                   1 if the child node did not exist and node creation succeded,
                  -1 otherwise.
*/
//==============================================================================
int cFileXML::gotoChild(const string& a_name, int a_index, bool a_create)
{
    // navigate to first child with matching name
    xml_node node = ((XML*)(m_xml))->m_currentNode.child(a_name.c_str());

    // navigate to the matching child at the desired index
    for (int index=0; index<a_index; index++)
    {
        node = node.next_sibling(a_name.c_str());
    }

    // if desired node exists, set current node to it and return 0
    if (!node.empty())
    {
        ((XML*)(m_xml))->m_currentNode = node;

        return 0;
    }

    // if desired node does not exist
    else
    {
        // if we are not supposed to create it, return error
        if (!a_create)
        {
            return -1;
        }

        // otherwise, create node and return 1
        else
        {
            ((XML*)(m_xml))->m_currentNode = ((XML*)(m_xml))->m_currentNode.append_child(a_name.c_str());

            return 1;
        }
    }
}


//==============================================================================
/*!
    Remove a specific child node from the current node.

    \param  a_name      Name of the child node to navigate to.
    \param  a_index     Index of the child, used when several children have
                        the same name.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::removeChild(const string& a_name, int a_index)
{
    // navigate to first child with matching name
    xml_node node = ((XML*)(m_xml))->m_currentNode.child(a_name.c_str());

    // navigate to the matching child at the desired index
    for (int index=0; index<a_index; index++)
    {
        node = node.next_sibling(a_name.c_str());
    }

    // if desired node exists, remove it and return success
    if (!node.empty ())
    {
        ((XML*)(m_xml))->m_currentNode.remove_child(node);

        return true;
    }

    // otherwise, return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Set the current node pointer to the parent of the current node.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::gotoParent()
{
    // set current pointer to parent node
    ((XML*)(m_xml))->m_currentNode = ((XML*)(m_xml))->m_currentNode.parent();

    return true;
}


//==============================================================================
/*!
    Set the current node pointer to the first child of the current node.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::gotoFirstChild()
{
    // set current pointer to first child
    xml_node node = ((XML*)(m_xml))->m_currentNode.first_child();

    // if child exists, return success
    if (!node.empty())
    {
        ((XML*)(m_xml))->m_currentNode = node;

        return true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Set the current node pointer to the next sibling of the current node.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::gotoNextSibling()
{
    // set current pointer to next sibling
    xml_node node = ((XML*)(m_xml))->m_currentNode.next_sibling();

    // if node exists, return success
    if (!node.empty())
    {
        ((XML*)(m_xml))->m_currentNode = node;

        return true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Get the name of the current node.

    \param  a_name  Holds the name of the current node on success.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::getName(string& a_name) const
{
    // check that we are not trying to get the root node name
    if (((XML*)(m_xml))->m_currentNode == ((XML*)(m_xml))->m_document)
    {
      return false;
    }

    // retrieve current node name
    a_name = ((XML*)(m_xml))->m_currentNode.name ();

    return true;
}


//==============================================================================
/*!
    Set the name of the current node.

    \param  a_name  Name to assign to the current node.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::setName(const string& a_name)
{
    // check that we are not trying to rename the root node
    if (((XML*)(m_xml))->m_currentNode == ((XML*)(m_xml))->m_document)
    {
      return false;
    }

    // otherwise, assign name to current node
    else 
    {
      ((XML*)(m_xml))->m_currentNode.set_value(a_name.c_str());

      return true;
    }

}


//==============================================================================
/*!
    Get the value of the current node.

    \param  a_val  Holds the value of the current node on success.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::getValue(bool& a_val) const
{
    string tmp;

    // retrieve current node value as a string
    if (getValue(tmp))
    {
        // convert string to bool
        if (tmp == "1")
        {
            a_val = true;
        }
        else
        {
            a_val = false;
        }

        // success
        return true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Get the value of the current node.

    \param  a_val  Holds the value of the current node on success.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::getValue(long int& a_val) const
{
    string tmp;

    // retrieve current node value as a string
    if (getValue(tmp))
    {
        // convert string to __long int__
        a_val = strToInt(tmp);

        // success
        return true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Get the value of the current node.

    \param  a_val  Holds the value of the current node on success.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::getValue(double& a_val) const
{
    string tmp;

    // retrieve current node value as a string
    if (getValue(tmp))
    {
        // convert string to __double__
        a_val = strToDouble(tmp);

        // success
        return true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Get the value of the current node.

    \param  a_val  Holds the value of the current node on success.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::getValue(string& a_val) const
{
    // check that we are not trying to get the root node value
    if (((XML*)(m_xml))->m_currentNode == ((XML*)(m_xml))->m_document)
    {
      return false;
    }

    // retrieve current node value as string
    a_val = string(((XML*)(m_xml))->m_currentNode.child_value());

    // if string is non null, return success
    if (a_val.length() > 0)
    {
        return  true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Set current node value.

    \param  a_val   Node value to assign to current node.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::setValue(const bool a_val)
{
    ostringstream o;

    // create string from value
    if (a_val)
    {
        o << "1";
    }
    else
    {
        o << "0";
    }

    // write to current node
    return setValue(o.str());
}


//==============================================================================
/*!
    Set current node value.

    \param  a_val   Node value to assign to current node.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::setValue(const long int a_val)
{
    ostringstream o;

    // create string from value
    o << a_val;

    // write to current node
    return setValue(o.str());
}


//==============================================================================
/*!
    Set current node value.

    \param  a_val   Node value to assign to current node.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::setValue(const double a_val)
{
    ostringstream o;

    // create string from value
    o << a_val;

    // write to current node
    return setValue(o.str());
}


//==============================================================================
/*!
    Set current node value.

    \param  a_val   Node value to assign to current node.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::setValue(const string a_val)
{
    // check that we are not trying to set the value of the root node
    if (((XML*)(m_xml))->m_currentNode == ((XML*)(m_xml))->m_document)
    {
        return false;
    }

    // write string to current node value
    if (((XML*)(m_xml))->m_currentNode.append_child(node_pcdata).set_value (a_val.c_str()))
    {
        return  true;
    }
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Get the value of a specific attribute of the current node.

    \param  a_attribute String holding the name of the attribute.
    \param  a_val       Holds the value of the requested attribute on success.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::getAttribute(const string& a_attribute, bool& a_val) const
{
    string tmp;

    // retrieve attribute value as a string
    if(getAttribute(a_attribute, tmp))
    {
        // convert to __bool__
        if (tmp == "1")
        {
            a_val = true;
        }
        else
        {
            a_val = false;
        }

        // success
        return true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Get the value of a specific attribute of the current node.

    \param  a_attribute String holding the name of the attribute.
    \param  a_val       Holds the value of the requested attribute on success.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::getAttribute(const string& a_attribute, long int& a_val) const
{
    string tmp;

    // retrieve attribute value as a string
    if(getAttribute(a_attribute, tmp))
    {
        // convert to __long int__
        a_val = strToInt(tmp);

        // success
        return true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Get the value of a specific attribute of the current node.

    \param  a_attribute String holding the name of the attribute.
    \param  a_val       Holds the value of the requested attribute on success.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::getAttribute(const string& a_attribute, double& a_val) const
{
    string tmp;

    // retrieve attribute value as a string
    if (getAttribute(a_attribute, tmp))
    {
        // convert to __double__
        a_val = strToDouble(tmp);

        // success
        return true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Get the value of a specific attribute of the current node.

    \param  a_attribute String holding the name of the attribute.
    \param  a_val       Holds the value of the requested attribute on success.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::getAttribute(const string& a_attribute, string& a_val) const
{
    // retrieve attribute value
    a_val = string(((XML*)(m_xml))->m_currentNode.attribute(a_attribute.c_str()).as_string());

    // if string is non null, return success
    if (a_val.length() > 0)
    {
        return  true;
    }

    // otherwise return failure
    else
    {
        return false;
    }
}


//==============================================================================
/*!
    Set an attribute value for the current node.

    \param  a_attribute String holding the name of the attribute to set.
    \param  a_val       Attribute value to assign to current node attribute.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::setAttribute(const string& a_attribute, const bool a_val)
{
    ostringstream o;

    // convert __string__
    if (a_val)
    {
        o << "1";
    }
    else
    {
        o << "0";
    }

    // write string to attribute
    return setAttribute(a_attribute, o.str());
}


//==============================================================================
/*!
    Set an attribute value for the current node.

    \param  a_attribute String holding the name of the attribute to set.
    \param  a_val       Attribute value to assign to current node attribute.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::setAttribute(const string& a_attribute, const long int a_val)
{
    ostringstream o;

    // convert to __string__
    o << a_val;

    // write string to attribute
    return setAttribute(a_attribute, o.str());
}


//==============================================================================
/*!
    Set an attribute value for the current node.

    \param  a_attribute String holding the name of the attribute to set.
    \param  a_val       Attribute value to assign to current node attribute.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::setAttribute(const string& a_attribute, const double a_val)
{
    ostringstream o;

    // convert to __string__
    o << a_val;

    // write string to attribute
    return setAttribute(a_attribute, o.str());
}


//==============================================================================
/*!
    Set an attribute value for the current node.

    \param  a_attribute String holding the name of the attribute to set.
    \param  a_val       Attribute value to assign to current node attribute.

    \return __true__ if in case of success, __false__ otherwise.
*/
//==============================================================================
bool cFileXML::setAttribute(const string& a_attribute, const string a_val)
{
    // set attribute value
    ((XML*)(m_xml))->m_currentNode.append_attribute(a_attribute.c_str()) = a_val.c_str();

    // success
    return true;
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
