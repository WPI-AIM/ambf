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
    \version   3.2.0 $Rev: 2199 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <cstring>
#include <iostream>
#include <iomanip>
#include <ostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>
using namespace std;
//---------------------------------------------------------------------------
#include "chai3d.h"
using namespace chai3d;
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// load shader from file
int loadFromFile(const string &shadername, vector<string> &vert, vector<string> &frag)
{
    // build helper data struct
    vector<string> extension;
    extension.push_back("vert");
    extension.push_back("frag");
    map<string, vector<string>*> res;
    res["vert"] = &vert;
    res["frag"] = &frag;

    // load each file
    unsigned long maxLen = 0;
    for (auto ext : extension)
    {
        // open file
        string filename = shadername + "." + ext;
        ifstream in(filename.c_str());
        if(!in.is_open()) return -1;

        // parse each line
        string str;
        while(getline(in, str))
        {
            // remove end-of-line characters
            str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
            str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());

            // escape special characters
            for (unsigned long int i=0; i<str.length(); i++)
            {
                switch (str.at(i)) {
                case '\\':
                case '\"':
                    str.insert(i++, "\\");
                    break;
                }
            }

            // store line
            (*(res[ext])).push_back(str);

            // keep track of the lenght of the longest line
            maxLen = max(maxLen, (unsigned long)(str.length()));
        }
        in.close();

        // remove trailing empty lines
        while((*(res[ext])).back().length() == 0)
        {
            (*(res[ext])).pop_back();
        }
    }

    // make all strings the same length
    for (auto ext : extension)
    {
        for (auto &str : (*(res[ext])))
        {
            string blank(maxLen - str.length(), ' ');
            str = "    " + str + blank + "    ";
        }
    }

    return 0;
}


// header file printer
int writeHeader(const vector<string> &vert, const vector<string> &frag, string shadername, string filename)
{
    // convert shadername to root name
    shadername = shadername.substr(shadername.find_last_of('/')+1, shadername.length());
    replace(shadername.begin(), shadername.end(), '.', '_');
    replace(shadername.begin(), shadername.end(), ' ', '_');
    replace(shadername.begin(), shadername.end(), '\'', '_');
    replace(shadername.begin(), shadername.end(), '-', '_');

    ofstream out(filename.c_str());

    out << "//===========================================================================" << endl;
    out << "/*" << endl;
    out << "    Header file containing the \"" << shadername << "\" shader." << endl;
    out << endl;
    out << "    Automatically generated using CHAI3D visualization and haptics library." << endl;
    out << "    http://www.chai3d.org" << endl;
    out << "*/" << endl;
    out << "//===========================================================================" << endl;

    // convert filename to root name
    filename = filename.substr(filename.find_last_of('/')+1, filename.length());
    filename = filename.substr(0, filename.find_first_of('.'));
    replace(filename.begin(), filename.end(), '.', '_');
    replace(filename.begin(), filename.end(), ' ', '_');
    replace(filename.begin(), filename.end(), '\'', '_');
    replace(filename.begin(), filename.end(), '-', '_');
    filename += "H";

    // upper-case shader name for variable creation
    transform(shadername.begin(), shadername.end(), shadername.begin(), ::toupper);

    // avoid conflicts
    out << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#ifndef " << filename << endl;
    out << "#define " << filename << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // include headers (required by convenience functions)
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#include \"system/CGlobals.h\"" << endl;
    out << "//---------------------------------------------------------------------------" << endl;

    // namespace
    out << "namespace chai3d {" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // print vert section info
    out << "const std::string C_SHADER_" << shadername << "_VERT = ";
    for (auto line : vert)
    {
        out << endl << "\"" << line << "\\n\"";
    }
    out << ";" << endl;

    // separator
    out << endl << endl;

    // print frag section info
    out << "const std::string C_SHADER_" << shadername << "_FRAG = ";
    for (auto line : frag)
    {
        out << endl << "\"" << line << "\\n\"";
    }
    out << ";" << endl;

    out << endl << endl;

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
    cout << endl << "cshader name [-o header.h] [-h]" << endl;
    cout << "\tname\tshader base filename (expects name.frag and name.vert files)" << endl;
    cout << "\t-o\tspecify output header filename (default is CShader<Name>.h)" << endl;
    cout << "\t-h\tdisplay this message" << endl << endl;

    return -1;
}


//===========================================================================
/*
    UTILITY:    cshader.cpp

    This utility converts a vertex and fragment shader into a C/C++ compatible
    header. This allows programmers to easily embed said shaders into their
    executables.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    string shadername;
    string filename;
    string altfilename;

    // process arguments
    if (argc < 2) return usage();
    for (int i=1; i<argc; i++)
    {
        if (argv[i][0] != '-') {
            if (shadername.length() > 0) return usage();
            shadername = string(argv[i]);
            filename = "CShader" + shadername;
            filename[7] = toupper(shadername[0]);
            filename += ".h";
        }
        else switch (argv[i][1]) {
        case 'o':
            if ((i < argc) && (argv[i+1][0] != '-')) {
                i++;
                altfilename = string(argv[i]);
            }
            else return usage ();
            break;

        case 'h':
            return usage ();
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
    cout << "Shader Converter" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl;
    cout << endl;

    // report action
    cout << "converting " << shadername << " .frag and " << shadername << ".vert to " << filename << "..." << endl;

    // read vert and frag
    vector<string> vert;
    vector<string> frag;
    if (loadFromFile(shadername, vert, frag) < 0)
    {
        cout << "error: cannot load shader from " << shadername << " .frag and " << shadername << ".vert" << endl;
        return -1;
    }
    else
    {
        cout << "shader vertex and fragment loaded successfully" << endl;
    }
    cout << endl;

    // export shader
    if (writeHeader (vert, frag, shadername, filename) < 0)
    {
        cout << "error: conversion failed" << endl;
        return -1;
    }
    else
    {
        cout << "conversion succeeded" << endl;
    }

    return 0;
}

//---------------------------------------------------------------------------
