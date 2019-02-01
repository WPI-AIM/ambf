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
    \version   3.2.0 $Rev: 2159 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CDisplayListH
#define CDisplayListH
//------------------------------------------------------------------------------
#include "system/CGlobals.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CDisplayList.h
    
    \brief
    Implementation of class that supports OpenGL display lists.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cDisplayList
    \ingroup    graphics

    \brief
    This class provides support for OpenGL display lists.

    \details
    This class provides support for OpenGL display lists.
*/
//==============================================================================
class cDisplayList
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
 
public:

    //! Constructor of cDisplayList.
    cDisplayList();
   
    //! Destructor of cDisplayList.
    ~cDisplayList();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method invalidates the current display list. Request for update.
    void invalidate();

    //! This method renders the display list.
    bool render(const bool a_useDisplayList);

    //! This method begins the creation of a display list.
    bool begin(const bool a_useDisplayList);

    //! This method finalizes the creation of a display list.
    void end(const bool a_executeDisplayList);

    //! This method returns the OpenGL display list number.
    unsigned int getDisplayListGL() { return (m_displayList); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:
    
    //! OpenGL display list.
    unsigned int m_displayList;

    //! If __true__, then the display list is currently in the process of being created.
    bool m_flagCreatingDisplayList;

    //! If __true__, then the display list should be deleted.
    bool m_flagMarkedForDeletion;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
