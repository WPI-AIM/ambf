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
    \version   3.2.0 $Rev: 2167 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CLevelH
#define CLevelH
//------------------------------------------------------------------------------
#include "widgets/CGenericWidget.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CLevel.h

    \brief
    <Implements a 2D level display widget
*/
//==============================================================================

//==============================================================================
/*!
    \class      cLevel
    \ingroup    widgets

    \brief
    This class implements a 2D level display widget.

    \details
    This class implements a 2D level display widget.
*/
//==============================================================================
class cLevel : public cGenericWidget
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cLevel.
    cLevel();

    //! Destructor of cLevel.
    virtual ~cLevel() {};


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! Color of activated increment lines.
    cColorf m_colorActive;

    //! Color of inactivated increment lines.
    cColorf m_colorInactive;


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of itself.
    virtual cLevel* copy(const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false, 
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = false);

    //! This method sets the width of this cLevel widget.
    void setWidth(const double a_width);

    //! This method sets the number of increments. This value can range from 2 to 200.
    void setNumIncrements(const int a_numIncrements);

    //! This method returns the number of increments.
    inline int getNumIncrements() const { return (m_numIncrements); }

    //! This method sets the range of input values which command the increment lines.
    void setRange(const double a_minValue, const double a_maxValue); 

    //! This method returns the minimum value of the range.
    inline double getRangeMin() const { return (m_minValue); }

    //! This method returns the maximum value of the range.
    inline double getRangeMax() const { return (m_maxValue); }

    //! This method sets a value to be displayed by the level.
    void setValue(const double a_value);

    //! This method returns the current value displayed by the level.
    double getValue() const { return (m_value); }

    //! This method enables or disables the use of single colored line increments to display the value.
    inline void setSingleIncrementDisplay(const bool a_singleIncrementDisplay) { m_flagSingleIncrementDisplay = a_singleIncrementDisplay; }

    //! This method returns the status about the display mode.
    inline bool getSingleIncrementDisplay() const { return (m_flagSingleIncrementDisplay); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Number of increments.
    int m_numIncrements;

    //! Range - minimum value.
    double m_minValue;

    //! Range - maximum value.
    double m_maxValue;

    //! Current value.
    double m_value;

    //! If __true__, then the single segment display is activated.
    bool m_flagSingleIncrementDisplay;


    //--------------------------------------------------------------------------
    // PROTECTED VIRTUAL METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method updates the mesh model.
    virtual void updateLevelMesh();

    //! This method copies all properties of this object to another.
    void copyLevelProperties(cLevel* a_obj,
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
