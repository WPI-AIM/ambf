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
    \version   3.2.0 $Rev: 2158 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CAudioDeviceH
#define CAudioDeviceH
//------------------------------------------------------------------------------
#include "audio/CAudioBuffer.h"
#include "audio/CAudioSource.h"
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CAudioDevice.h

    \brief 
    Implements an audio device for listening to audio sources.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cAudioDevice
    \ingroup    audio

    \brief
    This class implements an audio device context.

    \details
    This class implements an audio display context for outputting sounds to loud 
    speakers. Optionally, 3D spatial sound is modeled by defining the position and 
    orientation of a listener placed inside the virtual world (cWorld)
*/
//==============================================================================
class cAudioDevice
{    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cAudioDevice.
    cAudioDevice(bool a_createAudioContext = true);

    //! Destructor of cAudioDevice.
    virtual ~cAudioDevice();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method sets the position of the listener in world coordinates.
    bool setListenerPos(const cVector3d& a_listenerPos);

    //! This method returns the position of the listener in world coordinates.
    cVector3d getListenerPos() { return (m_listenerPos); }

    //! This method sets the orientation of the listener in world coordinates.
    bool setListenerRot(const cMatrix3d& a_listenerRot);

    //! This method sets orientation of the listener in world coordinates.
    bool setListenerRot(const cVector3d& a_lookAt, const cVector3d& a_up); 

    //! This method returns the position of the listener in world coordinates.
    cMatrix3d getListenerRot() { return (m_listenerRot); }

    //! This method sets the linear velocity of the listener in world coordinates.
    bool setListenerVel(const cVector3d& a_listenerVel);

    //! This method sets the linear velocity of the listener in world coordinates.
    cVector3d getListenerVel() { return (m_listenerVel); }


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method opens this audio device context.
    bool open();

    //! This method closes this audio device context.
    bool close();

    //! This method checks for any OpenAL errors.
    bool checkError();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Audio device (of type ALCdevice*).
    void* m_device;
    
    //! Audio context (of type ALCcontext*).
    void* m_context;

    //! Position of listener in world coordinates.
    cVector3d m_listenerPos;

    //! Orientation of listener in world coordinates.
    cMatrix3d m_listenerRot;

    //! Linear velocity of listener in world coordinates.
    cVector3d m_listenerVel;

    //! Flag that indicates if the connection to audio device was opened successfully by calling method open().
    bool m_active;

    //! Flag which indicates if this audio device has created an audio context.
    bool m_createAudioContext;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
