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
    \version   3.2.0 $Rev: 2181 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "AL/al.h"
#include "AL/alc.h"
//------------------------------------------------------------------------------
#include "audio/CAudioDevice.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cAudioDevice. The constructor takes a parameter which
    indicates if an audio context must be create. In principle this option
    should be set to __true__ unless you are already using a movie player or any 
    other component that is already creating an audio context under OpenAL. Creating 
    multiple contexts is not recommended as the behavior is not predictable across
    different operating systems.

    \param  a_createAudioContext  Indicates if audio contexts should be created.
*/
//==============================================================================
cAudioDevice::cAudioDevice(bool a_createAudioContext)
{
    // initialize all variables
    m_createAudioContext = a_createAudioContext;
    m_active = false;
    m_listenerPos.zero();
    m_listenerRot.identity();
    m_listenerVel.zero();
    m_device = NULL;

    // open device
    open();
};


//==============================================================================
/*!
    Destructor of cAudioDevice.
*/
//==============================================================================
cAudioDevice::~cAudioDevice()
{   
    // close device
    close();
};


//==============================================================================
/*!
    This method opens this audio device context.

    \return __true__ if no errors occurred, __false__ otherwise.
*/
//==============================================================================
bool cAudioDevice::open()
{
    // open device
    if (m_device == NULL)
    {
        m_device = static_cast<void*>(alcOpenDevice(NULL));
    }
    else
    {
        return false;
    }

    if (m_createAudioContext)
    {
        // create context
        m_context = static_cast<void*>(alcCreateContext(static_cast<ALCdevice*>(m_device), NULL));
    
        // set current audio context
        alcMakeContextCurrent(static_cast<ALCcontext*>(m_context));
    }

    // check for errors
    m_active = checkError();

    // set default values
    setListenerPos(cVector3d( 0.0, 0.0, 0.0));
    setListenerRot(cVector3d(-1.0, 0.0, 0.0), cVector3d( 0.0, 0.0, 1.0));
    setListenerVel(cVector3d( 0.0, 0.0, 0.0));

    // success
    return (m_active);
}


//==============================================================================
/*!
    This method closes this audio device context.

    \return __true__ if no errors occurred, __false__ otherwise.
*/
//==============================================================================
bool cAudioDevice::close()
{
    // sanity check
    if (!m_active) { return (C_ERROR); }

    // create audio context
    if (m_createAudioContext)
    {
        // delete context
        alcDestroyContext(static_cast<ALCcontext*>(m_context));
        m_context = NULL;
    }

    // close device
    if (m_device)
    {
        alcCloseDevice(static_cast<ALCdevice*>(m_device));
        m_device = NULL;
    }

    // disable system
    m_active = false;

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method sets the position of the listener in world coordinates.

    \param  a_listenerPos  Position of listener in world coordinates.

    \return __true__ if no errors occurred, __false__ otherwise.
*/
//==============================================================================
bool cAudioDevice::setListenerPos(const cVector3d& a_listenerPos)
{
    // sanity check
    if (!m_active) { return (C_ERROR); }

    // update value
    m_listenerPos = a_listenerPos;

    // send command
    float pos[3];
    pos[0] = (float)m_listenerPos(0);
    pos[1] = (float)m_listenerPos(1);
    pos[2] = (float)m_listenerPos(2);
    alListenerfv(AL_POSITION, pos );

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method sets the orientation of the listener in world coordinates.

    \param  a_listenerRot  Orientation of the listener in world coordinates.

    \return __true__ if no errors occurred, __false__ otherwise.
*/
//==============================================================================
bool cAudioDevice::setListenerRot(const cMatrix3d& a_listenerRot)
{
    cVector3d lookAt = m_listenerPos - a_listenerRot.getCol0();
    cVector3d up = a_listenerRot.getCol2();
    return (setListenerRot(lookAt, up));
}


//==============================================================================
/*!
    This method sets the orientation of the listener in world coordinates.

    \param  a_lookAt  Look at position of listener.
    \param  a_up      Up vector of listener.

    \return __true__ if no errors occurred, __false__ otherwise.
*/
//==============================================================================
bool cAudioDevice::setListenerRot(const cVector3d& a_lookAt, const cVector3d& a_up)
{
    // sanity check
    if (!m_active) { return (C_ERROR); }

    // update value
    m_listenerRot.setCol0(-cNormalize(m_listenerPos - a_lookAt));
    m_listenerRot.setCol2(cNormalize(a_up));
    m_listenerRot.setCol1(cCross(m_listenerRot.getCol2(), m_listenerRot.getCol0()));

    // send command
    float rot[6];
    rot[0] = (float)a_lookAt(0);
    rot[1] = (float)a_lookAt(1);
    rot[2] = (float)a_lookAt(2);
    rot[3] = (float)a_up(0);
    rot[4] = (float)a_up(1);
    rot[5] = (float)a_up(2);
    alListenerfv(AL_ORIENTATION, rot);
    
    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method sets the linear velocity of the listener in world coordinates.

    \param  a_listenerVel  Linear velocity of listener in world coordinates.

    \return __true__ if no errors occurred, __false__ otherwise.
*/
//==============================================================================
bool cAudioDevice::setListenerVel(const cVector3d& a_listenerVel)
{
    // sanity check
    if (!m_active) { return (C_ERROR); }

    // update value
    m_listenerVel = a_listenerVel;

    // send command
    float vel[3];
    vel[0] = (float)m_listenerVel(0);
    vel[1] = (float)m_listenerVel(1);
    vel[2] = (float)m_listenerVel(2);
    alListenerfv(AL_VELOCITY, vel);

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method checks for any OpenAL errors.

    \return __true__ if no errors occurred, __false__ otherwise.
*/
//==============================================================================
bool cAudioDevice::checkError()
{
    if(alGetError() == AL_NO_ERROR)
    {
        return (C_SUCCESS);
    }
    else
    {
        return (C_ERROR);
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
