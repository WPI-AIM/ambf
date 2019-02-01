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
    \version   3.2.0 $Rev: 2017 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "AL/al.h"
#include "AL/alc.h"
//------------------------------------------------------------------------------
#include "audio/CAudioSource.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cAudioSource.
*/
//==============================================================================
cAudioSource::cAudioSource()
{
    // init data
    m_source = 0;
    m_pitch = 1.0f;
    m_gain = 1.0f;
    m_sourcePos.zero();
    m_sourceVel.zero();
    m_loop = false;
    m_audioBuffer = NULL;

    // create audio source
    alGenSources(1, &m_source);
};


//==============================================================================
/*!
    Destructor of cAudioSource.
*/
//==============================================================================
cAudioSource::~cAudioSource()
{
    // delete OpenAL audio source
    if (m_source != 0)
    {
        alDeleteSources(1, &m_source);
    }
}


//==============================================================================
/*!
    This method assigns an audio buffer to this audio source.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::setAudioBuffer(cAudioBuffer* a_audioBuffer)
{
    // set buffer
    m_audioBuffer = a_audioBuffer;
    alSourcei(m_source, AL_BUFFER, m_audioBuffer->getBuffer());

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method begins playing the audio source.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::play()
{
    // play source
    alSourcePlay(m_source);

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method checks if the audio source is currently playing.

    \return __true__ if audio source is playing, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::isPlaying()
{
    ALint state = false;
    alGetSourcei(m_source, AL_SOURCE_STATE, &state);
    if (state == AL_PLAYING)
    {
        return (true);
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    This method pauses the audio source from playing.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::pause()
{
    // pause source
    alSourcePause(m_source);

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method stops the audio source from playing.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::stop()
{
    // stop audio source
    alSourceStop(m_source);

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method rewinds the audio source.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::rewind()
{
    // rewind source
    alSourceRewind(m_source);

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method sets the playing position of the audio source to a desired
    value \p a_time passed as argument and defined in seconds.

    \param  a_time  Playing position defined in seconds.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::setPosTime(const ALfloat a_time)
{
    // set playing position
    alSourcef(m_source, AL_SEC_OFFSET, a_time);

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method returns the current playing position of the audio source.

    \return Current playing position time in seconds.
*/
//==============================================================================
ALfloat cAudioSource::getPosTime()
{
    ALfloat pos = 0.0;
    alGetSourcef(m_source, AL_SEC_OFFSET, &pos);
    return (pos);
}


//==============================================================================
/*!
    This method returns the left sample at a given time.

    \param  a_time  Sample time.

    \return Current playing position time in seconds.
*/
//==============================================================================
short cAudioSource::getSampleLeft(double a_time)
{
    if (m_audioBuffer == NULL)
    {
        return (0);
    }
    else
    {
        return (m_audioBuffer->getSampleLeft(a_time, m_loop));
    }
}


//==============================================================================
/*!
    This method returns the right sample at a given time.

    \param  a_time  Sample time.

    \return Current playing position time in seconds.
*/
//==============================================================================
short cAudioSource::getSampleRight(double a_time)
{
    if (m_audioBuffer == NULL)
    {
        return (0);
    }
    else
    {
        return (m_audioBuffer->getSampleRight(a_time, m_loop));
    }
}


//==============================================================================
/*!
    This method sets the audio pitch.

    \param  a_pitch  Pitch value. This value must be positive.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::setPitch(const ALfloat a_pitch)
{
    // store new value
    m_pitch = a_pitch;

    // set pitch property
    alSourcef(m_source, AL_PITCH, fabs(m_pitch));

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method sets the audio gain.

    \param  a_gain  Gain value. The value must be positive.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::setGain(const ALfloat a_gain)
{
    // store new value
    m_gain = a_gain;

    // set gain property
    alSourcef(m_source, AL_GAIN, fabs(m_gain));

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method sets the position of the audio source in world coordinates.

    \param  a_sourcePos  Position of the audio source in world coordinates.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::setSourcePos(const cVector3d a_sourcePos)
{
    // store new value
    m_sourcePos = a_sourcePos;

    // set position of source
    float pos[3];
    pos[0] = (float)(m_sourcePos(0));
    pos[1] = (float)(m_sourcePos(1));
    pos[2] = (float)(m_sourcePos(2));
    alSourcefv(m_source, AL_POSITION, pos);

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method sets the velocity of the audio source in world coordinates.

    \param  a_sourceVel  Velocity of the audio source in world coordinates.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::setSourceVel(const cVector3d a_sourceVel)
{
    // store new value
    m_sourceVel = a_sourceVel;

    // set velocity of source
    float vel[3];
    vel[0] = (float)m_sourceVel(0);
    vel[1] = (float)m_sourceVel(1);
    vel[2] = (float)m_sourceVel(2);
    alSourcefv(m_source, AL_VELOCITY, vel);

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method enables or disables loop playing. Set argument \p a_loop to 
    __true__ to enable loop playing, __false__ otherwise.

    \param  a_loop  Loop playing mode.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::setLoop(const bool a_loop)
{
    // store new value
    m_loop = a_loop;

    // set loop property
    if (a_loop)
    {
        alSourcei(m_source, AL_LOOPING, AL_TRUE);
    }
    else
    {
        alSourcei(m_source, AL_LOOPING, AL_FALSE);
    }

    // check for errors
    return (checkError());
}


//==============================================================================
/*!
    This method checks for any OpenAL errors.

    \return __true__ if no errors occurred, __false__ otherwise.
*/
//==============================================================================
bool cAudioSource::checkError()
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
