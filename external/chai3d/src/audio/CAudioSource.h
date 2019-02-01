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
#ifndef CAudioSourceH
#define CAudioSourceH
//------------------------------------------------------------------------------
#include "audio/CAudioBuffer.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CAudioSource.h

    \brief 
    Implements of an audio source.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cAudioSource
    \ingroup    audio

    \brief
    This class implements an audio source.

    \details
    This class implements an audio source. An audio source is a sound generator
    that is placed inside the world at a specific location.
    A sound generator plays sounds that are stored by audio buffers.
*/
//==============================================================================
class cAudioSource
{    
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cAudioSource.
    cAudioSource();

    //! Destructor of cAudioSource.
    virtual ~cAudioSource();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method sets the audio buffer.
    bool setAudioBuffer(cAudioBuffer* a_audioBuffer);

    //! This method returns the audio buffer.
    cAudioBuffer* getAudioBuffer() { return (m_audioBuffer); }

    //! This method plays the audio buffer.
    bool play();

    //! This method returns __true__ if the audio source is currently playing, __false__ otherwise.
    bool isPlaying();

    //! This method pauses the audio source from playing.
    bool pause();

    //! This method stops the audio source from playing.
    bool stop();

    //! This method rewinds the audio source.
    bool rewind();

    //! This method set the current playing position of the audio buffer.
    bool setPosTime(const float a_time);

    //! This method returns the current playing position on audio buffer.
    float getPosTime();

    // This method returns a left sample at a given time.
    short getSampleLeft(double a_time);

    // This method returns a right sample at a given time.
    short getSampleRight(double a_time);

    //! This method sets the audio pitch. The default value is 1.0.
    bool setPitch(const float a_pitch);

    //! This method returns the audio pitch value.
    float getPitch() { return (m_pitch); }

    //! This method sets the audio gain value. Set value to 0.0 to disable sound.
    bool setGain(const float a_gain);

    //! This method returns the audio gain value.
    float getGain() { return (m_gain); }

    //! This method sets the position of the audio source in world coordinates.
    bool setSourcePos(const cVector3d a_sourcePos);

    //! This method returns the position of the audio source in world coordinates.
    cVector3d getSourcePos() { return (m_sourcePos); }

    //! This method sets the velocity of the audio source in world coordinates.
    bool setSourceVel(const cVector3d a_sourceVel);

    //! This method returns the velocity of the audio source in world coordinates.
    cVector3d getSourceVel() { return (m_sourceVel); }

    //! This method enables or disables loop playing.
    bool setLoop(const bool a_loop);

    //! This method returns the status of loop playing.
    bool getLoop() { return (m_loop); }


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method checks for any OpenAL errors.
    bool checkError();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! OpenAL audio source ID.
    unsigned int m_source;

    //! Audio pitch value.
    float m_pitch;

    //! Audio gain value.
    float m_gain;

    //! Position of source in world coordinates.
    cVector3d m_sourcePos;

    //! Velocity of source in world coordinates.
    cVector3d m_sourceVel;

    //! Loop playing mode.
    bool m_loop;

    //! Audio buffer.
    cAudioBuffer* m_audioBuffer;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
