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
#include "audio/CAudioBuffer.h"
#include "files/CFileAudioWAV.h"
#include "math/CMaths.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cAudioBuffer.
*/
//==============================================================================
cAudioBuffer::cAudioBuffer()
{
    // initialize all data (to mono 8 bits-per-sample)
    m_buffer = 0;
    m_filename = "";
    m_data = NULL;
    m_flagDeleteData = false;
    m_buffer = 0;
    m_size = 0;
    m_frequency = 0;
    m_stereo = false;
    m_bitsPerSample = 8;

    // generate OpenAL buffer
    alGenBuffers(1, &m_buffer);

    // check for any errors
    checkError();
};


//==============================================================================
/*!
    Destructor of cAudioBuffer.
*/
//==============================================================================
cAudioBuffer::~cAudioBuffer()
{
    // delete current buffer
    if (m_buffer != 0)
    {
        alDeleteBuffers(1, &m_buffer);
    }

    // cleanup memory
    cleanup();
}


//==============================================================================
/*!
    This method sets a buffer by passing a pointer to the audio data and 
    defines the data specifications which are passed by argument.

    \param  a_data           Pointer to the audio data.
    \param  a_size           Audio data size in bytes.
    \param  a_frequency      Audio data frequency.
    \param  a_stereo         __true__ for stereo, __false__ for mono.
    \param  a_bitsPerSample  Number of bits per sample (8 or 16).

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioBuffer::setup(unsigned char* a_data, const unsigned int a_size, int a_frequency, bool a_stereo, unsigned short a_bitsPerSample)
{
    // sanity check
    if (a_data == NULL)
    { 
        return (C_ERROR); 
    }

    // cleanup previous image
    if (cleanup() == C_ERROR)
    {
        return (C_ERROR);
    }

    // store new properties
    m_data = a_data;
    m_size = a_size;
    m_frequency = a_frequency;
    m_stereo = a_stereo;
    m_bitsPerSample = a_bitsPerSample;

    // mark the audio buffer as being owned externally,
    // and therefore NOT requiring deletion upon destruction
    m_flagDeleteData = false;
 
    // error check
    if (m_data == NULL)
    {
        cleanup();
        return (C_ERROR);
    }

    // determine format
    ALenum format;
    if (m_stereo)
    {
        if (m_bitsPerSample == 8)
            format = AL_FORMAT_STEREO8;
        else
            format = AL_FORMAT_STEREO16;
    }
    else 
    {
        if (m_bitsPerSample == 8)
            format = AL_FORMAT_MONO8;
        else
            format = AL_FORMAT_MONO16;
    }

    // create buffer
    alBufferData(m_buffer,
                 format,
                 m_data,
                 m_size,
                 m_frequency);

    // check error
    return (checkError());
}


//==============================================================================
/*!
    This method loads an audio file by passing the path and name as argument.

    \param  a_filename  Filename.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioBuffer::loadFromFile(const std::string& a_filename)
{
    // cleanup previous image
    if (cleanup() == false)
    {
        return (C_ERROR);
    }

    // find file extension
    string extension = cGetFileExtension(a_filename);

    // we need a file extension to figure out file type
    if (extension.length() == 0)
    {
        return (C_ERROR);
    }

    // convert string to lower extension
    string fileType = cStrToLower(extension);

    // result for loading file
    bool result = false;


    //--------------------------------------------------------------------
    // .WAV FORMAT
    //--------------------------------------------------------------------
    if (fileType == "wav")
    {
        result = cLoadFileWAV(a_filename, 
                              m_data,
                              &m_size, 
                              &m_frequency,
                              &m_stereo,
                              &m_bitsPerSample);
    }

    // mark the audio buffer as being owned internally,
    // and therefore requiring deletion upon destruction
    m_flagDeleteData = true;


    //--------------------------------------------------------------------
    // CREATE BUFFER
    //--------------------------------------------------------------------
    if (result)
    {
        // retrieve format
        ALenum format;
        if (m_stereo)
        {
            if (m_bitsPerSample == 8)
                format = AL_FORMAT_STEREO8;
            else
                format = AL_FORMAT_STEREO16;
        }
        else {
            if (m_bitsPerSample == 8)
                format = AL_FORMAT_MONO8;
            else
                format = AL_FORMAT_MONO16;
        }

        // assign data to the openAL buffer
        alBufferData(m_buffer, 
                     format,
                     m_data,
                     m_size, 
                     m_frequency);

        // check for errors
        result = checkError();
    }

    // return result
    return (result);
}


//==============================================================================
/*!
    This method converts an audio signal from __stereo__ to __mono__.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioBuffer::convertToMono()
{
    // wrong format
    if (!m_stereo)
    {
        return (C_ERROR);
    }

    if (m_bitsPerSample == 8)
    {
        ALbyte* data = (ALbyte*)m_data;
        int size = m_size / 2;
        int numSamples = size / sizeof(ALbyte);

        for (int i=0; i<numSamples; i++)
        {
            data[i] = (ALbyte)(0.5 * (data[2*i] + data[2*i]+1));
        }

        m_size = size;
        m_stereo = false;

        alBufferData(m_buffer, 
                     AL_FORMAT_MONO8,
                     m_data,
                     m_size, 
                     m_frequency);

        return (checkError());
    }
    else if (m_bitsPerSample == 16)
    {
        ALshort* data = (ALshort*)m_data;
        int size = m_size / 2;
        int numSamples = size / sizeof(ALshort);

        for (int i=0; i<numSamples; i++)
        {
            data[i] = ALshort(0.5 * (data[2*i] + data[2*i]+1));
        }

        m_size = size;
        m_stereo = false;

        alBufferData(m_buffer,
                     AL_FORMAT_MONO16,
                     m_data,
                     m_size, 
                     m_frequency);

        return (checkError());
    }

    // wrong format
    return (C_ERROR);
}


//==============================================================================
/*!
    This method clears audio data from memory.

    \return __true__ if operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cAudioBuffer::cleanup()
{
    // cleanup audio data
    if (m_flagDeleteData)
    {
        delete [] m_data;
        m_data = NULL;
    }

    // reset properties (to mono 8 bits-per-sample)
    m_filename = "";
    m_size = 0;
    m_frequency = 0;
    m_stereo = false;
    m_bitsPerSample = 8;

    // return success
    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method returns the number of samples of the audio data. Please note that
    this value does not correspond to the number of bytes of the audio buffer.

    \return Number of audio samples stored in this buffer.
*/
//==============================================================================
int cAudioBuffer::getNumSamples()
{
    int result = 0;

    if (m_stereo)
    {
        if (m_bitsPerSample == 8)
        {
            result = m_size / 2;
        }
        else if (m_bitsPerSample == 16)
        {
            result = m_size / 4;
        }
    }
    else
    {
        if (m_bitsPerSample == 8)
        {
            result = m_size;
        }
        else if (m_bitsPerSample == 16)
        {
            result = m_size / 2;
        }
    }

    return (result);
}


//==============================================================================
/*!
    This method returns the left sample at a given time. If the a_loop argument
    is set to __true__ then audiobuffer will return a sample value as it was 
    playing in loop mode.

    \param  a_time  Sample time.
    \param  a_loop  Loop mode.

    \return Current playing position time in seconds.
*/
//==============================================================================
short cAudioBuffer::getSampleLeft(const double a_time, const bool a_loop)
{
    int numSamples = getNumSamples ();
    int sample = (int)((double)(m_frequency) * a_time);

    if (a_loop)
    {
        sample = sample % numSamples;
    }
    else if (sample >= numSamples)
    {
        return 0;
    }

    int index = 0;
    if (m_stereo)
    {
        if (m_bitsPerSample == 8)
        {
            index = 2 * sample;
        }
        else if (m_bitsPerSample == 16)
        {
            index = 4 * sample;
        }
    }
    else
    {
        if (m_bitsPerSample == 8)
        {
            index = sample;
        }
        else if (m_bitsPerSample == 16)
        {
            index = 2 * sample;
        }
    }

    if (index >= m_size)
    {
        return (0);
    }
    else
    {
        if (m_bitsPerSample == 8)
        {
            return (short)(*((unsigned char*)(m_data+index)));
        }
        else if (m_bitsPerSample == 16)
        {
            return (short)(*((short*)(m_data+index)));
        }
    }

    return (0);
}


//==============================================================================
/*!
    This method returns the right sample at a given time. If the a_loop argument
    is set to __true__ then audiobuffer will return a sample value as it was 
    playing in loop mode.

    \param  a_time  Sample time.
    \param  a_loop  Loop mode.

    \return Current playing position time in seconds.
*/
//==============================================================================
short cAudioBuffer::getSampleRight(const double a_time, const bool a_loop)
{
    int numSamples = getNumSamples ();
    int sample = (int)((double)(m_frequency) * a_time);

    if (a_loop)
    {
      sample = sample % numSamples;
    }
    else if (sample >= numSamples)
    {
      return 0;
    }

    int index = 0;
    if (m_stereo)
    {
      if (m_bitsPerSample == 8)
      {
        index = 2 * sample + 1;
      }
      else if (m_bitsPerSample == 16)
      {
        index = 4 * sample + 2;
      }
    }
    else
    {
      if (m_bitsPerSample == 8)
      {
        index = sample;
      }
      else if (m_bitsPerSample == 16)
      {
        index = 2 * sample;
      }
    }

    if (index >= m_size)
    {
      return (0);
    }
    else
    {
      if (m_bitsPerSample == 8)
      {
        return (short)(*((unsigned char*)(m_data+index)));
      }
      else if (m_bitsPerSample == 16)
      {
        return (short)(*((short*)(m_data+index)));
      }
    }

    return (0);
}


//==============================================================================
/*!
    This methods checks for any OpenAL errors.

    \return __true__ if no errors have occurred, __false__ otherwise.
*/
//==============================================================================
bool cAudioBuffer::checkError()
{
    int result = alGetError();
    if(result == AL_NO_ERROR)
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
