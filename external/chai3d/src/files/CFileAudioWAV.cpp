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
    \version   3.2.0 $Rev: 2015 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "files/CFileAudioWAV.h"
//------------------------------------------------------------------------------
#include "math/CConstants.h"
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
using namespace chai3d;
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------

struct C_RIFF_Header
{
    char         chunkID[4];
    unsigned int chunkSize;
    char         format[4];
};

struct C_WAVE_Format
{
    char           subChunkID[4];
    unsigned int   subChunkSize;
    unsigned short audioFormat;
    unsigned short numChannels;
    unsigned int   sampleRate;
    unsigned int   byteRate;
    unsigned short blockAlign;
    unsigned short bitsPerSample;
};

struct C_WAVE_Data
{
    char         subChunkID[4];
    unsigned int subChunk2Size;
};

//------------------------------------------------------------------------------
#endif  // DOXYGEN_SHOULD_SKIP_THIS
//------------------------------------------------------------------------------


//==============================================================================
/*!
    This function loads an audio WAV file into memory. \n
    If the operation succeeds, then the functions returns __true__ and the 
    audio data is loaded into memory. The audio data, size, frequency and format are
    returned by argument. \n
    If the operation fails, then the function returns __false__. 

    \param  a_filename       Filename.
    \param  a_data           Returned pointer to audio data.
    \param  a_size           Returned number of samples contained in WAV file.
    \param  a_frequency      Returned sample frequency of WAV file.
    \param  a_stereo         Returns __true__ if stereo, __false__ otherwise.
    \param  a_bitsPerSample  Returned number of bits per sample (8 or 16).

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cLoadFileWAV(const std::string& a_filename, 
    unsigned char*& a_data,
    int* a_size,
    int* a_frequency,
    bool* a_stereo,
    unsigned short* a_bitsPerSample)
{
    FILE* soundFile = NULL;
    C_WAVE_Format wave_format;
    C_RIFF_Header riff_header;
    C_WAVE_Data wave_data;

    // load audio file
    soundFile = fopen(a_filename.c_str(), "rb");

    // check if file exists
    if (!soundFile)
    {
        return (C_ERROR);
    }

    // read in the first chunk into the struct
    if (fread(&riff_header, sizeof(C_RIFF_Header), 1, soundFile) <= 0)
    {
      fclose(soundFile);
      return (C_ERROR);
    }

    // check for RIFF and WAVE tag in memory
    if ((riff_header.chunkID[0] != 'R' ||
         riff_header.chunkID[1] != 'I' ||
         riff_header.chunkID[2] != 'F' ||
         riff_header.chunkID[3] != 'F')
        ||
        (riff_header.format[0] != 'W' ||
         riff_header.format[1] != 'A' ||
         riff_header.format[2] != 'V' ||
         riff_header.format[3] != 'E'))
    {
        fclose(soundFile);
        return (C_ERROR);
    }

    // read in the 2nd chunk for the wave info
    if (fread(&wave_format, sizeof(C_WAVE_Format), 1, soundFile) <= 0)
    {
      fclose(soundFile);
      return (C_ERROR);
    }

    // check for fmt tag in memory
    if (wave_format.subChunkID[0] != 'f' ||
        wave_format.subChunkID[1] != 'm' ||
        wave_format.subChunkID[2] != 't' ||
        wave_format.subChunkID[3] != ' ')
    {
        fclose(soundFile);
        return (C_ERROR);
    }

    // check for extra parameters;
    if (wave_format.subChunkSize > 16)
        fseek(soundFile, sizeof(short), SEEK_CUR);

    // read in the the last byte of data before the sound file
    if (fread(&wave_data, sizeof(C_WAVE_Data), 1, soundFile) <= 0)
    {
      fclose(soundFile);
      return (C_ERROR);
    }

    // check for data tag in memory
    if (wave_data.subChunkID[0] != 'd' ||
        wave_data.subChunkID[1] != 'a' ||
        wave_data.subChunkID[2] != 't' ||
        wave_data.subChunkID[3] != 'a')
    {
        fclose(soundFile);
        return (C_ERROR);
    }

    // allocate memory for data
    a_data = new unsigned char[wave_data.subChunk2Size];

    // read in the sound data into the soundData variable
    if (!fread(a_data, wave_data.subChunk2Size, 1, soundFile))
    {
        fclose(soundFile);
        return (C_ERROR);
    }

    // now we set the variables that we passed in with the
    // data from the structs
    *a_size = wave_data.subChunk2Size;
    *a_frequency = wave_format.sampleRate;

    // the format is worked out by looking at the number of
    // channels and the bits per sample.
    *a_bitsPerSample = wave_format.bitsPerSample;
    if (wave_format.numChannels == 1) 
    {
        *a_stereo = false;
    }
    else if (wave_format.numChannels == 2) 
    {
        *a_stereo = true;
    }

    // close file
    fclose(soundFile);

    // return success
    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
