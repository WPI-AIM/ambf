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
    \version   3.2.0 $Rev: 2163 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "graphics/CVideo.h"
//------------------------------------------------------------------------------
#include "system/CThread.h"
//------------------------------------------------------------------------------
#include "TheoraPlayer.h"
#include "OpenAL_AudioInterface.h"
#include <algorithm>
//------------------------------------------------------------------------------
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

// static members
cMutex       cVideo::m_sharedLock;
unsigned int cVideo::m_clipCount = 0;
void        *cVideo::m_manager   = NULL;
void        *cVideo::m_audio     = NULL;

//==============================================================================
/*!
    Default constructor of cVideo.
*/
//==============================================================================
cVideo::cVideo()
{
    // make sure static members are handled serially
    m_sharedLock.acquire();

    // allocate video manager
    if (m_clipCount == 0)
    {
        m_manager = new TheoraVideoManager;
        m_audio   = new OpenAL_AudioInterfaceFactory;

        // assign audio output to manager
        ((TheoraVideoManager*)m_manager)->setAudioInterfaceFactory((OpenAL_AudioInterfaceFactory*)m_audio);
    }

    // increase clip count
    m_clipCount++;

    // it is now safe to check/modify static members
    m_sharedLock.release();

    // init internal variables
    defaults();
}


//==============================================================================
/*!
    Destructor of cVideo.
*/
//==============================================================================
cVideo::~cVideo()
{
    // clean up memory
    cleanup();

    // make sure static members are handled serially
    m_sharedLock.acquire();

    // decrease clip count
    if (m_clipCount > 0)
    {
        m_clipCount--;
    }

    // deallocate video manager if necessary
    if (m_clipCount == 0)
    {
        delete (OpenAL_AudioInterfaceFactory*)m_audio;
        delete (TheoraVideoManager*)m_manager;

        m_audio = NULL;
        m_manager = NULL;
    }

    // it is now safe to check/modify static members
    m_sharedLock.release();
}


//==============================================================================
/*!
    This method initializes internal variables.
*/
//==============================================================================
void cVideo::defaults()
{
    // no filename or video name defined
    m_filename   = "";
    m_name       = "";

    // size of video is zero since not yet defined and allocated
    m_width      = 0;
    m_height     = 0;
    m_frameCount = 0;
    m_duration   = 0.0;
    m_fps        = 0.0;

    // video time and frame pointers are similarly not yet defined
    m_frameIndex = 0;
    m_lastUpdate = 0.0;
    m_firstFrame = false;

    // set play properties
    m_autoReplay = false;

    // video management objects are not allocated
    m_clip       = NULL;
    m_data       = NULL;
}


//==============================================================================
/*!
    This method frees memory that was used for video data, and re-initialize
    internal variables.
*/
//==============================================================================
void cVideo::cleanup()
{
    // stop playback
    stop();

    // delete video data
    if (m_clip != NULL )
    {
        ((TheoraVideoManager*)m_manager)->destroyVideoClip((TheoraVideoClip*)m_clip);
    }

    // delete local buffer
    if (m_data != NULL)
    {
        delete [] m_data;
    }

    // reset to default values
    defaults();
}


//==============================================================================
/*!
    This method loads this video from the specified file. Returns __true__ if all
    goes well.  Note that regardless of whether it succeeds,
    this overwrites any video that had previously been loaded by this object.

    \param  a_filename  Video filename.

    \return __true__ if file loaded successfully, __false__ otherwise.
*/
//==============================================================================
bool cVideo::loadFromFile(const string& a_filename)
{
    // cleanup previous video
    cleanup();
    defaults();

    // find extension
    string extension = cGetFileExtension(a_filename);

    // we need a file extension to figure out file type
    if (extension.length() == 0)
    {
        return (C_ERROR);
    }

    // convert string to lower extension
    string fileType = cStrToLower(extension);

    // result for loading file
    bool result = C_ERROR;

    //--------------------------------------------------------------------
    // .OGG FORMAT
    //--------------------------------------------------------------------
    if (fileType == "ogg" || fileType == "ogv" || fileType == "oga")
    {
        m_clip = ((TheoraVideoManager*)m_manager)->createVideoClip(a_filename);
        if (m_clip)
        {
            m_width = ((TheoraVideoClip*)m_clip)->getWidth();
            m_height = ((TheoraVideoClip*)m_clip)->getHeight();
            m_duration = ((TheoraVideoClip*)m_clip)->getDuration();
            m_frameCount = ((TheoraVideoClip*)m_clip)->getNumFrames();
            m_fps = ((TheoraVideoClip*)m_clip)->getFPS();

            m_filename = a_filename;
            m_name = cGetFilename(a_filename, false);

            m_data = new unsigned char[3 * m_width*m_height];

            reset();

            result = C_SUCCESS;
        }
    }

    return (result);
}


//==============================================================================
/*!
    This method start playing the video, and also starts audio decoding if
    available. At any given moment, the correct frame can be retrieved by calling
    \ref cVideo::getCurrentFramePointer() or \ref cVideo::getCurrentFrame().
*/
//==============================================================================
void cVideo::play()
{
    if (m_clip)
    {
        m_clock.start();
        ((TheoraVideoClip*)m_clip)->play();
    }
}


//==============================================================================
/*!
    This method sets the playback speed as a ratio of normal speed.

    \param  a_speed  Ratio of normal speed to use for playback.
*/
//==============================================================================
void cVideo::setPlaybackSpeed(double a_speed)
{
    if (m_clip)
    {
        ((TheoraVideoClip*)m_clip)->setPlaybackSpeed((float)a_speed);
    }
}


//==============================================================================
/*!
    This method retrieves the playback speed as a ratio of normal speed.

    \return
    The playback speed ratio (1.0 means normal speed). A return value of 0.0
    indicates an error.
*/
//==============================================================================
double cVideo::getPlaybackSpeed()
{
    if (m_clip)
    {
        return (double)((TheoraVideoClip*)m_clip)->getPlaybackSpeed();
    }

    return 0.0;
}


//==============================================================================
/*!
    This method pauses the video if it is playing. The current (frozen) frame
    can still be retrieved with \ref cVideo::getCurrentFramePointer() or
    \ref cVideo::getCurrentFrame(), but will not be updated until
    cVideo::play() is called again.
*/
//==============================================================================
void cVideo::pause()
{
    if (m_clip)
    {
        ((TheoraVideoClip*)m_clip)->pause();
        m_clock.stop();
    }
}


//==============================================================================
/*!
    This method stops the video. This resets the current frame and the time
    position to the beginning of the video. \ref cVideo::getCurrentFramePointer()
    or \ref cVideo::getCurrentFrame() return the first video frame.
*/
//==============================================================================
void cVideo::stop()
{
    reset();
}


//==============================================================================
/*!
    This method controls the auto-replay setting. When auto-replay is enabled,
    the video will automatically start playing again after it reaches the last
    frame. If auto-replay is disabled, \ref cVideo::play() must be called again
    to restart the video.

    \param  a_replay  Set to __true__ to make the movie auto-replay,
                      __false__ to disable auto-replay.
*/
//==============================================================================
void cVideo::setAutoReplay(bool a_replay)
{
    if (m_clip)
    {
        m_autoReplay = a_replay;
    }
}


//==============================================================================
/*!
    This internal method is used to reset the video to its first frame,
    launch the background decoder thread, and wait until enough frames have been
    buffered so that we should be safe to start playing the video.
*/
//==============================================================================
void cVideo::reset()
{
    if (m_clip)
    {
        // put the movie back to the beginning
        m_clock.stop();
        m_clock.reset();
        m_lastUpdate = 0.0;
        ((TheoraVideoClip*)m_clip)->stop();
        m_frameIndex = 0;

        // get initial frame
        TheoraVideoFrame *frame = NULL;
        while (!frame) frame = ((TheoraVideoClip*)m_clip)->getNextFrame();

        // flip first frame
        storeFrame(frame);

        // mark frame as first frame
        m_firstFrame = true;
    }
}


//==============================================================================
/*!
    This method checks if the video is paused, as opposed to playing. The paused
    state is entered after calling \ref cVideo::pause() or \ref cVideo::stop(),
    or after loading a new video using \ref cVideo::loadFromFile().
*/
//==============================================================================
bool cVideo::isPaused()
{
    if (m_clip)
    {
        return (((TheoraVideoClip*)m_clip)->isPaused());
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    This method seeks a given time position in the video stream. If the video
    is playing, keep playing from the new position.

    \param  a_time  The desired time position to seek in the video.

    \return __true__ if the requested time position is valid, __false__ otherwise.
*/
//==============================================================================
bool cVideo::seek(double a_time)
{
    return seekFrame((int)(a_time*m_fps));
}


//==============================================================================
/*!
    This method seeks a given frame in the video stream. If the video is playing,
    keep playing from the new frame position.

    \param  a_index  The desired frame index to seek in the video.

    \return __true__ if the requested frame index is valid, __false__ otherwise.
*/
//==============================================================================
bool cVideo::seekFrame(unsigned int a_index)
{
    // check that requested frame exists, and that video clip exists
    if (!m_clip || a_index >= m_frameCount)
    {
        return (C_ERROR);
    }

    // move clip to desired time
    ((TheoraVideoClip*)m_clip)->seekToFrame(a_index);

    // get initial frame
    TheoraVideoFrame *frame = NULL;
    while (!frame) frame = ((TheoraVideoClip*)m_clip)->getNextFrame();

    // flip frame
    storeFrame(frame);

    return (C_SUCCESS);
}


//==============================================================================
/*!
    This method updates the current frame pointer to the current time. This
    method is used internally by \ref cVideo::getCurrentFramePointer() and
    \ref cVideo::getCurrentFrame() to keep track of the correct frame over time.

    \return __true__ if a new frame is available for display, __false__ otherwise.
*/
//==============================================================================
bool cVideo::update()
{
    if (!m_clip)
    {
        return (C_ERROR);
    }

    bool newFrame = C_ERROR;

    // check if a new frame is available
    if (!((TheoraVideoClip*)m_clip)->isDone())
    {
        // update video timebase
        double t = m_clock.getCurrentTimeSeconds();
        ((TheoraVideoManager*)m_manager)->update((float)(t-m_lastUpdate));
        m_lastUpdate = t;

        // check if a new one is available
        TheoraVideoFrame *frame = ((TheoraVideoClip*)m_clip)->getNextFrame();
        if (frame)
        {
            m_frameIndex++;
            newFrame = C_SUCCESS;
            storeFrame(frame);
        }
    }

    // report beginning of stream
    if (m_firstFrame)
    {
        newFrame = C_SUCCESS;
        m_firstFrame = false;
    }

    // report end of stream
    if (((TheoraVideoClip*)m_clip)->isDone())
    {
        reset();
        newFrame = C_SUCCESS;

        // auto-restart
        if (m_autoReplay)
        {
            play();
        }
    }

    return (newFrame);
}


//==============================================================================
/*!
    This method creates a copy of itself.

    \return Pointer to new object.
*/
//==============================================================================
cVideoPtr cVideo::copy()
{
    // allocate new video
    cVideoPtr video = cVideo::create();

    // if we are empty, return empty video
    if (!m_clip)
    {
        return (video);
    }

    // otherwise load same video
    video->loadFromFile(m_filename);

    // set state
    if      (((TheoraVideoClip*)m_clip)->isDone())   video->stop();
    else if (((TheoraVideoClip*)m_clip)->isPaused()) video->pause();
    else                                             video->play();

    // move to same frame
    video->seekFrame(m_frameIndex);

    // return new video
    return (video);
}


//==============================================================================
/*!
    This method retrieves the video time position.

    \return The current video time position in seconds.
*/
//==============================================================================
double cVideo::getCurrentTimePosition()
{
    return ((TheoraVideoClip*)m_clip)->getTimePosition();
}


//==============================================================================
/*!
    This method fills a \ref cImage object to point to the image data of the
    current video frame. The actual data buffer is still owned by the video, and may
    be destroyed when the video moves on to the next frame (if it is playing).
    To get a permanent copy of a frame buffer, use \ref getCurrentFrame() instead.

    \param  a_image  Image object to set to frame data.

    \return __true__ if the frame is new, __false__ otherwise.
*/
//==============================================================================
bool cVideo::getCurrentFramePointer(cImage &a_image)
{
    // find frame at current time
    bool newFrame = update();

    // set image data to point to the actual frame buffer
    a_image.setData(m_data, 3*m_width*m_height, false);

    // set image properties
    if (a_image.setProperties(m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE) == C_ERROR)
    {
        return (C_ERROR);
    }

    return (newFrame);
}


//==============================================================================
/*!
    This method fills a \ref cImage object with a copy of the image data of
    the current video frame. The actual data buffer is copied from the video frame.
    To get a temporary copy of the current frame buffer, use
    \ref getCurrentFramePointer() instead.

    \param  a_image  Image object to set to frame data.

    \return __true__ if the frame is new, __false__ otherwise.
*/
//==============================================================================
bool cVideo::getCurrentFrame(cImage &a_image)
{
    // find frame at current time
    bool newFrame = update();

    // check if image needs to be initialized
    if (!a_image.isInitialized()         ||
         a_image.getWidth()  != m_width  ||
         a_image.getHeight() != m_height ||
         a_image.getFormat() != GL_RGB   ||
         a_image.getType()   != GL_UNSIGNED_BYTE)
    {
        a_image.allocate(m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE);
        if (a_image.setProperties(m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE) == C_ERROR)
        {
            return (C_ERROR);
        }
    }

    // copy the frame buffer to the image
    if (!memcpy (a_image.getData(), m_data, 3*m_width*m_height))
    {
        return (C_ERROR);
    }

    return (newFrame);
}


//==============================================================================
/*!
    Fill a \ref cImage object with a copy of the image data of a given
    video frame. The actual data buffer is copied from the video frame.

    \param  a_index  Index of the video frame to copy.
    \param  a_image  Image object to set to frame data.

    \return __true__ if the frame is new, __false__ otherwise.
*/
//==============================================================================
bool cVideo::getFramePointer(int a_index, cImage &a_image)
{
    // find frame at requested index
    bool newFrame = seekFrame(a_index);

    // set image data to point to the actual frame buffer
    a_image.setData(m_data, 3*m_width*m_height, false);

    // set image properties
    if (a_image.setProperties(m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE) == C_ERROR)
    {
        return (C_ERROR);
    }

    return (newFrame);
}


//==============================================================================
/*!
    This method fills a \ref cImage object with a copy of the image data of a
    given video frame. The actual data buffer is copied from the video frame.

    \param  a_index  Index of the video frame to copy.
    \param  a_image  Image object to set to frame data.

    \return __true__ if the frame is new, __false__ otherwise.
*/
//==============================================================================
bool cVideo::getFrame(int a_index, cImage &a_image)
{
    // find frame at requested index
    bool newFrame = seekFrame(a_index);

    // check if image needs to be initialized
    if (!a_image.isInitialized()         ||
         a_image.getWidth()  != m_width  ||
         a_image.getHeight() != m_height ||
         a_image.getFormat() != GL_RGB   ||
         a_image.getType()   != GL_UNSIGNED_BYTE)
    {
        a_image.allocate(m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE);
        if (a_image.setProperties(m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE) == C_ERROR)
        {
            return (C_ERROR);
        }
    }

    // copy the frame buffer to the image
    if (!memcpy (a_image.getData(), m_data, 3*m_width*m_height))
    {
        return (C_ERROR);
    }

    return (newFrame);
}


//==============================================================================
/*!
    This method flips a new frame the right way around before making it available
    to the outside world.
*/
//==============================================================================
void cVideo::storeFrame(void *frame)
{
    unsigned int  lineWidth = 3*m_width;
    unsigned char *dst  = m_data;
    unsigned char *src  = ((TheoraVideoFrame*)frame)->getBuffer() + (m_height-1)*lineWidth;

    // copy/flip frame to local buffer
    for(unsigned int i=0; i<m_height; i++)
    {
        memcpy(dst, src, lineWidth);
        dst += lineWidth;
        src -= lineWidth;
    }

    // pop frame from stack
    ((TheoraVideoClip*)m_clip)->popFrame();
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
