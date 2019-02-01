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
    \version   3.2.0 $Rev: 1884 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CVideoH
#define CVideoH
//------------------------------------------------------------------------------
#include "graphics/CImage.h"
#include "timers/CPrecisionClock.h"
#include "system/CMutex.h"
//------------------------------------------------------------------------------
#include <memory>
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CVideo.h

    \brief
    Implements support for video files.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cVideo;
typedef std::shared_ptr<cVideo> cVideoPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cVideo
    \ingroup    graphics

    \brief
    This class implements support for video files of the OGG/Vorbis format.

    \details
    This class implements support for video files of the OGG/Vorbis format.
    Audio is also supported.
*/
//==============================================================================
class cVideo
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Default constructor of cVideo.
    cVideo();

    //! Destructor of cVideo.
    virtual ~cVideo();

    //! Shared cVideo allocator.
    static cVideoPtr create() { return (std::make_shared<cVideo>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL COMMANDS:
    //--------------------------------------------------------------------------

public:

    //! This method creates a copy of this object.
    cVideoPtr copy();

    //! This method frees video from memory.
    void erase() { cleanup(); }

    //! This method returns __true__ if a video file has been loaded in memory, __false__ otherwise.
    inline bool isInitialized() const { return (m_clip != NULL); }

    //! This method returns the width of video image.
    inline unsigned int getWidth() const { return (m_width);  }

    //! This method returns the height of video image.
    inline unsigned int getHeight() const { return (m_height); }

    //! This method returns the number of frames of this video stream.
    inline unsigned int getFrameCount() const { return (m_frameCount); }

    //! This method returns the duration of this video in seconds.
    inline double getDuration() const { return (m_duration); }

    //! This method returns the rate of this video in frames per second.
    inline double getFPS() const { return (m_fps); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - MANIPULATING VIDEO:
    //--------------------------------------------------------------------------

public:

    //! This method starts playing the video.
    void play();

    //! This method control playback speed.
    void setPlaybackSpeed(double a_speed);

    //! This method retrieves playback speed.
    double getPlaybackSpeed();

    //! This method pauses the video.
    void pause();

    //! This method stops the video.
    void stop();

    //! This method enables or disables the auto-replay mode.
    void setAutoReplay(bool a_replay = true);

    //! This method returns the video playing status.
    bool isPaused();

    //! This method seeks a particular time in the video.
    bool seek(double a_time);

    //! This method seeks a particular frame in the video.
    bool seekFrame(unsigned int a_index);

    //! This method returns the index number of the current video frame.
    int getCurrentFrameIndex() { return (m_frameIndex); }

    //! This method returns the time position of the current video frame.
    double getCurrentTimePosition();

    //! This method returns a pointer to the current frame (does not allocate a copy).
    bool getCurrentFramePointer(cImage &a_image);

    //! This method returns a copy of the current frame.
    bool getCurrentFrame(cImage &a_image);

    //! This method returns a pointer to any frame (does not allocate a copy).
    bool getFramePointer(int a_index, cImage &a_image);

    //! This method returns a copy of any frame.
    bool getFrame(int a_index, cImage &a_image);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - FILES:
    //--------------------------------------------------------------------------

public:

    //! This method loads the video file by passing image path and name as argument.
    bool loadFromFile(const std::string& a_filename);

    //! This method returns the filename from which this video was last loaded or saved.
    std::string getFilename() const { return (m_filename); }

    //! This method returns the video title.
    std::string getName() const { return (m_name); }


    //--------------------------------------------------------------------------
    // PROTECTED  METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method initializes member variables.
    void defaults();

    //! This method deletes memory and removes any video that was previously loaded.
    void cleanup();

    //! This method updates the frame index and pointer to the current time.
    bool update();

    //! This method reset the video to the first frame and make it ready to play again.
    void reset();

    //! This method stores a frame locally (and flip horizontally).
    inline void storeFrame(void *frame);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Video filename.
    std::string m_filename;

    //! Video name.
    std::string m_name;

    //! Width in pixels of the current video.
    unsigned int m_width;

    //! Height in pixels of the current video.
    unsigned int m_height;

    //! Frame count of the current video.
    unsigned int m_frameCount;

    //! Current frame index of the current video.
    unsigned int m_frameIndex;

    //! Duration in seconds of the current video.
    double m_duration;

    //! Last time the frame index was updated.
    double m_lastUpdate;

    //! First frame flag.
    bool m_firstFrame;

    //! Auto replay flag.
    bool m_autoReplay;

    //! Frame per seconds of the current video.
    double m_fps;

    //! Video manager time base.
    cPrecisionClock m_clock;

    //! Video clip object.
    void *m_clip;

    //! Video frame data.
    unsigned char *m_data;

    //! Shared clip counter
    static unsigned int m_clipCount;

    //! Shared members lock
    static cMutex m_sharedLock;

    //! Shared video manager.
    static void *m_manager;

    //! Interface to audio control.
    static void *m_audio;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
