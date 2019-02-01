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
    \version   3.2.0 $Rev: 2048 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CPrecisionClockH
#define CPrecisionClockH
//------------------------------------------------------------------------------
#include "system/CGlobals.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CPrecisionClock.h
    \ingroup    timers

    \brief
    Implements a high precision clock.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cPrecisionClock
    \ingroup    timers

    \brief
    This class implements a high precision clock.

    \details
    __cPrecisionClock__ implements a high-precision clock. All measurements 
    are computed in seconds unless otherwise-specified. \n
  
    __cPrecisionClock__ behaves just like a real chronograph: It can be started, 
    stopped and restarted at a later time. When a clock is running (__ON__), 
    time is accumulated until the next stop event (__OFF__). The value of a 
    clock can be read by calling method getCurrentTimeSeconds(). When a clock
    is disabled (__OFF__), time is no longer accumulated.
*/
//==============================================================================

class cPrecisionClock
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
 
public:

    //! Constructor of cPrecisionClock.
    cPrecisionClock();

    //! Destructor of cPrecisionClock.
    virtual ~cPrecisionClock() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method resets the clock.
    void reset(const double a_currentTime = 0.0);

    //! This method starts the clock with optional reset.
    double start(bool a_resetClock = false);

    //! This method stops the clock and return the elapsed time.
    double stop();

    //! This method returns __true__ if the clock is running (__ON__), otherwise __false__ if the clock is paused (__OFF__).
    bool on() const { return (m_on); };

    //! This method returns the current clock time in seconds.
    double getCurrentTimeSeconds() const;

    //! This method sets the period in seconds before a _timeout_ occurs (you need to poll for this).
    void setTimeoutPeriodSeconds(const double a_timeoutPeriod);

    //! This method reads the programmed _timeout_ period is seconds.
    double getTimeoutPeriodSeconds() const { return (m_timeoutPeriod); }

    //! This method returns __true__ if _timeout_ has occurred, otherwise return  __false__.
    bool timeoutOccurred() const ;

    //! This method returns __true__ if the high resolution CPU clock is available on this computer, __false__. otherwise.
    bool highResolution() const { return (m_highres); };

    //! This method returns the raw CPU time in seconds.
    static double getCPUTimeSeconds();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    //! This method initializes all static variables of cPrecisionClock.
    static void initialize();


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

#if defined(WIN32) | defined(WIN64)
    //! Stores information about CPU high precision clock.
    static LARGE_INTEGER m_freq;
#endif

    //! If __true__, then high precision CPU clock is available.
    static bool m_highres;

    //! If __true__, then static members of cPrecisionClock have been initialized.
    static bool m_initialized;

    //! Time accumulated between previous calls to start() and stop().
    double m_timeAccumulated;

    //! CPU time in seconds when clock was started.
    double m_timeStart;

    //! Timeout period in seconds.
    double m_timeoutPeriod;

    //! Clock time in seconds when timer was started. 
    double m_timeoutStart;

    //! If __true__, then clock is currently __ON__.
    bool m_on;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
