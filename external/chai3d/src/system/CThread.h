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
    \author    Sebastien Grange
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CThreadH
#define CThreadH
//------------------------------------------------------------------------------
#include "system/CGlobals.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CThread.h
    \ingroup    system

    \brief
    Implements support for handling threads.
*/
//==============================================================================

//------------------------------------------------------------------------------
/*!
    Defines two basic thread priorities for handling __graphics__ and 
    __haptics__ rendering loops.
*/
//------------------------------------------------------------------------------
enum CThreadPriority
{
    CTHREAD_PRIORITY_GRAPHICS,    // lower priority
    CTHREAD_PRIORITY_HAPTICS      // higher priority
};


//==============================================================================
/*!
    \class      cThread
    \ingroup    system

    \brief
    This class implements support for threads.

    \details
    A process can create one or more threads to execute a portion of the 
    program code associated with the process. Use method start() to specify 
    the program code executed by a thread. The method also allows you to 
    pass data to the thread procedure.\n
    
    A scheduling priority level, as defined in __CThreadPriority__, can be 
    requested for a thread, but is not guaranteed to be honored by the 
    operating system.

*/
//==============================================================================

class cThread
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
    
public:

    //! Constructor of cThread.
    cThread();

    //! Destructor of cThread.
    virtual ~cThread();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method sets the thread parameters (and start it).
    void start(void(*a_function)(void ), const CThreadPriority a_level);

    //! This method sets the thread parameters (and start it).
    void start(void(*a_function)(void*), const CThreadPriority a_level, void *a_arg);

    //! This method terminates the thread (not recommended!).
    void stop();

    //! This method sets the thread priority level.
    void setPriority(CThreadPriority a_level);

    //! This method returns the current thread priority level.
    CThreadPriority getPriority() const { return (m_priorityLevel); }


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

#if defined(WIN32) | defined(WIN64)
    //! Thread handle.
    DWORD m_threadId;
#endif

#if defined(LINUX) || defined(MACOSX)
    //! Thread handle.
    pthread_t m_handle;
#endif

    //! Pointer to thread function.
    void* m_function;

    //! Thread priority level.
    CThreadPriority m_priorityLevel;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
