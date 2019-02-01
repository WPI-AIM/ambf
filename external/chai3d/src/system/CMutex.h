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
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CMutexH
#define CMutexH
//------------------------------------------------------------------------------
#include "system/CGlobals.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMutex.h
    \ingroup    system

    \brief
    Implements support for mutex objects.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cMutex
    \ingroup    system

    \brief
    This class implements a mutex.

    \details
    A mutex object is a synchronization object whose state is set to 
    signaled when it is not owned by any thread, and non-signaled when
    it is owned. Only one thread at a time can own a mutex object, whose 
    name comes from the fact that it is useful in coordinating mutually 
    exclusive access to a shared resource. For example, to prevent two 
    threads from writing to shared memory at the same time, each thread 
    waits for ownership of a mutex object before executing the code that 
    accesses the memory. After writing to the shared memory, the thread 
    releases the mutex object.
*/
//==============================================================================
class cMutex
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cMutex.
    cMutex();

    //! Destructor of cMutex.
    virtual ~cMutex();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method acquires the mutex.
    bool acquire();

    //! This method acquires the mutex only if it is not owned by another thread.
    bool tryAcquire();

    //! This method releases the mutex.
    void release();


    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

#if defined(WIN32) | defined(WIN64)
    //! Mutex handle.
    CRITICAL_SECTION m_mutex;
#endif

#if defined(LINUX) || defined(MACOSX)
    //! Mutex handle.
    pthread_mutex_t m_mutex;
#endif
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
