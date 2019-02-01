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
#include "system/CMutex.h"
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cMutex.
*/
//==============================================================================
cMutex::cMutex()
{
#if defined(WIN32) | defined(WIN64)
    InitializeCriticalSection(&m_mutex);
#endif

#if defined (LINUX) || defined (MACOSX)
    pthread_mutexattr_t mattr;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_settype(&mattr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&m_mutex, &mattr);
#endif
}


//==============================================================================
/*!
    Destructor of cMutex.
*/
//==============================================================================
cMutex::~cMutex()
{
#if defined(WIN32) | defined(WIN64)
    DeleteCriticalSection(&m_mutex);
#endif

#if defined (LINUX) || defined (MACOSX)
    pthread_mutex_destroy(&m_mutex);
#endif
}


//==============================================================================
/*!
    This method requests ownership of the mutex. The calling thread will wait until
    the mutex becomes available, or until a system timeout occurs (in which
    case the call is not successful).

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cMutex::acquire()
{
    // create thread
#if defined(WIN32) | defined(WIN64)
    EnterCriticalSection(&m_mutex);
    return true;
#endif

#if defined (LINUX) || defined (MACOSX)
    return (pthread_mutex_lock(&m_mutex) == 0);
#endif
}


//==============================================================================
/*!
    This method requests ownership of the mutex. The calling thread will 
    return __false__ immediately if the mutex is already owned by another thread.
    Otherwise, the calling thread is granted ownership of the mutex.

    \return __true__ in case of success, __false__ otherwise.
*/
//==============================================================================
bool cMutex::tryAcquire()
{
    // create thread
#if defined(WIN32) | defined(WIN64)
    return (TryEnterCriticalSection(&m_mutex) != 0);
#endif

#if defined (LINUX) || defined (MACOSX)
    return (pthread_mutex_trylock(&m_mutex) == 0);
#endif
}


//==============================================================================
/*!
    This method makes the calling thread release ownership of the mutex.
*/
//==============================================================================
void cMutex::release()
{
    // create thread
#if defined(WIN32) | defined(WIN64)
    LeaveCriticalSection(&m_mutex);
#endif

#if defined (LINUX) || defined (MACOSX)
    pthread_mutex_unlock(&m_mutex);
#endif
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
