/************************************************************************************
This source file is part of the Theora Video Playback Library
For latest info, see http://libtheoraplayer.googlecode.com
*************************************************************************************
Copyright (c) 2008-2014 Kresimir Spes (kspes@cateia.com)
This program is free software; you can redistribute it and/or modify it under
the terms of the BSD license: http://opensource.org/licenses/BSD-3-Clause
*************************************************************************************/
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <pthread.h>
#endif

#include "TheoraAsync.h"
#include "TheoraUtil.h"
#include "TheoraVideoManager.h"

#ifdef _WINRT
#include <wrl.h>
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// Mutex
///////////////////////////////////////////////////////////////////////////////////////////////////

TheoraMutex::TheoraMutex()
{
#ifdef _WIN32
#ifndef _WINRT // WinXP does not have CreateTheoraMutexEx()
	mHandle = CreateMutex(0, 0, 0);
#else
	mHandle = CreateMutexEx(NULL, NULL, 0, SYNCHRONIZE);
#endif
#else
	mHandle = (pthread_mutex_t*)malloc(sizeof(pthread_mutex_t));
	pthread_mutex_init((pthread_mutex_t*)mHandle, 0);
#endif
}

TheoraMutex::~TheoraMutex()
{
#ifdef _WIN32
	CloseHandle(mHandle);
#else
	pthread_mutex_destroy((pthread_mutex_t*)mHandle);
	free((pthread_mutex_t*)mHandle);
	mHandle = NULL;
#endif
}

void TheoraMutex::lock()
{
#ifdef _WIN32
	WaitForSingleObjectEx(mHandle, INFINITE, FALSE);
#else
	pthread_mutex_lock((pthread_mutex_t*)mHandle);
#endif
}

void TheoraMutex::unlock()
{
#ifdef _WIN32
	ReleaseMutex(mHandle);
#else
	pthread_mutex_unlock((pthread_mutex_t*)mHandle);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// ScopeLock
///////////////////////////////////////////////////////////////////////////////////////////////////

TheoraMutex::ScopeLock::ScopeLock(TheoraMutex* mutex, bool logUnhandledUnlocks) : mMutex(NULL)
{
	mLogUnhandledUnlocks = logUnhandledUnlocks;
	acquire(mutex);
}

TheoraMutex::ScopeLock::~ScopeLock()
{
	if (release() && mLogUnhandledUnlocks)
	{
		th_writelog("A mutex has been scope-unlocked automatically!");
	}
}

bool TheoraMutex::ScopeLock::acquire(TheoraMutex* mutex)
{
	if (mMutex == NULL && mutex != NULL)
	{
		mMutex = mutex;
		mMutex->lock();
		return true;
	}
	return false;
}

bool TheoraMutex::ScopeLock::release()
{
	if (mMutex != NULL)
	{
		mMutex->unlock();
		mMutex = NULL;
		return true;
	}
	return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Thread
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef _WINRT
using namespace Windows::Foundation;
using namespace Windows::System::Threading;
#endif

#ifdef _WIN32
unsigned long WINAPI theoraAsyncCall(void* param)
#else
void* theoraAsyncCall(void* param)
#endif
{
	TheoraThread* t = (TheoraThread*)param;
	t->execute();
#ifdef _WIN32
	return 0;
#else
	pthread_exit(NULL);
	return NULL;
#endif
}

#ifdef _WINRT
struct TheoraAsyncActionWrapper
{
public:
	IAsyncAction^ mAsyncAction;
	TheoraAsyncActionWrapper(IAsyncAction^ asyncAction)
	{
		mAsyncAction = asyncAction;
	}
};
#endif
	
TheoraThread::TheoraThread() : mId(0), mRunning(false)
{
#ifndef _WIN32
	mId = (pthread_t*)malloc(sizeof(pthread_t));
#endif
}

TheoraThread::~TheoraThread()
{
	TheoraMutex::ScopeLock lock(&mRunningMutex);
	bool running = mRunning;
	lock.release();
	if (running)
	{
		stop();
	}
	if (mId != NULL)
	{
#ifdef _WIN32
#ifndef _WINRT
		CloseHandle(mId);
#else
		delete mId;
#endif
#else
		free((pthread_t*)mId);
#endif
		mId = NULL;
	}
}

void TheoraThread::start()
{
	TheoraMutex::ScopeLock lock(&mRunningMutex);
	mRunning = true;
	lock.release();
#ifdef _WIN32
#ifndef _WINRT
	mId = CreateThread(0, 0, &theoraAsyncCall, this, 0, 0);
#else
	mId = new TheoraAsyncActionWrapper(ThreadPool::RunAsync(
		ref new WorkItemHandler([&](IAsyncAction^ work_item)
		{
			execute();
		}),
		WorkItemPriority::Normal, WorkItemOptions::TimeSliced));
#endif
#else
	pthread_create((pthread_t*)mId, NULL, &theoraAsyncCall, this);
#endif
}

bool TheoraThread::isRunning()
{
	TheoraMutex::ScopeLock lock(&mRunningMutex);
	bool result = mRunning;
	lock.release();
	return result;
}

void TheoraThread::join()
{
	TheoraMutex::ScopeLock lock(&mRunningMutex);
	mRunning = false;
	lock.release();
#ifdef _WIN32
#ifndef _WINRT
	WaitForSingleObject(mId, INFINITE);
	if (mId != NULL)
	{
		CloseHandle(mId);
		mId = NULL;
	}
#else
	IAsyncAction^ action = ((TheoraAsyncActionWrapper*)mId)->mAsyncAction;
	int i = 0;
	while (action->Status != AsyncStatus::Completed &&
		action->Status != AsyncStatus::Canceled &&
		action->Status != AsyncStatus::Error &&
		i < 100)
	{
		_psleep(50);
		++i;
	}
	if (i >= 100)
	{
		i = 0;
		action->Cancel();
		while (action->Status != AsyncStatus::Completed &&
			action->Status != AsyncStatus::Canceled &&
			action->Status != AsyncStatus::Error &&
			i < 100)
		{
			_psleep(50);
			++i;
		}
	}
#endif
#else
	pthread_join(*((pthread_t*)mId), 0);
#endif
}
	
void TheoraThread::resume()
{
#ifdef _WIN32
#ifndef _WINRT
	ResumeThread(mId);
#else
	// not available in WinRT
#endif
#endif
}
	
void TheoraThread::pause()
{
#ifdef _WIN32
#ifndef _WINRT
	SuspendThread(mId);
#else
	// not available in WinRT
#endif
#endif
}
	
void TheoraThread::stop()
{
	bool stop = false;
	TheoraMutex::ScopeLock lock(&mRunningMutex);
	if (mRunning)
	{
		mRunning = false;
		stop = true;
	}
	lock.release();
	if (stop)
	{
#ifdef _WIN32
#ifndef _WINRT
		TerminateThread(mId, 0);
#else
		((TheoraAsyncActionWrapper*)mId)->mAsyncAction->Cancel();
#endif
#elif defined(_ANDROID)
		pthread_kill(*((pthread_t*)mId), 0);
#else
		pthread_cancel(*((pthread_t*)mId));
#endif
	}
}
	
