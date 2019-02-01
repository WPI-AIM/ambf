/************************************************************************************
This source file is part of the Theora Video Playback Library
For latest info, see http://libtheoraplayer.googlecode.com
*************************************************************************************
Copyright (c) 2008-2014 Kresimir Spes (kspes@cateia.com)
This program is free software; you can redistribute it and/or modify it under
the terms of the BSD license: http://opensource.org/licenses/BSD-3-Clause
*************************************************************************************/
#ifdef _WIN32
#pragma warning( disable: 4251 ) // MSVC++

#ifdef _DEBUG
#include "windows.h"
const DWORD MS_VC_EXCEPTION=0x406D1388;

#pragma pack(push,8)
typedef struct tagTHREADNAME_INFO
{
	DWORD dwType; // Must be 0x1000.
	LPCSTR szName; // Pointer to name (in user addr space).
	DWORD dwThreadID; // Thread ID (-1=caller thread).
	DWORD dwFlags; // Reserved for future use, must be zero.
} THREADNAME_INFO;
#pragma pack(pop)

void SetThreadName( DWORD dwThreadID, char* threadName)
{
	THREADNAME_INFO info;
	info.dwType = 0x1000;
	info.szName = threadName;
	info.dwThreadID = dwThreadID;
	info.dwFlags = 0;

	__try
	{
		RaiseException( MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR),       (ULONG_PTR*)&info );
	}
	__except(EXCEPTION_EXECUTE_HANDLER)
	{
		
	}
}

#endif
#endif
#include "TheoraWorkerThread.h"
#include "TheoraVideoManager.h"
#include "TheoraVideoClip.h"
#include "TheoraAsync.h"
#include "TheoraUtil.h"

#ifdef _THREAD_NAMING
static int threadCounter = 1;
static TheoraMutex counterMutex;
#endif

TheoraWorkerThread::TheoraWorkerThread() : TheoraThread()
{
	mClip = NULL;
}

TheoraWorkerThread::~TheoraWorkerThread()
{

}

void TheoraWorkerThread::execute()
{
	TheoraMutex::ScopeLock lock;
#ifdef _THREAD_NAMING
    {
        lock.acquire(&counterMutex);
        char name[64];
        sprintf(name, "TheoraWorkerThread %d", threadCounter++);
        pthread_setname_np(name);
		lock.release();
	}
#endif
	while (isRunning())
	{
		mClip = TheoraVideoManager::getSingleton().requestWork(this);
		if (!mClip)
		{
			_psleep(100);
			continue;
		}

		lock.acquire(mClip->mThreadAccessMutex);
		// if user requested seeking, do that then.
		if (mClip->mSeekFrame >= 0) mClip->doSeek();

		if (!mClip->decodeNextFrame())
			_psleep(1); // this happens when the video frame queue is full.

		mClip->mAssignedWorkerThread = NULL;
		lock.release();
		mClip = NULL;
	}
}
