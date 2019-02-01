/************************************************************************************
This source file is part of the Theora Video Playback Library
For latest info, see http://libtheoraplayer.sourceforge.net/
*************************************************************************************
Copyright (c) 2008-2013 Kresimir Spes (kspes@cateia.com)
This program is free software; you can redistribute it and/or modify it under
the terms of the BSD license: http://www.opensource.org/licenses/bsd-license.php
*************************************************************************************/
#include <stdio.h>
#include "OpenAL_AudioInterface.h"

ALCdevice* gDevice=0;
ALCcontext* gContext=0;


short float2short(float f)
{
	if      (f >  1) f= 1;
	else if (f < -1) f=-1;
	return (short) (f*32767);
}

OpenAL_AudioInterface::OpenAL_AudioInterface(TheoraVideoClip* owner,int nChannels,int freq) :
	TheoraAudioInterface(owner,nChannels,freq), TheoraTimer()
{
	mMaxBuffSize = freq * mNumChannels * 2;
	mBuffSize = 0;
	mNumProcessedSamples = 0;
	mCurrentTimer = 0;
	mTempBuffer = new short[mMaxBuffSize];
	alGenSources(1, &mSource);
	owner->setTimer(this);
	mNumPlayedSamples = 0;
}

OpenAL_AudioInterface::~OpenAL_AudioInterface()
{
	if (mTempBuffer) delete [] mTempBuffer;
	
	if (mSource)
	{
	    alSourceStop(mSource);
		alSourcei(mSource, AL_BUFFER, 0);
		alDeleteSources(1, &mSource);
	}
	while (mBufferQueue.size() > 0)
	{
		alDeleteBuffers(1, &mBufferQueue.front().id);
		mBufferQueue.pop();
	}
}

float OpenAL_AudioInterface::getQueuedAudioSize()
{
	return ((float) (mNumProcessedSamples - mNumPlayedSamples)) / mFreq;
}

void OpenAL_AudioInterface::insertData(float* data, int nSamples)
{
	for (int i = 0; i < nSamples; i++)
	{
		if (mBuffSize < mMaxBuffSize)
		{
			mTempBuffer[mBuffSize++]=float2short(data[i]);
		}
		if (mBuffSize == (int)(mSpeed*mFreq * mNumChannels / 10))
		{
			OpenAL_Buffer buff;
			alGenBuffers(1,&buff.id);
			ALuint format = (mNumChannels == 1) ? AL_FORMAT_MONO16 : AL_FORMAT_STEREO16;
			alBufferData(buff.id,format,mTempBuffer,mBuffSize*2,(int)(mSpeed*mFreq));
			alSourceQueueBuffers(mSource, 1, &buff.id);
			buff.nSamples=mBuffSize/mNumChannels;
			mNumProcessedSamples+=mBuffSize/mNumChannels;
			mBufferQueue.push(buff);
			mBuffSize=0;
			int state;
			alGetSourcei(mSource,AL_SOURCE_STATE,&state);
			if (state != AL_PLAYING)
			{
				alSourcePlay(mSource);
			}
		}
	}
}

void OpenAL_AudioInterface::update(float time_increase)
{
	int i, nProcessed;
	OpenAL_Buffer buff;

	// process played buffers
	alGetSourcei(mSource,AL_BUFFERS_PROCESSED,&nProcessed);
	for (i=0;i<nProcessed;i++)
	{
		buff=mBufferQueue.front();
		mBufferQueue.pop();
		mNumPlayedSamples+=buff.nSamples;
		alSourceUnqueueBuffers(mSource,1,&buff.id);
		alDeleteBuffers(1,&buff.id);
	}
	if (nProcessed != 0)
	{
		// update offset
		alGetSourcef(mSource,AL_SEC_OFFSET,&mCurrentTimer);
	}

	// control playback and return time position
    mCurrentTimer += time_increase * mSpeed;
	mTime = mCurrentTimer + (float) mNumPlayedSamples/mFreq;
	float duration=mClip->getDuration();
	if (mTime > duration) mTime=duration;
}

void OpenAL_AudioInterface::pause()
{
	alSourcePause(mSource);
	TheoraTimer::pause();
}

void OpenAL_AudioInterface::play()
{
	alSourcePlay(mSource);
	TheoraTimer::play();
}

void OpenAL_AudioInterface::seek(float time)
{
	OpenAL_Buffer buff;

	alSourceStop(mSource);
	while (!mBufferQueue.empty())
	{
		buff=mBufferQueue.front();
		mBufferQueue.pop();
		alSourceUnqueueBuffers(mSource,1,&buff.id);
		alDeleteBuffers(1,&buff.id);
	}

	mBuffSize=0;
	mCurrentTimer = 0;
	mNumPlayedSamples=mNumProcessedSamples=(int) (time*mFreq);
	mTime = time;
}

OpenAL_AudioInterfaceFactory::OpenAL_AudioInterfaceFactory()
{
	gDevice = alcOpenDevice("");
	if (alcGetError(gDevice) != ALC_NO_ERROR) goto Fail;
	gContext = alcCreateContext(gDevice, NULL);
	if (alcGetError(gDevice) != ALC_NO_ERROR) goto Fail;
	alcMakeContextCurrent(gContext);
	if (alcGetError(gDevice) != ALC_NO_ERROR) goto Fail;

	return;
Fail:
	gDevice=NULL;
	gContext=NULL;
}

OpenAL_AudioInterfaceFactory::~OpenAL_AudioInterfaceFactory()
{
	if (gDevice)
	{
		alcMakeContextCurrent(NULL);
		alcDestroyContext(gContext);
		alcCloseDevice(gDevice);
	}
}

OpenAL_AudioInterface* OpenAL_AudioInterfaceFactory::createInstance(TheoraVideoClip* owner,int nChannels,int freq)
{
	return new OpenAL_AudioInterface(owner,nChannels,freq);
}
