/************************************************************************************
This source file is part of the Theora Video Playback Library
For latest info, see http://libtheoraplayer.sourceforge.net/
*************************************************************************************
Copyright (c) 2008-2013 Kresimir Spes (kspes@cateia.com)
This program is free software; you can redistribute it and/or modify it under
the terms of the BSD license: http://www.opensource.org/licenses/bsd-license.php
*************************************************************************************/
#ifndef _OpenAL_AudioInterface_h
#define _OpenAL_AudioInterface_h

#include "TheoraAudioInterface.h"
#include "TheoraVideoClip.h"
#include "TheoraTimer.h"

#include "AL/al.h"
#include "AL/alc.h"
 #include <queue>
	
class OpenAL_AudioInterface : public TheoraAudioInterface, TheoraTimer
{
	int mMaxBuffSize;
	int mBuffSize;
	short *mTempBuffer;
	float mCurrentTimer;

	struct OpenAL_Buffer
	{
		ALuint id;
		int nSamples;
	};
	std::queue<OpenAL_Buffer> mBufferQueue;

	ALuint mSource;
	int mNumProcessedSamples,mNumPlayedSamples;
public:
	OpenAL_AudioInterface(TheoraVideoClip* owner,int nChannels,int freq);
	virtual ~OpenAL_AudioInterface();
	void insertData(float* data,int nSamples);

	//! queued audio buffers, expressed in seconds
	float getQueuedAudioSize();

	void update(float time_increase);

	void pause();
	void play();
	void seek(float time);
};



class OpenAL_AudioInterfaceFactory : public TheoraAudioInterfaceFactory
{
public:
	OpenAL_AudioInterfaceFactory();
	virtual ~OpenAL_AudioInterfaceFactory();
	OpenAL_AudioInterface* createInstance(TheoraVideoClip* owner,int nChannels,int freq);
};

#endif
