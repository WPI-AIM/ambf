/************************************************************************************
This source file is part of the Theora Video Playback Library
For latest info, see http://libtheoraplayer.googlecode.com
*************************************************************************************
Copyright (c) 2008-2014 Kresimir Spes (kspes@cateia.com)
This program is free software; you can redistribute it and/or modify it under
the terms of the BSD license: http://opensource.org/licenses/BSD-3-Clause
*************************************************************************************/
#include <stdio.h>
#include <sys/stat.h>
#include <memory.h>
#include "TheoraDataSource.h"
#include "TheoraException.h"
#include "TheoraVideoManager.h"
#include "TheoraUtil.h"

TheoraDataSource::~TheoraDataSource()
{

}

TheoraFileDataSource::TheoraFileDataSource(std::string filename)
{
	mFilename = filename;
	mFilePtr = NULL;
}

TheoraFileDataSource::~TheoraFileDataSource()
{
	if (mFilePtr)
	{
		fclose(mFilePtr);
		mFilePtr = NULL;
	}
}

void TheoraFileDataSource::openFile()
{
	if (mFilePtr == NULL)
	{
		mFilePtr = fopen(mFilename.c_str(), "rb");
		if (!mFilePtr)
		{
			std::string msg = "Can't open video file: " + mFilename;
			th_writelog(msg);
			throw TheoraGenericException(msg);
		}

		
#ifdef _WIN32
		struct _stat64 s;
		_fstati64(_fileno(mFilePtr), &s);
#else
		struct stat s;
		fstat(fileno(mFilePtr), &s);
#endif
		mSize = (uint64_t) s.st_size;
	}
}

int TheoraFileDataSource::read(void* output, int nBytes)
{
	if (mFilePtr == NULL) openFile();
	uint64_t n = fread(output, 1, nBytes, mFilePtr);
	return (int) n;
}

void TheoraFileDataSource::seek(uint64_t byte_index)
{
	if (mFilePtr == NULL) openFile();
#ifdef LINUX //fpos_t is not a scalar in Linux, for more info refer here: https://code.google.com/p/libtheoraplayer/issues/detail?id=6
	fpos_t fpos = { 0 };
	fpos.__pos = byte_index;
#else
	fpos_t fpos = byte_index;
#endif
	fsetpos(mFilePtr, &fpos);
}

uint64_t TheoraFileDataSource::size()
{
	if (mFilePtr == NULL) openFile();
	return mSize;
}

uint64_t TheoraFileDataSource::tell()
{
	if (mFilePtr == NULL) return 0;
#ifdef LINUX
	fpos_t pos;
	fgetpos(mFilePtr, &pos);
	return (uint64_t) pos.__pos;
#else
	fpos_t pos;
	fgetpos(mFilePtr, &pos);
	return (uint64_t) pos;
#endif
}

TheoraMemoryFileDataSource::TheoraMemoryFileDataSource(std::string filename) :
	mReadPointer(0),
	mData(0)
{
	mFilename = filename;
	FILE* f = fopen(filename.c_str(),"rb");
	if (!f) throw TheoraGenericException("Can't open video file: " + filename);

#ifdef _WIN32
	struct _stat64 s;
	_fstati64(_fileno(f), &s);
#else
	struct stat s;
	fstat(fileno(f), &s);
#endif
	mSize = (uint64_t) s.st_size;
	if (mSize > 0xFFFFFFFF)
	{
		throw TheoraGenericException("TheoraMemoryFileDataSource doesn't support files larger than 4GB!");
	}
	mData = new unsigned char[(unsigned int) mSize];
	if (mSize < UINT_MAX)
	{
		fread(mData, 1, (size_t) mSize, f);
	}
	else
	{
		throw TheoraGenericException("Unable to preload file to memory, file is too large.");
	}

	fclose(f);
}

TheoraMemoryFileDataSource::TheoraMemoryFileDataSource(unsigned char* data, long size, const std::string& filename)
{
	mFilename = filename;
	mData = data;
	mSize = size;
	mReadPointer = 0;
}

TheoraMemoryFileDataSource::~TheoraMemoryFileDataSource()
{
	if (mData) delete[] mData;
}

int TheoraMemoryFileDataSource::read(void* output, int nBytes)
{
	int n = (int) ((mReadPointer+nBytes <= mSize) ? nBytes : mSize - mReadPointer);
	if (!n) return 0;
	memcpy(output, mData + mReadPointer, n);
	mReadPointer += n;
	return n;
}

void TheoraMemoryFileDataSource::seek(uint64_t byte_index)
{
	mReadPointer = byte_index;
}

uint64_t TheoraMemoryFileDataSource::size()
{
	return mSize;
}

uint64_t TheoraMemoryFileDataSource::tell()
{
	return mReadPointer;
}
