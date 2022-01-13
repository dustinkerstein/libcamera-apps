/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd. / Dustin Kerstein
 *
 * control_output.cpp
 */

#include <thread>
#include <pthread.h>
#include <chrono>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "control_output.hpp"
#include "core/control.hpp"
using namespace std::chrono;

// We're going to align the frames within the buffer to friendly byte boundaries
static constexpr int ALIGN = 16; // power of 2, please

struct Header
{
	unsigned int length;
	bool keyframe;
	int64_t timestamp;
};
static_assert(sizeof(Header) % ALIGN == 0, "Header should have aligned size");

ControlOutput::ControlOutput() : Output(), cb_(), framesBuffered_(0), framesWritten_(0)
{

}

ControlOutput::~ControlOutput()
{

}

void ControlOutput::WriteOut()
{
	if (fp_timestamps_)
		fclose(fp_timestamps_);
	fp_timestamps_ = nullptr;
	if (Control::enableBuffer) 
	{
		// We do have to skip to the first I frame before dumping stuff to disk. If there are
		// no I frames you will get nothing. Caveat emptor, methinks.
		unsigned int total = 0, frames = 0;
		bool seen_keyframe = false;
		Header header;
		FILE *fp = fp_; // can't capture a class member in a lambda
		bool padded = false;
		while (!cb_.Empty())
		{
			if (Control::mode == 3 && frames == 10 && !padded) {
				padded = true;
				// frames = 0;
				// total = 0;
				cb_.ResetReadPtr();
			}
			uint8_t *dst = (uint8_t *)&header;
			cb_.Read(
				[&dst](void *src, int n) {
					memcpy(dst, src, n);
					dst += n;
				},
				sizeof(header));
			seen_keyframe |= header.keyframe;
			if (seen_keyframe)
			{
				cb_.Read([fp](void *src, int n) { fwrite(src, 1, n, fp); }, header.length);
				cb_.Skip((ALIGN - header.length) & (ALIGN - 1));
				total += header.length;
				frames++;
			}
			else
				cb_.Skip((header.length + ALIGN - 1) & ~(ALIGN - 1));
		}
		fclose(fp_);
		std::cerr << "Wrote " << total << " bytes (" << frames << " frames)" << std::endl;
	}
}

void ControlOutput::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	if (!Control::enableBuffer) 
	{	
		// auto start = high_resolution_clock::now();
		if (fwrite(mem, size, 1, fp_) != 1)
			throw std::runtime_error("failed to write output bytes");
		else
			framesWritten_++;
		// auto stop = high_resolution_clock::now();
		// auto duration = duration_cast<milliseconds>(stop - start);
		// std::cerr << "LIBCAMERA: Write took: " << duration.count() << "ms" << std::endl;
	}
	else 
	{
		framesBuffered_++;
		auto start = high_resolution_clock::now();

		// First make sure there's enough space.
		int pad = (ALIGN - size) & (ALIGN - 1);
		while (size + pad + sizeof(Header) > cb_.Available())
		{
			if (cb_.Empty())
				throw std::runtime_error("circular buffer too small");
			Header header;
			uint8_t *dst = (uint8_t *)&header;
			cb_.Read(
				[&dst](void *src, int n) {
					memcpy(dst, src, n);
					dst += n;
				},
				sizeof(header));
			cb_.Skip((header.length + ALIGN - 1) & ~(ALIGN - 1));
		}
		Header header = { static_cast<unsigned int>(size), !!(flags & FLAG_KEYFRAME), timestamp_us };
		cb_.Write(&header, sizeof(header));
		cb_.Write(mem, size);
		cb_.Pad(pad);

		auto stop = high_resolution_clock::now();
		auto duration = duration_cast<milliseconds>(stop - start);
		std::cerr << "LIBCAMERA: Copy took: " << duration.count() << "ms, Frames Buffered: " << framesBuffered_ << std::endl;

		if (Control::mode == 3 && fwrite(mem, size, 1, gp_) != 1)
			throw std::runtime_error("failed to write output bytes for SMS dual preview");
	}
}

void ControlOutput::Reset()
{
	std::cerr << "LIBCAMERA: RESETTING BUFFER" << std::endl;
	framesWritten_ = 0;
	framesBuffered_ = 0;
	flags = 2;
	state_ = WAITING_KEYFRAME;
	last_timestamp_ = 0;
	fp_ = nullptr;
	gp_ = nullptr;
}

void ControlOutput::Initialize()
{
	if (Control::mode < 3) {
		if (!fp_) {
			char * myfifo = new char [14];
			strcpy(myfifo, "/dev/shm/pipe");
			mkfifo(myfifo, 0666);
			std::cerr << "LIBCAMERA: PIPE CREATED" << std::endl;
			fp_ = fopen("/dev/shm/pipe", "w");
			std::cerr << "LIBCAMERA: PIPE OPENED BY CONSUMER" << std::endl;
		}
	}
	else {
		if (!gp_) {
			char * myfifo2 = new char [17];
			strcpy(myfifo2, "/dev/shm/smspipe");
			mkfifo(myfifo2, 0666);
			std::cerr << "LIBCAMERA: SMS DUAL PREVIEW PIPE CREATED" << std::endl;
			gp_ = fopen("/dev/shm/smspipe", "w");
			std::cerr << "LIBCAMERA: SMS DUAL PREVIEW PIPE OPENED BY CONSUMER" << std::endl;
		}
	}
}

void ControlOutput::ConfigTimestamp()
{
	if (!Control::timestampsFile.empty())
	{
		fp_timestamps_ = fopen(Control::timestampsFile.c_str(), "w");
		if (!fp_timestamps_)
			throw std::runtime_error("LIBCAMERA: Failed to open timestamp file " + Control::timestampsFile);
		fprintf(fp_timestamps_, "# timecode format v2\n");
	}
}