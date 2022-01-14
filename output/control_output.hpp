/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd. / Dustin Kerstein
 *
 * control_output.hpp
 */

#pragma once

#include "output.hpp"

class ControlBuffer
{
public:
	ControlBuffer() : size_(6723993600), buf_(6723993600), rptr_(0), wptr_(0), prev_rptr_(0) {}
	void SaveReadPtr() { prev_rptr_ = rptr_; }
	void ResetReadPtr() { rptr_ = prev_rptr_; }
	bool Empty() const { return rptr_ == wptr_; }
	size_t Available() const { return (size_ - wptr_ + rptr_) % size_ - 1; }
	void Skip(unsigned int n) { rptr_ = (rptr_ + n) % size_; }
	// The dst function allows bytes read to go straight to memory or a file etc.
	void Read(std::function<void(void *src, unsigned int n)> dst, unsigned int n)
	{
		if (rptr_ + n >= size_)
		{
			dst(&buf_[rptr_], size_ - rptr_);
			n -= size_ - rptr_;
			rptr_ = 0;
		}
		dst(&buf_[rptr_], n);
		rptr_ += n;
	}
	void Pad(unsigned int n) { wptr_ = (wptr_ + n) % size_; }
	void Write(const void *ptr, unsigned int n)
	{
		if (wptr_ + n >= size_)
		{
			memcpy(&buf_[wptr_], ptr, size_ - wptr_);
			n -= size_ - wptr_;
			ptr = static_cast<const uint8_t *>(ptr) + size_ - wptr_;
			wptr_ = 0;
		}
		memcpy(&buf_[wptr_], ptr, n);
		wptr_ += n;
	}

private:
	const size_t size_;
	std::vector<uint8_t> buf_;
	size_t rptr_, wptr_, prev_rptr_;
};

class ControlOutput : public Output
{
	public:
		ControlOutput();
		~ControlOutput();
		void WriteOut();
		void Reset();
		void Initialize();
		void ConfigTimestamp();

	protected:
		void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) override;

	private:
		ControlBuffer cb_;
		std::atomic_uint framesBuffered_;
		std::atomic_uint framesWritten_;
		FILE *fp_;
		FILE *gp_;
};
