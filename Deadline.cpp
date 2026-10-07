/***********************************************************************************************************************
*                                                                                                                      *
* xptools                                                                                                              *
*                                                                                                                      *
* Copyright (c) 2026 Shiz <hi@shiz.me>                                                                                 *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file Deadline.cpp
	@brief Implementation of Deadline class
 */

#include "Deadline.h"

using namespace std;


#ifdef _WIN32
static double ticksPerNs;

static void GetTime(LARGE_INTEGER *t)
{
	QueryPerformanceCounter(t);
}

static uint64_t GetTimeDiff(const LARGE_INTEGER *start, const LARGE_INTEGER *now)
{
	if (ticksPerNs == 0)
	{
		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);
		ticksPerNs = 1e9 / static_cast<double>(freq.QuadPart);
	}

	uint64_t ticks = now->QuadPart - start->QuadPart;
	return static_cast<uint64_t>(ticks / ticksPerNs);
}
#else
/* Not all systems define CLOCK_MONOTONIC_RAW, fallback to CLOCK_MONOTONIC in that case */
#ifndef CLOCK_MONOTONIC_RAW
#define CLOCK_MONOTONIC_RAW CLOCK_MONOTONIC
#endif

static void GetTime(struct timespec *t)
{
	clock_gettime(CLOCK_MONOTONIC_RAW, t);
}

static uint64_t GetTimeDiff(const struct timespec *start, const struct timespec *now)
{
	time_t secs = now->tv_sec - start->tv_sec;
	uint64_t nsecs;
	if (now->tv_nsec < start->tv_nsec)
	{
		secs--;
		nsecs = static_cast<uint64_t>(Deadline::SECONDS_PER_NS - start->tv_nsec + now->tv_nsec);
	}
	else
	{
		nsecs = static_cast<uint64_t>(now->tv_nsec - start->tv_nsec);
	}
	if (secs < 0)
		return 0;
	return static_cast<uint64_t>(secs) * Deadline::SECONDS_PER_NS + nsecs;
}
#endif


/**
	@brief Creates a deadline

	@param duration_ns Deadline duration in nanoseconds
 */
Deadline::Deadline(uint64_t duration_ns)
	: duration(duration_ns)
{
}

void Deadline::Start(void)
{
	GetTime(&startTime);
}

uint64_t Deadline::GetElapsed(void) const
{
	Instant now;
	GetTime(&now);
	return GetTimeDiff(&startTime, &now);
}

uint64_t Deadline::GetRemaining(void) const
{
	uint64_t elapsed = GetElapsed();
	return (elapsed > duration) ? 0 : (duration - elapsed);
}
