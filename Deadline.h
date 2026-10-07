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
	@file Deadline.h
	@brief Declaration of Deadline class
 */
#ifndef Deadline_h
#define Deadline_h

#include <cstdint>

#ifndef _WIN32
#include <ctime>
#else
#include <windows.h>
#endif

/**
	@brief Class representing a timer deadline
 */
class Deadline
{
public:
	static const uint64_t SECONDS_PER_NS = 1000000000ull;

	Deadline(uint64_t duration_ns);
	~Deadline() = default;

	//Start deadline timer
	void Start(void);
	//Return amount of elapsed nanoseconds
	uint64_t GetElapsed(void) const;
	//Return amount of remaining nanoseconds
	uint64_t GetRemaining(void) const;
	//Return whether deadline has elapsed
	bool HasElapsed(void) const { return GetRemaining() == 0; }

protected:
#ifdef _WIN32
	typedef LARGE_INTEGER Instant;
#else
	typedef struct timespec Instant;
#endif

	uint64_t duration;
	Instant startTime;
};

#endif
