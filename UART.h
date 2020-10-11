/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2016 Andrew D. Zonenberg                                                                          *
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
	@file
	@author Andrew D. Zonenberg
	@brief Declaration of UART
 */

#ifndef UART_h
#define UART_h

#include "../log/log.h"
#include <string>
#include "Socket.h"

#ifdef _WIN32

typedef HANDLE FILE_DESCRIPTOR;
#define INVALID_FILE_DESCRIPTOR INVALID_HANDLE_VALUE

#else

typedef int FILE_DESCRIPTOR;
#define INVALID_FILE_DESCRIPTOR -1

#endif

/**
	@brief Wrapper class for a serial port
 */
class UART
{
public:

	UART();
	UART(const std::string& devfile, int baud);
	bool Connect(const std::string& devfile, int baud);
	void Close();
	virtual ~UART();

	bool Read(unsigned char* data, int len);
	bool Write(const unsigned char* data, int len);

	FILE_DESCRIPTOR GetHandle()
	{ return m_fd; }

	bool IsValid() const
	{
		if (m_networked)
			return m_socket.IsValid();

		return (m_fd != INVALID_FILE_DESCRIPTOR);
	}

protected:
	bool m_networked;
	FILE_DESCRIPTOR m_fd;
	Socket m_socket;
};

#endif
