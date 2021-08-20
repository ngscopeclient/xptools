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
	@brief Implementation of UART
 */

//#include "jtaghal.h"
#include "UART.h"
#include <stdio.h>
#include <memory.h>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>

#ifdef __linux__
#include <asm/termios.h>

//asm/termios.h seems to conflict with sys/ioctl.h and termios.h
//so just pull these by hand
//define __THROW to avoid build issue when it's not available from the libc
#ifndef __THROW
# define __THROW
#endif
extern "C" int tcflush (int __fd, int __queue_selector) __THROW;
extern "C" int ioctl (int __fd, unsigned long int __request, ...) __THROW;

#else 
#include <termios.h>
#endif // __linux__

#endif

using namespace std;

/**
	@brief Connects to a serial port

	@throw JtagException on failure

	@param devfile 	The device file
	@param baud		Baud rate to use (in bits per second)
 */
UART::UART(const std::string& devfile, int baud)
	: m_networked(false)
	, m_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)
{
	Connect(devfile, baud);
}

/**
	@brief Constructor
 */
UART::UART()
	: m_networked(false)
	, m_fd(INVALID_FILE_DESCRIPTOR)
	, m_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)
{
}


/**
	@brief Destructor
 */
UART::~UART() {
	Close();
}

/**
	@brief Connects to a serial port

	@throw JtagException on failure

	@param devfile 	The device file
	@param baud		Baud rate to use (in bits per second)
 */
bool UART::Connect(const std::string& devfile, int baud)
{
	if(devfile.find(":") != string::npos)
	{
		//It's a socket, connect to it
		m_networked = true;

		char host[128];
		unsigned int port;
		fflush(stdout);
		sscanf(devfile.c_str(), "%127[^:]:%6u", host, &port);
		//LogTrace("Connecting to %s:%d\n", host, port);
		return m_socket.Connect(host, port);
	}
	else
	{
	#ifdef _WIN32
		(const void)devfile;
		(const void)baud;
		m_fd = INVALID_HANDLE_VALUE;
		LogError("Windows UART stuff not implemented");
		return false;
	#else
		//Open the UART
		//LogTrace("Opening TTY %s\n", devfile.c_str());
		m_fd = open(devfile.c_str(), O_RDWR);
		if(m_fd < 0)
		{
			LogError("Could not open UART file %s\n", devfile.c_str());
			return false;
		}

		//Set flags - linux doesn't support custom bauds in termios
#ifdef __linux__
		termios2 flags;
		memset(&flags, 0, sizeof(flags));
		ioctl(m_fd, TCGETS2, &flags);
		flags.c_cflag = CS8 | CLOCAL | CREAD | BOTHER;
		flags.c_iflag = IGNBRK | IGNPAR;
		flags.c_oflag = 0;
		flags.c_cc[VMIN] = 1;
		flags.c_ispeed = baud;
		flags.c_ospeed = baud;
		if(0 != tcflush(m_fd, TCIFLUSH))
		{
			LogError("Fail to flush tty\n");
			return false;
		}

		if(0 != ioctl(m_fd, TCSETS2, &flags))
		{
			LogError("Fail to set attr\n");
			return false;
		}
#else
		termios flags;
		memset(&flags, 0, sizeof(flags));
		tcgetattr(m_fd, &flags);
		flags.c_cflag = CS8 | CLOCAL | CREAD; 
		flags.c_iflag = IGNBRK | IGNPAR;
		flags.c_oflag = 0;
		flags.c_cc[VMIN] = 1;
		flags.c_ispeed = baud;
		flags.c_ospeed = baud;
		if(0 != tcflush(m_fd, TCIFLUSH))
		{
			LogError("Fail to flush tty\n");
			return false;
		}
		if(0 != tcsetattr(m_fd, TCSANOW, &flags))
		{
			LogError("Fail to set attr\n");
			return false;
		}
#endif
		/*
		//Put the file in nonblocking mode temporarily
		int f = fcntl(m_fd, F_GETFL, 0);
		if(f >= 0)
			fcntl(m_fd, F_SETFL, f | O_NONBLOCK);

		//Do nonblocking reads for 500ms to clear out junk left over from board reset
		//TODO: see if this should be default or not?
		double t = GetTime() + 0.5;
		unsigned char unused;
		while(GetTime() < t)
			read(m_fd, &unused, 1);

		//Return to blocking mode
		fcntl(m_fd, F_SETFL, f);
		*/
	#endif
	}

	return true;
}

/**
	@brief Disconnects from the serial port
 */
void UART::Close()
{
	if (m_networked)
		return m_socket.Close();

#ifdef _WIN32
	//TODO
#else
	close(m_fd);
#endif

	m_fd = INVALID_FILE_DESCRIPTOR;
}

bool UART::Read(unsigned char* data, int len)
{
	if(m_networked)
		return m_socket.RecvLooped(data, len);
	else
	{
		#ifdef _WIN32
			(void)data;
			(void)len;
			LogError("UART stuff not implemented\n");
			return false;
		#else
			int x = 0;
			while( (x = read(m_fd, (char*)data, len)) > 0)
			{
				len -= x;
				data += x;
				if(len == 0)
					break;
			}

			if(x < 0)
			{
				LogWarning("UART read failed\n");
				return false;
			}
			else if(x == 0)
			{
				//LogWarning("Socket closed unexpectedly\n");
				return false;
			}

			return true;
		#endif
	}
}

bool UART::Write(const unsigned char* data, int len)
{
	if(m_networked)
		return m_socket.SendLooped(data, len);
	else
	{
		#ifdef _WIN32
			(void)data;
			(void)len;
			LogError("UART stuff not implemented\n");
			return false;
		#else
			int x = 0;
			while( (x = write(m_fd, (const char*)data, len)) > 0)
			{
				len -= x;
				data += x;
				if(len == 0)
					break;
			}

			if(x < 0)
			{
				LogWarning("UART write failed\n");
				return false;
			}
			else if(x == 0)
			{
				//LogWarning("Socket closed unexpectedly\n");
				return false;
			}

			return true;
		#endif
	}
}
