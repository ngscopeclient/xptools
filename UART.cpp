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
#include <termios.h>
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
	if(devfile.find(":") != string::npos)
	{
		//It's a socket, connect to it
		m_networked = true;

		char host[128];
		unsigned int port;
		fflush(stdout);
		sscanf(devfile.c_str(), "%127[^:]:%6u", host, &port);
		//printf("[UART] Connecting to %s:%d\n", host, port);
		m_socket.Connect(host, port);
	}
	else
	{
	#ifdef _WIN32
		(const void)devfile;
		(const void)baud;
		m_fd = INVALID_HANDLE_VALUE;
		LogError("Windows UART stuff not implemented",);
		return;
	#else
		//Open the UART
		m_fd = open(devfile.c_str(), O_RDWR);
		if(m_fd < 0)
		{
			LogError("Could not open UART file %s\n", devfile.c_str());
			return;
		}

		/*
			TODO
			B50
			B75
			B110
			B134
			B150
			B200
			B300
			B600
			B1200
			B1800
			B2400
			B4800
			B9600
			B19200
			B38400
			B57600
			B115200
			B230400
		*/
		int baud_const = 0;
		switch(baud)
		{
		case 9600:
			baud_const = B9600;
			break;

		case 115200:
			baud_const = B115200;
			break;

		default:
			LogError("Invalid baud rate specified\n");
			return;
		}

		//Set flags
		termios flags;
		memset(&flags, 0, sizeof(flags));
		tcgetattr(m_fd, &flags);
		flags.c_cflag = baud_const | CS8 | CLOCAL | CREAD;
		flags.c_iflag = IGNBRK | IGNPAR;
		flags.c_cc[VMIN] = 1;
		if(0 != tcflush(m_fd, TCIFLUSH))
		{
			LogError("Fail to flush tty\n");
			return;
		}
		if(0 != tcsetattr(m_fd, TCSANOW, &flags))
		{
			LogError("Fail to set attr\n");
			return;
		}

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
}

/**
	@brief Disconnects from the serial port
 */
UART::~UART()
{
#ifdef _WIN32
	//TODO
#else
	close(m_fd);
	m_fd = 0;
#endif
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
			while( (x = recv(m_fd, (char*)data, len, 0)) > 0)
			{
				len -= x;
				data += x;
				if(len == 0)
					break;
			}

			if(x < 0)
			{
				LogWarning("Socket read failed\n");
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
			while( (x = send(m_fd, (const char*)data, len, 0)) > 0)
			{
				len -= x;
				data += x;
				if(len == 0)
					break;
			}

			if(x < 0)
			{
				LogWarning("Socket write failed\n");
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
