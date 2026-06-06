/***********************************************************************************************************************
*                                                                                                                      *
* xptools                                                                                                              *
*                                                                                                                      *
* Copyright (c) 2012-2026 Andrew D. Zonenberg and contributors                                                         *
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

#include "UART.h"
#include <stdio.h>
#include <memory.h>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>

#ifdef USE_TERMIOS2
#include <asm/termios.h>

//asm/termios.h seems to conflict with sys/ioctl.h and termios.h
//so just pull these by hand. musl libc doesn't define __THROW so define it as blank
#ifndef __THROW
#define __THROW
#endif
extern "C" int tcflush (int __fd, int __queue_selector) __THROW;
extern "C" int ioctl (int __fd, unsigned long int __request, ...) __THROW;

#else
#include <termios.h>
#endif // __linux__

#else
#include <Windows.h>
#include <setupapi.h>
#include <devguid.h>
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

	@param devfile 		The device file
	@param baud			Baud rate to use (in bits per second)
	@param dtrEnable	True if DTR line should be enabled (needed by some devices like the NanoVNA to communicate)
	@param txUs			Transmit timeout in microseconds (default 50,000 = 50 milliseconds)
	@param rxUs			Recieve timeout in microseconds (default 500,000 = 500 milliseconds)

	//TODO timeouts only implemented for WIN32
 */
bool UART::Connect(
	const string& devfile,
	int baud,
	[[maybe_unused]] bool dtrEnable,
	[[maybe_unused]] unsigned int txUs,
	[[maybe_unused]]unsigned int rxUs)
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
		string win32_portname;

		// Specify the COM port name using \\.\COM10
		// this is needed for COM10 and above on windows
		// note the escape character is also backslash
		if(devfile.find("COM")==0)
		{
			// pre-append "\\.\"
			win32_portname = R"(\\.\)" + devfile;
		}
		else if(devfile.find(R"(\\.\COM)")==0)
		{
			// the "\\.\"" is already there
			win32_portname = devfile;
		}
		else
		{
			// the name of the COM port is not valid
			LogError("COM port name %s invalid, expected COM1 or \\\\.\\COM1\n",devfile.c_str());
			return false;
		}

		m_fd = CreateFileA(win32_portname.c_str(),			  // port name
							GENERIC_READ | GENERIC_WRITE, // Read/Write
							0,                            // No Sharing
							NULL,                         // No Security
							OPEN_EXISTING,// Open existing port only
							0,            // Non Overlapped I/O
							NULL);        // Null for Comm Devices

		if (m_fd == INVALID_HANDLE_VALUE)
		{
			LogError("Could not open COM port %s\n", win32_portname.c_str());
			return false;
		}
		// Configure port
		DCB dcbSerialParams; // Initializing DCB structure
		SecureZeroMemory(&dcbSerialParams, sizeof(DCB));
   		dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
		// Read current config
		bool result = GetCommState(m_fd, &dcbSerialParams);
		if(!result)
		{
			LogError("Could not get state for COM port %s\n", win32_portname.c_str());
			return false;
		}
		// Set values
		dcbSerialParams.BaudRate = baud;  		// Setting BaudRate
		dcbSerialParams.ByteSize = 8;         	// Setting ByteSize = 8
		dcbSerialParams.StopBits = ONESTOPBIT;	// Setting StopBits = 1
		dcbSerialParams.Parity   = NOPARITY;  	// Setting Parity = None
		if(dtrEnable)
			dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;	// Needed by some devices like the NanoVNA to communicate
		// Set port configuration
		result = SetCommState(m_fd, &dcbSerialParams);
		if(!result)
		{
			LogError("Could not set state for COM port %s\n", win32_portname.c_str());
			return false;
		}
		// Set timeouts

		if(!SetTimeouts(txUs,rxUs))
			return false;

		return true;
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
#ifdef USE_TERMIOS2
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
	@brief Sets timeouts on a UART that is open

	@param txUs			Transmit timeout in microseconds (default 50,000 = 50 milliseconds)
	@param rxUs			Recieve timeout in microseconds (default 500,000 = 500 milliseconds)

	//TODO timeouts only implemented for WIN32
 */
bool UART::SetTimeouts([[maybe_unused]] unsigned int txUs, [[maybe_unused]] unsigned int rxUs)
{
	#ifdef WIN32
		// check if the file handle is open before attempting to use it
		if(IsValid())
		{
			COMMTIMEOUTS timeouts;
			SecureZeroMemory(&timeouts, sizeof(COMMTIMEOUTS));
			timeouts.ReadIntervalTimeout        = 50; // Timeout between each byte (in milliseconds)
			timeouts.ReadTotalTimeoutConstant   = rxUs/1000;// Total timeout for a read operation (in milliseconds)
			timeouts.ReadTotalTimeoutMultiplier = 1;  // Multiplier for each byte
			timeouts.WriteTotalTimeoutConstant  = txUs/1000; // in milliseconds
			timeouts.WriteTotalTimeoutMultiplier = 10;// in milliseconds
			bool result = SetCommTimeouts(m_fd, &timeouts);
			if(!result)
			{
				LogError("Could not set timeouts for COM port\n");
				return false;
			}
		}
		else
		{
			LogError("Did not set timeouts on UART that is not open\n");
		}

	#else
		LogError("UART::SetTimeouts is unimplemented on non-Windows platforms\n");
		return true;
	#endif

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
	// Closing the Serial Port
	CloseHandle(m_fd);
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
			long unsigned int x = 0;
			while(ReadFile( 	m_fd,			//Handle of the Serial port
             					(char*)data,    //Temporary character
             					len,			//Size of TempChar
             					&x,   			//Number of bytes read
             					NULL)
								&& (x != 0))	// Stop when stream is empty
			{
				len -= x;
				data += x;
				if(len == 0)
					break;
			}

			if(x == 0)
			{
				//LogWarning("Socket closed unexpectedly\n");
				return false;
			}

			return true;
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
			long unsigned int x = 0;
			while(WriteFile(	m_fd,        // Handle to the Serial port
                   				(const char*)data,     // Data to be written to the port
                   				len,  //No of bytes to write
                   				&x, //Bytes written
                   				NULL))
			{
				len -= x;
				data += x;
				if(len == 0)
					break;
			}

			if(x == 0)
			{
				//LogWarning("Socket closed unexpectedly\n");
				return false;
			}

			return true;
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

std::vector<UartDescriptor> UART::EnumerateUarts()
{
    std::vector<UartDescriptor> ports;

#ifdef _WIN32
    HDEVINFO deviceInfoSet = SetupDiGetClassDevs(
        &GUID_DEVCLASS_PORTS,
        nullptr,
        nullptr,
        DIGCF_PRESENT
    );

    if (deviceInfoSet == INVALID_HANDLE_VALUE)
        return ports;

    SP_DEVINFO_DATA devInfo;
    devInfo.cbSize = sizeof(SP_DEVINFO_DATA);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(deviceInfoSet, i, &devInfo); i++)
    {
        char desc[256] = {0};

        SetupDiGetDeviceRegistryPropertyA(
            deviceInfoSet,
            &devInfo,
            SPDRP_FRIENDLYNAME,
            nullptr,
            (PBYTE)desc,
            sizeof(desc),
            nullptr
        );

        // FriendlyName has the form "USB-SERIAL CH340 (COM3)"
        std::string friendly(desc);
        size_t begin = friendly.find("(COM");
        size_t end   = friendly.find(")");

        if (begin != std::string::npos && end != std::string::npos)
        {
            UartDescriptor info;
            info.port = friendly.substr(begin + 1, end - begin - 1); // "COM3"
            info.description = friendly.substr(0, begin - 1);        // Description without "(COM3)"
            ports.push_back(info);
        }
    }
    SetupDiDestroyDeviceInfoList(deviceInfoSet);
#else
    for (const auto& entry : std::filesystem::directory_iterator("/dev/serial/by-id"))
    {
        UartDescriptor info;
        info.description = entry.path().filename().string();
        info.port = std::filesystem::read_symlink(entry.path()).string();
        ports.push_back(info);
    }
#endif
    return ports;
}
