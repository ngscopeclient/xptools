/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
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
	@file Socket.cpp
	@brief Implementation of Socket class
 */

#include "Socket.h"
#include "../log/log.h"
#include "TimeUtil.h"
#include <cmath>
#include <memory.h>

#ifndef _WIN32
#include <netinet/tcp.h>
#endif

using namespace std;

/**
	@brief Creates a socket

	@param af Address family of the socket (layer 3 protocol selection)
	@param type Type of the socket (stream or datagram)
	@param protocol Protocol of the socket (layer 4 protocol selection)
 */
Socket::Socket(int af, int type, int protocol)
	: m_af(af), m_type(type), m_protocol(protocol), m_rxtimeout(0.0), m_txtimeout(0.0)
{
#ifdef _WIN32
	WSADATA wdat;
	if(0 != WSAStartup(MAKEWORD(2, 2), &wdat))
		LogError("Failed to initialize socket library\n");
#endif

	Open();
}

void Socket::Open()
{
	//For once - a nice, portable call, no #ifdefs required.
	m_socket = socket(m_af, m_type, m_protocol);

	if(!IsValid())
		LogError("Failed to create socket\n");
}

/**
	@brief Wraps an existing socket

	@param sock Socket to encapsulate
	@param af Address family of the provided socket
 */
Socket::Socket(ZSOCKET sock, int af) : m_af(af), m_rxtimeout(0.0), m_txtimeout(0.0), m_socket(sock)
{
	//TODO: get actual values?
	m_type = SOCK_STREAM;
	m_protocol = IPPROTO_TCP;

#ifdef _WIN32
	WSADATA wdat;
	if(0 != WSAStartup(MAKEWORD(2, 2), &wdat))
		LogFatal("Failed to initialize socket library\n");
#endif
}

/**
	@brief Closes a socket
 */
Socket::~Socket(void)
{
	Close();

#ifdef _WIN32
	WSACleanup();
#endif
}

void Socket::Close()
{
//There are a couple of different ways to close a socket...
#ifdef _WIN32
	if(m_socket != INVALID_SOCKET)
	{
		closesocket(m_socket);
		m_socket = INVALID_SOCKET;
	}
#else
	if(m_socket >= 0)
	{
		close(m_socket);
		m_socket = -1;
	}
#endif
}

/**
	@brief Establishes a TCP connection to a remote host

	@param host DNS name or string IP address of remote host
	@param port Port to connect to (host byte order)

	@return true on success, false on fail
 */
bool Socket::Connect(const std::string& host, uint16_t port)
{
	addrinfo hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;	//allow both v4 and v6
	hints.ai_socktype = m_type;

#ifndef _WIN32
	hints.ai_flags = AI_NUMERICSERV;	//numeric port number, implied on windows
#endif

	//Make ASCII port number
	char sport[6];
	snprintf(sport, sizeof(sport), "%5d", port);

#ifdef _WIN32
	//Do a DNS lookup
	ADDRINFO* address = NULL;
#else
	addrinfo* address = NULL;
#endif

	if(0 != (getaddrinfo(host.c_str(), sport, &hints, &address)))
	{
		LogWarning("DNS lookup for %s failed\n", host.c_str());
		return false;
	}
	if(address == NULL)
	{
		LogWarning("DNS lookup for %s failed\n", host.c_str());
		return false;
	}

	//Try actually connecting
	bool connected = false;
	for(addrinfo* p = address; p != NULL; p = p->ai_next)
	{
		m_af = p->ai_family;
		m_protocol = p->ai_protocol;
		Close();
		Open();

		//Connect to the socket
		if(0 == connect(m_socket, p->ai_addr, p->ai_addrlen))
		{
			connected = true;
			break;
		}
	}

	//BUGFIX: clean up here so we don't leak the address if the connection failed
	freeaddrinfo(address);

	//Connect to the socket
	if(!connected)
	{
		//Close the socket so destructor code won't try to send stuff to us
		Close();

		LogWarning("Failed to connect to %s\n", host.c_str());
		return false;
	}

	return true;
}

/**
	@brief Sends data over the socket

	@param buf Buffer to send
	@param count Length of data buffer

	@return true on success, false on fail
 */
bool Socket::SendLooped(const unsigned char* buf, int count)
{
	const unsigned char* p = buf;
	int bytes_left = count;
	int x = 0;

	double start = GetTime();

	while((x = send(m_socket, (const char*)p, bytes_left, 0)) > 0)
	{
		bytes_left -= x;
		p += x;
		if(bytes_left == 0)
			break;

		if((m_txtimeout > 0.0) && ((GetTime() - start) < m_txtimeout))
		{
			LogWarning("send timeout\n");
			return false;
		}
	}

	if(x < 0)
	{
		LogWarning("Socket write failed (errno=%d, %s)\n", errno, strerror(errno));
		return false;
	}
	else if(x == 0)
	{
		//LogWarning("Socket closed unexpectedly\n");
		return false;
	}

	return bytes_left == 0;
}

/**
	@brief Recives data from a UDP socket

	@param buf Output buffer
	@param len Length of the buffer
	@param addr IP address of the sender
	@param flags Socket flags

	@return Number of bytes read
 */
/*
size_t Socket::RecvFrom(void* buf, size_t len, sockaddr_in& addr,  int flags)
{
	socklen_t slen = sizeof(addr);
	return recvfrom(m_socket, buf, len, flags, reinterpret_cast<sockaddr*>(&addr), &slen);
}
*/

/**
	@brief Sends data to a UDP socket

	@param buf Input buffer
	@param len Length of the buffer
	@param addr IP address of the recipient
	@param flags Socket flags

	@return Number of bytes sent
 */
/*
size_t Socket::SendTo(void* buf, size_t len, sockaddr_in& addr,  int flags)
{
	size_t ret = sendto(m_socket, buf, len, flags, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
	if(ret != len)
	{
		throw JtagExceptionWrapper(
			"Socket sendto failed",
			"",
			JtagException::EXCEPTION_TYPE_NETWORK);
	}
	return ret;
}
*/

/**
	@brief Recieves data from the socket

	@param buf The buffer to read into
	@param len Length of read buffer

	@return true on success, false on fail
 */
bool Socket::RecvLooped(unsigned char* buf, int len)
{
	unsigned char* p = buf;
	int bytes_left = len;
	int x = 0;

	double start = GetTime();

	while(true)
	{
		bytes_left -= x;
		p += x;
		if(bytes_left == 0)
			break;

		if((m_rxtimeout > 0.0) && (((GetTime() - start) > m_rxtimeout)))
		{
			LogWarning("Socket read timed out\n");
			return false;
		}

		x = recv(m_socket, (char*)p, bytes_left, MSG_WAITALL);

		//Handle EINTR and EAGAIN
		#ifndef _WIN32
		if((x < 0) && ((errno == EINTR) || (errno == EAGAIN)))
		{
			x = 0;
			continue;
		}
		#endif

		if(x <= 0)
			break;
	}

	if(x < 0)
	{
		LogWarning("Socket read failed (errno=%d, %s)\n", errno, strerror(errno));
		return false;
	}
	else if(x == 0)
	{
		//LogWarning("Socket closed unexpectedly\n");
		return false;
	}

	return bytes_left == 0;
}

/**
	@brief Flush RX buffer

	@return true on success, false on fail
 */
void Socket::FlushRxBuffer(void)
{
	/* Why oh why has this never been made a SO_ ? */
	char dummyBuffer[2000];
#ifndef _WIN32
	while(recv(m_socket, dummyBuffer, sizeof(dummyBuffer), MSG_DONTWAIT) > 0)
		;
#else
	/* Windows does not support MSG_DONTWAIT... */
	unsigned long i;
	while(1)
	{
		ioctlsocket(m_socket, FIONREAD, &i);
		if(i == 0)
			break;
		recv(m_socket, dummyBuffer, (i > sizeof(dummyBuffer)) ? sizeof(dummyBuffer) : i, 0);
	}
#endif
}

bool Socket::SetTxBuffer(int bufsize)
{
	if(0 != setsockopt((int)m_socket, SOL_SOCKET, SO_SNDBUF, (char*)&bufsize, sizeof(bufsize)))
		return false;
	return true;
}

bool Socket::SetRxBuffer(int bufsize)
{
	if(0 != setsockopt((int)m_socket, SOL_SOCKET, SO_RCVBUF, (char*)&bufsize, sizeof(bufsize)))
		return false;
	return true;
}

/**
	@brief Binds the socket to an address

	TODO: allow binding to specific addresses etc

	@param port Port to listen on

	@return true on success, false on fail
 */
bool Socket::Bind(unsigned short port)
{
	sockaddr* addr;
	socklen_t len;
	sockaddr_in name;
	sockaddr_in6 name6;

	if(m_af == AF_INET)
	{
		memset(&name, 0, sizeof(name));

		//Set port number
		name.sin_family = m_af;
		name.sin_port = htons(port);
		addr = reinterpret_cast<sockaddr*>(&name);
		len = sizeof(name);
    } else {
#ifdef _WIN32
        // attempt to listen on dual-stack instead of v6-only
        // don't fail if this can't be set, just listen on v6-only
        // this option only available on Vista and later
        // ref: https://learn.microsoft.com/en-us/windows/win32/winsock/dual-stack-sockets
        DWORD flag = 0; // defaults to 1
        if (0 != setsockopt((ZSOCKET)m_socket, IPPROTO_IPV6, IPV6_V6ONLY, (const char *)&flag, sizeof(DWORD)))
            LogWarning("Can't enable dual-stack socket, listening on IPv6 only\n");
#endif

		memset(&name6, 0, sizeof(name6));

		//Set port number
		name6.sin6_family = m_af;
		name6.sin6_port = htons(port);
		addr = reinterpret_cast<sockaddr*>(&name6);
		len = sizeof(name6);
	}

	//Try binding the socket
	if(0 != ::bind(m_socket, addr, len))
	{
		LogError("Unable to bind socket\n");
		return false;
	}

	return true;
}

/**
	@brief Puts the socket in listening mode
 */
bool Socket::Listen()
{
	if(0 != listen(m_socket, SOMAXCONN))
	{
		LogWarning("Can't listen to socket\n");
		return false;
	}
	return true;
}

/**
	@brief Accepts an IPv4 connection on the socket

	@brief addr Output address of accepted connection
	@brief len Size of the output buffer

	@return Socket for the client connection
 */
Socket Socket::Accept(sockaddr_in* addr, ZSOCKLEN len)
{
	ZSOCKET sock = accept(m_socket, reinterpret_cast<sockaddr*>(addr), &len);

	//Error check
#ifdef _WIN32
	if(sock == INVALID_SOCKET)
#else
	if(sock < 0)
#endif
	{
		LogError("Failed to accept socket connection (make sure socket is in listening mode)\n");
	}

	return Socket(sock, m_af);
}

/**
	@brief Accepts a connection on the socket

	@brief addr Output address of accepted connection
	@brief len Size of the output buffer

	@return Socket for the client connection
 */
Socket Socket::Accept()
{
	sockaddr_storage addr;
	socklen_t len = sizeof(addr);
	ZSOCKET sock = accept(m_socket, reinterpret_cast<sockaddr*>(&addr), &len);

	//Error check
#ifdef _WIN32
	if(sock == INVALID_SOCKET)
#else
	if(sock < 0)
#endif
	{
		LogError("Failed to accept socket connection (make sure socket is in listening mode)\n");

#ifdef _WIN32
		return Socket(INVALID_SOCKET, m_af);
#else
		return Socket(-1, m_af);
#endif
	}

	return Socket(sock, m_af);
}

/**
	@brief Accepts a IPv6 connection on the socket

	@brief addr Output address of accepted connection
	@brief len Size of the output buffer

	@return Socket for the client connection
 */
Socket Socket::Accept(sockaddr_in6* addr, ZSOCKLEN len)
{
	ZSOCKET sock = accept(m_socket, reinterpret_cast<sockaddr*>(addr), &len);

	//Error check
#ifdef _WIN32
	if(sock == INVALID_SOCKET)
#else
	if(sock < 0)
#endif
	{
		LogError("Failed to accept socket connection (make sure socket is in listening mode)\n");
#ifdef _WIN32
		return Socket(INVALID_SOCKET, m_af);
#else
		return Socket(-1, m_af);
#endif
	}

	return Socket(sock, m_af);
}

/**
	@brief Detaches the socket from this object

	@return Socket handle. The caller is responsible for closing the handle.
 */
ZSOCKET Socket::Detach()
{
	ZSOCKET s = m_socket;
#ifdef _WIN32
	m_socket = INVALID_SOCKET;
#else
	m_socket = -1;
#endif
	return s;
}

/**
	@brief Sends a string to a socket

	@param fd		Socket handle
	@param str		String to send

	@return true on success, false on fail
 */
bool Socket::SendPascalString(const std::string& str)
{
	if(str.length() > 0xFFFFFFFF)
	{
		LogError("SendPascalString() requires input <4 GB");
		return false;
	}

	uint32_t len = str.length();
	if(!SendLooped((unsigned char*)&len, 4))
		return false;
	if(!SendLooped((unsigned char*)str.c_str(), len))
		return false;

	return true;
}

/**
	@brief Reads a Pascal-style string from a socket

	@return true on success, false on fail
 */
bool Socket::RecvPascalString(string& str)
{
	uint32_t len;
	if(!RecvLooped((unsigned char*)&len, 4))
		return false;
	int64_t tlen = static_cast<int64_t>(len) + 1;	 //use larger int to avoid risk of overflow if str len == 4GB
	char* rbuf = new char[tlen];
	bool err = RecvLooped((unsigned char*)rbuf, len);
	rbuf[len] = 0;				//null terminate the string
	str = string(rbuf, len);	//use sequence constructor since buffer may have embedded nulls
	delete[] rbuf;

	return err;
}

/**
	@brief Disable the Nagle algorithm on the socket so that messages get sent right away

	@return true on success, false on fail
 */
bool Socket::DisableNagle()
{
	int flag = 1;
	if(0 != setsockopt((int)m_socket, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag)))
		return false;

	return true;
}

/**
	@brief Disable delayed-ACK so that we send ACKs immediately upon packet receipt

	@return true on success, false on fail
 */
bool Socket::DisableDelayedACK()
{
#ifdef TCP_QUICKACK
	int flag = 1;
	if(0 != setsockopt((int)m_socket, IPPROTO_TCP, TCP_QUICKACK, (char*)&flag, sizeof(flag)))
		return false;
#endif

	//TODO: can this be done on Windows too?

	return true;
}

/**
	@brief Set SO_REUSEADDR on our socket, allowing binding to it again without waiting for
	       timeout if our task crashed hard.

	@return true on success, false on fail
 */
bool Socket::SetReuseaddr(bool on)
{
	int flag = on;
    if (0 != setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&flag, sizeof(flag)))
        return false;
    return true;
}

bool Socket::SetRxTimeout(unsigned int microSeconds)
{
#ifdef _WIN32
	// WinSock2 expects a DWORD here, that contains the timeout in milliseconds.
	DWORD timeout = (DWORD)(ceil((float)microSeconds / 1000.0f));
	if(0 != setsockopt((ZSOCKET)m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(DWORD)))
		return false;
#else
	struct timeval tv;
	tv.tv_sec = microSeconds / 1000000;
	tv.tv_usec = (suseconds_t)(microSeconds % 1000000);

	if(0 != setsockopt((ZSOCKET)m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)))
		return false;
#endif

	m_rxtimeout = (double)microSeconds / 1000000.0;
	return true;
}

bool Socket::SetTxTimeout(unsigned int microSeconds)
{
#ifdef _WIN32
	// WinSock2 expects a DWORD here, that contains the timeout in milliseconds.
	DWORD timeout = (DWORD)(ceil((float)microSeconds / 1000.0f));
	if(0 != setsockopt((ZSOCKET)m_socket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(DWORD)))
		return false;
#else
	struct timeval tv;
	tv.tv_sec = microSeconds / 1000000;
	tv.tv_usec = (suseconds_t)(microSeconds % 1000000);
	if(0 != setsockopt((ZSOCKET)m_socket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)))
		return false;
#endif

	m_txtimeout = (double)microSeconds / 1000000.0 ;
	return true;
}

Socket& Socket::operator=(ZSOCKET rhs)
{
	m_socket = rhs;
	return *this;
}
