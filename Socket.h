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
    @file Socket.h
    @brief Declaration of Socket class
 */
#ifndef Socket_h
#define Socket_h

#include <string>

//Pull in some OS specific stuff
#ifdef _WIN32

#include <ws2tcpip.h>
#define ZSOCKLEN int
#define ZSOCKET SOCKET

#else

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#define ZSOCKLEN socklen_t
#define ZSOCKET int

#endif

/**
    @brief Class representing a network socket.
 */
class Socket
{
   public:
	Socket(int af, int type, int protocol);

	//Create a Socket object from an existing socket
	Socket(ZSOCKET sock, int af = PF_INET);

	//Destructor
	virtual ~Socket(void);

	//Connect to a host (automatic DNS resolution)
	virtual bool Connect(const std::string& host, uint16_t port);

	//Bind to a port (any available interface)
	virtual bool Bind(unsigned short port);

	//Put us in listening mode
	virtual bool Listen();

	//Accept a new connection
	virtual Socket Accept(sockaddr_in* addr, ZSOCKLEN len);
	virtual Socket Accept(sockaddr_in6* addr, ZSOCKLEN len);
	virtual Socket Accept();

	//Disconnect us from the socket object
	virtual ZSOCKET Detach();

	//Send / receive rawdata
	virtual bool SendLooped(const unsigned char* buf, int count);
	virtual bool RecvLooped(unsigned char* buf, int len);
	virtual bool RecvLooped(unsigned char* buf, int len, int timeout);
	//size_t RecvFrom(void* buf, size_t len, sockaddr_in& addr, int flags = 0);
	//size_t SendTo(void* buf, size_t len, sockaddr_in& addr, int flags = 0);

	//Send/receive a string
	virtual bool RecvPascalString(std::string& str);
	virtual bool SendPascalString(const std::string& str);

	//Set TCP_NODELAY on our socket
	bool DisableNagle();

	//Set RX/TX timeouts
	bool SetRxTimeout(unsigned int microSeconds);
	bool SetTxTimeout(unsigned int microSeconds);

	/**
    	@brief Convert us to the native OS socket type
    	@return A reference to our socket handle
	 */
	operator ZSOCKET&() { return m_socket; }

	bool IsValid() const
	{
#ifdef _WIN32
		return (m_socket != INVALID_SOCKET);
#else
		return (m_socket >= 0);
#endif
	}

	virtual void Close();

   protected:
	virtual void Open();

	/**
    	@brief Address family of this socket (typically AF_INET or AF_INET6)
	 */
	int m_af;

	//Type of the socket
	int m_type;

	//Protocol of the socket
	int m_protocol;

	/**
    	@brief The socket handle
	 */
	ZSOCKET m_socket;
};

#endif
