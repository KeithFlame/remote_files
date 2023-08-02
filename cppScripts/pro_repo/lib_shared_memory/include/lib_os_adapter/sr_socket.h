/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: *******.h              Version: 1.0                            */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Type: class definition            Programming language: C++               */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Author: Dai Liang                 Date:16.04.2019                         */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description:                                                              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
#ifndef _SR_SOCKET_H_
#define _SR_SOCKET_H_

#include "sr_portable.h"
#ifndef linux
#include <winsock2.h>
#define SR_FD_MAX_ID 0x7FFFFFF
#else
#include <sys/select.h>
#define SR_FD_MAX_ID FD_SETSIZE
#endif


namespace sr
{
	namespace socket
	{
        enum SocketProtocol
		{
			PROTOCOL_TCP,
			PROTOCOL_UDP
		};
        enum SocketType
		{
			TYPE_STREAM,
			TYPE_DGRAM
		};
		struct Sockaddr {
			U16   port;
			S8    ip[16];
		};
		enum
		{
			SR_FD_SETSIZE = 64
		};
		enum SocketOption
		{
			SNDBUF,        /* send buffer size */
			RCVBUF,        /* receive buffer size */
			SNDTIMEO,      /* send timeout */
			RCVTIMEO       /* receive timeout */
		};

		typedef U64 SOCKET;
		typedef struct sr_fd_set {
			U32   fd_count;               /* how many are SET? */
			SOCKET  fd_array[SR_FD_SETSIZE];   /* an array of SOCKETs */
			static void SR_FD_SET(SOCKET fd, ::sr::socket::sr_fd_set& set)
			{
				if (set.fd_count < SR_FD_SETSIZE)
				{
					set.fd_array[set.fd_count++] = fd;
				}
			}
			static void SR_FD_CLR(SOCKET fd, ::sr::socket::sr_fd_set& set)
			{
				for (U32 i = 0; i < set.fd_count; i++)
				{
					if (set.fd_array[i] == fd)
					{
						while (i < set.fd_count - 1)
						{
							set.fd_array[i] = set.fd_array[i + 1];
							i++;
						}
						set.fd_count--;
						break;
					}
				}
			}
			static void SR_FD_ZERO(::sr::socket::sr_fd_set& set)
			{
				set.fd_count = 0;
			}
			static bool SR_FD_ISSET(SOCKET fd, ::sr::socket::sr_fd_set& set)
			{
				for (U32 i = 0; i < set.fd_count; i++)
				{
					if (set.fd_array[i] == fd)
					{
						return true;
					}
				}
				return false;
			}
		} sr_fd_set;

		const SOCKET SR_INVALID_SOCKET = (~0);
		const S32 SR_SOCKET_ERROR = -1;
		const S32 SR_SOCKET_OK = 0;
	
		SOCKET socket(SocketType type, SocketProtocol protocol);
		S32 closeSocket(SOCKET s);

		S32 bind(SOCKET s, const Sockaddr& addr);
		S32 connect(SOCKET s, const Sockaddr& addr);
		S32 listen(SOCKET s,S32 backlog);
		SOCKET accept(SOCKET s, const Sockaddr&addr);
		//UDP
		S32 recvFrom(SOCKET s, void* const buff, U32 buffer_size, Sockaddr& from, S32 flags = 0);
		S32 sendTo(SOCKET s, const void* const buff, U32 length, const Sockaddr& to, S32 flags = 0);
		//TCP
		S32 recv(SOCKET s, void* const buff, U32 buffer_size, S32 flags = 0);
		S32 send(SOCKET s, const void* const buff, U32 length, S32 flags = 0);
		//option
		S32 getSockOpt(SOCKET s, SocketOption optname, void * const optval, S32 *optlen);
		S32 setSockOpt(SOCKET s, SocketOption optname, const void * const optval, S32 optlen);

		S32 select(S32 nfds, void* const readfds, void* const writefds, void* const exceptfds, U32 ms);
	}
	
}
#endif
