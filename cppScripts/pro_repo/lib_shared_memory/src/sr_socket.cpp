/*****************************************************************************/
/*                                                                           */
/*              Copyright (c) SURGERII. All rights reserved                  */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* File name: *******.cpp            Version: 1.0                            */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Type: class definition            Programming language: C++               */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Author: Dai Liang                 Date: 16.04.2019                        */
/*                                                                           */
/*****************************************************************************/
/*                                                                           */
/* Description:                                                              */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/* Date     | History                                                        */
/* ---------+--------------------------------------------------------------- */
/*          |                                                                */
/*          |                                                                */
/*****************************************************************************/
#include "sr_socket.h"
#include <winsock2.h>
#include <Ws2tcpip.h>

#pragma comment (lib,"ws2_32.lib")
namespace sr
{
	namespace socket
	{
		SOCKET socket(SocketType type, SocketProtocol protocol)
		{
			WSADATA wsa;
			//Initialize win sock	
			if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
			{
				//log
				return SR_INVALID_SOCKET;
			}
			else
			{
				S32 temp_type = SOCK_DGRAM;
				switch (type)
				{
				case sr::socket::TYPE_STREAM:
					temp_type = SOCK_STREAM ;
					break;
				case sr::socket::TYPE_DGRAM:
					temp_type = SOCK_DGRAM;
					break;
				default:
					break;
				}
				S32 temp_protocol = IPPROTO_UDP;
				switch (protocol)
				{
				case sr::socket::PROTOCOL_TCP:
					temp_protocol = IPPROTO_TCP;
					break;
				case sr::socket::PROTOCOL_UDP:
					temp_protocol = IPPROTO_UDP;
					break;
				default:
					break;
				}
				return ::socket(AF_INET, temp_type, temp_protocol);
			}
		}

		S32 closeSocket(SOCKET s)
		{
			WSACleanup();
			return ::closesocket(s);
		}


		S32 bind(SOCKET s, const Sockaddr& addr)
		{
			struct sockaddr_in serverAddr;
			serverAddr.sin_family = AF_INET;
			//serverAddr.sin_addr.s_addr = inet_addr(addr.ip);
			InetPton(AF_INET, addr.ip, &serverAddr.sin_addr.s_addr);
			serverAddr.sin_port = htons(addr.port);
			if (::bind(s, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
			{
				return SR_SOCKET_ERROR;
			}
			return SR_SOCKET_OK;
		}


		S32 connect(SOCKET s, const Sockaddr& addr)
		{
			struct sockaddr_in serverAddr;
			serverAddr.sin_family = AF_INET;
			//serverAddr.sin_addr.s_addr = inet_addr(addr.ip);
			InetPton(AF_INET, addr.ip, &serverAddr.sin_addr.s_addr);
			serverAddr.sin_port = htons(addr.port);
			if (::connect(s, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
			{
				return SR_SOCKET_ERROR;
			}
			return SR_SOCKET_OK;
		}


		S32 listen(SOCKET s, S32 backlog)
		{
			if (::listen(s, backlog) == SOCKET_ERROR)
			{
				return SR_SOCKET_ERROR;
			}
			return SR_SOCKET_OK;
		}


		SOCKET accept(SOCKET s, const Sockaddr& addr)
		{
			struct sockaddr client_addr;
			int size = sizeof(client_addr);
			SOCKET client_sock = ::accept(s, &client_addr, &size);
			return client_sock;
		}


		S32 recvFrom(SOCKET s, void* const buff, U32 buffer_size, Sockaddr& from, S32 flags /*= 0*/)
		{
			struct sockaddr_in si_other;
			S32 slen = sizeof(si_other);
			S32 result =::recvfrom(s, (char *)buff, buffer_size, flags, (struct sockaddr *) &si_other, (int*)&slen);
			//char* addr = inet_ntoa(si_other.sin_addr);
			char addr[INET_ADDRSTRLEN];
			inet_ntop(AF_INET, &(si_other.sin_addr), addr, INET_ADDRSTRLEN);
			//if (addr != NULL)
			{
				memcpy(from.ip, addr, sizeof(from.ip));
			}
			from.port = ntohs(si_other.sin_port);
			return result;
		}


		S32 sendTo(SOCKET s, const void* const buff, U32 length, const Sockaddr& to, S32 flags /*= 0*/)
		{
			struct sockaddr_in to_ddr_in;
			to_ddr_in.sin_family = AF_INET;
			//to_ddr_in.sin_addr.s_addr = inet_addr(to.ip);
			InetPton(AF_INET, to.ip, &to_ddr_in.sin_addr.s_addr);
			to_ddr_in.sin_port = htons(to.port);
			return ::sendto(s, (const char *)buff, length, flags, (struct sockaddr *) &to_ddr_in, sizeof(struct sockaddr_in));
		}


		S32 recv(SOCKET s, void* const buff, U32 buffer_size, S32 flags/* = 0*/)
		{
			return ::recv(s, (char*)buff, buffer_size, flags);
		}

		S32 send(SOCKET s, const void* const buff, U32 length, S32 flags /*= 0*/)
		{
			return ::send(s, (char*)buff, length, flags);
		}

		bool getOptionByName(SocketOption optname, S32& option)
		{
			bool if_support = true;
			switch (optname)
			{
			case sr::socket::SNDBUF:
				option = SO_SNDBUF;
				break;
			case sr::socket::RCVBUF:
				option = SO_RCVBUF;
				break;
			case sr::socket::SNDTIMEO:
				option = SO_SNDTIMEO;
				break;
			case sr::socket::RCVTIMEO:
				option = SO_RCVTIMEO;
				break;
			default:
				if_support = false;
				break;
			}
			return if_support;
		}

		S32 getSockOpt(SOCKET s, SocketOption optname, void * const optval, S32 *optlen)
		{
			S32 temp_option;
			bool if_support = getOptionByName(optname, temp_option);
			if (if_support)
			{
				return ::getsockopt(s, 0, temp_option, (char*)optval, (int*)optlen);
			}
			return SR_SOCKET_ERROR;
		}


		S32 setSockOpt(SOCKET s, SocketOption optname, const void * const  optval, S32 optlen)
		{
			S32 temp_option;
			bool if_support = getOptionByName(optname, temp_option);
			if (if_support)
			{
				return ::setsockopt(s, 0, temp_option, (char*)optval, (int)optlen);
			}
			return SR_SOCKET_ERROR;
		}

		void convert_srfdset_sysfdset(const ::sr::socket::sr_fd_set& fdset,
			::fd_set& wfd_set) {
			//wfd_set.fd_count = fdset.fd_count;
			for (U32 i = 0; i < fdset.fd_count; i++) {
				//wfd_set.fds_bits[i] = fdset.fd_array[i];
				FD_SET(fdset.fd_array[i], &wfd_set);
			}
		}

		void convert_sysfdset_srfdset( ::sr::socket::sr_fd_set& fdset,
			const::fd_set& wfd_set){
			for (U32 i = 0; i < wfd_set.fd_count; i++){
				::sr::socket::sr_fd_set::SR_FD_SET(wfd_set.fd_array[i], fdset);
			}
		}

		S32 select(S32 nfds, void* const readfds, void* const writefds, void* const exceptfds, U32 timeout) {
			::fd_set rd_fd_set;
			::fd_set* p_rd_fd_set = NULL;
			FD_ZERO(&rd_fd_set);
			if (readfds){
				convert_srfdset_sysfdset(*(sr::socket::sr_fd_set*)readfds, rd_fd_set);
				p_rd_fd_set = &rd_fd_set;
			}
			
			::fd_set wt_fd_set;
			::fd_set* p_wt_fd_set = NULL;
			FD_ZERO(&wt_fd_set);
			if (writefds){
				convert_srfdset_sysfdset(*(sr::socket::sr_fd_set*)writefds, wt_fd_set);
				p_wt_fd_set = &wt_fd_set;
			}
			
			::fd_set ex_fd_set;
			::fd_set* p_ex_fd_set = NULL;
			FD_ZERO(&ex_fd_set);
			if (exceptfds){
				convert_srfdset_sysfdset(*(sr::socket::sr_fd_set*)exceptfds, ex_fd_set);
				p_ex_fd_set = &ex_fd_set;
			}
			
			timeval tv;
			auto sec = timeout / 1000;
			auto ms = timeout - sec * 1000;
			tv.tv_sec = sec;
			tv.tv_usec = 1000 * ms;

			int rc=  ::select(nfds, p_rd_fd_set, p_wt_fd_set, p_ex_fd_set, &tv);

			if (readfds){
				sr::socket::sr_fd_set::SR_FD_ZERO(*(sr::socket::sr_fd_set*)readfds);
				convert_sysfdset_srfdset(*(sr::socket::sr_fd_set*)readfds, rd_fd_set);
			}
			if (writefds){
				sr::socket::sr_fd_set::SR_FD_ZERO(*(sr::socket::sr_fd_set*)writefds);
				convert_sysfdset_srfdset(*(sr::socket::sr_fd_set*)writefds, wt_fd_set);
			}
			if (exceptfds){
				sr::socket::sr_fd_set::SR_FD_ZERO(*(sr::socket::sr_fd_set*)exceptfds);
				convert_sysfdset_srfdset(*(sr::socket::sr_fd_set*)exceptfds, ex_fd_set);
			}

			return rc;

		}
	}
}
