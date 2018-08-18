/* ci_sock.h: OS-dependent socket routines header file

   Copyright (c) 2001-2009, Robert M Supnik

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   ROBERT M SUPNIK BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of Robert M Supnik shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Robert M Supnik.
*/

#ifndef _CI_SOCK_H_
#define _CI_SOCK_H_    0

#if defined (_WIN32)                                    /* Windows */
#undef INT_PTR                                          /* hack, hack */
#include <winsock.h>
#include <ws2tcpip.h>

#elif !defined (__OS2__) || defined (__EMX__)           /* VMS, Mac, Unix, OS/2 EMX */
#define WSAGetLastError()       errno                   /* Windows macros */
#define SOCKET          int32
#define WSAEWOULDBLOCK  EWOULDBLOCK
#define WSAEINPROGRESS  EINPROGRESS
#define INVALID_SOCKET  -1 
#define SOCKET_ERROR    -1
#include <sys/types.h>                                  /* for fcntl, getpid */
#include <sys/socket.h>                                 /* for sockets */
#include <fcntl.h>
#include <unistd.h>
#include <netinet/in.h>                                 /* for sockaddr_in */
#include <netinet/tcp.h>                                /* for TCP_NODELAY */
#include <netdb.h>
#include <sys/time.h>                                   /* for EMX */
#endif

#if defined (VMS)                                       /* VMS unique */
#include <ioctl.h>                                      /* for ioctl */
#if !defined (timerclear)
#define timerclear(tvp)         (tvp)->tv_sec = (tvp)->tv_usec = 0
#endif
#endif
#if defined(__EMX__)                                    /* OS/2 unique */
#if !defined (timerclear)
#define timerclear(tvp)         (tvp)->tv_sec = (tvp)->tv_usec = 0
#endif
#endif

#define SCPN_TCP 0
#define SCPN_UDP 1
#define SCPN_MCS 2

SOCKET ci_master_sock (int32 port, int32 type);
SOCKET ci_connect_sock (int32 ip, int32 port);
SOCKET ci_create_sock (int32 type);
SOCKET ci_accept_conn (SOCKET master, uint32 *ipaddr);
int32 ci_check_conn (SOCKET sock, t_bool rd);
int32 ci_read_sock_tcp (SOCKET sock, char *buf, int32 nbytes);
int32 ci_write_sock_tcp (SOCKET sock, char *msg, int32 nbytes);
int32 ci_read_sock_udp (SOCKET sock, char *buf, int32 nbytes, int32 *ip, int32 *port);
int32 ci_write_sock_udp (SOCKET sock, char *msg, int32 nbytes, int32 ip, int32 port);
void ci_close_sock (SOCKET sock, t_bool master);
int32 ci_settcpnodelay (SOCKET sock);
int32 ci_setnoportlock (SOCKET sock);
int32 ci_setipmulticast (SOCKET sock, int32 multi_ip);
SOCKET ci_setnonblock (SOCKET sock);
t_stat get_ipaddr (CONST char *cptr, uint32 *ipa, uint32 *ipp);

#endif
