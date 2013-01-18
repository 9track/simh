/* ci_sock.c: OS-dependent socket routines

   Copyright (c) 2001-2005, Robert M Supnik

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

#include "sim_defs.h"
#include "ci_sock.h"
#include <signal.h>

/* OS dependent routines

   ci_master_sock      create master socket
   ci_accept_conn      accept connection
   ci_read_sock        read from socket
   ci_write_sock       write from socket
   ci_close_sock       close socket
   ci_setnonblock      set socket non-blocking
   ci_msg_sock         send message to socket
*/

int32 ci_sock_cnt = 0;

/* First, all the non-implemented versions */

#if defined (__OS2__) && !defined (__EMX__)

SOCKET ci_master_sock (int32 port)
{
return INVALID_SOCKET;
}

SOCKET ci_connect_sock (int32 ip, int32 port)
{
return INVALID_SOCKET;
}

SOCKET ci_accept_conn (SOCKET master, uint32 *ipaddr)
{
return INVALID_SOCKET;
}

int32 ci_read_sock (SOCKET sock, char *buf, int32 nbytes)
{
return -1;
}

int32 ci_write_sock (SOCKET sock, char *msg, int32 nbytes)
{
return 0;
}

void ci_close_sock (SOCKET sock, t_bool master)
{
return;
}

SOCKET ci_setnonblock (SOCKET sock)
{
return SOCKET_ERROR;
}

#else                                                   /* endif unimpl */

/* UNIX, Win32, Macintosh, VMS, OS2 (Berkeley socket) routines */

SOCKET ci_err_sock (SOCKET s, char *emsg, int32 flg)
{
int32 err = WSAGetLastError ();

printf ("Sockets: %s error %d\n", emsg, err);
ci_close_sock (s, flg);
return INVALID_SOCKET;
}

SOCKET ci_create_sock (int32 type)
{
SOCKET newsock;
int32 err;

#if defined (_WIN32)
WORD wVersionRequested; 
WSADATA wsaData; 
wVersionRequested = MAKEWORD (1, 1); 

if (ci_sock_cnt == 0) {
    err = WSAStartup (wVersionRequested, &wsaData);     /* start Winsock */ 
    if (err != 0) {
        printf ("Winsock: startup error %d\n", err);
        return INVALID_SOCKET;
        }
    }
ci_sock_cnt = ci_sock_cnt + 1;
#endif                                                  /* endif Win32 */
#if defined (SIGPIPE)
signal (SIGPIPE, SIG_IGN);                              /* no pipe signals */
#endif

if (type == SCPN_MCS) type = SCPN_UDP;
if (type == SCPN_TCP)
    newsock = socket (AF_INET, SOCK_STREAM, 0);         /* create tcp socket */
else if (type == SCPN_UDP)
    newsock = socket (AF_INET, SOCK_DGRAM, 0);          /* create udp socket */
else return INVALID_SOCKET;
if (newsock == INVALID_SOCKET) {                        /* socket error? */
    err = WSAGetLastError ();
    printf ("Sockets: socket error %d\n", err);
    return INVALID_SOCKET;
    }
return newsock;
}

SOCKET ci_master_sock (int32 port, int32 type)
{
SOCKET newsock;
struct sockaddr_in name;
int32 sta;

newsock = ci_create_sock (type);                           /* create socket */
if (newsock == INVALID_SOCKET) return newsock;              /* socket error? */

name.sin_family = AF_INET;                                  /* name socket */
name.sin_port = htons ((unsigned short) port);              /* insert port */
name.sin_addr.s_addr = htonl (INADDR_ANY);                  /* insert addr */

if (type == SCPN_MCS) {
    sta = ci_setnoportlock (newsock);                      /* allow port to be re-used */
    if (sta == SOCKET_ERROR)                                /* setsockopt error? */
        return ci_err_sock (newsock, "setsockopt", 1);
}
sta = bind (newsock, (struct sockaddr *) &name, sizeof (name));
if (sta == SOCKET_ERROR)                                    /* bind error? */
    return ci_err_sock (newsock, "bind", 1);
sta = ci_setnonblock (newsock);                            /* set nonblocking */
if (sta == SOCKET_ERROR)                                    /* fcntl error? */
    return ci_err_sock (newsock, "fcntl", 1);
if (type == SCPN_TCP) {
//    sta = ci_settcpnodelay (newsock);                      /* set nodelay */
//    if (sta == SOCKET_ERROR)                                /* setsockopt error? */
//        return ci_err_sock (newsock, "setsockopt", 1);
    sta = listen (newsock, 1);                              /* listen on socket */
    if (sta == SOCKET_ERROR)                                /* listen error? */
        return ci_err_sock (newsock, "listen", 1);
}
return newsock;                                             /* got it! */
}

SOCKET ci_connect_sock (int32 ip, int32 port)
{
SOCKET newsock;
struct sockaddr_in name;
int32 sta;

newsock = ci_create_sock (SCPN_TCP);                   /* create socket */
if (newsock == INVALID_SOCKET) return newsock;          /* socket error? */

name.sin_family = AF_INET;                              /* name socket */
name.sin_port = htons ((unsigned short) port);          /* insert port */
name.sin_addr.s_addr = htonl (ip);                      /* insert addr */

sta = ci_setnonblock (newsock);                        /* set nonblocking */
if (sta == SOCKET_ERROR)                                /* fcntl error? */
    return ci_err_sock (newsock, "fcntl", 1);
//sta = ci_settcpnodelay (newsock);                      /* set nodelay */
//if (sta == SOCKET_ERROR)                                /* setsockopt error? */
//    return ci_err_sock (newsock, "setsockopt", 1);
sta = connect (newsock, (struct sockaddr *) &name, sizeof (name));
if ((sta == SOCKET_ERROR) && 
    (WSAGetLastError () != WSAEWOULDBLOCK) &&
    (WSAGetLastError () != WSAEINPROGRESS))
    return ci_err_sock (newsock, "connect", 1);

return newsock;                                         /* got it! */
}

SOCKET ci_accept_conn (SOCKET master, uint32 *ipaddr)
{
int32 sta, err;
#if defined (macintosh) || defined (__linux) || \
    defined (__APPLE__) || defined (__OpenBSD__)
socklen_t size;
#elif defined (_WIN32) || defined (__EMX__) ||\
     (defined (__ALPHA) && defined (__unix__))
int size;
#else 
size_t size; 
#endif
SOCKET newsock;
struct sockaddr_in clientname;

if (master == 0) return INVALID_SOCKET;                 /* not attached? */
size = sizeof (clientname);
newsock = accept (master, (struct sockaddr *) &clientname, &size);
if (newsock == INVALID_SOCKET) {                        /* error? */
    err = WSAGetLastError ();
    if (err != WSAEWOULDBLOCK)
        printf ("Sockets: accept error %d\n", err);
    return INVALID_SOCKET;
    }
if (ipaddr != NULL) *ipaddr = ntohl (clientname.sin_addr.s_addr);

sta = ci_setnonblock (newsock);                        /* set nonblocking */
if (sta == SOCKET_ERROR)                                /* fcntl error? */
    return ci_err_sock (newsock, "fcntl", 0);
return newsock;
}

int32 ci_check_conn (SOCKET sock, t_bool rd)
{
fd_set rw_set, er_set;
fd_set *rw_p = &rw_set;
fd_set *er_p = &er_set;
struct timeval tz;

timerclear (&tz);
FD_ZERO (rw_p);
FD_ZERO (er_p);
FD_SET (sock, rw_p);
FD_SET (sock, er_p);
if (rd) select ((int) sock + 1, rw_p, NULL, er_p, &tz);
else select ((int) sock + 1, NULL, rw_p, er_p, &tz);
if (FD_ISSET (sock, rw_p)) return 1;
if (FD_ISSET (sock, er_p)) return -1;
return 0;
}

int32 ci_read_sock_tcp (SOCKET sock, char *buf, int32 nbytes)
{
int32 rbytes, err;

rbytes = recv (sock, buf, nbytes, 0);
if (rbytes == SOCKET_ERROR) {
    err = WSAGetLastError ();
    if (err == WSAEWOULDBLOCK) return 0;                /* no data */
    printf ("Sockets: read error %d\n", err);
    return -1;
    }
return rbytes;
}

int32 ci_read_sock_udp (SOCKET sock, char *buf, int32 nbytes, int32 *ip, int32 *port)
{
int32 rbytes, err;
struct sockaddr_in name;
int32 namelen;

namelen = sizeof(name);
rbytes = recvfrom (sock, buf, nbytes, 0, (struct sockaddr *) &name, &namelen);
if (rbytes == SOCKET_ERROR) {
    err = WSAGetLastError ();
    if (err == WSAEWOULDBLOCK) return 0;                /* no data */
    printf ("Sockets: read error %d\n", err);
    return -1;
    }
*ip = ntohl(name.sin_addr.s_addr);
*port = ntohs(name.sin_port);
return rbytes;
}

int32 ci_write_sock_tcp (SOCKET sock, char *msg, int32 nbytes)
{
return send (sock, msg, nbytes, 0);
}

int32 ci_write_sock_udp (SOCKET sock, char *msg, int32 nbytes, int32 ip, int32 port)
{
struct sockaddr_in name;
int32 namelen;

name.sin_family = AF_INET;                              /* name socket */
name.sin_port = htons ((unsigned short) port);          /* insert port */
name.sin_addr.s_addr = htonl (ip);                      /* insert addr */
namelen = sizeof(name);

return sendto (sock, msg, nbytes, 0, (struct sockaddr *) &name, namelen);
}

void ci_close_sock (SOCKET sock, t_bool master)
{
#if defined (_WIN32)
closesocket (sock);
if (master) {
    ci_sock_cnt = ci_sock_cnt - 1;
    if (ci_sock_cnt <= 0) {
        WSACleanup ();
        ci_sock_cnt = 0;
        }
    }
#else
close (sock);
#endif
return;
}

int32 ci_settcpnodelay (SOCKET sock)
{
int flag = 1;
int result;

result = setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
if (result < 0) return SOCKET_ERROR;
return 0;
}

int32 ci_setnoportlock (SOCKET sock)
{
int flag = 1;
int result;
result = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *) &flag, sizeof(int));
if (result < 0) return SOCKET_ERROR;
return 0;
}

int32 ci_setipmulticast (SOCKET sock, int32 multi_ip)
{
struct ip_mreq mreq;
int result, sock_type, sock_type_len;

sock_type_len = sizeof(sock_type);
result = getsockopt(sock, SOL_SOCKET, SO_TYPE, (char *) &sock_type, &sock_type_len);
if (result < 0) return SOCKET_ERROR;
if (sock_type != SOCK_DGRAM) return SOCKET_ERROR;

mreq.imr_multiaddr.s_addr = htonl(multi_ip);
mreq.imr_interface.s_addr = htonl(INADDR_ANY);
result = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq));
if (result < 0) return SOCKET_ERROR;
return SCPE_OK;
}

#if defined (_WIN32)                                    /* Windows */
SOCKET ci_setnonblock (SOCKET sock)
{
unsigned long non_block = 1;

return ioctlsocket (sock, FIONBIO, &non_block);         /* set nonblocking */
}

#elif defined (VMS)                                     /* VMS */
SOCKET ci_setnonblock (SOCKET sock)
{
int non_block = 1;

return ioctl (sock, FIONBIO, &non_block);               /* set nonblocking */
}

#else                                                   /* Mac, Unix, OS/2 */
int32 ci_setnonblock (SOCKET sock)
{
int32 fl, sta;

fl = fcntl (sock, F_GETFL,0);                           /* get flags */
if (fl == -1) return SOCKET_ERROR;
sta = fcntl (sock, F_SETFL, fl | O_NONBLOCK);           /* set nonblock */
if (sta == -1) return SOCKET_ERROR;
#if !defined (macintosh) && !defined (__EMX__)          /* Unix only */
sta = fcntl (sock, F_SETOWN, getpid());                 /* set ownership */
if (sta == -1) return SOCKET_ERROR;
#endif
return 0;
}

#endif                                                  /* endif !Win32 && !VMS */

/* get_ipaddr           IP address:port

   Inputs:
        cptr    =       pointer to input string
   Outputs:
        ipa     =       pointer to IP address (may be NULL), 0 = none
        ipp     =       pointer to IP port (may be NULL), 0 = none
        result  =       status
*/

t_stat get_ipaddr (CONST char *cptr, uint32 *ipa, uint32 *ipp)
{
char gbuf[CBUFSIZE];
char *addrp, *portp, *octetp;
uint32 i, addr, port, octet;
t_stat r;

if ((cptr == NULL) || (*cptr == 0))
    return SCPE_ARG;
strncpy (gbuf, cptr, CBUFSIZE);
addrp = gbuf;                                           /* default addr */
if (portp = strchr (gbuf, ':'))                         /* x:y? split */
    *portp++ = 0;
else if (strchr (gbuf, '.'))                            /* x.y...? */
    portp = NULL;
else {
    portp = gbuf;                                       /* port only */
    addrp = NULL;                                       /* no addr */
    }
if (portp) {                                            /* port string? */
    if (ipp == NULL)                                    /* not wanted? */
        return SCPE_ARG;
    port = (int32) get_uint (portp, 10, 65535, &r);
    if ((r != SCPE_OK) || (port == 0))
        return SCPE_ARG;
    }
else port = 0;
if (addrp) {                                            /* addr string? */
    if (ipa == NULL)                                    /* not wanted? */
        return SCPE_ARG;
    for (i = addr = 0; i < 4; i++) {                    /* four octets */
        octetp = strchr (addrp, '.');                   /* find octet end */
        if (octetp != NULL)                             /* split string */
            *octetp++ = 0;
        else if (i < 3)                                 /* except last */
            return SCPE_ARG;
        octet = (int32) get_uint (addrp, 10, 255, &r);
        if (r != SCPE_OK)
            return SCPE_ARG;
        addr = (addr << 8) | octet;
        addrp = octetp;
        }
    if (((addr & 0377) == 0) || ((addr & 0377) == 255))
        return SCPE_ARG;
    }
else addr = 0;
if (ipp)                                                /* return req values */
    *ipp = port;
if (ipa)
    *ipa = addr;
return SCPE_OK;   
}

#endif                                                  /* end else !implemented */
