/* vax_ci.h: Computer Interconnect adapter

   Copyright (c) 2017, Matt Burke

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
   THE AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of the author shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from the author.
*/

#ifndef _VAX_CI_H_
#define _VAX_CI_H_

/* Debug Streams */

#define DBG_REG         0x0001                          /* registers */
#define DBG_WRN         0x0002                          /* warnings */
#define DBG_REQID       0x0004                          /* request IDs/reponses */
#define DBG_SCSDG       0x0008                          /* SCS datagrams */
#define DBG_SCSMSG      0x0010                          /* SCS Messages */
#define DBG_PPDDG       0x0020                          /* PPD Datagrams */
#define DBG_BLKTF       0x0040                          /* block transfer requests/responses/confirms */
#define DBG_LCMD        0x0080                          /* local commands */
#define DBG_CONN        0x0100                          /* connections */
#define DBG_TRC         0x0200                          /* trace */

/* Port States */

#define PORT_UNINIT     0                               /* Uninitialised */
#define PORT_INIT       10                              /* Initialised */
#define PORT_ENABLED    20                              /* Enabled */

/* Datastructure types */

#define DYN_SCSDG       0x3B
#define DYN_SCSMSG      0x3C

/* PPD Offsets */

#define PPD_FLINK       0x00                            /* forward link */
#define PPD_BLINK       0x04                            /* backward link */
#define PPD_SIZE        0x08                            /* structure size */
#define PPD_TYPE        0x0A                            /* structure type */
#define PPD_SWFLAG      0x0B                            /* software flags */
#define PPD_PORT        0x0C                            /* src/dest port */
#define PPD_STATUS      0x0D                            /* status */
#define PPD_OPC         0x0E                            /* opcode */
#define PPD_FLAGS       0x0F                            /* flags */
#define PPD_LENGTH      0x10                            /* message length */
#define PPD_MTYPE       0x12                            /* message type */
#define PPD_STYPE       0x14                            /* SCS message type */
#define PPD_VCMMSK      0x10                            /* VCD modify mask */
#define PPD_VCMVAL      0x14                            /* VCD modify value */
#define PPD_VCPVAL      0x18                            /* VCD previous value */

/* PPD Sizes */

#define PPD_HDR         0x10                            /* PPD header size */
#define PPD_DGHDR       (PPD_HDR + 0x2)                 /* datagram header size */
#define PPD_MSGHDR      (PPD_HDR + 0x2)                 /* message header size */

/* Transmit Opcodes */

#define OPC_SNDDG       0x01                            /* send datagram */
#define OPC_SNDMSG      0x02                            /* send message */
#define OPC_RETCNF      0x03                            /* confirm return */
#define OPC_REQID       0x05                            /* request ID */
#define OPC_SNDRST      0x06                            /* send reset */
#define OPC_SNDSTRT     0x07                            /* send start */
#define OPC_REQDAT      0x08                            /* request data (at priority 0) */
#define OPC_REQDAT1     0x09                            /* request data (at priority 1) */
#define OPC_REQDAT2     0x0A                            /* request data (at priority 2) */
#define OPC_RETID       0x0B                            /* send ID */
#define OPC_SNDLB       0x0D                            /* send loopback */
#define OPC_REQMDAT     0x0E                            /* request maintenance data */
#define OPC_SNDDAT      0x10                            /* send data */
#define OPC_RETDAT      0x11                            /* return data */
#define OPC_SNDMDAT     0x12                            /* send maintenance data */

/* Local Opcodes */

#define OPC_INVTC       0x18                            /* invalidate translation cache */
#define OPC_SETCKT      0x19                            /* set circuit */
#define OPC_RDCNT       0x1A                            /* read counters */

/* Receive Opcodes */

#define OPC_M_RECV      0x20                            /* opcode receive flag */
#define OPC_DGREC       (OPC_M_RECV | OPC_SNDDG)        /* datagram received */
#define OPC_MSGREC      (OPC_M_RECV | OPC_SNDMSG)       /* message received */
#define OPC_CNFREC      (OPC_M_RECV | OPC_RETCNF)       /* confirm received */
#define OPC_REQREC      (OPC_M_RECV | OPC_REQID)        /* request ID recevied */
#define OPC_REQDATREC   (OPC_M_RECV | OPC_REQDAT)       /* request data received */
#define OPC_MCNFREC     0x29                            /* maintenance confirm received */
#define OPC_IDREC       (OPC_M_RECV | OPC_RETID)        /* ID received */
#define OPC_LBREC       (OPC_M_RECV | OPC_SNDLB)        /* loopback received */
#define OPC_SNDDATREC   (OPC_M_RECV | OPC_SNDDAT)       /* send data received */
#define OPC_DATREC      (OPC_M_RECV | OPC_RETDAT)       /* data recieved */
#define OPC_MDATREC     0x33                            /* maintenance data recieved */

/* PPD Flags */

#define PPD_RSP         1                               /* respond to command */
#define PPD_V_PS        1                               /* path select */
#define PPD_M_PS        0x3
#define  PPD_PSAUTO     0                               /* automatic */
#define  PPD_PS0        1                               /* use path 0 */
#define  PPD_PS1        2                               /* use path 1 */
#define GET_PATH(x)     ((x >> PPD_V_PS) & PPD_M_PS)
#define PPD_V_SP        4                               /* send path */
#define PPD_M_SP        0x3
#define PPD_V_M         4                               /* multiple value, blk xfer only */
#define PPD_M_M         0x7
#define PPD_P           0x80                            /* 0/1 for 512/576 data pkt size */
                                                        /*  or packing format for messages */
                                                        /*  to PDP 10/20 ports. */
#define PPD_FORCE       0x80                            /* force reset */
#define PPD_DSTART      0x80                            /* default start addr */
#define PPD_EXTCNT      0x80                            /* extended counters */

#define CI_GET16(p,w)   (((uint16) p[w]) | \
                        (((uint16) p[(w)+1]) << 8))
#define CI_GET32(p,w)   (((uint32) p[w]) | \
                        (((uint32) p[(w)+1]) << 8) | \
                        (((uint32) p[(w)+2]) << 16) | \
                        (((uint32) p[(w)+3]) << 24))

#define CI_PUT16(p,w,x) p[w] = (x) & 0xFF; \
                        p[(w)+1] = ((x) >> 8) & 0xFF
#define CI_PUT32(p,w,x) p[w] = (x) & 0xFF; \
                        p[(w)+1] = ((x) >> 8) & 0xFF; \
                        p[(w)+2] = ((x) >> 16) & 0xFF; \
                        p[(w)+3] = ((x) >> 24) & 0xFF

#define CI_MAXFR        1024                            /* max xfer */
#define CI_MAX_NODES    16

typedef struct {
    uint32 type;
    uint32 addr;
    size_t size;
    size_t length;
    uint8 data[CI_MAXFR];
} CI_PKT;

extern uint32 ci_state;
extern uint32 ci_node;
extern DEVICE ci_dev;

void ci_set_state (uint32 state);
t_stat ci_route_ppd (CI_PKT *pkt);
t_stat ci_send_packet (CI_PKT *pkt, size_t length);
t_stat ci_receive_packet (CI_PKT *pkt, uint8 port);
t_stat ci_open_vc (uint8 port);
t_stat ci_close_vc (uint8 port);
t_bool ci_check_vc (uint8 port);
t_stat ci_show_node (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat ci_set_node (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ci_show_group (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat ci_set_group (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ci_attach (UNIT *uptr, CONST char *cptr);
t_stat ci_detach (UNIT *uptr);
t_stat ci_svc (UNIT *uptr);
t_stat ci_port_reset (DEVICE *dptr);

#endif
