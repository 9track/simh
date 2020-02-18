/* vax_ci.h: Computer Interconnect adapter

   Copyright (c) 2017-2019, Matt Burke

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
#define DBG_SCSMSG      0x0010                          /* SCS messages */
#define DBG_PPDDG       0x0020                          /* PPD datagrams */
#define DBG_BLKTF       0x0040                          /* block transfers */
#define DBG_LCMD        0x0080                          /* local commands */
#define DBG_CONN        0x0100                          /* connections */
#define DBG_TRC         0x0200                          /* trace */
#define DBG_PKT         0x0400                          /* packet data */

/* Unit Specific Data */

#define ci_node         u3                              /* port addr */
#define ci_state        u4                              /* port state */

/* Return status codes */

#define CISE_OK         0                               /* status OK */
#define CISE_INVOPC     1                               /* invalid opcode */
#define CISE_VCERR      2                               /* VC error */
#define CISE_QFULL      3                               /* queue full */
#define CISE_NOPKT      4                               /* queue empty */
#define CISE_QERR       5                               /* queue error */
#define CISE_MEMERR     6                               /* memory read/write error */

/* CI Port Types */

#define CI_CI780        2                               /* SBI bus */
#define CI_CI750        2                               /* CMI bus */
#define CI_HSC          4                               /* HSC50/HSC70 */
#define CI_KL10         6                               /* PDP10 */
#define CI_BCA          11                              /* BI bus */
#define CI_CIXCD        14                              /* XMI bus */
#define CI_CITCA        17                              /* TC bus */
#define CI_CIPCA        18                              /* PCI bus */
#define CI_SHAC         34
#define CI_RF70         48                              /* DSSI disks */
#define CI_RF71         48
#define CI_RF30         49
#define CI_RF31         50
#define CI_RF72         51
#define CI_RF32         52
#define CI_RF73         53
#define CI_RF31F        54
#define CI_RF35         55
#define CI_RF36         58
#define CI_RF37         59
#define CI_RF74         60
#define CI_RF75         61
#define CI_TF70         64                              /* DSSI tapes */
#define CI_TF30         65
#define CI_TF85         65
#define CI_TF86         66
#define CI_HSJ          80
#define CI_HSD          81

#define CI_DUALPATH     0x80000000

/* Port States */

#define PORT_UNINIT     0                               /* Uninitialised */
#define PORT_INIT       10                              /* Initialised */
#define PORT_ENABLED    20                              /* Enabled */

/* Datastructure types */

#define DYN_CIDG        0x3B                            /* CI datagram */
#define DYN_CIMSG       0x3C                            /* CI message */

/* PPD Offsets */

#define PPD_FLINK       0x00                            /* forward link */
#define PPD_BLINK       0x04                            /* backward link */
#define PPD_SIZE        0x08                            /* structure size */
#define  PPD_DG         0                               /* datagram */
#define  PPD_MSG        1                               /* message */
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

#define PPD_XCTID       0x10                            /* transaction ID */
#define PPD_LCONID      0x10                            /* local connection id */
#define PPD_RSPID       0x14                            /* local response id */
#define PPD_XFRSZ       0x18                            /* transfer size */
#define PPD_SBNAM       0x1C                            /* send buffer name */
#define PPD_SBOFF       0x20                            /* send buffer offset */
#define PPD_RBNAM       0x24                            /* receive buffer name */
#define PPD_RBOFF       0x28                            /* receive buffer offset */

#define PPD_SYSID       0x14                            /* sending system ID */
#define PPD_PROTO       0x1A                            /* protocol revision */
#define  PPD_BASE       0                               /* 1st rev */
#define  PPD_ELOG       1                               /* 2nd rev, supports ELOG */
#define PPD_MAXDG       0x1C                            /* max datagram size */
#define PPD_MAXMSG      0x1E                            /* max message size */
#define PPD_SWTYPE      0x20                            /* software type */
#define PPD_SWVER       0x24                            /* software version */
#define PPD_SWIN        0x28                            /* software incarnation */
#define PPD_HWTYPE      0x30                            /* hardware type */
#define PPD_HWVER       0x34                            /* hardware version */
#define PPD_NODE        0x40                            /* node name */
#define PPD_TIME        0x48                            /* current time */

/* SCS Offsets */

#define SCS_LENGTH      0x10                            /* message length */
#define SCS_MTYPE       0x14                            /* message type */
#define SCS_CREDIT      0x16                            /* credit extension */
#define SCS_DSTCON      0x18                            /* destination connection ID */
#define SCS_SRCCON      0x1C                            /* source connection ID */
#define SCS_APPL        0x20                            /* application message */
#define SCS_MINCR       0x20                            /* mininum send credit */
#define SCS_STATUS      0x22                            /* status */
#define SCS_DSTPROC     0x24                            /* destination process */
#define SCS_SRCPROC     0x34                            /* source process */
#define SCS_CONDAT      0x44                            /* connection data */

/* SCS Message Types */

#define SCS_CONREQ      0x00                            /* connect request */
#define SCS_CONRSP      0x01                            /* connect response */
#define SCS_ACCREQ      0x02                            /* accept request */
#define SCS_ACCRSP      0x03                            /* accept respone */
#define SCS_REJREQ      0x04                            /* reject request */
#define SCS_REJRSP      0x05                            /* reject respone */
#define SCS_DISREQ      0x06                            /* disconnect request */
#define SCS_DISRSP      0x07                            /* disconnect response */
#define SCS_CRREQ       0x08                            /* credit request */
#define SCS_CRRSP       0x09                            /* credit response */
#define SCS_APPMSG      0x0A                            /* application message */
#define SCS_APPDG       0x0B                            /* application datagram */

/* SCS Status Codes */

#define SCS_STNORM      0x01                            /* success */
#define SCS_STNOMAT     0x0A                            /* no matching listener */
#define SCS_STNORS      0x12                            /* no resources */
#define SCS_STDISC      0x19                            /* disconnected */
#define SCS_STNOCR      0x21                            /* insufficient credit */
#define SCS_BALANCE     0x29                            /* load balance disconnect */
#define SCS_UAP         0x2A                            /* use alternate port */

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
#define OPC_RSTREC      (OPC_M_RECV | OPC_SNDRST)       /* reset received */
#define OPC_STRTREC     (OPC_M_RECV | OPC_SNDSTRT)      /* start received */
#define OPC_REQDATREC   (OPC_M_RECV | OPC_REQDAT)       /* request data received */
#define OPC_MCNFREC     0x29                            /* maintenance confirm received */
#define OPC_IDREC       (OPC_M_RECV | OPC_RETID)        /* ID received */
#define OPC_LBREC       (OPC_M_RECV | OPC_SNDLB)        /* loopback received */
#define OPC_SNDDATREC   (OPC_M_RECV | OPC_SNDDAT)       /* send data received */
#define OPC_DATREC      (OPC_M_RECV | OPC_RETDAT)       /* data recieved */
#define OPC_MDATREC     0x33                            /* maintenance data recieved */

/* Datagram and Message Types */

#define DG_START        0                               /* start */
#define DG_STACK        1                               /* start acknowledge */
#define DG_ACK          2                               /* acknowledge */
#define DG_SCSDG        3                               /* SCS datagram */
#define DG_SCSMSG       4                               /* SCS message */
#define DG_ELOG         5                               /* error log */
#define DG_HOSTSHUT     6                               /* host shutdown */
#define DG_FUDG         7                               /* firmware update datagram */
#define DG_MTYPE        0x7FFF                          /* message type */
#define DG_CCLR         0x8000                          /* cache clear */

/* PPD Status */

#define STS_OK          0x00                            /* OK status */
#define STS_ERR         0x01                            /* error flag */
#define STS_P0ACK       0x00                            /* success or not used */
#define STS_P0NAK       0x02                            /* negative ack */
#define STS_P0RSP       0x04                            /* no response */
#define STS_P0ARB       0x06                            /* arbitration timeout */
#define STS_P1NAK       0x08                            /* negative ack */
#define STS_P1RSP       0x10                            /* no response */
#define STS_P1ARB       0x18                            /* arbitration timeout */
#define STS_VCC         (STS_ERR | 0x20)                /* VC closed */
#define STS_INVBN       (STS_ERR | 0x40)                /* invalid buffer name */
#define STS_BLV         (STS_ERR | 0x60)                /* buffer length violation */
#define STS_ACCV        (STS_ERR | 0x80)                /* access violation */
#define STS_NP          (STS_ERR | 0xA0)                /* no path */
#define STS_BMSE        (STS_ERR | 0xC0)                /* buffer memory system error */
#define STS_OTHER       (STS_ERR | 0xE0)                /* other (see below) */
#define STS_PSV         (STS_OTHER)                     /* pkt size violation */
#define STS_URP         (STS_OTHER | 0x02)              /* unrecognized pkt */
#define STS_INVDP       (STS_OTHER | 0x04)              /* invalid destination port */
#define STS_URC         (STS_OTHER | 0x06)              /* unrecognized command */
#define STS_ABO         (STS_OTHER | 0x08)              /* abort (port disabled) */
#define STS_INVSN       (STS_OTHER | 0x0C)              /* invalid sequence number */
#define STS_UNIMCMD     (STS_OTHER | 0x0E)              /* Unimplemented command (BVP) */
#define STS_INVFS       (STS_OTHER | 0x10)              /* Invalid flags or status (BVP) */

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
#define PPD_LP          0x8                             /* last packet (blk xfer) */
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

/* Block transfer constants */

#define CI_DATHDR       (PPD_RBOFF + 4)
#define CI_MAXDAT       512

typedef struct {
    uint32 type;                                        /* adpater specific use */
    uint32 addr;                                        /* associated memory address */
    size_t size;
    size_t length;                                      /* packet length */
    uint8 data[CI_MAXFR];                               /* packet data */
} CI_PKT;

void ci_set_state (UNIT *uptr, uint32 state);
t_stat ci_send_ppd (UNIT *uptr, CI_PKT *pkt);
t_stat ci_receive_ppd (UNIT *uptr, CI_PKT *pkt);
t_stat ci_open_vc (UNIT *uptr, uint8 port);
t_stat ci_close_vc (UNIT *uptr, uint8 port);
t_bool ci_check_vc (UNIT *uptr, uint8 port);
uint8 ci_get_node (UNIT *uptr);
t_stat ci_show_node (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat ci_set_node (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ci_show_group (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat ci_set_group (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ci_attach (UNIT *uptr, CONST char *cptr);
t_stat ci_detach (UNIT *uptr);
t_stat ci_port_svc (UNIT *uptr);
t_stat ci_port_reset (DEVICE *dptr);

#endif
