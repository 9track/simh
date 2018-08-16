/* vax_ci_vcd.c: Computer Interconnect adapter

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

   Port Initialisation:
      In order to get the operating system to use the CI port, the console
      storage device must contain the port microcode.

   Virtual Circuits:
      After the port has been initialised ID requested will be sent
      every few seconds on both paths to all ports that do not have
      a virtual circuit currently established. Note that ID requests
      are also sent to the local port.
      These packets are sent via UDP multicast in this implementation.
      When a port receives an ID request (OP_REQID) it responds with
      an OP_IDREC packet which contains information about the local
      port. Once ID information is received by a system a virtual
      circuit is opened between the two nodes. (Note that a virtual
      circuit is also established with the local port).
      The virtual circuit is implemented using a TCP connection.

   Block Transfers:
      These packets allow the host to transfer large amounts of
      data from one system to another without host intervention.
      Block transfers consist of one of the following sequences:

      [ Node 1 ] Send Data (OP_SNDDAT)
      [ Node 2 ] Data Received (OP_SNDDAT)
      [ Node 2 ] Send Confirmation (OP_CNFREC)
      [ Node 1 ] Confirmation Received (OP_CNFREC)

      [ Node 1 ] Request Data (OP_REQDAT)
      [ Node 2 ] Data Requested (OP_REQDAT)
      [ Node 2 ] Send Data (OP_DATREC)
      [ Node 1 ] Data Received (OP_DATREC)

*/

#include "vax_defs.h"
#include "vax_ci.h"
#include "ci_sock.h"
#include <time.h>

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
#define OPC_SNDID       0x0B                            /* send ID */
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
#define OPC_IDREC       (OPC_M_RECV | OPC_SNDID)        /* ID received */
#define OPC_LBREC       (OPC_M_RECV | OPC_SNDLB)        /* loopback received */
#define OPC_SNDDATREC   (OPC_M_RECV | OPC_SNDDAT)       /* send data received */
#define OPC_DATREC      (OPC_M_RECV | OPC_RETDAT)       /* data recieved */
#define OPC_MDATREC     0x33                            /* maintenance data recieved */

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
#define PPD_DGHDR       (PPD_HDR + 0x4)                 /* datagram header size */
#define PPD_MSGHDR      (PPD_HDR + 0x4)                 /* message header size */

/* Virtual Circuit States */

#define VCST_CLOSED     0                               /* VC closed */
#define VCST_WAIT       1                               /* VC opening */
#define VCST_OPEN       2                               /* VC open */

#define CI_NET_BUF_SIZE 1024

const char* ppd_types[] = {                             /* PPD debug definitions */
    "START",
    "STACK",
    "ACK",
    "SCSDG",
    "SCSMSG"
};

const char* scs_msg_types[] = {                         /* SCS message debug definitions */
    "Connect Request",
    "Connect Response",
    "Accept Request",
    "Accept Response",
    "Reject Request",
    "Reject Response",
    "Disconnect Request",
    "Disconnect Response",
    "Credit Request",
    "Credit Response",
    "Application Sequenced Message"
};

const char* ci_path_names[] = {
    "AUTO",
    "A",
    "B"
};

/* Virtual Circuit Descriptor */

typedef struct {
    uint32    socket;                                   /* TCP Socket of this VC */
    int32     ipa;                                      /* IP of remote system */
    int32     ipp;                                      /* Port of remote system */
    int32     pri;                                      /* Priority */
    uint32    vcd_val;                                  /* VCD value */
    int32     vc_state;                                 /* VC state */
} VCD;

/* Block Transfer Descriptor */

typedef struct {
    t_bool    incomplete;                               /* Transfer is incomplete (true/false) */
    uint8     opcode;                                   /* The transfer type */
    uint32    total_data_len;                           /* Bytes remaining of transfer */
    uint32    data_pte;                                 /* PTE of first page of buffer */
    uint32    data_offset;                              /* Offset into buffer */
    uint32    start_offset;                             /* Current buffer pointer */
    uint32    page_offset;                              /* Offset into buffer */
    uint8     conf_buf[0x2c];                           /* Optional buffer for confirm */
} BLKTF;

typedef struct {
  int32       src;                                      /* receive (0=setup, 1=loopback, 2=normal) */
  uint8       packet[CI_NET_BUF_SIZE];
} CI_ITEM;

typedef struct {
  int32       max;
  int32       count;
  int32       head;
  int32       tail;
  int32       loss;
  int32       high;
  CI_PKT     *item;
} CI_QUE;

#define CI_QUE_MAX           100                        /* message queue array */

int32 ci_multi_ip;                                      /* multicast ip address */
int32 ci_multi_port;                                    /* multicast port */
int32 ci_tcp_port;                                      /* local tcp port */
int32 ci_pri;                                           /* connection priority */
SOCKET ci_multi_sock = 0;                               /* multicast socket */
SOCKET ci_tcp_sock = 0;                                 /* VC socket */
SOCKET ci_wait_sock[CI_MAX_NODES];                      /* pending connections */

VCD ci_vcd[CI_MAX_NODES];                               /* VC descriptors */
BLKTF blktf_table[CI_MAX_NODES];                        /* block transfer descriptors */
CI_QUE ci_rx_queue;                                     /* packet receive queue */
CI_QUE ci_tx_queue;                                     /* packet transmit queue */

uint8 buffer[CI_NET_BUF_SIZE];
uint8 net_blk_buf[CI_NET_BUF_SIZE];

extern FILE *sim_log;
extern FILE *sim_deb;
extern int32 sim_switches;
extern int32 tmxr_poll;
extern DEVICE ci_dev;
extern UNIT ci_unit;

t_stat ci_send_packet (uint8 *packet, uint16 size, int32 opcode, int32 port, int32 path);
t_stat ci_send_data (int32 opcode, uint8 *buffer, int32 port, int32 path);
t_stat ci_receive_data (int32 opcode, int32 src, uint8 *buffer);
t_stat ci_open_vc (int32 port);
t_stat ci_close_vc (int32 port);
void ci_vc_fail (uint8 port);
t_stat ci_process_net_buffer (CI_ITEM *item);
void put_rsp_queue (int32 addr);
void put_dgf_queue (int32 addr);
void put_msf_queue (int32 addr);
int32 get_dgf_queue (void);
int32 get_msf_queue (void);
t_stat ciq_init (CI_QUE *que, int32 max);               /* initialize FIFO queue */
void ciq_clear  (CI_QUE *que);                          /* clear FIFO queue */
void ciq_remove (CI_QUE *que);                          /* remove item from FIFO queue */
void ciq_insert (CI_QUE *que, int32 src, uint8 *packet); /* insert item into FIFO queue */

extern void ci_read_packet (int32 addr, int32 size, uint8 *buffer);
extern void ci_write_packet (int32 addr, int32 size, uint8 *buffer);
extern void ci_set_int (void);
extern void ci_clr_int (void);

//
// TODO: Need generic read/write block functions in GVP code.
//       i.e. ReadData, WriteData, ReadDataPTE and WriteDataPTE
//

/* CI port state change */

void ci_set_state (uint32 state)
{
int32 r;
uint32 src_ipa, src_ipp;

if (state < PORT_INIT) {                                /* change to uninit */
    if (ci_state >= PORT_INIT) {                        /* currently initialised? */
        sim_cancel (&ci_unit);                          /* stop poll */
        sim_debug (DBG_CONN, &ci_dev, "deactivating unit\n");
        ciq_clear (&ci_rx_queue);                       /* clear queue */
        do r = ci_read_sock_udp (ci_multi_sock, buffer, CI_NET_BUF_SIZE, &src_ipa, &src_ipp);
        while (r > 0);                                  /* clear network buffer */
        }
    }
else if (state >= PORT_INIT) {                          /* change to init */
    if (ci_state < PORT_INIT) {                         /* currently uninit? */
        sim_clock_coschedule (&ci_unit, tmxr_poll);     /* start poll */
        sim_debug (DBG_CONN, &ci_dev, "activating unit\n");
        }
   }
ci_state = state;                                       /* update state */
}

t_stat ci_ppd (CI_PKT *pkt)
{
int32 status;
t_stat r;
uint32 opcode, port_num, cmd_flags;
uint32 ci_path;

port_num = pkt->data[PPD_PORT];
opcode = pkt->data[PPD_OPC];
cmd_flags = pkt->data[PPD_FLAGS];
status = pkt->data[PPD_STATUS];

ci_path = (cmd_flags >> 1) & 0x3;

switch (opcode) {

    case OPC_SNDDG:
        r = ci_snddg (pkt);
        break;

    case OPC_SNDMSG:
        r = ci_sndmsg (pkt);
        break;

    case OPC_REQID:
        r = ci_reqid (pkt);
        break;

    case OPC_REQDAT:
        r = ci_reqdat (pkt);
        break;

    case OPC_SNDLB:
        r = ci_sndlb (pkt);
        break;

    case OPC_SNDDAT:
        r = ci_snddat (pkt);
        break;

    case OPC_INVTC:
        r = ci_invtc (pkt);
        break;

    case OPC_SETCKT:
        r = ci_setckt (pkt);
        break;

    case OPC_RDCNT:
        r = ci_rdcnt (pkt);
        break;

    case OPC_DGREC:
        r = ci_dgrec (pkt);
        break;

    case OPC_MSGREC:
        r = ci_msgrec (pkt);
        break;

    case OPC_REQREC:
        r = ci_reqrec (pkt);
        break;

    case OPC_REQDATREC:
        r = ci_reqdatrec (pkt);
        break;

    case OPC_IDREC:
        r = ci_idrec (pkt);
        break;

    case OPC_DATREC:
        r = ci_datrec (pkt);
        break;

    case OPC_SNDDATREC:
        r = ci_snddatrec (pkt);
        break;

    case OPC_CNFREC:
        r = ci_cnfrec (pkt);
        break;

    default:
        sim_debug (DBG_WRN, &ci_dev, "Unimplemented Opcode: %d\n", opcode);
        r = SCPE_IOERR;
        break;
        }
#if 0
// Need to put packet to response or datagram free queue    
if (cmd_flags & 0x1) {
    sim_debug (DBG_LCMD, &ci_dev, "response requested\n");
    }
else {
    sim_debug (DBG_LCMD, &ci_dev, "no response requested\n");
    }
#endif
return r;
}

/*               < SNDDG >                 *
 *                                         *
 *                                         *
 *  ........ 0x0    A = Port Num           *
 *  ........ 0x4    B = Staus              *
 *  ........ 0x8    C = Opcode             *
 *  DDCCBBAA 0xC    D = Flags              *
 *  FFFFEEEE 0x10   E = DG Length          *
 *  xxxxxxxx 0x14   F = DG Type            *
 *  xxxxxxxx 0x18   x = Message            *
 *                                         *
 *  Length = E + 0x14                      *
 *                                         */
t_stat ci_snddg (CI_PKT *pkt)
{
uint16 dg_type = CI_GET16 (pkt, PPD_MTYPE);
uint16 msg_size = CI_GET16 (pkt, PPD_LENGTH) + PPD_DGHDR;

switch (dg_type) {

    case 0:
        ci_ppd_debug ("==> SNDDG - START");
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - START, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
        break;

    case 1:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - STACK dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
        break;

    case 2:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - ACK, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
        break;

    case 3:
        sim_debug (DBG_SCSDG, &ci_dev, "==> SNDDG - SCSDG, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
        break;

    case 4:
        sim_debug (DBG_WRN, &ci_dev, "==> SNDDG - SCSMSG, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
        break;
        }

r = ci_send_packet (pkt, msg_size);
if (r != SCPE_OK)
    return r;
if (cmd_flags & 0x1)
    ci_put_rsp (pkt);                                   /* driver wants it back */
else
    ci_put_dfq (pkt);                                   /* dispose of packet */
}

/*               < SNDMSG >                *
 *                                         *
 *                                         *
 *  ........ 0x0    A = Port Num           *
 *  ........ 0x4    B = Staus              *
 *  ........ 0x8    C = Opcode             *
 *  DDCCBBAA 0xc    D = Flags              *
 *  FFFFEEEE 0x10   E = MSG Length         *
 *  ....GGGG 0x14   F = MSG Type           *
 *  xxxxxxxx 0x18   G = SCS Message Type   *
 *  xxxxxxxx 0x1c   x = Message            *
 *                                         *
 *  Length = E + 0x14                      *
 *                                         */
t_stat ci_sndmsg (CI_PKT *pkt)
{
uint16 dg_type = CI_GET16 (pkt, PPD_MTYPE);
uint16 msg_size = CI_GET16 (pkt, PPD_LENGTH) + PPD_MSGHDR;
uint16 msg_type;

switch (dg_type) {

    case 0:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - START, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
        //
        // The following code may be related to the VC failure mechanism.
        // Maybe put this in ci_send_packet when VC is not open?
        //
        pkt->data[PPD_STATUS] = 0xA1;                   /* Status: No Path */
        ci_put_rsp (pkt);
        return SCPE_OK;
        break;

    case 1:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - STACK dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
        break;

    case 2:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - ACK, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
        break;

    case 3:
        sim_debug (DBG_WRN, &ci_dev, "==> SNDMSG - SCSDG, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
        break;

    case 4:
        msg_type = CI_GET16 (pkt->data, PPD_STYPE);
        sim_debug (DBG_SCSMSG, &ci_dev, "==> SCSMSG - %s\n", scs_msg_types[msg_type]);
        break;
        }

r = ci_send_packet (pkt, msg_size);
if (r != SCPE_OK)
    return r;
if (cmd_flags & 0x1)
    ci_put_rsp (pkt);                                   /* driver wants it back */
else
    ci_put_dfq (pkt);                                   /* dispose of packet */
}

/*             < REQID >               *
 *                                     *
 *                                     *
 *  ........ 0x0    A = Port Num       *
 *  ........ 0x4    B = Staus          *
 *  ........ 0x8    C = Opcode         *
 *  DDCCBBAA 0xc    D = Flags          *
 *                                     *
 *  Length = 0x10                      *
 *                                     */
t_stat ci_reqid (CI_PKT *pkt)
{
sim_debug (DBG_REQID, &ci_dev, "==> REQID, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
pkt->data[PPD_STATUS] = 0;                              /* Status OK */
r = ci_send_packet (pkt, PPD_HDR);
if (r != SCPE_OK)
    return r;
if (cmd_flags & 0x1)                                    /* response requested? */
    ci_put_rsp (pkt);                                   /* driver wants it back */
else
    ci_put_dfq (pkt);                                   /* dispose of packet */
}

/*               < REQDAT >                           *
 *                                                    *
 *                                                    *
 *  ........ 0x0    A = Port Num                      *
 *  ........ 0x4    B = Staus                         *
 *  ........ 0x8    C = Opcode                        *
 *  DDCCBBAA 0xc    D = Flags                         *
 *  EEEEEEEE 0x10   E = Transaction ID                *
 *  EEEEEEEE 0x14   F = Transfer Size                 *
 *  FFFFFFFF 0x18   G = Sending Buffer Name           *
 *  ....GGGG 0x1c   H = Sending Buffer Byte Offset    *
 *  HHHHHHHH 0x20   I = Receiving Buffer Name         *
 *  ....IIII 0x24   J = Receiving Buffer Byte Offset  *
 *  JJJJJJJJ 0x28                                     *
 *                                                    *
 *  Length = 0x2c                                     *
 *                                                    */
t_stat ci_reqdat (CI_PKT *pkt)
{
sim_debug (DBG_BLKTF, &ci_dev, "==> REQDAT, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);
/* msg_size = req_dat + gvp_headers + queue_pointers */
msg_size = 0x2c;
pkt->data[PPD_STATUS] = 0;                              /* Status OK */
r = ci_send_packet (pkt, msg_size);
if (r != SCPE_OK)
    return r;
if (cmd_flags & 0x1)                                    /* response requested? */
    ci_put_rsp (pkt);                                   /* driver wants it back */
else
    ci_put_dfq (pkt);                                   /* dispose of packet */
}

/*             < SNDLB >               *
 *                                     *
 *                                     *
 *  ........ 0x0    A = Port Num       *
 *  ........ 0x4    B = Staus          *
 *  ........ 0x8    C = Opcode         *
 *  DDCCBBAA 0xc    D = Flags          *
 *  FFFFEEEE 0x10   E = LB Size        *
 *  FFFFFFFF 0x14   F = LB Data        *
 *    ....          G = CRC            *
 *  GGGGFFFF 0x??                      *
 *      GGGG 0x??                      *
 *                                     *
 *  Length = 0x??                      *
 *                                     */
t_stat ci_sndlb (CI_PKT *pkt)
{
sim_debug (DBG_LCMD, &ci_dev, "==> SNDLB, path: %X\n", ci_path);
// TODO: should this go via normal ci_send_packet?
if (uptr->flags & UNIT_ATT) {                           /* CI cables connected? */
    ci_vcd[(ci_ppr & PPR_NODE)].vcd_val |= 0x300;
    ci_vcd[(ci_ppr & PPR_NODE)].mod_vcd_val |= 0x300;
    pkt->data[PPD_STATUS] = 0;                          /* status OK */
    pkt->data[PPD_OPC] = OPC_LBREC;                     /* loopback received */
    ci_put_rsp (pkt);
//    sim_debug (DBG_LCMD, &ci_dev, "<== LBREC, path: %d\n", ci_path);
    }
else {                                                  /* no, packet lost */
    ci_vcd[(ci_ppr & PPR_NODE)].vcd_val &= ~0x300;
    ci_vcd[(ci_ppr & PPR_NODE)].mod_vcd_val &= ~0x300;
    ci_put_dfq (ent_addr);                              /* dispose of packet */
    }
}

/*
   Block Transfers:
      These packets allow the large amounts of data to be
      transferred from one system to another without host intervention.
      Block transfers consist of one of the following sequences:

      [ Node 1 ] Send Data (SNDDAT)
      [ Node 2 ] Data Received (SNDDATREC)
      [ Node 2 ] Send Confirmation (RETCNF)
      [ Node 1 ] Confirmation Received (CNFREC)

      [ Node 1 ] Request Data (REQDAT)
      [ Node 2 ] Data Requested (REQDATREC)
      [ Node 2 ] Send Data (RETDAT)
      [ Node 1 ] Data Received (DATREC)

   Packet Formats:
      SNDDAT,SNDDATREC,REQDAT,REQDATREC
 
      ........ 0x0    A = Port Num
      ........ 0x4    B = Staus
      ........ 0x8    C = Opcode
      DDCCBBAA 0xc    D = Flags
      EEEEEEEE 0x10   E = Transaction ID
      EEEEEEEE 0x14   F = Transfer Size
      FFFFFFFF 0x18   G = Sending Buffer Name
      ....GGGG 0x1c   H = Sending Buffer Byte Offset
      HHHHHHHH 0x20   I = Receiving Buffer Name
      ....IIII 0x24   J = Receiving Buffer Byte Offset
      JJJJJJJJ 0x28
 
      Length = 0x2c
 */
t_stat ci_snddat (CI_PKT *pkt)
{
sim_debug (DBG_BLKTF, &ci_dev, "==> SNDDAT, dest: %d, path: %s\n", port_num, ci_path_names[ci_path]);

ci_send_data (OPC_SNDDAT, &buffer[0], port_num, ci_path);

if (cmd_flags & 0x1) {
    sim_debug (DBG_WRN, &ci_dev, "  SNDDAT response requested!\n");
    pkt->data[PPD_STATUS] = 0;                          /* status OK */
    ci_put_rsp (pkt);
    }
else {
    ci_put_mfq (pkt);
    }
}

t_stat ci_invtc (CI_PKT *pkt)
{
sim_debug (DBG_LCMD, &ci_dev, "==> INVTC\n");
if (cmd_flags & 0x1) {                                  /* respond to command? */
    pkt->data[PPD_STATUS] = 0;                          /* status OK */
    ci_put_rsp (pkt);
    sim_debug (DBG_LCMD, &ci_dev, "<== INVTC OK\n");
    }
else
    ci_put_dfq (ent_addr);                              /* dispose of packet */
break;
}

/*             < SETCKT >                *
 *                                       *
 *                                       *
 *  ........ 0x0    A = Port Num         *
 *  ........ 0x4    B = Staus            *
 *  ........ 0x8    C = Opcode           *
 *  DDCCBBAA 0xc    D = Flags            *
 *  ....EEEE 0x10   E = VCB Modify Mask  *
 *  ....FFFF 0x14   F = VCB Modify Value *
 *  GGGGGGGG 0x18   G = VCB val before   *
 *                                       *
 *  Length = 0x14                        *
 *                                       */
t_stat ci_setckt (CI_PKT *pkt)
{
uint32 vcd_mmsk = CI_GET32 (pkt->data, PPD_VCMMSK);
uint32 vcd_mval = CI_GET32 (pkt->data, PPD_VCMVAL);
uint32 vcd_val = ci_vcd[port_num].vcd_val;              /* current VCD value */
uint32 vcd_nval = ci_vcd[port_num].vcd_val;             /* new VCD value */

sim_debug (DBG_LCMD, &ci_dev, "==> SETCKT, port: %d, mask: %X value: %X\n", port_num, vcd_mmsk, vcd_mval);

vcd_nval &= ~vcd_mmsk;
vcd_nval |= vcd_mval;

if (VC_OPEN (vcd_nval) && !VC_OPEN (vcd_val)) {         /* Opening VC */
    r = ci_open_vc (port_num);
    if (r != SCPE_OK) {
        sim_debug (DBG_CONN, &ci_dev, "Unable to establish VC to node %d\n", port_num);
        put_dgf_queue (ent_addr);
        break;
        }
    sim_debug (DBG_LCMD, &ci_dev, "<== SETCKT, VC Open Complete\n");
    }
else if (!VC_OPEN (vcd_nval) && VC_OPEN (vcd_val)) {    /* Closing VC */
    r = ci_close_vc (port_num);
    sim_debug (DBG_LCMD, &ci_dev, "<== SETCKT, VC Close Complete\n");
    }

CI_PUT32 (pkt->data, PPD_VCPVAL, vcd_val);
pkt->data[PPD_STATUS] = 0;                              /* status OK */
ci_vcd[port_num].vcd_val = vcd_nval;
ci_put_rsp (pkt)
}

t_stat ci_rdcnt (CI_PKT *pkt)
{
sim_debug (DBG_LCMD, &ci_dev, "==> RDCNT\n");
CI_PUT32 (pkt->data, 0x10, 0);                          /* ACKs on path 0 */
CI_PUT32 (pkt->data, 0x14, 0);                          /* NAKs on path 0 */
CI_PUT32 (pkt->data, 0x18, 0);                          /* NORSPs on path 0 */
CI_PUT32 (pkt->data, 0x1c, 0);                          /* ACKs on path 1 */
CI_PUT32 (pkt->data, 0x20, 0);                          /* NAKs on path 1 */
CI_PUT32 (pkt->data, 0x24, 0);                          /* NORSPs on path 1 */
CI_PUT32 (pkt->data, 0x28, 0);                          /* DGS discarded */

if (cmd_flags & 0x1) {                                  /* Respond to command? */
    pkt->data[PPD_STATUS] = 0;                          /* status OK */
    ci_put_rsp (pkt)
    sim_debug (DBG_LCMD, &ci_dev, "<== CNTRD\n");
    }
else
    put_dgf_queue (ent_addr);  			    /* dispose of packet */
}

/*               < DGREC >                 *
 *                                         *
 *                                         *
 *  ........ 0x0    A = Port Num           *
 *  ........ 0x4    B = Staus              *
 *  ........ 0x8    C = Opcode             *
 *  DDCCBBAA 0xc    D = Flags              *
 *  FFFFEEEE 0x10   E = DG Length          *
 *  xxxxxxxx 0x14   F = DG Type            *
 *  xxxxxxxx 0x18   x = Message            *
 *                                         *
 *  Length = E + 0x14                      *
 *                                         */
t_stat ci_dgrec (CI_PKT *pkt)
{
uint32 msg_size = GET_INT16 (pkt->data, PPD_LENGTH) + PPD_DGHDR;

dg_type = pkt->data[PPD_MTYPE];
ci_get_dfq (pkt);                                       /* get a free datagram */

ci_write_packet (pkt, msg_size);

switch (dg_type) {

    case 0:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - START, src: %d, path: %s\n", src, ci_path_names[ci_path]);
        break;

    case 1:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - STACK, src: %d, path: %s\n", src, ci_path_names[ci_path]);
        break;

    case 2:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - ACK, src: %d, path: %s\n", src, ci_path_names[ci_path]);
        break;

    case 3:
        sim_debug (DBG_SCSDG, &ci_dev, "<== DGREC - SCSDG, src: %d, path: %s\n", src, ci_path_names[ci_path]);
        break;

    case 4:
        sim_debug (DBG_WRN, &ci_dev, "<== DGREC - SCSMSG, src: %d, path: %s\n", src, ci_path_names[ci_path]);
        break;
}

ci_put_rsp (pkt);                                       /* pass it to system */
}

/*               < MSGREC >                *
 *                                         *
 *                                         *
 *  ........ 0x0    A = Port Num           *
 *  ........ 0x4    B = Staus              *
 *  ........ 0x8    C = Opcode             *
 *  DDCCBBAA 0xc    D = Flags              *
 *  FFFFEEEE 0x10   E = MSG Length         *
 *  ....GGGG 0x14   F = MSG Type           *
 *  xxxxxxxx 0x18   G = SCS Message Type   *
 *  xxxxxxxx 0x1c   x = Message            *
 *                                         *
 *  Length = E + 0x14                      *
 *                                         */
t_stat ci_msgrec (CI_PKT *pkt)
{
uint32 msg_size = GET_INT16 (pkt->data, PPD_LENGTH) + PPD_MSGHDR;

dg_type = pkt->data[PPD_MTYPE];
ci_get_mfq (pkt);                                       /* get a free message */

ci_write_packet (pkt, msg_size);

switch (dg_type) {

    case 0:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - START, src: %d, path: %s\n", src, ci_path_names[ci_path]);
        break;

    case 1:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - STACK, src: %d, path: %s\n", src, ci_path_names[ci_path]);
        break;

    case 2:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - ACK, src: %d, path: %s\n", src, ci_path_names[ci_path]);
        break;

    case 3:
        sim_debug (DBG_WRN, &ci_dev, "<== MSGREC - SCSDG, src: %d, path: %s\n", src, ci_path_names[ci_path]);
        break;

    case 4:
        msg_type = pkt->data[PPD_STYPE];
        sim_debug (DBG_SCSMSG, &ci_dev, "<== SCSMSG - %s\n", scs_msg_types[msg_type]);
        break;
}

ci_put_rsp (pkt);                                       /* pass it to system */
}

/*             < REQREC >              *
 *                                     *
 *                                     *
 *  ........ 0x0    A = Port Num       *
 *  ........ 0x4    B = Staus          *
 *  ........ 0x8    C = Opcode         *
 *  DDCCBBAA 0xc    D = Flags          *
 *                                     *
 *  Length = 0x10                      *
 *                                     */
t_stat ci_reqrec (CI_PKT *pkt)
{
if (ci_vcd[src].vc_state != VCST_CLOSED)                /* don't send ID once VC is open */
    return SCPE_OK;
sim_debug (DBG_REQID, &ci_dev, "<== REQID, src: %d, path: %s\n", src, ci_path_names[ci_path]);

// TODO: these should be moved to ci_receive_packet
port_num = CI_GET16 (pkt->data, 0x10);
ci_vcd[src].pri = CI_GET16 (pkt->data, 0x14);
//

pkt->data[PPD_PORT] = src;                              /* add remote port number */
pkt->data[PPD_STATUS] = 0;                              /* status: OK */
pkt->data[PPD_OPC] = OPC_RETID;                         /* set opcode */
pkt->data[PPD_FLAGS] = (pkt->data[PPD_FLAGS] & ~0x1) | (ci_path << 4);  /* Clear RSP flag */
CI_PUT32 (pkt->data, 0x10, 0);                          /* transaction ID low (MBZ to trigger START) */
CI_PUT32 (pkt->data, 0x14, 0);                          /* transaction ID high (MBZ to trigger START) */
CI_PUT32 (pkt->data, 0x18, 0x80000002);                 /* remote port type */
CI_PUT32 (pkt->data, 0x1c, 0x00700040);                 /* code revision */
CI_PUT32 (pkt->data, 0x20, 0xFFFF0F00);                 /* function mask */
pkt->data[0x24] = 0x0;                                  /* resetting port */
// TODO: should also respond when disabled?
pkt->data[0x25] = 0x4;                                  /* port state (maint = 0, state = enabled) */
msg_size = 0x26;

//ci_vcd[src].ipa = src_ip;                               /* Save IP address and port number for */
ci_vcd[src].ipp = port_num;                             /* opening of VC */

r = ci_send_packet(pkt, msg_size);
sim_debug (DBG_REQID, &ci_dev, "==> RETID, dest: %d, path: %s\n", src, ci_path_names[ci_path]);
}

/*               < REQDATREC >                        *
 *                                                    *
 *                                                    *
 *  ........ 0x0    A = Port Num                      *
 *  ........ 0x4    B = Staus                         *
 *  ........ 0x8    C = Opcode                        *
 *  DDCCBBAA 0xc    D = Flags                         *
 *  EEEEEEEE 0x10   E = Transaction ID                *
 *  EEEEEEEE 0x14   F = Transfer Size                 *
 *  FFFFFFFF 0x18   G = Sending Buffer Name           *
 *  ....GGGG 0x1c   H = Sending Buffer Byte Offset    *
 *  HHHHHHHH 0x20   I = Receiving Buffer Name         *
 *  ....IIII 0x24   J = Receiving Buffer Byte Offset  *
 *  JJJJJJJJ 0x28                                     *
 *                                                    *
 *  Length = 0x2c                                     *
 *                                                    */
t_stat ci_reqdatrec (CI_PKT *pkt)
{
sim_debug (DBG_BLKTF, &ci_dev, "<== REQDATREC, src: %d, path: %s\n", src, ci_path_names[ci_path]);
ci_send_data (OPC_RETDAT, item->packet, src, ci_path);
sim_debug (DBG_BLKTF, &ci_dev, "==> RETDAT, dest: %d\n", src);
}

/*               < IDREC >                 *
 *                                         *
 *                                         *
 *  ........ 0x0    A = Port Num           *
 *  ........ 0x4    B = Staus              *
 *  ........ 0x8    C = Opcode             *
 *  DDCCBBAA 0xc    D = Flags              *
 *  EEEEEEEE 0x10   E = Transaction ID     *
 *  EEEEEEEE 0x14   F = Remote Port Type   *
 *  FFFFFFFF 0x18   G = Code Revision      *
 *  GGGGGGGG 0x1c   H = Function Mask      *
 *  HHHHHHHH 0x20   I = Resetting Port     *
 *  ....JJII 0x24   J = Port State         *
 *                                         *
 *  Length = 0x26                          *
 *                                         */
t_stat ci_idrec (CI_PKT *pkt)
{
sim_debug (DBG_REQID, &ci_dev, "<== IDREC, src: %d, path: %s\n", src, ci_path_names[ci_path]);
/* copy_end = idrec_size + gvp_header + copy_start */
msg_size = 0x26;
ci_get_dfq (pkt);
ci_write_packet (pkt, msg_size);
ci_put_rsp (pkt)
}

t_stat ci_datrec (CI_PKT *pkt)
{
t_stat r;
r = ci_receive_data (opcode, src, item->packet);
if (r != SCPE_OK)
    return r;
ci_get_mfq (pkt);
msg_size = 0x2c;
// TODO: how to handle confirmation buffer?
// create packet from scratch?
ci_write_packet (pkt, msg_size, blktf_table[src].conf_buf);
sim_debug (DBG_BLKTF, &ci_dev, "putting packet on queue\n", packet_len);
return ci_put_rsp (pkt)                        /* Pass it to system */
}

t_stat ci_snddatrec (CI_PKT *pkt)
{
t_stat r;
r = ci_receive_data (opcode, src, item->packet);
if (r != SCPE_OK)
    return r;
r = ci_send_packet (blktf_table[src].conf_buf, 0x2c, OPC_RETCNF, src, ci_path);
sim_debug (DBG_BLKTF, &ci_dev, "==> RETCNF, dest: %d, path: %d\n", src, ci_path);
return r;
}

t_stat ci_cnfrec (CI_PKT *pkt)
{
sim_debug (DBG_BLKTF, &ci_dev, "<== CNFREC, src: %d, path: %s\n", src, ci_path_names[ci_path]);

ci_get_mfq (pkt);

/* copy_end = datrec_size + gvp_header + copy_start */
msg_size = 0x2c;
ci_write_packet (pkt, msg_size);
ci_put_rsp (pkt)                       /* Pass it to system */
}

t_stat ci_open_vc (uint8 port)
{
SOCKET newsock;

if ((ci_vcd[port].socket > 0) || (ci_vcd[port].vc_state == VCST_OPEN))
    return SCPE_OK;
if (port == (ci_ppr & PPR_NODE)) {
    ci_vcd[port].vc_state = VCST_OPEN;
    return SCPE_OK;
}
if (ci_pri < ci_vcd[port].pri)
    return SCPE_OK;

sim_debug (DBG_CONN, &ci_dev, "Connecting to node %d at %08X on port %d...\n", port, ci_vcd[port].ipa, ci_vcd[port].ipp);
newsock = ci_connect_sock (ci_vcd[port].ipa, ci_vcd[port].ipp);
if (newsock == INVALID_SOCKET) {
    sim_debug (DBG_CONN, &ci_dev, "Unable to establish VC to node %d\n", port);
    return SCPE_OPENERR;
}
sim_debug (DBG_CONN, &ci_dev, "Connected to node %d, socket %d\n", port, newsock);
ci_vcd[port].socket = newsock;
ci_vcd[port].vc_state = VCST_WAIT;
return SCPE_OK;
}

t_stat ci_close_vc (uint8 port)
{
if (port == (ci_ppr & PPR_NODE)) {
    ci_vcd[port].vc_state = VCST_CLOSED;
    return SCPE_OK;
}
if ((ci_vcd[port].socket == 0) || (ci_vcd[port].vc_state == VCST_CLOSED))
    return SCPE_OK;
sim_debug (DBG_CONN, &ci_dev, "Closing VC with node %d...\n", port);
ci_close_sock (ci_vcd[port].socket, 0);
ci_vcd[port].socket = 0;
ci_vcd[port].vc_state = VCST_CLOSED;
return SCPE_OK;
}

/* Notifies the operating system that the connection with the *
 * remote node has been lost by inserting a START datagram on *
 * the response queue in order to trigger VC failure          */

void ci_vc_fail (uint8 port)
{
CI_PKT pkt;
ci_get_dfq (&pkt);                                      /* get a free datagram */
pkt.data[PPD_PORT] = port;                              /* port number */
pkt.data[PPD_STATUS] = 0;                               /* status: OK */
pkt.data[PPD_OPC] = OPC_DGREC;                          /* opcode */
pkt.data[PPD_FLAGS] = 0;                                /* flags */
pkt.data[PPD_LENGTH] = 0;                               /* message length */
pkt.data[PPD_MTYPE] = 0;                                /* message type: START */
ci_put_rsp (&pkt);                                      /* put to response queue */
}

t_stat ci_send_packet (uint8 *packet, uint16 size, int32 opcode, int32 port, int32 path)
{
int32 r;

// Packet header needs to contain:
    // Size
    // Priority (ci_pri)
    // Local TCP port (ci_tcp_port)
    // Local port number
    // CI path
    // Data type (PPD or block transfer)?

// Opcode needs to be or'd with 0x20 to convert
// send opcode to receive opcode

packet[0x0] = size & 0xff;
packet[0x1] = ((size & 0xff00) >> 8);
packet[0x2] = (ci_ppr & PPR_NODE);                      /* local port number */
packet[0x3] = opcode & 0xFF;                            /* packet type */
packet[0x4] = path & 0x3;                               /* CI path */

if (port == (ci_ppr & PPR_NODE)) {
    sim_debug (DBG_CONN, &ci_dev, "packet destination is local node\n");
    ciq_insert (&ci_rx_queue, port, packet);
    return SCPE_OK;
}

if ((ci_vcd[port].vc_state == VCST_OPEN) && (opcode != OPC_REQID) && (opcode != OPC_IDREC)) {
    if (ci_tx_queue.count == 0) {
        r = ci_write_sock_tcp (ci_vcd[port].socket, packet, CI_NET_BUF_SIZE); /* send packet */
        if (r < 0)
            ciq_insert (&ci_tx_queue, port, packet);
    }
    else ciq_insert (&ci_tx_queue, port, packet);
}
else r = ci_write_sock_udp (ci_multi_sock, packet, size, ci_multi_ip, ci_multi_port); /* send packet */

return SCPE_OK;
}

t_stat ciq_init (CI_QUE* que, int32 max)
{

if (!que->item) {                                       /* create dynamic queue if it does not exist */
    size_t size = sizeof(CI_ITEM) * max;
    que->max = max;
    que->item = (CI_ITEM *)malloc (size);
    if (que->item)
        memset (que->item, 0, size);                    /* init dynamic memory */
    else {                                              /* failed to allocate memory */
        char* msg = "CIQ: failed to allocate dynamic queue[%d]\r\n";
        printf (msg, max);
        if (sim_log)
            fprintf (sim_log, msg, max);
        return SCPE_MEM;
        };
    };
return SCPE_OK;
}

void ciq_clear (CI_QUE* que)
{
memset (que->item, 0, sizeof(CI_ITEM) * que->max); /* clear packet array */
que->count = que->head = que->tail = que->loss = que->high = 0; /* clear rest of structure */
}

void ciq_remove (CI_QUE* que)
{
CI_ITEM* item = &que->item[que->head];

if (que->count) {
    memset (item, 0, sizeof(CI_ITEM));
    if (++que->head == que->max)
        que->head = 0;
    que->count--;
    }
}

void ciq_insert (CI_QUE* que, int32 src, uint8 *pack)
{
CI_ITEM* item;

if (!que->count) {                                      /* if queue empty, set pointers to beginning */
    que->head = 0;
    que->tail = -1;
    }
if (++que->tail == que->max)
    que->tail = 0;                                      /* find new tail of the circular queue */
if (++que->count > que->max) {
    que->count = que->max;                              /* lose oldest packet */
    if (++que->head == que->max)
        que->head = 0;
    que->loss++;
    }
if (que->count > que->high)
    que->high = que->count;

item = &que->item[que->tail];                           /* set information in (new) tail item */
item->src = src;
memcpy (item->packet, pack, CI_NET_BUF_SIZE);
}

t_stat ci_svc (UNIT *uptr)
{
SOCKET newsock;
uint32 port_num, i;
uint8 src;
uint8  src_ci_port;
uint8  dest_ci_port;
uint32  src_ipa, src_ipp;
t_stat r;
CI_ITEM *item;

while (ci_rx_queue.count > 0) {
    if (ci_rx_queue.loss > 0)
        printf("Warning: CI queue packet loss is %d\n", ci_rx_queue.loss);
    item = &ci_rx_queue.item[ci_rx_queue.head];
    ci_process_net_buffer (item);
    ciq_remove (&ci_rx_queue);                          /* remove processed packet from queue */
}

if ((uptr->flags & UNIT_ATT) == 0) {
    sim_clock_coschedule (uptr, tmxr_poll);
    return SCPE_OK;
    }

while (ci_tx_queue.count > 0) {
    if (ci_tx_queue.loss > 0)
        printf ("Warning: CI queue packet loss is %d\n", ci_tx_queue.loss);
    item = &ci_tx_queue.item[ci_tx_queue.head];
    r = ci_write_sock_tcp (ci_vcd[item->src].socket, item->packet, CI_NET_BUF_SIZE);
    if (r < 0) break;
    ciq_remove (&ci_tx_queue);                          /* remove processed packet from queue */
}

if (ci_state >= PORT_ENABLED) {                         /* Check multicast channel for any REQID packets */
    do {
        r = ci_read_sock_udp (ci_multi_sock, buffer, CI_NET_BUF_SIZE, &src_ipa, &src_ipp);
        if (r > 0) {
            src_ci_port = buffer[0x2] & 0xF;
            dest_ci_port = buffer[0xc] & 0xF;
            if ((dest_ci_port == (ci_ppr & PPR_NODE)) && (src_ci_port != (ci_ppr & PPR_NODE))) {
                ci_vcd[src_ci_port].ipa = src_ipa;
                //printf("multicast packet - src: %d, dest: %d\n", src_ci_port, dest_ci_port);
                ciq_insert (&ci_rx_queue, src_ci_port, buffer);
            }
        }
        if (r < 0) {
            printf ("CI: multicast socket read error\n");
            return SCPE_IOERR;
        }
    } while (r > 0);
}

for (i = 0; i < CI_MAX_NODES; i++) {
    if (ci_wait_sock[i] > 0) {
        r = ci_check_conn (ci_wait_sock[i], TRUE);
        if (r < 0) continue;
        r = ci_read_sock_tcp (ci_wait_sock[i], buffer, CI_NET_BUF_SIZE);
        if (r == 0) continue;
        src_ci_port = buffer[0x2] & 0xF;
        if (ci_vcd[src_ci_port].socket > 0) {
            ci_close_sock (ci_vcd[i].socket, 0);
            ci_wait_sock[i] = 0;
        }
        else {
            sim_debug (DBG_CONN, &ci_dev, "Accepting connection from node %d, socket: %d\n", src_ci_port, ci_wait_sock[i]);
            ciq_insert (&ci_rx_queue, src_ci_port, buffer);
            ci_vcd[src_ci_port].socket = ci_wait_sock[i];
            ci_vcd[src_ci_port].vc_state = VCST_OPEN;
            ci_wait_sock[i] = 0;
        }
    }
}

for (i = 0; i < CI_MAX_NODES; i++) {                    /* Poll open VCs */
    if (i == (ci_ppr & PPR_NODE))
        continue;

    switch (ci_vcd[i].vc_state) {

        case VCST_CLOSED:
            break;

        case VCST_WAIT:
            r = ci_check_conn (ci_vcd[i].socket, TRUE);
            if (r < 0)
                continue;
            ci_vcd[i].vc_state++;
            break;

        case VCST_OPEN:
            do {
                r = ci_read_sock_tcp (ci_vcd[i].socket, buffer, CI_NET_BUF_SIZE);
                if (r > 0) ciq_insert (&ci_rx_queue, i, buffer);
                if (r < 0) {
                    sim_debug (DBG_CONN, &ci_dev, "Node %d closed VC, socket: %d\n", i, ci_vcd[i].socket);
                    ci_close_sock (ci_vcd[i].socket, 0);
                    ci_vcd[i].vc_state = VCST_CLOSED;
                    ci_vcd[i].vcd_val &= ~0x8000;
                    ci_vc_fail (i);
                }
            } while (r > 0);
            break;
    }
}

while (ci_rx_queue.count > 0) {
    if (ci_rx_queue.loss > 0)
        printf ("Warning: CI queue packet loss is %d\n", ci_rx_queue.loss);
    item = &ci_rx_queue.item[ci_rx_queue.head];
    ci_process_net_buffer (item);
    ciq_remove (&ci_rx_queue);                          /* remove processed packet from queue */
}

/* Check for any nodes establishing a virtual circuit to this node. */

newsock = ci_accept_conn (ci_tcp_sock, &src_ipa);
if (newsock != INVALID_SOCKET)  {
    for (i = 0; i < CI_MAX_NODES; i++) {
        if (ci_wait_sock[i] == 0) {
            ci_wait_sock[i] = newsock;                  /* add to pending connections */
            break;
            }
        }
    }

sim_clock_coschedule (uptr, tmxr_poll);
return SCPE_OK;
}

t_stat ci_process_net_buffer (CI_ITEM *item)
{
uint8 ci_path;
uint8 src;

src = item->packet[0x2] & 0xF;
opcode = item->packet[0x3];
ci_path = item->packet[0x4] & 0x3;
//item->packet[0xc] = src;

if (ci_path == 0) {
    ci_path = 2;
//    item->packet[0xf] |= (ci_path << 4);
}

if (blktf_table[src].incomplete) {
    if ((opcode != OPC_SNDDAT) && (opcode != OPC_DATREC)) {
        item->packet[0xc] = src;
        item->packet[0xf] |= (ci_path << 4);
        item->packet[0xf] = item->packet[0xf] & ~0x1;   /* Clear RSP bit */
        }
    }
else {
    item->packet[0xc] = src;
    item->packet[0xf] |= (ci_path << 4);
    item->packet[0xf] = item->packet[0xf] & ~0x1;       /* Clear RSP bit */
    }

return ci_ppd ();
}

/* Reset CI adapter */

t_stat ci_port_reset (DEVICE *dptr)
{
int32 i;
t_stat r;

ci_state = PORT_UNINIT;

r = ciq_init (&ci_rx_queue, CI_QUE_MAX);                /* init read queue */
if (r != SCPE_OK)
    return r;

ciq_clear (&ci_rx_queue);

r = ciq_init (&ci_tx_queue, CI_QUE_MAX);                /* init write queue */
if (r != SCPE_OK)
    return r;

ciq_clear (&ci_tx_queue);

for (i = 0; i < CI_MAX_NODES; i++) {
    if (ci_vcd[i].socket > 0) {
        ci_close_sock (ci_vcd[i].socket, 0);
        ci_vcd[i].socket = 0;
        }
    ci_vcd[i].vcd_val = 0;                              /* clear VCD table */
    ci_vcd[i].mod_vcd_val = 0;
    ci_vcd[i].vc_state = VCST_CLOSED;
    ci_vcd[i].ipa = 0;
    ci_vcd[i].ipp = 0;

    blktf_table[i].incomplete = 0;                      /* clear block transfer table */
    blktf_table[i].opcode = 0;
    blktf_table[i].total_data_len = 0;
    blktf_table[i].data_pte = 0;
    blktf_table[i].data_offset = 0;
    blktf_table[i].start_offset = 0;
    blktf_table[i].page_offset = 0;

    ci_wait_sock[i] = 0;
    }

srand (time (NULL));
ci_pri = (rand() % 100);                                /* generate priority */

ci_clr_int ();
sim_cancel (&ci_unit);                                  /* stop poll */
return SCPE_OK;
}

t_stat ci_attach (UNIT *uptr, CONST char *cptr)
{
char* tptr;
t_stat r;
int32 ipa, ipp;
SOCKET multi_sock, tcp_sock;

if (ci_tcp_port == 0) {
    fprintf (stderr, "Must set TCP port first\n");
    return SCPE_ARG;
    }
r = get_ipaddr (cptr, &ipa, &ipp);                      /* get multicast addr */
if (r != SCPE_OK)
    return SCPE_ARG;

if (ipp == ci_tcp_port) {
    fprintf (stderr, "Multicast port cannot be same as TCP port\n");
    return SCPE_ARG;
    }

if (ipa < 0xE0000000) {                                 /* check it is valid */
    fprintf (stderr, "Invalid multicast address\n");
    return SCPE_ARG;
    }

tptr = (char *) malloc (strlen (cptr) + 1);             /* get string buf */
if (tptr == NULL)                                       /* no more mem? */
    return SCPE_MEM;

multi_sock = ci_master_sock (ipp, SCPN_MCS);            /* make multicast socket */
if (multi_sock == INVALID_SOCKET) {
    free (tptr);                                        /* release buf */
    return SCPE_OPENERR;                                /* open error */
    }
r = ci_setipmulticast(multi_sock, ipa);
if (r != SCPE_OK) {
    free (tptr);                                        /* release buf */
    return SCPE_OPENERR;                                /* open error */
    }
printf ("Listening on multicast port %d (socket %d)\n", ipp, multi_sock);
if (sim_log) fprintf (sim_log,
    "Listening on multicast port %d (socket %d)\n", ipp, multi_sock);

tcp_sock = ci_master_sock (ci_tcp_port, SCPN_TCP);      /* make tcp socket */
if (tcp_sock == INVALID_SOCKET) {
    free (tptr);                                        /* release buf */
    return SCPE_OPENERR;                                /* open error */
    }
printf ("Listening on TCP port %d (socket %d)\n", ci_tcp_port, tcp_sock);
if (sim_log) fprintf (sim_log,
    "Listening on TCP port %d (socket %d)\n", ci_tcp_port, tcp_sock);

ci_multi_ip = ipa;                                      /* save ip */
ci_multi_port = ipp;                                    /* save port */
ci_multi_sock = multi_sock;                             /* save master socket */
ci_tcp_sock = tcp_sock;
strcpy (tptr, cptr);                                    /* copy port */
uptr->filename = tptr;                                  /* save */
uptr->flags = uptr->flags | UNIT_ATT;                   /* no more errors */
return SCPE_OK;
}

t_stat ci_detach (UNIT *uptr)
{
if (!(uptr->flags & UNIT_ATT))                          /* attached? */
    return SCPE_OK;
ci_close_sock (ci_multi_sock, 1);                       /* close master socket */
ci_close_sock (ci_tcp_sock, 1);                         /* close master socket */
free (uptr->filename);                                  /* free port string */
uptr->filename = NULL;
uptr->flags = uptr->flags & ~UNIT_ATT;                  /* not attached */
return SCPE_OK;
}

/* Show CI adapter parameters */

t_stat ci_show_node (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
fprintf (st, "node=%d", (ci_ppr & PPR_NODE));
return SCPE_OK;
}

t_stat ci_show_tcp (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
fprintf (st, "port=%d", ci_tcp_port);
return SCPE_OK;
}

t_stat ci_set_node (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
int32 r;
uint32 newnode;

if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
newnode = (uint32) get_uint (cptr, 10, PPR_M_NODE, &r);
if (r != SCPE_OK)
    return r;
if ((newnode > 15) || (newnode < 0))
    return SCPE_ARG;
ci_ppr = (ci_ppr & ~PPR_NODE) | (newnode & PPR_M_NODE);
return SCPE_OK;
}

t_stat ci_set_tcp (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
int32 r;
uint32 newport;

if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
newport = (uint32) get_uint (cptr, 10, 65535, &r);
if (r != SCPE_OK)
    return r;
ci_tcp_port = newport;
return SCPE_OK;
}
