/* vax_ci.c: Computer Interconnect adapter

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
#include "sim_sock.h"
#include <time.h>

/* Virtual Circuit Flags */

#define VCD_SSN         0x0003                          /* send sequence number */
#define VCD_RSN         0x000C                          /* receive sequence number */
#define VCD_RDP         0x0040                          /* RDP supported */
#define VCD_FSN         0x0080                          /* full sequence number */
#define VCD_SAS         0x0100                          /* subnode addressing */
#define VCD_PAG         0x0200                          /* path A good */
#define VCD_PBG         0x0400                          /* path B good */
#define VCD_NADP        0x0800                          /* NADP supported */
#define VCD_IDG         0x1000                          /* inhibit datagrams */
#define VCD_SS          0x2000                          /* send sequence number */
#define VCD_RS          0x4000                          /* receive sequence number */
#define VCD_CST         0x8000                          /* circuit state */
#define VCD_OPEN(x)     (x & VCD_CST)
#define LINK_OPEN(p)    (uptr->ci_state > LINK_CLOSED)  /* link open */
#define DG_INHIBIT(x)   (x & VCD_IDG)

/* Network packet header */

#define HDR_LENGTH      0                               /* packet length */
#define HDR_SOURCE      2                               /* source CI port */
#define HDR_VCPORT      4                               /* virtual circuit TCP port */

/* CI Link States */

#define LINK_CLOSED     0                               /* link closed */
#define LINK_LOCAL      1                               /* link local */
#define LINK_WAIT       2                               /* link opening */
#define LINK_REMOTE     3                               /* link open */

/* Unit Specific Data */

#define port_ctx        up7                             /* port context */

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

typedef struct {
  uint8       port;
  CI_PKT      pkt;
} CI_ITEM;

typedef struct {
  int32       max;
  int32       count;
  int32       head;
  int32       tail;
  int32       loss;
  int32       high;
  CI_ITEM    *item;
} CI_QUE;

#define CI_QUE_MAX           100                        /* message queue array */

typedef struct {
    uint32    conn;                                     /* link state */
    SOCKET    socket;                                   /* TCP link */
    char      host[CBUFSIZE];                           /* IP address of remote system */
    UNIT      *uptr;                                    /* local unit */
    uint32    vcd;                                      /* VC descriptor */
} CI_NODE;

typedef struct {
    char      group[CBUFSIZE];                          /* multicast group address */
    char      tcp_port[CBUFSIZE];                       /* VC port */
    SOCKET    multi_sock;                               /* multicast socket */
    SOCKET    tcp_sock;                                 /* VC socket */
    SOCKET    wait_sock[CI_MAX_NODES];                  /* pending connections */
    CI_NODE   nodes[CI_MAX_NODES];                      /* node context */
    CI_QUE    rx_queue;                                 /* packet receive queue */
    CI_QUE    tx_queue;                                 /* packet transmit queue */
} CI_PORT;

CI_PKT rcv_pkt;                                         /* receive packet buffer */

extern int32 tmxr_poll;

t_stat ci_snddg (UNIT *uptr, CI_PKT *pkt);
t_stat ci_sndmsg (UNIT *uptr, CI_PKT *pkt);
t_stat ci_reqid (UNIT *uptr, CI_PKT *pkt);
t_stat ci_reqdat (UNIT *uptr, CI_PKT *pkt);
t_stat ci_retid (UNIT *uptr, CI_PKT *pkt);
t_stat ci_sndlb (UNIT *uptr, CI_PKT *pkt);
t_stat ci_snddat (UNIT *uptr, CI_PKT *pkt);
t_stat ci_retdat (UNIT *uptr, CI_PKT *pkt);
t_stat ci_retcnf (UNIT *uptr, CI_PKT *pkt);
t_stat ci_invtc (UNIT *uptr, CI_PKT *pkt);
t_stat ci_setckt (UNIT *uptr, CI_PKT *pkt);
t_stat ci_rdcnt (UNIT *uptr, CI_PKT *pkt);
t_stat ci_dgrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_msgrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_reqrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_reqdatrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_idrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_datrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_snddatrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_cnfrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_lbrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_sndrst (UNIT *uptr, CI_PKT *pkt);
t_stat ci_sndstrt (UNIT *uptr, CI_PKT *pkt);
t_stat ci_rstrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_strtrec (UNIT *uptr, CI_PKT *pkt);
t_stat ci_send_packet (UNIT *uptr, CI_PKT *pkt);
t_stat ci_receive_packet (UNIT *uptr, CI_PKT *pkt);
t_stat ci_open_vc (UNIT *uptr, uint8 port);
t_stat ci_close_vc (UNIT *uptr, uint8 port);
t_stat ci_fail_vc (UNIT *uptr, uint8 port);
t_stat ciq_init (CI_QUE *que, int32 max);               /* initialize FIFO queue */
void ciq_clear  (CI_QUE *que);                          /* clear FIFO queue */
void ciq_remove (CI_QUE *que);                          /* remove item from FIFO queue */
void ciq_insert (CI_QUE *que, CI_PKT *pkt);             /* insert item into FIFO queue */

void ci_dump_packet (UNIT *uptr, char* buffer, uint32 length)
{
uint8 tbyte;
int32 i, j;
return;
for (i = 0; i < length; i+=4) {
    sim_debug_unit (DBG_SCSMSG, uptr, "  %04X: ", i);
//    sim_debug_unit (DBG_SCSMSG, ci_dev, "  %s: ", prefix);
    sim_debug_unit (DBG_SCSMSG, uptr, "  ");
    for (j = 3; j >= 0; j--) {
        if ((i + j) < length) {
            tbyte = buffer[i+j];
            sim_debug_unit (DBG_SCSMSG, uptr, "%02X ", tbyte);
            }
        else
            sim_debug_unit (DBG_SCSMSG, uptr, "   ", tbyte);
        }
    sim_debug_unit (DBG_SCSMSG, uptr, " ");
    for (j = 0; j < 4; j++) {
        if ((i + j) < length) {
            tbyte = buffer[i+j];
            if ((tbyte > 0x1F) && (tbyte < 0x7F))
                sim_debug_unit (DBG_SCSMSG, uptr, "%c", tbyte);
            else
                sim_debug_unit (DBG_SCSMSG, uptr, ".");
            }
        else
            sim_debug_unit (DBG_SCSMSG, uptr, " ");
        }
    sim_debug_unit (DBG_SCSMSG, uptr, "\n");
    }
}

/* CI port state change */

void ci_set_state (UNIT *uptr, uint32 state)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
int32 r;

if (state < PORT_INIT) {                                /* change to uninit */
    if (uptr->ci_state >= PORT_INIT) {                  /* currently initialised? */
        sim_cancel (uptr);                              /* stop poll */
        sim_debug_unit (DBG_CONN, uptr, "deactivating unit\n");
        ciq_clear (&cp->rx_queue);                      /* clear queue */
        do r = sim_read_sock (cp->multi_sock, rcv_pkt.data, CI_MAXFR);
        while (r > 0);                                  /* clear network buffer */
        }
    }
else if (state >= PORT_INIT) {                          /* change to init */
    if (uptr->ci_state < PORT_INIT) {                   /* currently uninit? */
        sim_clock_coschedule (uptr, tmxr_poll);         /* start poll */
        sim_debug_unit (DBG_CONN, uptr, "activating unit\n");
        }
   }
uptr->ci_state = state;                                 /* update state */
}

/* Local port commands */

t_stat ci_local_ppd (UNIT *uptr, CI_PKT *pkt)
{
uint32 opcode = pkt->data[PPD_OPC];
t_stat r;

switch (opcode) {

    case OPC_INVTC:
        return ci_invtc (uptr, pkt);

    case OPC_SETCKT:
        return ci_setckt (uptr, pkt);

    case OPC_RDCNT:
        return ci_rdcnt (uptr, pkt);

    default:
        sim_printf ("CI: unimplemented opcode (send) %02X\n", opcode);
        }
return SCPE_IOERR; // TODO: need local status
}

t_stat ci_send_ppd (UNIT *uptr, CI_PKT *pkt)
{
uint32 opcode = pkt->data[PPD_OPC];
t_stat r;

switch (opcode) {

    case OPC_SNDDG:
        r = ci_snddg (uptr, pkt);
        break;

    case OPC_SNDMSG:
        r = ci_sndmsg (uptr, pkt);
        break;

    case OPC_RETCNF:
        r = ci_retcnf (uptr, pkt);
        break;

    case OPC_REQID:
        r = ci_reqid (uptr, pkt);
        break;

    case OPC_SNDRST:
        r = ci_sndrst (uptr, pkt);
        break;

    case OPC_SNDSTRT:
        r = ci_sndstrt (uptr, pkt);
        break;

    case OPC_REQDAT:
        r = ci_reqdat (uptr, pkt);
        break;

    case OPC_RETID:
        r = ci_retid (uptr, pkt);
        break;

    case OPC_SNDLB:
        r = ci_sndlb (uptr, pkt);
        break;

    case OPC_SNDDAT:
        r = ci_snddat (uptr, pkt);
        break;

    case OPC_RETDAT:
        r = ci_retdat (uptr, pkt);
        break;

    default:
        return ci_local_ppd (uptr, pkt);                /* check for local PPD */
        }
if (r == SCPE_OK)
    r = ci_send_packet (uptr, pkt);
return r;
}

t_stat ci_receive_ppd (UNIT *uptr, CI_PKT *pkt)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
uint32 opcode, port;
t_stat r;

for ( ;; ) {
    r = ci_receive_packet (uptr, pkt);                  /* get next packet */
    if (r != SCPE_OK)                                   /* none available? */
        return r;                                       /* done */
    opcode = pkt->data[PPD_OPC];
    if (opcode == OPC_DGREC) {                          /* datagram? */
        port = pkt->data[PPD_PORT];                     /* get remote port */
        if (DG_INHIBIT (cp->nodes[port].vcd))           /* inhibited? */
            continue;                                   /* try next packet */
        }
        break;                                          /* process packet */
    }

switch (opcode) {

    case OPC_DGREC:                                     /* datagram */
        return ci_dgrec (uptr, pkt);

    case OPC_MSGREC:                                    /* message */
        return ci_msgrec (uptr, pkt);

    case OPC_REQREC:                                    /* ID request */
        return ci_reqrec (uptr, pkt);

    case OPC_RSTREC:                                    /* port reset */
        return ci_rstrec (uptr, pkt);

    case OPC_STRTREC:                                   /* port start */
        return ci_strtrec (uptr, pkt);

    case OPC_REQDATREC:                                 /* data requested */
        return ci_reqdatrec (uptr, pkt);

    case OPC_IDREC:                                     /* ID response */
        return ci_idrec (uptr, pkt);

    case OPC_DATREC:                                    /* data received */
        return ci_datrec (uptr, pkt);

    case OPC_SNDDATREC:                                 /* data received */
        return ci_snddatrec (uptr, pkt);

    case OPC_CNFREC:                                    /* confirmation */
        return ci_cnfrec (uptr, pkt);

    case OPC_LBREC:                                     /* loopback */
        return ci_lbrec (uptr, pkt);

    default:
        sim_printf ("CI: unimplemented opcode (receive) %02X\n", opcode);
        }
return SCPE_IOERR; // TODO: need local status
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
t_stat ci_snddg (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
uint16 dg_type = CI_GET16 (pkt->data, PPD_MTYPE);
uint32 dg_size = CI_GET16 (pkt->data, PPD_LENGTH) + PPD_DGHDR;
t_stat r;

switch (dg_type) {

    case 0:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDDG - START, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 1:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDDG - STACK dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 2:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDDG - ACK, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 3:
        sim_debug_unit (DBG_SCSDG, uptr, "==> SNDDG - SCSDG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 4:
        sim_debug_unit (DBG_WRN, uptr, "==> SNDDG - SCSMSG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 5:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDDG - ELOG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 6:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDDG - HOSTSHUT, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 7:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDDG - FU_DG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    default:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDDG - %X, dest: %d, path: %s\n", dg_type, port, ci_path_names[path]);
        break;
        }

ci_dump_packet (uptr, pkt->data, dg_size);
return SCPE_OK;
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
t_stat ci_sndmsg (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
uint32 msg_size = CI_GET16 (pkt->data, PPD_LENGTH) + PPD_MSGHDR;
uint16 dg_type = CI_GET16 (pkt->data, PPD_MTYPE) & 0x7FFF;
uint16 msg_type;

switch (dg_type) {

    case 0:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDMSG - START, dest: %d, path: %s\n", port, ci_path_names[path]);
        //
        // The following code may be related to the VC failure mechanism.
        // Maybe put this in ci_send_packet when VC is not open?
        //
        sim_debug_unit (DBG_PPDDG, uptr, "<== SNDMSG - START, No Path\n");
        pkt->data[PPD_STATUS] = 0xA1;                   /* Status: No Path */
        // FIXME: Handle this via normal ci_send_ppd() route
#if 0
        ci_respond (pkt);
#endif
        return SCPE_OK;
        break;

    case 1:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDMSG - STACK dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 2:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDMSG - ACK, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 3:
        sim_debug_unit (DBG_WRN, uptr, "==> SNDMSG - SCSDG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 4:
        msg_type = CI_GET16 (pkt->data, PPD_STYPE);
        sim_debug_unit (DBG_SCSMSG, uptr, "==> SCSMSG - %s, dest: %d, path: %s\n", scs_msg_types[msg_type], port, ci_path_names[path]);
//        ci_dump_packet (uptr, pkt->data, msg_size);
        break;

    case 5:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDMSG - ELOG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 6:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDMSG - HOSTSHUT, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 7:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDMSG - FU_DG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    default:
        sim_debug_unit (DBG_PPDDG, uptr, "==> SNDMSG - %X, dest: %d, path: %s\n", dg_type, port, ci_path_names[path]);
        break;
        }

ci_dump_packet (uptr, pkt->data, msg_size);
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
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
t_stat ci_reqid (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_REQID, uptr, "==> REQID, dest: %d, path: %s\n", port, ci_path_names[path]);
ci_dump_packet (uptr, pkt->data, pkt->length);
pkt->data[PPD_STATUS] = 0;                              /* Status OK */
return SCPE_OK;
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
t_stat ci_reqdat (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_BLKTF, uptr, "==> REQDAT, dest: %d, path: %s\n", port, ci_path_names[path]);
ci_dump_packet (uptr, pkt->data, pkt->length);
pkt->data[PPD_STATUS] = 0;                              /* Status OK */
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
}

t_stat ci_retid (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
CI_NODE *node = &cp->nodes[port];
sim_debug_unit (DBG_REQID, uptr, "==> RETID, dest: %d, path: %s\n", port, ci_path_names[path]);
ci_dump_packet (uptr, pkt->data, pkt->length);
pkt->data[PPD_STATUS] = 0;                              /* Status OK */
if (VCD_OPEN (node->vcd))                               /* VC open? */
    pkt->data[0x12] = 1;                                /* set transaction ID */
return SCPE_OK;
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
t_stat ci_sndlb (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_LCMD, uptr, "==> SNDLB, dest: %d, path: %s\n", port, ci_path_names[path]);
ci_dump_packet (uptr, pkt->data, pkt->length);
return SCPE_OK;
}

/*
   Block Transfers:
      These packets allow large amounts of data to be
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
t_stat ci_snddat (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_BLKTF, uptr, "==> SNDDAT, dest: %d, path: %s\n", port, ci_path_names[path]);
ci_dump_packet (uptr, pkt->data, pkt->length);
pkt->data[PPD_STATUS] = 0;                              /* status OK */
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
}

t_stat ci_retdat (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_BLKTF, uptr, "==> RETDAT, dest: %d, path: %s\n", port, ci_path_names[path]);
ci_dump_packet (uptr, pkt->data, pkt->length);
pkt->data[PPD_STATUS] = 0;                              /* status OK */
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
}

t_stat ci_retcnf (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_BLKTF, uptr, "==> RETCNF, dest: %d, path: %s\n", port, ci_path_names[path]);
ci_dump_packet (uptr, pkt->data, pkt->length);
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
}

t_stat ci_sndrst (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_PPDDG, uptr, "==> SNDRST, dest: %d, path: %s\n", port, ci_path_names[path]);
ci_dump_packet (uptr, pkt->data, 0x10);
return SCPE_OK;
}

t_stat ci_sndstrt (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_PPDDG, uptr, "==> SNDSTRT, dest: %d, path: %s\n", port, ci_path_names[path]);
ci_dump_packet (uptr, pkt->data, 0x10);
return SCPE_OK;
}

t_stat ci_invtc (UNIT *uptr, CI_PKT *pkt)
{
sim_debug_unit (DBG_LCMD, uptr, "==> INVTC\n");
pkt->data[PPD_STATUS] = 0;                              /* status OK */
return SCPE_OK;
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
 *  ....GGGG 0x18   G = VCB val before   *
 *                                       *
 *  Length = 0x14                        *
 *                                       */
t_stat ci_setckt (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
CI_NODE *node = &cp->nodes[port];
uint32 vcd_mmsk = CI_GET32 (pkt->data, PPD_VCMMSK);
uint32 vcd_mval = CI_GET32 (pkt->data, PPD_VCMVAL);
uint32 vcd_val = node->vcd;                              /* current VCD value */
uint32 vcd_nval = vcd_val;                               /* new VCD value */
t_stat r = SCPE_OK;

sim_debug_unit (DBG_LCMD, uptr, "==> SETCKT, port: %d, mask: %X value: %X\n", port, vcd_mmsk, vcd_mval);
ci_dump_packet (uptr, pkt->data, pkt->length);

vcd_nval &= ~vcd_mmsk;
vcd_nval |= vcd_mval;

if (VCD_OPEN (vcd_nval) && !VCD_OPEN (vcd_val))         /* Opening VC */
    r = ci_open_vc (uptr, port);
else if (!VCD_OPEN (vcd_nval) && VCD_OPEN (vcd_val))    /* Closing VC */
    r = ci_close_vc (uptr, port);

if (r != SCPE_OK)
    return r;
CI_PUT32 (pkt->data, PPD_VCPVAL, vcd_val);
pkt->data[PPD_STATUS] = 0;                              /* status OK */
node->vcd = vcd_nval;                                   /* set new VCD value */
return r;
}

t_stat ci_rdcnt (UNIT *uptr, CI_PKT *pkt)
{
sim_debug_unit (DBG_LCMD, uptr, "==> RDCNT\n");
CI_PUT32 (pkt->data, 0x10, 0);                          /* ACKs on path 0 */
CI_PUT32 (pkt->data, 0x14, 0);                          /* NAKs on path 0 */
CI_PUT32 (pkt->data, 0x18, 0);                          /* NORSPs on path 0 */
CI_PUT32 (pkt->data, 0x1c, 0);                          /* ACKs on path 1 */
CI_PUT32 (pkt->data, 0x20, 0);                          /* NAKs on path 1 */
CI_PUT32 (pkt->data, 0x24, 0);                          /* NORSPs on path 1 */
CI_PUT32 (pkt->data, 0x28, 0);                          /* DGS discarded */
pkt->data[PPD_STATUS] = 0;                              /* status OK */
return SCPE_OK;
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
t_stat ci_dgrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
uint32 dg_type = pkt->data[PPD_MTYPE];

switch (dg_type) {

    case 0:
        sim_debug_unit (DBG_PPDDG, uptr, "<== DGREC - START, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 1:
        sim_debug_unit (DBG_PPDDG, uptr, "<== DGREC - STACK, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 2:
        sim_debug_unit (DBG_PPDDG, uptr, "<== DGREC - ACK, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 3:
        sim_debug_unit (DBG_SCSDG, uptr, "<== DGREC - SCSDG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 4:
        sim_debug_unit (DBG_WRN, uptr, "<== DGREC - SCSMSG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 5:
        sim_debug_unit (DBG_PPDDG, uptr, "<== DGREC - ELOG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 6:
        sim_debug_unit (DBG_PPDDG, uptr, "<== DGREC - HOSTSHUT, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 7:
        sim_debug_unit (DBG_PPDDG, uptr, "<== DGREC - FU_DG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    default:
        sim_debug_unit (DBG_PPDDG, uptr, "<== DGREC - %X, src: %d, path: %s\n", dg_type, port, ci_path_names[path]);
        break;
        }
return SCPE_OK;
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
t_stat ci_msgrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
uint32 msg_size = CI_GET16 (pkt->data, PPD_LENGTH) + PPD_MSGHDR;
uint32 dg_type = pkt->data[PPD_MTYPE];
uint32 msg_type;

switch (dg_type) {

    case 0:
        sim_debug_unit (DBG_PPDDG, uptr, "<== MSGREC - START, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 1:
        sim_debug_unit (DBG_PPDDG, uptr, "<== MSGREC - STACK, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 2:
        sim_debug_unit (DBG_PPDDG, uptr, "<== MSGREC - ACK, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 3:
        sim_debug_unit (DBG_WRN, uptr, "<== MSGREC - SCSDG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 4:
        msg_type = pkt->data[PPD_STYPE];
        sim_debug_unit (DBG_SCSMSG, uptr, "<== SCSMSG - %s, src: %d, path: %s\n", scs_msg_types[msg_type], port, ci_path_names[path]);
//        ci_dump_packet (uptr, pkt->data, msg_size);
        break;

    case 5:
        sim_debug_unit (DBG_PPDDG, uptr, "<== MSGREC - ELOG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 6:
        sim_debug_unit (DBG_PPDDG, uptr, "<== MSGREC - HOSTSHUT, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 7:
        sim_debug_unit (DBG_PPDDG, uptr, "<== MSGREC - FU_DG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    default:
        sim_debug_unit (DBG_PPDDG, uptr, "<== MSGREC - %X, src: %d, path: %s\n", dg_type, port, ci_path_names[path]);
        break;
        }

pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
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
t_stat ci_reqrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_REQID, uptr, "<== REQREC, src: %d, path: %s\n", port, ci_path_names[path]);
return SCPE_OK;
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
t_stat ci_reqdatrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_BLKTF, uptr, "<== REQDATREC, src: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
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
t_stat ci_idrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_REQID, uptr, "<== IDREC, src: %d, path: %s\n", port, ci_path_names[path]);
return SCPE_OK;
}

t_stat ci_datrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_BLKTF, uptr, "<== DATREC, src: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
}

t_stat ci_snddatrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_BLKTF, uptr, "<== SNDDATREC, src: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
}

t_stat ci_cnfrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_BLKTF, uptr, "<== CNFREC, src: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
}

/*             < LBREC >               *
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
t_stat ci_lbrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
CI_NODE *node = &cp->nodes[port];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_LCMD, uptr, "<== LBREC, path: %s\n", ci_path_names[path]);
node->vcd |= (VCD_PAG | VCD_PBG);                       /* path A and B good */
return SCPE_OK;
}

t_stat ci_rstrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_PPDDG, uptr, "<== RSTREC, src: %d, path: %s\n", port, ci_path_names[path]);
//ci_dump_packet (uptr, pkt->data, 0x10);
return SCPE_OK;
}

t_stat ci_strtrec (UNIT *uptr, CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug_unit (DBG_PPDDG, uptr, "<== STRTREC, src: %d, path: %s\n", port, ci_path_names[path]);
//ci_dump_packet (uptr, pkt->data, 0x10);
return SCPE_OK;
}

/* Open a link to a remote node */

t_stat ci_open_vc (UNIT *uptr, uint8 port)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
CI_NODE *node = &cp->nodes[port];
SOCKET newsock;

if (node->conn == LINK_LOCAL)                           /* local port? */
    return SCPE_OK;                                     /* yes, done */
if (LINK_OPEN (port))                                   /* already open? */
    return SCPE_OK;                                     /* yes, done */
if (uptr->ci_node > port) {                             /* are we the server? */
    node->conn = LINK_WAIT;
    return SCPE_OK;                                     /* yes, done */
    }
sim_debug_unit (DBG_CONN, uptr, "Connecting to node %d at %s...\n", port, node->host);
newsock = sim_connect_sock (node->host, NULL, NULL);
if (newsock == INVALID_SOCKET) {
    sim_debug_unit (DBG_CONN, uptr, "Unable to establish VC to node %d\n", port);
    return SCPE_OPENERR;
    }
sim_debug_unit (DBG_CONN, uptr, "Connected to node %d, socket %d\n", port, newsock);
node->socket = newsock;
node->conn = LINK_REMOTE;
return SCPE_OK;
}

/* Close a link to a remote node */

t_stat ci_close_vc (UNIT *uptr, uint8 port)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
CI_NODE *node = &cp->nodes[port];

if (!LINK_OPEN (port))                                  /* already closed? */
    return SCPE_OK;                                     /* yes, done */
if (node->conn == LINK_LOCAL)                           /* local port? */
    return SCPE_OK;                                     /* yes, done */
sim_debug_unit (DBG_CONN, uptr, "Closing VC with node %d...\n", port);
sim_close_sock (node->socket);
node->socket = 0;
node->conn = LINK_CLOSED;
return SCPE_OK;
}

t_stat ci_check_vc (UNIT *uptr, uint8 port)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
CI_NODE *node = &cp->nodes[port];
return VCD_OPEN (node->vcd) ? TRUE : FALSE;
}

/* Notifies the operating system that the connection with the *
 * remote node has been lost by inserting a START datagram on *
 * the response queue in order to trigger VC failure          */

t_stat ci_fail_vc (UNIT *uptr, uint8 port)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
CI_NODE *node = &cp->nodes[port];
CI_PKT pkt;

ci_close_vc (uptr, port);                               /* close the VC */
node->vcd &= ~VCD_CST;                                  /* update descriptor */
pkt.data[PPD_PORT] = port;                              /* port number */
pkt.data[PPD_STATUS] = 0;                               /* status: OK */
pkt.data[PPD_OPC] = OPC_DGREC;                          /* opcode */
pkt.data[PPD_FLAGS] = 0;                                /* flags */
pkt.data[PPD_LENGTH] = 0;                               /* message length */
// TODO: Should message be HOSTSHUT rather than START?
pkt.data[PPD_MTYPE] = DG_START;                         /* message type: START */
pkt.addr = 0;
pkt.length = 0x14;                                      // TODO: Need #define for this
ciq_insert (&cp->rx_queue, &pkt);                       /* put to response queue */
return SCPE_OK;
}

t_stat ci_send_packet (UNIT *uptr, CI_PKT *pkt)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
uint32 port = pkt->data[PPD_PORT];
CI_NODE *node = &cp->nodes[port];
CI_PKT tpkt;
int32 r;

memcpy (&tpkt, pkt, sizeof (CI_PKT));                   /* make local copy */
tpkt.data[HDR_SOURCE] = uptr->ci_node;                  /* local port number */

if (node->conn == LINK_LOCAL) {                         /* loopback? */
    sim_debug_unit (DBG_CONN, uptr, "packet destination is local node\n");
    uptr = node->uptr;                                  /* get local unit */
    cp = (CI_PORT *)uptr->port_ctx;                     /* get local port */
    // TODO: check for queue full
    ciq_insert (&cp->rx_queue, &tpkt);                  /* add to queue */
    sim_cancel (uptr);
    sim_activate (uptr, 0);
    return SCPE_OK;
    }

CI_PUT16 (tpkt.data, HDR_LENGTH, tpkt.length);
strncpy (&tpkt.data[HDR_VCPORT], cp->tcp_port, 5);      /* VC port */

if (LINK_OPEN (port)) {                                 /* link established? */
    if (cp->tx_queue.count == 0) {                      /* need to queue? */
        r = sim_write_sock (node->socket, tpkt.data, tpkt.length);
        if (r < 0)
            ciq_insert (&cp->tx_queue, &tpkt);          /* try later */
        }
    else
        ciq_insert (&cp->tx_queue, &tpkt);
    }
else if (uptr->flags & UNIT_ATT)                        /* no, use multicast */
    r = sim_write_sock_ex (cp->multi_sock, tpkt.data, tpkt.length, cp->group, SIM_SOCK_OPT_DATAGRAM);
//FIXME: No path
return SCPE_OK;
}

t_stat ci_poll_mcs (UNIT *uptr)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
char *src_addr;
uint8 src, dst;
t_stat r;

for (;;) {
    if (cp->rx_queue.count == CI_QUE_MAX);              /* queue full? */
        break;                                          /* yes, done */
    r = sim_read_sock_ex (cp->multi_sock, rcv_pkt.data, CI_MAXFR, &src_addr);
    if (r < 0) {
        sim_printf ("CI: multicast socket read error\n");
        return SCPE_IOERR;
        }
    if (r == 0)                                         /* no packet? */
        break;                                          /* done */
    src = rcv_pkt.data[HDR_SOURCE];
    dst = rcv_pkt.data[PPD_PORT];
    if (dst == uptr->ci_node) {
        sprintf (cp->nodes[src].host, "[%s]:%.5s", src_addr, &rcv_pkt.data[HDR_VCPORT]);
        rcv_pkt.length = CI_GET16 (rcv_pkt.data, HDR_LENGTH);
        ciq_insert (&cp->rx_queue, &rcv_pkt);
        }
    free (src_addr);
    }
return SCPE_OK;
}

t_stat ci_poll_tcp (UNIT *uptr)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
uint32 i;
uint8 dst;
t_stat r;

for (i = 0; i < CI_MAX_NODES; i++) {
    if (cp->rx_queue.count == CI_QUE_MAX)               /* queue full? */
        break;                                          /* yes, don't receive any more */
    if (cp->nodes[i].conn == LINK_LOCAL)                /* local node? */
        continue;                                       /* yes, skip */
    if (cp->nodes[i].socket != 0) {                     /* connection established? */
        do {
            if (cp->rx_queue.count == CI_QUE_MAX)       /* queue full? */
                break;                                  /* yes, don't receive any more */
            r = sim_read_sock (cp->nodes[i].socket, rcv_pkt.data, 2);
            if (r > 0) {
                rcv_pkt.length = CI_GET16 (rcv_pkt.data, 0);
                r = sim_read_sock (cp->nodes[i].socket, &rcv_pkt.data[2], rcv_pkt.length - 2);
                }
            dst = rcv_pkt.data[PPD_PORT];
            if (dst != uptr->ci_node)                   /* not for this node? */
                continue;                               /* drop packet */
            if (r > 0) ciq_insert (&cp->rx_queue, &rcv_pkt);
            if (r < 0) {
                sim_debug_unit (DBG_CONN, uptr, "Node %d closed VC, socket: %d\n", i, cp->nodes[i].socket);
                ci_fail_vc (uptr, i);                   /* notify VC failure */
                }
            } while (r > 0);
        }
    }
return SCPE_OK;
}

t_stat ci_receive_packet (UNIT *uptr, CI_PKT *pkt)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
uint32 path;
t_stat r;
CI_ITEM *item;

if (cp->rx_queue.count == 0) {                          /* receive queue empty? */
    if (uptr->flags & UNIT_ATT) {
        if (uptr->ci_state >= PORT_ENABLED) {           /* port enabled? */
            r = ci_poll_mcs (uptr);                     /* poll multicast channel */
            if (r != SCPE_OK)
                return r;
            r = ci_poll_tcp (uptr);                     /* poll open VCs */
            if (r != SCPE_OK)
                return r;
            }
        }
    }
if (cp->rx_queue.count > 0) {                           /* process receive queue */
    if (cp->rx_queue.loss > 0)
        sim_printf ("Warning: CI queue packet loss is %d\n", cp->rx_queue.loss);
    item = &cp->rx_queue.item[cp->rx_queue.head];
    memcpy (pkt, &item->pkt, sizeof (CI_PKT));          /* copy to output packet */
    path = GET_PATH (pkt->data[PPD_FLAGS]);
    if (path == PPD_PSAUTO)                             /* auto select path? */
        path = PPD_PS1;                                 /* default to path 1 */
    pkt->data[PPD_OPC] |= OPC_M_RECV;                   /* convert to rcv opcode */
    pkt->data[PPD_PORT] = pkt->data[HDR_SOURCE];        /* set source port */
    pkt->data[PPD_FLAGS] |= (path << PPD_V_SP);         /* set send path */
    pkt->data[PPD_FLAGS] &= ~PPD_RSP;                   /* clear response bit */
    pkt->addr = 0;
    ciq_remove (&cp->rx_queue);                         /* remove processed packet from queue */
    return SCPE_OK;
    }

return SCPE_EOF;
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
        sim_printf ("CI: failed to allocate dynamic queue[%d]\n", max);
        return SCPE_MEM;
        };
    };
return SCPE_OK;
}

void ciq_clear (CI_QUE* que)
{
memset (que->item, 0, sizeof(CI_ITEM) * que->max);      /* clear packet array */
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

void ciq_insert (CI_QUE* que, CI_PKT *pkt)
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
memcpy (&item->pkt, pkt, sizeof (CI_PKT)); // TODO: don't copy whole packet?
}

t_stat ci_svc (UNIT *uptr)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;
SOCKET newsock;
uint32 i;
uint8  src_ci_port;
t_stat r;
CI_ITEM *item;

if ((uptr->flags & UNIT_ATT) == 0)
    return SCPE_OK;

while (cp->tx_queue.count > 0) {                        /* process transmit queue */
    if (cp->tx_queue.loss > 0)
        sim_printf ("Warning: CI queue packet loss is %d\n", cp->tx_queue.loss);
    item = &cp->tx_queue.item[cp->tx_queue.head];
    r = sim_write_sock (cp->nodes[item->port].socket, item->pkt.data, CI_MAXFR);
    if (r < 0) break;
    ciq_remove (&cp->tx_queue);                         /* remove processed packet from queue */
    }

for (i = 0; i < CI_MAX_NODES; i++) {                    /* check pending connections */
    if (cp->wait_sock[i] > 0) {
        r = sim_check_conn (cp->wait_sock[i], TRUE);    /* anything to receive? */
        if (r < 0) continue;                            /* no, continue */
        r = sim_read_sock (cp->wait_sock[i], rcv_pkt.data, CI_MAXFR);
        if (r == 0) continue;                           /* nothing received, continue */
        src_ci_port = rcv_pkt.data[HDR_SOURCE];
        if (cp->nodes[src_ci_port].socket > 0) {        /* VC already open? */
            sim_close_sock (cp->wait_sock[i]);          /* reject connection */
            cp->wait_sock[i] = 0;                       /* clear pending connection */
            }
        else {
            sim_debug_unit (DBG_CONN, uptr, "Accepting connection from node %d, socket: %d\n", src_ci_port, cp->wait_sock[i]);
            rcv_pkt.length = CI_GET16 (rcv_pkt.data, HDR_LENGTH);
            ciq_insert (&cp->rx_queue, &rcv_pkt);
            cp->nodes[src_ci_port].socket = cp->wait_sock[i];
            cp->nodes[src_ci_port].conn = LINK_REMOTE;
            cp->wait_sock[i] = 0;                       /* clear pending connection */
            }
        }
    }

newsock = sim_accept_conn (cp->tcp_sock, NULL);         /* check for new VCs */
if (newsock != INVALID_SOCKET)  {
    for (i = 0; i < CI_MAX_NODES; i++) {                /* find free queue entry */
        if (cp->wait_sock[i] == 0) {
            cp->wait_sock[i] = newsock;                 /* save socket in queue */
            break;
            }
        }
    }

return SCPE_OK;
}

/* Reset CI adapter */

t_stat ci_port_reset (DEVICE *dptr)
{
UNIT *uptr = dptr->units;
UNIT *ruptr;
DEVICE *rdptr;
CI_PORT *cp, *rcp;
t_stat r;
int32 i;

uptr->dptr = dptr;
uptr->port_ctx = (void *) realloc (uptr->port_ctx, sizeof (CI_PORT));
cp = (CI_PORT *)uptr->port_ctx;

for (i = 0; i < CI_MAX_NODES; i++) {
    if (cp->nodes[i].socket != 0) {                     /* close open VCs */
        sim_close_sock (cp->nodes[i].socket);
        cp->nodes[i].socket = 0;
        }
    if (cp->wait_sock[i] != 0) {                        /* close pending VCs */
        sim_close_sock (cp->wait_sock[i]);
        cp->wait_sock[i] = 0;
        }
    cp->nodes[i].host[0] = '\0';
    cp->nodes[i].conn = LINK_CLOSED;
    cp->nodes[i].vcd = 0;                               /* clear VCD table */
    cp->nodes[i].uptr = NULL;
    }

for (i = 0; (rdptr = sim_devices[i]) != NULL; i++) {    /* loop thru dev */
    if (!(rdptr->flags & DEV_DIS)) {                    /* enabled? */
        if (rdptr->flags & DEV_CI) {                    /* CI? */
            if (rdptr == dptr)                          /* self? */
                continue;                               /* skip */
            ruptr = rdptr->units;
            rcp = ruptr->port_ctx;
            if (rcp == NULL)                            /* port not setup? */
                continue;
            if (((ruptr->flags ^ uptr->flags) & UNIT_ATT) != 0) /* same attach state? */
                continue;
            if ((ruptr->flags & uptr->flags) & UNIT_ATT) { /* both attached? */
                if (strcmp (rcp->group, cp->group) != 0)    /* same group? */
                    continue;
                }
            cp->nodes[ruptr->ci_node].conn = LINK_LOCAL; /* found local node */
            cp->nodes[ruptr->ci_node].uptr = ruptr;
            }
        }
    }

cp->nodes[uptr->ci_node].conn = LINK_LOCAL;             /* add self */
cp->nodes[uptr->ci_node].uptr = uptr;
uptr->ci_state = PORT_UNINIT;                           /* now uninitialised */

r = ciq_init (&cp->rx_queue, CI_QUE_MAX);               /* init read queue */
if (r != SCPE_OK)
    return r;
ciq_clear (&cp->rx_queue);

r = ciq_init (&cp->tx_queue, CI_QUE_MAX);               /* init write queue */
if (r != SCPE_OK)
    return r;
ciq_clear (&cp->tx_queue);

sim_cancel (uptr);                                      /* stop poll */
return SCPE_OK;
}

t_stat ci_attach (UNIT *uptr, CONST char *cptr)
{
char* tptr;
t_stat r;
SOCKET multi_sock, tcp_sock;
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;

if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
if (cp->group[0] == '\0') {
    sim_printf ("Must set group address first\n");
    return SCPE_ARG;
    }

multi_sock = sim_master_sock_ex (cp->group, NULL,
    (SIM_SOCK_OPT_REUSEADDR | SIM_SOCK_OPT_MULTICAST)); /* make multicast socket */
if (multi_sock == INVALID_SOCKET)
    return SCPE_OPENERR;                                /* open error */
sim_printf ("Listening on multicast address %s (socket %d)\n", cp->group, multi_sock);

sim_parse_addr (cptr, NULL, 0, NULL, cp->tcp_port, sizeof (cp->tcp_port), NULL, NULL);
tcp_sock = sim_master_sock (cptr, NULL);                /* make tcp socket */
if (tcp_sock == INVALID_SOCKET)
    return SCPE_OPENERR;                                /* open error */
sim_printf ("Listening on TCP address %s (socket %d)\n", cptr, tcp_sock);

tptr = (char *) malloc (strlen (cptr) + 1);             /* get string buf */
if (tptr == NULL)                                       /* no more mem? */
    return SCPE_MEM;

cp->multi_sock = multi_sock;                            /* save master socket */
cp->tcp_sock = tcp_sock;
strcpy (tptr, cptr);                                    /* copy port */
uptr->filename = tptr;                                  /* save */
uptr->flags = uptr->flags | UNIT_ATT;
return SCPE_OK;
}

t_stat ci_detach (UNIT *uptr)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;

if (!(uptr->flags & UNIT_ATT))                          /* attached? */
    return SCPE_OK;
sim_close_sock (cp->multi_sock);                        /* close master socket */
sim_close_sock (cp->tcp_sock);                          /* close master socket */
free (uptr->filename);                                  /* free port string */
uptr->filename = NULL;
uptr->flags = uptr->flags & ~UNIT_ATT;                  /* not attached */
return SCPE_OK;
}

/* Show CI adapter parameters */

t_stat ci_show_node (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
fprintf (st, "node=%d", uptr->ci_node);
return SCPE_OK;
}

t_stat ci_show_group (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;

if (cp->group[0])
    fprintf (st, "group=%s", cp->group);
else
    fprintf (st, "no group");
return SCPE_OK;
}

t_stat ci_set_node (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
int32 r;
uint32 node;

if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
node = (uint32) get_uint (cptr, 10, CI_MAX_NODES, &r);
if (r != SCPE_OK)
    return r;
if ((node >= CI_MAX_NODES) || (node < 0))
    return SCPE_ARG;
uptr->ci_node = node;
return SCPE_OK;
}

t_stat ci_set_group (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
char host[CBUFSIZE], port[CBUFSIZE];
CI_PORT *cp = (CI_PORT *)uptr->port_ctx;

if ((!cptr) || (!*cptr))
    return SCPE_ARG;
if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
if (sim_parse_addr (cptr, host, sizeof(host), NULL, port, sizeof(port), NULL, NULL))
    return SCPE_ARG;
if (host[0] == '\0')
    return SCPE_ARG;
strncpy (cp->group, cptr, CBUFSIZE-1);
return SCPE_OK;
}
