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
#define LINK_OPEN(p)    (ci_nodes[p].state > LINK_CLOSED) /* link open */
#define DG_INHIBIT(x)   (x & VCD_IDG)

/* Network packet header */

#define HDR_LENGTH      0                               /* packet length */
#define HDR_SOURCE      2                               /* source CI port */
#define HDR_PRIORITY    4                               /* TCP connection priority */
#define HDR_VCPORT      6                               /* virtual circuit TCP port */

/* CI Link States */

#define LINK_CLOSED     0                               /* link closed */
#define LINK_LOCAL      1                               /* link local */
#define LINK_WAIT       2                               /* link opening */
#define LINK_REMOTE     3                               /* link open */

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
    SOCKET    socket;                                   /* TCP Socket of this VC */
    char      host[CBUFSIZE];                           /* IP address of remote system */
    int32     pri;                                      /* Priority */
    uint32    vcd_val[CI_MAX_NODES];                    /* VCD table */
    uint32    conn;                                     /* link state */
    uint32    state;                                    /* port state */
} CI_NODE;

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

char ci_group[CBUFSIZE];                                /* multicast group address */
char ci_tcp_port[CBUFSIZE];                             /* VC port */
SOCKET ci_multi_sock = 0;                               /* multicast socket */
SOCKET ci_tcp_sock = 0;                                 /* VC socket */
SOCKET ci_wait_sock[CI_MAX_NODES];                      /* pending connections */
CI_NODE ci_nodes[CI_MAX_NODES];
int32 ci_pri;                                           /* connection priority */
uint32 ci_state;                                        /* port state */
CI_QUE ci_rx_queue;                                     /* packet receive queue */
CI_QUE ci_tx_queue;                                     /* packet transmit queue */
CI_PKT rcv_pkt;                                         /* receive packet buffer */

extern int32 tmxr_poll;

t_stat ci_snddg (CI_PKT *pkt);
t_stat ci_sndmsg (CI_PKT *pkt);
t_stat ci_reqid (CI_PKT *pkt);
t_stat ci_reqdat (CI_PKT *pkt);
t_stat ci_retid (CI_PKT *pkt);
t_stat ci_sndlb (CI_PKT *pkt);
t_stat ci_snddat (CI_PKT *pkt);
t_stat ci_retdat (CI_PKT *pkt);
t_stat ci_retcnf (CI_PKT *pkt);
t_stat ci_invtc (CI_PKT *pkt);
t_stat ci_setckt (CI_PKT *pkt);
t_stat ci_rdcnt (CI_PKT *pkt);
t_stat ci_dgrec (CI_PKT *pkt);
t_stat ci_msgrec (CI_PKT *pkt);
t_stat ci_reqrec (CI_PKT *pkt);
t_stat ci_reqdatrec (CI_PKT *pkt);
t_stat ci_idrec (CI_PKT *pkt);
t_stat ci_datrec (CI_PKT *pkt);
t_stat ci_snddatrec (CI_PKT *pkt);
t_stat ci_cnfrec (CI_PKT *pkt);
t_stat ci_lbrec (CI_PKT *pkt);
t_stat ci_sndrst (CI_PKT *pkt);
t_stat ci_sndstrt (CI_PKT *pkt);
t_stat ci_rstrec (CI_PKT *pkt);
t_stat ci_strtrec (CI_PKT *pkt);
t_stat ci_open_vc (uint8 port);
t_stat ci_close_vc (uint8 port);
t_stat ci_send_packet (CI_PKT *pkt);
t_stat ci_receive_packet (CI_PKT *pkt);
void ci_fail_vc (uint8 port);
t_stat ciq_init (CI_QUE *que, int32 max);               /* initialize FIFO queue */
void ciq_clear  (CI_QUE *que);                          /* clear FIFO queue */
void ciq_remove (CI_QUE *que);                          /* remove item from FIFO queue */
void ciq_insert (CI_QUE *que, uint8 port, CI_PKT *pkt); /* insert item into FIFO queue */

/* Adapter specific functions */

extern t_stat ci_receive (CI_PKT *pkt);
extern t_stat ci_respond (CI_PKT *pkt);
extern t_stat ci_dispose (CI_PKT *pkt);
extern t_stat ci_send_data (CI_PKT *pkt);
extern t_stat ci_receive_data (CI_PKT *pkt);
extern t_bool ci_can_receive ();

void dump_packet (char* buffer, uint32 length)
{
uint8 tbyte;
int32 i, j;

for (i = 0; i < length; i+=4) {
    sim_debug (DBG_SCSMSG, &ci_dev, "  %04X: ", i);
//    sim_debug (DBG_SCSMSG, ci_dev, "  %s: ", prefix);
    sim_debug (DBG_SCSMSG, &ci_dev, "  ");
    for (j = 3; j >= 0; j--) {
        if ((i + j) < length) {
            tbyte = buffer[i+j];
            sim_debug (DBG_SCSMSG, &ci_dev, "%02X ", tbyte);
            }
        else
            sim_debug (DBG_SCSMSG, &ci_dev, "   ", tbyte);
        }
    sim_debug (DBG_SCSMSG, &ci_dev, " ");
    for (j = 0; j < 4; j++) {
        if ((i + j) < length) {
            tbyte = buffer[i+j];
            if ((tbyte > 0x1F) && (tbyte < 0x7F))
                sim_debug (DBG_SCSMSG, &ci_dev, "%c", tbyte);
            else
                sim_debug (DBG_SCSMSG, &ci_dev, ".");
            }
        else
            sim_debug (DBG_SCSMSG, &ci_dev, " ");
        }
    sim_debug (DBG_SCSMSG, &ci_dev, "\n");
    }
}

/* CI port state change */

void ci_set_state (uint32 state)
{
int32 r;
UNIT *uptr = &ci_dev.units[0];

if (state < PORT_INIT) {                                /* change to uninit */
    if (ci_state >= PORT_INIT) {                        /* currently initialised? */
        sim_cancel (uptr);                              /* stop poll */
        sim_debug (DBG_CONN, &ci_dev, "deactivating unit\n");
        ciq_clear (&ci_rx_queue);                       /* clear queue */
        do r = sim_read_sock (ci_multi_sock, rcv_pkt.data, CI_MAXFR);
        while (r > 0);                                  /* clear network buffer */
        }
    }
else if (state >= PORT_INIT) {                          /* change to init */
    if (ci_state < PORT_INIT) {                         /* currently uninit? */
        sim_clock_coschedule (uptr, tmxr_poll);         /* start poll */
        sim_debug (DBG_CONN, &ci_dev, "activating unit\n");
        }
   }
ci_state = state;                                       /* update state */
}

/* Local port commands */

t_stat ci_local_ppd (CI_PKT *pkt)
{
uint32 opcode = pkt->data[PPD_OPC];
t_stat r;

switch (opcode) {

    case OPC_INVTC:
        return ci_invtc (pkt);

    case OPC_SETCKT:
        return ci_setckt (pkt);

    case OPC_RDCNT:
        return ci_rdcnt (pkt);

    default:
        sim_printf ("CI: unimplemented opcode (send) %02X\n", opcode);
        }
return SCPE_IOERR; // TODO: need local status
}

t_stat ci_send_ppd (CI_PKT *pkt)
{
uint32 opcode = pkt->data[PPD_OPC];
t_stat r;

switch (opcode) {

    case OPC_SNDDG:
        r = ci_snddg (pkt);
        break;

    case OPC_SNDMSG:
        r = ci_sndmsg (pkt);
        break;

    case OPC_RETCNF:
        r = ci_retcnf (pkt);
        break;

    case OPC_REQID:
        r = ci_reqid (pkt);
        break;

    case OPC_SNDRST:
        r = ci_sndrst (pkt);
        break;

    case OPC_SNDSTRT:
        r = ci_sndstrt (pkt);
        break;

    case OPC_REQDAT:
        r = ci_reqdat (pkt);
        break;

    case OPC_RETID:
        r = ci_retid (pkt);
        break;

    case OPC_SNDLB:
        r = ci_sndlb (pkt);
        break;

    case OPC_SNDDAT:
        r = ci_snddat (pkt);
        break;

    case OPC_RETDAT:
        r = ci_retdat (pkt);
        break;

    default:
        return ci_local_ppd (pkt);                      /* check for local PPD */
        }
if (r == SCPE_OK)
    r = ci_send_packet (pkt);
return r;
}

t_stat ci_receive_ppd (CI_PKT *pkt)
{
uint32 opcode;
t_stat r;

r = ci_receive_packet (pkt);
if (r != SCPE_OK)
    return r;
opcode = pkt->data[PPD_OPC];

switch (opcode) {

    case OPC_DGREC:
        return ci_dgrec (pkt);

    case OPC_MSGREC:
        return ci_msgrec (pkt);

    case OPC_REQREC:
        return ci_reqrec (pkt);

    case OPC_RSTREC:
        return ci_rstrec (pkt);

    case OPC_STRTREC:
        return ci_strtrec (pkt);

    case OPC_REQDATREC:
        return ci_reqdatrec (pkt);

    case OPC_IDREC:
        return ci_idrec (pkt);

    case OPC_DATREC:
        return ci_datrec (pkt);

    case OPC_SNDDATREC:
        return ci_snddatrec (pkt);

    case OPC_CNFREC:
        return ci_cnfrec (pkt);

    case OPC_LBREC:
        return ci_lbrec (pkt);

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
t_stat ci_snddg (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
uint16 dg_type = CI_GET16 (pkt->data, PPD_MTYPE);
t_stat r;

switch (dg_type) {

    case 0:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - START, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 1:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - STACK dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 2:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - ACK, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 3:
        sim_debug (DBG_SCSDG, &ci_dev, "==> SNDDG - SCSDG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 4:
        sim_debug (DBG_WRN, &ci_dev, "==> SNDDG - SCSMSG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 5:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - ELOG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 6:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - HOSTSHUT, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 7:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - FU_DG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    default:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDDG - %X, dest: %d, path: %s\n", dg_type, port, ci_path_names[path]);
        break;
        }

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
t_stat ci_sndmsg (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
uint32 msg_size = CI_GET16 (pkt->data, PPD_LENGTH) + PPD_MSGHDR;
uint16 dg_type = CI_GET16 (pkt->data, PPD_MTYPE) & 0x7FFF;
uint16 msg_type;

switch (dg_type) {

    case 0:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - START, dest: %d, path: %s\n", port, ci_path_names[path]);
        //
        // The following code may be related to the VC failure mechanism.
        // Maybe put this in ci_send_packet when VC is not open?
        //
        sim_debug (DBG_PPDDG, &ci_dev, "<== SNDMSG - START, No Path\n");
        pkt->data[PPD_STATUS] = 0xA1;                   /* Status: No Path */
        ci_respond (pkt);
        return SCPE_OK;
        break;

    case 1:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - STACK dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 2:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - ACK, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 3:
        sim_debug (DBG_WRN, &ci_dev, "==> SNDMSG - SCSDG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 4:
        msg_type = CI_GET16 (pkt->data, PPD_STYPE);
        sim_debug (DBG_SCSMSG, &ci_dev, "==> SCSMSG - %s, dest: %d, path: %s\n", scs_msg_types[msg_type], port, ci_path_names[path]);
        dump_packet (pkt->data, msg_size);
        break;

    case 5:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - ELOG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 6:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - HOSTSHUT, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 7:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - FU_DG, dest: %d, path: %s\n", port, ci_path_names[path]);
        break;

    default:
        sim_debug (DBG_PPDDG, &ci_dev, "==> SNDMSG - %X, dest: %d, path: %s\n", dg_type, port, ci_path_names[path]);
        break;
        }

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
t_stat ci_reqid (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_REQID, &ci_dev, "==> REQID, dest: %d, path: %s\n", port, ci_path_names[path]);
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
t_stat ci_reqdat (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_BLKTF, &ci_dev, "==> REQDAT, dest: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_STATUS] = 0;                              /* Status OK */
return SCPE_OK;
}

t_stat ci_retid (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
uint32 vcd_val = ci_nodes[pkt->port].vcd_val[port];      /* current VCD value */
sim_debug (DBG_REQID, &ci_dev, "==> RETID, dest: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_STATUS] = 0;                              /* Status OK */
if (VCD_OPEN (vcd_val))                                 /* VC open? */
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
t_stat ci_sndlb (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_LCMD, &ci_dev, "==> SNDLB, dest: %d, path: %s\n", port, ci_path_names[path]);
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
t_stat ci_snddat (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_BLKTF, &ci_dev, "==> SNDDAT, dest: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_STATUS] = 0;                              /* status OK */
return SCPE_OK;
}

t_stat ci_retdat (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_BLKTF, &ci_dev, "==> RETDAT, dest: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_STATUS] = 0;                              /* status OK */
return SCPE_OK;
}

t_stat ci_retcnf (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_BLKTF, &ci_dev, "==> RETCNF, dest: %d, path: %s\n", port, ci_path_names[path]);
return SCPE_OK;
}

t_stat ci_sndrst (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_PPDDG, &ci_dev, "==> SNDRST, dest: %d, path: %s\n", port, ci_path_names[path]);
dump_packet (pkt->data, 0x10);
return SCPE_OK;
}

t_stat ci_sndstrt (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_PPDDG, &ci_dev, "==> SNDSTRT, dest: %d, path: %s\n", port, ci_path_names[path]);
dump_packet (pkt->data, 0x10);
return SCPE_OK;
}

t_stat ci_invtc (CI_PKT *pkt)
{
sim_debug (DBG_LCMD, &ci_dev, "==> INVTC\n");
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
t_stat ci_setckt (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 vcd_mmsk = CI_GET32 (pkt->data, PPD_VCMMSK);
uint32 vcd_mval = CI_GET32 (pkt->data, PPD_VCMVAL);
uint32 vcd_val = ci_nodes[pkt->port].vcd_val[port];      /* current VCD value */
uint32 vcd_nval = vcd_val;                               /* new VCD value */
t_stat r;

sim_debug (DBG_LCMD, &ci_dev, "==> SETCKT, port: %d, mask: %X value: %X\n", port, vcd_mmsk, vcd_mval);

vcd_nval &= ~vcd_mmsk;
vcd_nval |= vcd_mval;

if (VCD_OPEN (vcd_nval) && !VCD_OPEN (vcd_val)) {       /* Opening VC */
    r = ci_open_vc (port);
    if (r != SCPE_OK) {
        sim_debug (DBG_CONN, &ci_dev, "Unable to establish VC to node %d\n", port);
        ci_dispose (pkt);
        return SCPE_EOF; // TODO: need local status codes
        }
    sim_debug (DBG_LCMD, &ci_dev, "<== SETCKT, VC Open Complete\n");
    }
else if (!VCD_OPEN (vcd_nval) && VCD_OPEN (vcd_val)) {  /* Closing VC */
    r = ci_close_vc (port);
    sim_debug (DBG_LCMD, &ci_dev, "<== SETCKT, VC Close Complete\n");
    }

CI_PUT32 (pkt->data, PPD_VCPVAL, vcd_val);
pkt->data[PPD_STATUS] = 0;                              /* status OK */
ci_nodes[pkt->port].vcd_val[port] = vcd_nval;           /* set new VCD value */
return r;
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
t_stat ci_dgrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
uint32 dg_type = pkt->data[PPD_MTYPE];

switch (dg_type) {

    case 0:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - START, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 1:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - STACK, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 2:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - ACK, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 3:
        sim_debug (DBG_SCSDG, &ci_dev, "<== DGREC - SCSDG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 4:
        sim_debug (DBG_WRN, &ci_dev, "<== DGREC - SCSMSG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 5:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - ELOG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 6:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - HOSTSHUT, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 7:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - FU_DG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    default:
        sim_debug (DBG_PPDDG, &ci_dev, "<== DGREC - %X, src: %d, path: %s\n", dg_type, port, ci_path_names[path]);
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
t_stat ci_msgrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
uint32 msg_size = CI_GET16 (pkt->data, PPD_LENGTH) + PPD_MSGHDR;
uint32 dg_type = pkt->data[PPD_MTYPE];
uint32 msg_type;

switch (dg_type) {

    case 0:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - START, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 1:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - STACK, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 2:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - ACK, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 3:
        sim_debug (DBG_WRN, &ci_dev, "<== MSGREC - SCSDG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 4:
        msg_type = pkt->data[PPD_STYPE];
        sim_debug (DBG_SCSMSG, &ci_dev, "<== SCSMSG - %s, src: %d, path: %s\n", scs_msg_types[msg_type], port, ci_path_names[path]);
        dump_packet (pkt->data, msg_size);
        break;

    case 5:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - ELOG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 6:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - HOSTSHUT, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    case 7:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - FU_DG, src: %d, path: %s\n", port, ci_path_names[path]);
        break;

    default:
        sim_debug (DBG_PPDDG, &ci_dev, "<== MSGREC - %X, src: %d, path: %s\n", dg_type, port, ci_path_names[path]);
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
t_stat ci_reqrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_REQID, &ci_dev, "<== REQREC, src: %d, path: %s\n", port, ci_path_names[path]);
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
t_stat ci_reqdatrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_BLKTF, &ci_dev, "<== REQDATREC, src: %d, path: %s\n", port, ci_path_names[path]);
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
t_stat ci_idrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_REQID, &ci_dev, "<== IDREC, src: %d, path: %s\n", port, ci_path_names[path]);
return SCPE_OK;
}

t_stat ci_datrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_BLKTF, &ci_dev, "<== DATREC, src: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
}

t_stat ci_snddatrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_BLKTF, &ci_dev, "<== SNDDATREC, src: %d, path: %s\n", port, ci_path_names[path]);
pkt->data[PPD_TYPE] = DYN_SCSMSG;
return SCPE_OK;
}

t_stat ci_cnfrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_BLKTF, &ci_dev, "<== CNFREC, src: %d, path: %s\n", port, ci_path_names[path]);
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
t_stat ci_lbrec (CI_PKT *pkt)
{
CI_NODE *node = &ci_nodes[pkt->port];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_LCMD, &ci_dev, "<== LBREC, path: %s\n", ci_path_names[path]);
node->vcd_val[pkt->port] |= (VCD_PAG | VCD_PBG);        /* path A and B good */
return SCPE_OK;
}

t_stat ci_rstrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_PPDDG, &ci_dev, "<== RSTREC, src: %d, path: %s\n", port, ci_path_names[path]);
dump_packet (pkt->data, 0x10);
return SCPE_OK;
}

t_stat ci_strtrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 path = GET_PATH (pkt->data[PPD_FLAGS]);
sim_debug (DBG_PPDDG, &ci_dev, "<== STRTREC, src: %d, path: %s\n", port, ci_path_names[path]);
dump_packet (pkt->data, 0x10);
return SCPE_OK;
}

/* Open a link to a remote node */

t_stat ci_open_vc (uint8 port)
{
CI_NODE *node = &ci_nodes[port];
SOCKET newsock;

if (node->conn == LINK_LOCAL)                           /* local port? */
    return SCPE_OK;                                     /* yes, done */
if (LINK_OPEN (port)) {                                 /* already open? */
    node->conn++;                                       /* increment usage */
    return SCPE_OK;                                     /* yes, done */
    }
if (ci_pri < node->pri) {                               /* are we the server? */
    node->conn = LINK_WAIT;
    return SCPE_OK;                                     /* yes, done */
    }
sim_debug (DBG_CONN, &ci_dev, "Connecting to node %d at %s...\n", port, node->host);
newsock = sim_connect_sock (node->host, NULL, NULL);
if (newsock == INVALID_SOCKET) {
    sim_debug (DBG_CONN, &ci_dev, "Unable to establish VC to node %d\n", port);
    return SCPE_OPENERR;
    }
sim_debug (DBG_CONN, &ci_dev, "Connected to node %d, socket %d\n", port, newsock);
node->socket = newsock;
node->conn = LINK_REMOTE;
return SCPE_OK;
}

/* Close a link to a remote node */

t_stat ci_close_vc (uint8 port)
{
CI_NODE *node = &ci_nodes[port];

if (!LINK_OPEN (port))                                  /* already closed? */
    return SCPE_OK;                                     /* yes, done */
if (node->conn == LINK_LOCAL)                           /* local port? */
    return SCPE_OK;                                     /* yes, done */
node->conn--;
if (LINK_OPEN (port))                                   /* still in use? */
    return SCPE_OK;                                     /* yes, done */
sim_debug (DBG_CONN, &ci_dev, "Closing VC with node %d...\n", port);
sim_close_sock (node->socket);
node->socket = 0;
node->conn = LINK_CLOSED;
return SCPE_OK;
}

t_stat ci_check_vc (uint8 lport, uint8 rport)
{
uint32 vcd_val = ci_nodes[lport].vcd_val[rport];        /* current VCD value */
return VCD_OPEN (vcd_val) ? TRUE : FALSE;
}

/* Notifies the operating system that the connection with the *
 * remote node has been lost by inserting a START datagram on *
 * the response queue in order to trigger VC failure          */

void ci_fail_vc (uint8 port)
{
// TODO: Should message be HOSTSHUT rather than START?
CI_PKT pkt;
CI_NODE *node;
uint32 i;

ci_close_vc (port);                                     /* close the VC */
// TODO: Work out which nodes have an open VC
for (i = 0; i < CI_MAX_NODES; i++) {
    node = &ci_nodes[i];
    if (node->conn == LINK_LOCAL)                       /* local port? */
    {
        if (VCD_OPEN (node->vcd_val[port])) {           /* with open VC to failed node? */
            node->vcd_val[port] &= ~VCD_CST;            /* update descriptor */
            pkt.data[PPD_PORT] = port;                  /* port number */
            pkt.data[PPD_STATUS] = 0;                   /* status: OK */
            pkt.data[PPD_OPC] = OPC_DGREC;              /* opcode */
            pkt.data[PPD_FLAGS] = 0;                    /* flags */
            pkt.data[PPD_LENGTH] = 0;                   /* message length */
            pkt.data[PPD_MTYPE] = 0;                    /* message type: START */
            pkt.addr = 0;
            pkt.length = 0x14; // TODO: Need #define for this
            ciq_insert (&ci_rx_queue, i, &pkt);         /* put to response queue */
            }
        }
    }
}

t_stat ci_send_packet (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 opcode = pkt->data[PPD_OPC];
CI_NODE *node = &ci_nodes[port];
CI_PKT tpkt;
int32 r;

memcpy (&tpkt, pkt, sizeof (CI_PKT));                   /* make local copy */

CI_PUT16 (tpkt.data, HDR_LENGTH, tpkt.length);
tpkt.data[HDR_SOURCE] = pkt->port;                      /* local port number */
CI_PUT16 (tpkt.data, HDR_PRIORITY, ci_pri);             /* priority */
strncpy (&tpkt.data[HDR_VCPORT], ci_tcp_port, 5);       /* VC port */

if (node->conn == LINK_LOCAL) {                         /* loopback? */
    sim_debug (DBG_CONN, &ci_dev, "packet destination is local node\n");
    ciq_insert (&ci_rx_queue, port, &tpkt);             /* add to queue */
    return SCPE_OK;
    }

if ((LINK_OPEN (port)) && (opcode != OPC_REQID) && (opcode != OPC_RETID)) {
    if (ci_tx_queue.count == 0) {
        r = sim_write_sock (node->socket, tpkt.data, CI_MAXFR); /* send packet */
        if (r < 0)
            ciq_insert (&ci_tx_queue, port, &tpkt);
        }
    else
        ciq_insert (&ci_tx_queue, port, &tpkt);
    }
else
    r = sim_write_sock_ex (ci_multi_sock, tpkt.data, tpkt.length, ci_group, SIM_SOCK_OPT_DATAGRAM); /* send packet */

return SCPE_OK;
}

t_stat ci_receive_packet (CI_PKT *pkt)
{
uint32 i;
uint32 path;
uint8  src_ci_port;
uint8  dest_ci_port;
t_stat r;
CI_ITEM *item;
char *src_addr;

if (ci_rx_queue.count == 0) {                           /* receive queue empty? */
    if (ci_state >= PORT_ENABLED) {                     /* check multicast channel for any REQID packets */
        do {
            if (ci_rx_queue.count == CI_QUE_MAX)
                break;
            r = sim_read_sock_ex (ci_multi_sock, rcv_pkt.data, CI_MAXFR, &src_addr);
            if (r > 0) {
                src_ci_port = rcv_pkt.data[HDR_SOURCE];
                dest_ci_port = rcv_pkt.data[PPD_PORT];
                if ((ci_nodes[dest_ci_port].conn == LINK_LOCAL) && (ci_nodes[src_ci_port].conn != LINK_LOCAL)) {
                    sprintf (ci_nodes[src_ci_port].host, "[%s]:%.5s", src_addr, &rcv_pkt.data[HDR_VCPORT]);
                    ci_nodes[src_ci_port].pri = CI_GET16 (rcv_pkt.data, HDR_PRIORITY);
                    rcv_pkt.length = CI_GET16 (rcv_pkt.data, HDR_LENGTH);
                    ciq_insert (&ci_rx_queue, dest_ci_port, &rcv_pkt);
                    }
                free (src_addr);
                }
            if (r < 0) {
                sim_printf ("CI: multicast socket read error\n");
                return SCPE_IOERR;
                }
            } while (r > 0);
        }

    for (i = 0; i < CI_MAX_NODES; i++) {                /* poll open VCs */
        if (ci_rx_queue.count == CI_QUE_MAX)            /* queue full? */
            break;                                      /* yes, don't receive any more */
        if (ci_nodes[i].conn == LINK_LOCAL)             /* local node? */
            continue;                                   /* yes, skip */
        if (ci_nodes[i].socket != 0) {                  /* connection established? */
            do {
                if (ci_rx_queue.count == CI_QUE_MAX)    /* queue full? */
                    break;                              /* yes, don't receive any more */
                r = sim_read_sock (ci_nodes[i].socket, rcv_pkt.data, CI_MAXFR);
                rcv_pkt.length = CI_GET16 (rcv_pkt.data, 0);
                dest_ci_port = rcv_pkt.data[PPD_PORT];
                if (r > 0) ciq_insert (&ci_rx_queue, dest_ci_port, &rcv_pkt);
                if (r < 0) {
                    sim_debug (DBG_CONN, &ci_dev, "Node %d closed VC, socket: %d\n", i, ci_nodes[i].socket);
                    ci_fail_vc (i);                     /* notify VC failure */
                    }
                } while (r > 0);
            }
        }
    }
if (ci_rx_queue.count > 0) {                            /* process receive queue */
    if (ci_rx_queue.loss > 0)
        sim_printf ("Warning: CI queue packet loss is %d\n", ci_rx_queue.loss);
    item = &ci_rx_queue.item[ci_rx_queue.head];
    if (item->port != pkt->port)                        /* not for this node? */
        return SCPE_EOF;
    memcpy (pkt, &item->pkt, sizeof (CI_PKT));          /* copy to output packet */
    pkt->port = item->port;                             /* set destination port */
    path = GET_PATH (pkt->data[PPD_FLAGS]);
    if (path == PPD_PSAUTO)                             /* auto select path? */
        path = PPD_PS1;                                 /* default to path 1 */
    pkt->data[PPD_OPC] |= OPC_M_RECV;                   /* convert to rcv opcode */
    pkt->data[PPD_PORT] = pkt->data[HDR_SOURCE];        /* set source port */
    pkt->data[PPD_FLAGS] |= (path << PPD_V_SP);         /* set send path */
    pkt->data[PPD_FLAGS] &= ~PPD_RSP;                   /* clear response bit */
    pkt->addr = 0;
    ciq_remove (&ci_rx_queue);                          /* remove processed packet from queue */
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

void ciq_insert (CI_QUE* que, uint8 port, CI_PKT *pkt)
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
item->port = port;
memcpy (&item->pkt, pkt, sizeof (CI_PKT)); // TODO: don't copy whole packet?
}

t_stat ci_svc (UNIT *uptr)
{
SOCKET newsock;
uint32 i;
uint8  src_ci_port;
t_stat r;
CI_ITEM *item;

while (ci_tx_queue.count > 0) {                         /* process transmit queue */
    if (ci_tx_queue.loss > 0)
        sim_printf ("Warning: CI queue packet loss is %d\n", ci_tx_queue.loss);
    item = &ci_tx_queue.item[ci_tx_queue.head];
    r = sim_write_sock (ci_nodes[item->port].socket, item->pkt.data, CI_MAXFR);
    if (r < 0) break;
    ciq_remove (&ci_tx_queue);                          /* remove processed packet from queue */
    }

for (i = 0; i < CI_MAX_NODES; i++) {                    /* check pending connections */
    if (ci_wait_sock[i] > 0) {
        r = sim_check_conn (ci_wait_sock[i], TRUE);     /* anything to receive? */
        if (r < 0) continue;                            /* no, continue */
        r = sim_read_sock (ci_wait_sock[i], rcv_pkt.data, CI_MAXFR);
        if (r == 0) continue;                           /* nothing received, continue */
        src_ci_port = rcv_pkt.data[HDR_SOURCE];
        if (ci_nodes[src_ci_port].socket > 0) {         /* VC already open? */
            sim_close_sock (ci_wait_sock[i]);           /* reject connection */
            ci_wait_sock[i] = 0;                        /* clear pending connection */
            }
        else {
            sim_debug (DBG_CONN, &ci_dev, "Accepting connection from node %d, socket: %d\n", src_ci_port, ci_wait_sock[i]);
            rcv_pkt.length = CI_GET16 (rcv_pkt.data, HDR_LENGTH);
            ciq_insert (&ci_rx_queue, src_ci_port, &rcv_pkt);
            ci_nodes[src_ci_port].socket = ci_wait_sock[i];
            ci_nodes[src_ci_port].conn = LINK_REMOTE;
            ci_wait_sock[i] = 0;                        /* clear pending connection */
            }
        }
    }

newsock = sim_accept_conn (ci_tcp_sock, NULL);          /* check for new VCs */
if (newsock != INVALID_SOCKET)  {
    for (i = 0; i < CI_MAX_NODES; i++) {                /* find free queue entry */
        if (ci_wait_sock[i] == 0) {
            ci_wait_sock[i] = newsock;                  /* save socket in queue */
            break;
            }
        }
    }

return SCPE_OK;
}

#if 0
t_stat ci_svc_old (UNIT *uptr)
{
SOCKET newsock;
uint32 i;
uint8  src_ci_port;
uint8  dest_ci_port;
uint32  ipa, ipp;
t_stat r;
CI_ITEM *item;
char *src_addr;

while ((ci_rx_queue.count > 0) && ci_can_receive ()) {  /* process receive queue */
    if (ci_rx_queue.loss > 0)
        sim_printf ("Warning: CI queue packet loss is %d\n", ci_rx_queue.loss);
    item = &ci_rx_queue.item[ci_rx_queue.head];
    ci_receive_packet (&item->pkt, item->port);
    ciq_remove (&ci_rx_queue);                          /* remove processed packet from queue */
    }

if ((uptr->flags & UNIT_ATT) == 0) {                    /* cables attached? */
    sim_clock_coschedule (uptr, tmxr_poll);             /* no, done */
    return SCPE_OK;
    }

while (ci_tx_queue.count > 0) {                         /* process transmit queue */
    if (ci_tx_queue.loss > 0)
        sim_printf ("Warning: CI queue packet loss is %d\n", ci_tx_queue.loss);
    item = &ci_tx_queue.item[ci_tx_queue.head];
    r = sim_write_sock (ci_vcd[item->port].socket, item->pkt.data, CI_MAXFR);
    if (r < 0) break;
    ciq_remove (&ci_tx_queue);                          /* remove processed packet from queue */
    }

if (ci_state >= PORT_ENABLED) {                         /* Check multicast channel for any REQID packets */
    do {
        if (ci_rx_queue.count == CI_QUE_MAX)
            break;
        r = sim_read_sock_ex (ci_multi_sock, rcv_pkt.data, CI_MAXFR, &src_addr);
        if (r > 0) {
            src_ci_port = rcv_pkt.data[HDR_SOURCE];
            dest_ci_port = rcv_pkt.data[PPD_PORT];
            if ((dest_ci_port == ci_node) && (src_ci_port != ci_node)) {
                sprintf (ci_vcd[src_ci_port].host, "[%s]:%.5s", src_addr, &rcv_pkt.data[HDR_VCPORT]);
                ci_vcd[src_ci_port].pri = CI_GET16 (rcv_pkt.data, HDR_PRIORITY);
                rcv_pkt.length = CI_GET16 (rcv_pkt.data, HDR_LENGTH);
                ciq_insert (&ci_rx_queue, src_ci_port, &rcv_pkt);
                }
            free (src_addr);
            }
        if (r < 0) {
            sim_printf ("CI: multicast socket read error\n");
            return SCPE_IOERR;
            }
        } while (r > 0);
    }

for (i = 0; i < CI_MAX_NODES; i++) {                    /* check pending connections */
    if (ci_wait_sock[i] > 0) {
        r = sim_check_conn (ci_wait_sock[i], TRUE);     /* anything to receive? */
        if (r < 0) continue;                            /* no, continue */
        r = sim_read_sock (ci_wait_sock[i], rcv_pkt.data, CI_MAXFR);
        if (r == 0) continue;                           /* nothing received, continue */
        src_ci_port = rcv_pkt.data[HDR_SOURCE];
        if (ci_vcd[src_ci_port].socket > 0) {           /* VC already open? */
            sim_close_sock (ci_wait_sock[i]);           /* reject connection */
            ci_wait_sock[i] = 0;                        /* clear pending connection */
            }
        else {
            sim_debug (DBG_CONN, &ci_dev, "Accepting connection from node %d, socket: %d\n", src_ci_port, ci_wait_sock[i]);
            rcv_pkt.length = CI_GET16 (rcv_pkt.data, HDR_LENGTH);
            ciq_insert (&ci_rx_queue, src_ci_port, &rcv_pkt);
            ci_vcd[src_ci_port].socket = ci_wait_sock[i];
            ci_vcd[src_ci_port].state = VCST_OPEN;
            ci_wait_sock[i] = 0;                        /* clear pending connection */
            }
        }
    }

for (i = 0; i < CI_MAX_NODES; i++) {                    /* poll open VCs */
    if (ci_rx_queue.count == CI_QUE_MAX)                /* queue full? */
        break;                                          /* yes, don't receive any more */
    if (i == ci_node)                                   /* local node? */
        continue;                                       /* yes, skip */
    if (ci_vcd[i].socket != 0) {                        /* connection established? */
        do {
            if (ci_rx_queue.count == CI_QUE_MAX)        /* queue full? */
                break;                                  /* yes, don't receive any more */
            r = sim_read_sock (ci_vcd[i].socket, rcv_pkt.data, CI_MAXFR);
            rcv_pkt.length = CI_GET16 (rcv_pkt.data, 0);
            if (r > 0) ciq_insert (&ci_rx_queue, i, &rcv_pkt);
            if (r < 0) {
                sim_debug (DBG_CONN, &ci_dev, "Node %d closed VC, socket: %d\n", i, ci_vcd[i].socket);
                ci_fail_vc (i);                         /* notify VC failure */
                }
            } while (r > 0);
        }
    }

while ((ci_rx_queue.count > 0) && ci_can_receive ()) {  /* process receive queue */
    if (ci_rx_queue.loss > 0)
        sim_printf ("Warning: CI queue packet loss is %d\n", ci_rx_queue.loss);
    item = &ci_rx_queue.item[ci_rx_queue.head];
    ci_receive_packet (&item->pkt, item->port);
    ciq_remove (&ci_rx_queue);                          /* remove processed packet from queue */
}

newsock = sim_accept_conn (ci_tcp_sock, NULL);          /* check for new VCs */
if (newsock != INVALID_SOCKET)  {
    for (i = 0; i < CI_MAX_NODES; i++) {                /* find free queue entry */
        if (ci_wait_sock[i] == 0) {
            ci_wait_sock[i] = newsock;                  /* save socket in queue */
            break;
            }
        }
    }

sim_clock_coschedule (uptr, tmxr_poll);
return SCPE_OK;
}
#endif

/* Reset CI adapter */

t_stat ci_port_reset (DEVICE *dptr)
{
int32 i, j;
t_stat r;
UNIT *uptr = &ci_dev.units[0];

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
    if (ci_nodes[i].socket != 0) {                      /* close open VCs */
        sim_close_sock (ci_nodes[i].socket);
        ci_nodes[i].socket = 0;
        }
    if (ci_wait_sock[i] != 0) {                         /* close pending VCs */
        sim_close_sock (ci_wait_sock[i]);
        ci_wait_sock[i] = 0;
        }
    for (j = 0; j < CI_MAX_NODES; j++)
        ci_nodes[i].vcd_val[j] = 0;                     /* clear VCD table */
    ci_nodes[i].host[0] = '\0';
    if (ci_nodes[i].conn != LINK_LOCAL)
        ci_nodes[i].conn = LINK_CLOSED;
    }

ci_nodes[uptr->CI_NODE].conn = LINK_LOCAL;
srand (time (NULL));
ci_pri = (rand() % 100);                                /* generate priority */

sim_cancel (uptr);                                      /* stop poll */
return SCPE_OK;
}

t_stat ci_attach (UNIT *uptr, CONST char *cptr)
{
char* tptr;
t_stat r;
SOCKET multi_sock, tcp_sock;

if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
if (ci_group[0] == '\0') {
    sim_printf ("Must set group address first\n");
    return SCPE_ARG;
    }

multi_sock = sim_master_sock_ex (ci_group, NULL,
    (SIM_SOCK_OPT_REUSEADDR | SIM_SOCK_OPT_MULTICAST)); /* make multicast socket */
if (multi_sock == INVALID_SOCKET)
    return SCPE_OPENERR;                                /* open error */
sim_printf ("Listening on multicast address %s (socket %d)\n", ci_group, multi_sock);

sim_parse_addr (cptr, NULL, 0, NULL, ci_tcp_port, sizeof (ci_tcp_port), NULL, NULL);
tcp_sock = sim_master_sock (cptr, NULL);                /* make tcp socket */
if (tcp_sock == INVALID_SOCKET)
    return SCPE_OPENERR;                                /* open error */
sim_printf ("Listening on TCP address %s (socket %d)\n", cptr, tcp_sock);

tptr = (char *) malloc (strlen (cptr) + 1);             /* get string buf */
if (tptr == NULL)                                       /* no more mem? */
    return SCPE_MEM;

ci_multi_sock = multi_sock;                             /* save master socket */
ci_tcp_sock = tcp_sock;
strcpy (tptr, cptr);                                    /* copy port */
uptr->filename = tptr;                                  /* save */
uptr->flags = uptr->flags | UNIT_ATT;
return SCPE_OK;
}

t_stat ci_detach (UNIT *uptr)
{
if (!(uptr->flags & UNIT_ATT))                          /* attached? */
    return SCPE_OK;
sim_close_sock (ci_multi_sock);                         /* close master socket */
sim_close_sock (ci_tcp_sock);                           /* close master socket */
free (uptr->filename);                                  /* free port string */
uptr->filename = NULL;
uptr->flags = uptr->flags & ~UNIT_ATT;                  /* not attached */
return SCPE_OK;
}

/* Show CI adapter parameters */

t_stat ci_show_node (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
fprintf (st, "node=%d", uptr->CI_NODE);
return SCPE_OK;
}

t_stat ci_show_group (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
if (ci_group[0])
    fprintf (st, "group=%s", ci_group);
else
    fprintf (st, "no group");
return SCPE_OK;
}

t_stat ci_set_node (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
int32 r;
uint32 newnode;

if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
newnode = (uint32) get_uint (cptr, 10, CI_MAX_NODES, &r);
if (r != SCPE_OK)
    return r;
if ((newnode > 15) || (newnode < 0))
    return SCPE_ARG;
ci_nodes[uptr->CI_NODE].conn = LINK_CLOSED;
ci_nodes[newnode].conn = LINK_LOCAL;
uptr->CI_NODE = newnode;
return SCPE_OK;
}

t_stat ci_set_group (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
char host[CBUFSIZE], port[CBUFSIZE];

if ((!cptr) || (!*cptr))
    return SCPE_ARG;
if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
if (sim_parse_addr (cptr, host, sizeof(host), NULL, port, sizeof(port), NULL, NULL))
    return SCPE_ARG;
if (host[0] == '\0')
    return SCPE_ARG;
strncpy(ci_group, cptr, CBUFSIZE-1);
return SCPE_OK;
}
