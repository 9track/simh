/* vax_ciqba.c: QBus Computer Interconnect adapter

   Copyright (c) 2019, Matt Burke

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

   ci             CIQBA adapter
*/

#include "vax_defs.h"
#include "vax_ci.h"
#include "ciqba_pra0_bin.h"

/* Register offsets */

#define CI_CONTROL_REG       0x0000
#define CI_STATUS_REG        0x0001
#define CI_INTERRUPT_VEC     0x0002
#define CI_FIRMWARE_VER      0x0003
#define CI_PORT_NUM          0x0004
#define CI_MEM_ADDR_LO       0x0005
#define CI_MEM_ADDR_HI       0x0006
#define CI_MEM_DATA          0x0007
#define CI_SEND_HI_RING      0x0008
#define CI_SEND_LO_RING      0x0010
#define CI_DISPOSE_RING      0x0020
#define CI_RECEIVE_RING      0x0030

/* Control register */

#define CCR_SEND_HI_ATTN     0x0001
#define CCR_SEND_LO_ATTN     0x0002
#define CCR_DISPOSE_AVAIL    0x0004
#define CCR_RECEIVE_AVAIL    0x0008
#define CCR_INITIALIZE       0x0010
#define CCR_START            0x0020
#define CCR_RUNDOWN          0x0040
#define CCR_HALT             0x0080
#define CCR_ENABLE_INT       0x0100
#define CCR_RESET_STATUS     0x0200
#define CCR_SOFT_RESET       0x1000
#define CCR_WRITE_EEPROM     0x2000
#define CCR_WRITE_MEMORY     0x4000
#define CCR_READ_MEMORY      0x8000

/* Status register */

#define CSR_SEND_HI_AVAIL    0x0001
#define CSR_SEND_LO_AVAIL    0x0002
#define CSR_DISPOSE_ATTN     0x0004
#define CSR_RECEIVE_ATTN     0x0008
#define CSR_ATTN             0x000F
#define CSR_INITIALIZED      0x0010
#define CSR_RUNNING          0x0020
#define CSR_DMA_TIMEOUT      0x0040
#define CSR_CI_TX_TIMEOUT    0x0080
#define CSR_DISPOSE_ERROR    0x0100
#define CSR_CI_OUT_ERROR     0x0200
#define CSR_CI_IN_ERROR      0x0400
#define CSR_CABLE_CROSS      0x0800
#define CSR_RING_STALL       0x1000
#define CSR_QBUS_TIMEOUT     0x2000
#define CSR_NO_INTERRUPT     0x4000
#define CSR_ERROR            0x8000

#define CIQBA_FWVER          1                          /* firmware version */

#define CIS_DATA             0x7FFF
#define CIS_OP_COMPLETE      0x80000000

#define CIS_RECEIVE_RING_ENTRIES     8
#define CIS_DISPOSE_RING_ENTRIES     8
#define CIS_SEND_LO_RING_ENTRIES     8
#define CIS_SEND_HI_RING_ENTRIES     4

#define VTB_NVR_BASE         0x178000
#define VTB_NVR_SIZE         0x200                      /* NVR size */

/* Ring entry */

#define RENT_ADDR_LO         0x0000FFFF
#define RENT_ADDR_HI         0x003F0000
#define RENT_ADDR            0x003FFFFF
#define RENT_XMIT_FAIL       0x00400000
#define RENT_V_LENGTH        22
#define RENT_LENGTH          0x3FC00000
#define RENT_PACKET_TYPE     0x40000000
#define RENT_CIQBA_OWNED     0x80000000

/* CIQBA PPD format - receive */

#define CIPPD_LEN_HI    0x0B                            /* packet size high byte */
#define CIPPD_LEN_LO    0x0C                            /* packet size low byte */
#define CIPPD_DEST      0x0D                            /* destination node */
#define CIPPD_NOT_DEST  0x0E                            /* destination node (1s complement) */
#define CIPPD_SRC       0x0F                            /* source node */
#define CIPPD_OPC       0x10                            /* opcode */
#define CIPPD_SEQ_CNTRL 0x11                            /* sequence control */

/* CIQBA PPD format - internal */

#define CIPPD_ADDR      0x12                            /* QBus address */
#define CIPPD_LENGTH    0x16                            /* map length */

/* CIQBA PPD format - block transfers */

#define CIPPD_NAME      0x1A                            /* receive buffer name */
#define CIPPD_OFFSET    0x1E                            /* receive buffer offset */
#define CIPPD_Q22HAN    0x22                            /* ? */
#define CIPPD_BCNT      0x26                            /* byte count? */
#define CIPPD_TLOFF     0x2A                            /* xfer local offset? */
#define CIPPD_SVAPTE    0x2F                            /* system VA page table entry */
#define CIPPD_BLEFT     0x33                            /* bytes left */
#define CIPPD_SMAPR     0x37                            /* start map register */
#define CIPPD_NMAPR     0x39                            /* number of map registers */

/* CIQBA PPD flags */

#define CIPPD_LAST      0x01                            /* last packet */
#define CIPPD_BLOCK     0x04                            /* block transfer */

/* CIQBA specific opcodes */

#define OPC_QUERY       0x00                            /* watchdog request */
#define OPC_INIT        0x01
#define OPC_PROMISC_EN  0x02
#define OPC_PROMISC_DIS 0x03
#define OPC_RESET       0x04                            /* toggle VC state */
#define OPC_MAP         0x05                            /* map buffer */
#define OPC_UNMAP       0x06                            /* unmap buffer */
#define OPC_QUERY_RSP   0x80                            /* watchdog response */
#define OPC_PROMISC_PKT 0x82

#define IOBA_CI              (IOPAGEBASE + 0xE00)
#define IOLN_CI              0x80

#define PQ_BASE              0x80E1D880                 /* clustering disbled */
//#define PQ_BASE              0x80E1F400                 /* clustering enabled */

#define MAX_BMAP        20                              /* max mapped buffers */

typedef struct {
    char *symbol;
    uint32 value;
} SYMTBL;

const SYMTBL pq_syms[] = {
    { "PQ$DDT", 0xC4 },
    { "T_FAKE_CONREG", 0x1A4 },
    { "P_PQUCB", 0x2A4 },
    { "R_BOUNDS_TABLE", 0x2A8 },
    { "R_GRANT_TABLE", 0x2AC },
    { "R_OFFSET_TABLE", 0x2B0 },
    { "R_OWN_TABLE", 0x2B4 },
    { "R_DISPOSE_TABLE", 0x2B8 },
    { "R_THROTTLE_TABLE", 0x2BC },
    { "B_ILLMSG_SAVED", 0x2C0 },
    { "R_ILLMSG_BUFFER", 0x2C4 },
    { "L_SNAPSHOT_VALID", 0x300 },
    { "L_LAST_ERRTYPE", 0x304 },
    { "PQ_GR_SNAPSHOT", 0x308 },
    { "PQ_L_DEBUGCHECK", 0x388 },
    { "PQ_L_CIQBA_ERRBRK", 0x38C },
    { "PQ_L_CIQBA_ERRDUMP", 0x390 },
    { "PQ__BLOCK_INFLUX", 0x431 },
    { "PQ__CAPTURE", 0x52F },
    { "PQ__CI_TO_SCS_PPD", 0x58D },
    { "PQ__CIQBA_ERROR", 0x631 },
    { "PQ__CIQBA_MESSAGE", 0x6DC },
    { "PQ__COPY", 0x85B },
    { "PQ__CREATE_PORT_STRUCT", 0x95C },
    { "PQ__DISPOSE", 0xA6E },
    { "PQ__ENABLE_RECEIVE", 0xBBD },
    { "PQ__ENINT", 0xC5A },
    { "PQ__EXTRACT_RING", 0xCD3 },
    { "PQ__FLUSH_RING", 0xDF1 },
    { "PQ__GET_PPD_SIZE", 0xE86 },
    { "PQ__HOST_OWN", 0xEB2 },
    { "PQ__INIT_PDT", 0xED7 },
    { "PQ__INIT_RING", 0xF01 },
    { "PQ__INSERT_RING", 0xF6A },
    { "PQ__INS_COMQH", 0x10F1 },
    { "PQ__INS_COMQL", 0x1135 },
    { "PQ__INS_DFREQ", 0x117A },
    { "PQ__INS_MFREQ", 0x11AB },
    { "PQ__INS_RSPQ", 0x11DC },
    { "PQ__LAST_RETDAT", 0x1219 },
    { "PQ__LOG_ERROR", 0x12AC },
    { "PQ__MEM_DUMP", 0x12D6 },
    { "PQ__NEXT_ENTRY", 0x12D7 },
    { "PQ__POKE_PORT", 0x12FA },
    { "PQ__POWERFAIL", 0x12FB },
    { "PQ__Q22_TO_SYS", 0x12FF },
    { "PQ__RECYCLE", 0x1375 },
    { "PQ__RECEIVE", 0x13AD },
    { "PQ__REG_DISP", 0x14D0 },
    { "PQ__REQDAT", 0x14FC },
    { "PQ__REQUEUE", 0x1513 },
    { "PQ__REVECTOR", 0x157F },
    { "PQ__ROUTE_PPD", 0x164A },
    { "PQ__SEND_BLOCK", 0x179E },
    { "PQ__SEND_ID", 0x17DC },
    { "PQ__SEND_SEGMENTS", 0x1853 },
    { "PQ__SEND_SETUP", 0x18EA },
    { "PQ__SHUT_ALL_VC", 0x19B8 },
    { "PQ__START_UCODE", 0x19CB },
    { "PQ__STOP_UCODE", 0x1B1B },
    { "PQ__SYS_TO_Q22", 0x1D1A },
    { "PQ__TIMER", 0x1DDE },
    { "PQ__TRACE_EVENT", 0x1EED },
    { "PQ__TRACE_PPD", 0x1F29 },
    { "PQ__UNMAP", 0x1F70 },
    { "PQ__UNMAP_PDT", 0x207F },
    { "PQ__UNWAIT", 0x2080 },
    { "PQ_CIQBA_INTERRUPT", 0x20AF },
    { "PQ_CONTROLLER_INIT", 0x2114 },
    { "PQ_MAP", 0x211B },
    { "PQ_REG_DUMP", 0x2269 },
    { "PQ_SNAPSHOT", 0x22D4 },
    { "PQ_UNIT_INIT", 0x22FF },
    { "PQ_UNIT_INIT_FORK", 0x2330 },
    { "PQ__BOGUS_PAK", 0x2715 },
    { "IRP_TRACE", 0x275C },
    { "IRP_TRACE_I", 0x2B5C },
    { "FIN_TRACE", 0x2B60 },
    { "FIN_TRACE_I", 0x2F60 },
    { "PQCL_ACCESS", 0x2F61 },
    { "PQCL_CANCEL", 0x2F62 },
    { "PQCL_DEACCESS", 0x2F78 },
    { "PQCL_READ", 0x2F79 },
    { "PQCL_SENSEMODE", 0x2FF1 },
    { "PQCL_SETMODE", 0x2FF2 },
    { "PQCL_WRITE", 0x30D5 },
    { "PQMAR_CREATE_FORK", 0x30D8 },
    { "PQMAR_CONVERT_PPD", 0x30E5 },
    { "PQMAR_ELAPSED_TIME", 0x3120 },
    { "PQMAR_WAITSRBITS", 0x3188 },
    { "PQMAR_WAITSRBITC", 0x31BC },
    { "PQMAR_INTERRUPT", 0x31F0 },
    { "PQMAR_INVALIDATE_TB", 0x324E },
    { "PQMAR_INV_Q22MAP", 0x326F },
    { "PQMAR_MAP_IRP", 0x3296 },
    { "PQMAR_MAP_IRPBYP", 0x3296 },
    { "PQMAR_MAP", 0x32A0 },
    { "PQMAR_MAPBYPASS", 0x32A0 },
    { "PQMAR_UNSTALL", 0x332F },
    { "PQMAR_RESUME_CDRP", 0x3367 },
    { "PQMAR_PORT_INTERRUPT", 0x3376 },
    { "PQMAR_WAIT", 0x3386 },
    { "PQ_CIQBA_DUMP", 0x3620 },
    { "PQ_CIQBA_READWORD", 0x368C },
    { "PQ_HOST_SHUT_PROCESSING", 0x36DB },
    { "PQ_SHUT_CIQBA", 0x3844 },
    { "PQ___PROVIDE_RECEIVE_PACKET", 0x395C },
    { "PQ_VERSION", 0x3A08 },
    { NULL, 0 }
    };

const char *ci_rgd[] = {
    "Control Register",
    "Status Register",
    "Interrupt Vector",
    "Firmware Version",
    "Port Num",
    "Mem Addr (Low)",
    "Mem Addr (High)",
    "Mem Data",
    "Send High [0L]",
    "Send High [0H]",
    "Send High [1L]",
    "Send High [1H]",
    "Send High [2L]",
    "Send High [2H]",
    "Send High [3L]",
    "Send High [3H]",
    "Send Low [0L]",
    "Send Low [0H]",
    "Send Low [1L]",
    "Send Low [1H]",
    "Send Low [2L]",
    "Send Low [2H]",
    "Send Low [3L]",
    "Send Low [3H]",
    "Send Low [4L]",
    "Send Low [4H]",
    "Send Low [5L]",
    "Send Low [5H]",
    "Send Low [6L]",
    "Send Low [6H]",
    "Send Low [7L]",
    "Send Low [7H]",
    "Dispose [0L]",
    "Dispose [0H]",
    "Dispose [1L]",
    "Dispose [1H]",
    "Dispose [2L]",
    "Dispose [2H]",
    "Dispose [3L]",
    "Dispose [3H]",
    "Dispose [4L]",
    "Dispose [4H]",
    "Dispose [5L]",
    "Dispose [5H]",
    "Dispose [6L]",
    "Dispose [6H]",
    "Dispose [7L]",
    "Dispose [7H]",
    "Receive [0L]",
    "Receive [0H]",
    "Receive [1L]",
    "Receive [1H]",
    "Receive [2L]",
    "Receive [2H]",
    "Receive [3L]",
    "Receive [3H]",
    "Receive [4L]",
    "Receive [4H]",
    "Receive [5L]",
    "Receive [5H]",
    "Receive [6L]",
    "Receive [6H]",
    "Receive [7L]",
    "Receive [7H]"
    };

typedef struct {
    uint32 addr;                                        /* QBus address */
    uint32 length;                                      /* mapped length */
    uint32 bnam;                                        /* buffer name */
} CI_MAP;

typedef struct {
    uint32 entries;
    uint16 *ring;
    uint32 ptr;
} CI_QUEUE;

#define NEW_QUEUE(n,s)      uint16 n##_d[(s * 2)]; \
                            CI_QUEUE n = { s, &n##_d[0] }

uint16 ci_ccr = 0;                                      /* command register */
uint16 ci_csr = 0;                                      /* status register */
uint32 ci_vtba = 0;                                     /* VTB address */
uint32 ci_vtbd = 0;                                     /* VTB data */
uint16 ci_nvr[0x20] = { 0 };
NEW_QUEUE (ci_rcv, CIS_RECEIVE_RING_ENTRIES);
NEW_QUEUE (ci_dsp, CIS_DISPOSE_RING_ENTRIES);
NEW_QUEUE (ci_sndl, CIS_SEND_LO_RING_ENTRIES);
NEW_QUEUE (ci_sndh, CIS_SEND_HI_RING_ENTRIES);
char ci_sym_text[30];
t_bool ci_vc_open[32];
CI_MAP ci_map[MAX_BMAP];

t_stat ci_rd (int32 *data, int32 PA, int32 access);
t_stat ci_wr (int32 data, int32 PA, int32 access);
t_stat ciqba_attach (UNIT *uptr, CONST char *cptr);
t_stat ciqba_detach (UNIT *uptr);
t_stat ciqba_svc (UNIT *uptr);
int32 ci_vtb_rd (int32 pa);
void ci_vtb_wr (int32 pa, int32 data);
uint32 ciqba_map_buffer (uint32 bnam, uint32 off);
t_stat ci_receive_int (CI_PKT *pkt, t_bool internal);
t_stat ci_svc_queue (CI_QUEUE *queue, uint32 *processed);
t_stat ci_sw_reset (DEVICE *dptr);
t_stat ci_reset (DEVICE *dptr);
int32 ci_inta (void);
char *ci_sym (uint32 addr);


/* CIQBA adapter data structures

   ci_dev       CI device descriptors
   ci_unit      CI unit
   ci_reg       CI register list
*/

DIB ci_dib = {
    IOBA_AUTO, IOLN_CI, &ci_rd, &ci_wr,
    1, IVCL (CI), 0, { &ci_inta }
    };

UNIT ci_unit[] = {
    { UDATA (&ciqba_svc, UNIT_IDLE|UNIT_ATTABLE, 0) },
    { UDATA (&ciqba_svc, UNIT_IDLE|UNIT_FIX|UNIT_ATTABLE|UNIT_BUFABLE|UNIT_MUSTBUF, VTB_NVR_SIZE) }
    };

REG ci_reg[] = {
    { NULL }
    };

MTAB ci_mod[] = {
    { MTAB_XTD|MTAB_VDV, 0, "NODE", "NODE",
      &ci_set_node, &ci_show_node },
    { MTAB_XTD|MTAB_VDV, 0, "GROUP", "GROUP",
      &ci_set_group, &ci_show_group },
    { MTAB_XTD|MTAB_VDV|MTAB_VALR, 004, "ADDRESS", "ADDRESS",
      &set_addr, &show_addr, NULL, "Bus address" },
    { MTAB_XTD|MTAB_VDV|MTAB_VALR, 0, "VECTOR", "VECTOR",
      &set_vec, &show_vec,  NULL, "Interrupt vector" },
    { 0 }
    };

DEBTAB ci_debug[] = {
    { "REG",    DBG_REG },
    { "WARN",   DBG_WRN },
    { "REQID",  DBG_REQID },
    { "SCSDG",  DBG_SCSDG },
    { "SCSMSG", DBG_SCSMSG },
    { "PPDDG",  DBG_PPDDG },
    { "BLKTF",  DBG_BLKTF },
    { "LCMD",   DBG_LCMD },
    { "CONN",   DBG_CONN },
    { "TRACE",  DBG_TRC },
    { "PKT",    DBG_PKT },
    { 0 }
    };

DEVICE ci_dev = {
    "CI", ci_unit, ci_reg, ci_mod,
    2, DEV_RDX, 20, 1, DEV_RDX, 16,
    NULL, NULL, &ci_reset,
    NULL, &ciqba_attach, &ciqba_detach,
    &ci_dib, DEV_QBUS | DEV_DEBUG | DEV_CI, 0,
    ci_debug, 0, 0
    };


t_stat ci_rd (int32 *data, int32 PA, int32 access)
{
int32 rg = (PA >> 1) &  0x3F;
DIB *dibp = (DIB *)ci_dev.ctxt;

*data = 0;
switch (rg) {

    case CI_CONTROL_REG:
        CLR_INT (CI);
        break;

    case CI_STATUS_REG:
        *data = ci_csr;
        ci_csr = ci_csr & ~CSR_ATTN;
        break;

    case CI_INTERRUPT_VEC:
        *data = (dibp->vec >> 2);
        break;

    case CI_FIRMWARE_VER:
        *data = CIQBA_FWVER;
        break;

    case CI_PORT_NUM:
        *data = ci_unit[0].ci_node;
        break;

    case CI_MEM_ADDR_LO:
        *data = ci_vtba & 0xFFFF;
        break;

    case CI_MEM_ADDR_HI:
        *data = (ci_vtba >> 16) & 0xFFFF;
        break;

    case CI_MEM_DATA:
        *data = ci_vtbd;
        break;

    default:
        if (rg >= CI_RECEIVE_RING) {
            rg = rg - CI_RECEIVE_RING;
            *data = ci_rcv.ring[rg];
//            sim_debug (DBG_REG, &ci_dev, "ci_rd: ci_rcv[%d] = %04X\n", rg, *data);
            break;
            }
        if (rg >= CI_DISPOSE_RING) {
            rg = rg - CI_DISPOSE_RING;
            *data = ci_dsp.ring[rg];
//            sim_debug (DBG_REG, &ci_dev, "ci_rd: ci_dsp[%d] = %04X\n", rg, *data);
            break;
            }
        if (rg >= CI_SEND_LO_RING) {
            rg = rg - CI_SEND_LO_RING;
            *data = ci_sndl.ring[rg];
//            sim_debug (DBG_REG, &ci_dev, "ci_rd: ci_sndl[%d] = %04X\n", rg, *data);
            break;
            }
        if (rg >= CI_SEND_HI_RING) {
            rg = rg - CI_SEND_HI_RING;
            *data = ci_sndh.ring[rg];
//            sim_debug (DBG_REG, &ci_dev, "ci_rd: ci_sndh[%d] = %04X\n", rg, *data);
            }
            break;
        }
rg = (PA >> 1) &  0x3F;
sim_debug (DBG_REG, &ci_dev, "ci_rd: %08X = %04X at %08X %s (%s)\n", PA, *data, fault_PC, ci_rgd[rg], ci_sym(fault_PC));
return SCPE_OK;
}

t_stat ci_wr (int32 data, int32 PA, int32 access)
{
int32 rg = (PA >> 1) &  0x3F;
uint32 entry, addr, length, i, j;
uint8 pkt[1024];
uint32 processed = 0;
DIB *dibp = (DIB *)ci_dev.ctxt;

sim_debug (DBG_REG, &ci_dev, "ci_wr: %08X = %04X at %08X %s (%s)\n", PA, data, fault_PC, ci_rgd[rg], ci_sym(fault_PC));
    
switch (rg) {

    case CI_CONTROL_REG:
        if (data & CCR_READ_MEMORY) {
            ci_vtbd = ci_vtb_rd (ci_vtba);
            ci_vtba |= CIS_OP_COMPLETE;
            }
        else if (data & CCR_WRITE_MEMORY) {
            ci_vtb_wr (ci_vtba, ci_vtbd);
            ci_vtba |= CIS_OP_COMPLETE;
            }
        else if (data & CCR_WRITE_EEPROM)
            ci_vtba |= CIS_OP_COMPLETE;
        else if (data & CCR_INITIALIZE) {
            ci_sw_reset (&ci_dev);
            ci_csr |= CSR_INITIALIZED;
            ci_set_state (&ci_unit[0], PORT_INIT);
            }
        else if (data & CCR_START) {
            ci_csr |= CSR_RUNNING;
            ci_set_state (&ci_unit[0], PORT_ENABLED);
            }
        else if (data & CCR_SEND_HI_ATTN) {
            sim_debug (DBG_REG, &ci_dev, "ci_wr: CCR_SEND_HI_ATTN\n");
            sim_activate_abs (&ci_unit[0], 20);
            }
        else if (data & CCR_SEND_LO_ATTN) {
            sim_debug (DBG_REG, &ci_dev, "ci_wr: CCR_SEND_LO_ATTN\n");
            sim_activate_abs (&ci_unit[0], 20);
            }
        else if (data & CCR_DISPOSE_AVAIL) {
            sim_debug (DBG_REG, &ci_dev, "ci_wr: CCR_DISPOSE_AVAIL\n");
            sim_activate_abs (&ci_unit[0], 20);
            }
        else if (data & CCR_RECEIVE_AVAIL) {
            sim_debug (DBG_REG, &ci_dev, "ci_wr: CCR_RECEIVE_AVAIL\n");
            sim_activate_abs (&ci_unit[0], 20);
            }
        else {
            // TODO: handle transition back to halted state in vax_ci.c
            if (data & CCR_RUNDOWN)
                ci_csr &= ~CSR_RUNNING;
            if (data & CCR_HALT) {
                ci_csr &= ~CSR_RUNNING;
                ci_csr &= ~CSR_INITIALIZED;
                ci_set_state (&ci_unit[0], PORT_UNINIT);
                }
            }
        ci_ccr = data & CCR_ENABLE_INT;
        break;

    case CI_STATUS_REG:
        break;

    case CI_INTERRUPT_VEC:
        dibp->vec = (data << 2);
        break;

    case CI_FIRMWARE_VER:
        break;

    case CI_PORT_NUM:
        break;

    case CI_MEM_ADDR_LO:
        ci_vtba = ci_vtba & ~0xFFFF;
        ci_vtba = ci_vtba | data;
        break;

    case CI_MEM_ADDR_HI:
        ci_vtba = ci_vtba & ~0xFFFF0000;
        ci_vtba = ci_vtba | (data << 16);
        break;

    case CI_MEM_DATA:
        ci_vtbd = data;
        break;

    default:
        if (rg >= CI_RECEIVE_RING) {
            rg = rg - CI_RECEIVE_RING;
//            sim_debug (DBG_REG, &ci_dev, "ci_wr: ci_rcv[%d] = %04X\n", rg, data);
            ci_rcv.ring[rg] = data;
            break;
            }
        if (rg >= CI_DISPOSE_RING) {
            rg = rg - CI_DISPOSE_RING;
//            sim_debug (DBG_REG, &ci_dev, "ci_wr: ci_dsp[%d] = %04X\n", rg, data);
            ci_dsp.ring[rg] = data;
            break;
            }
        if (rg >= CI_SEND_LO_RING) {
            rg = rg - CI_SEND_LO_RING;
//            sim_debug (DBG_REG, &ci_dev, "ci_wr: ci_sndl[%d] = %04X\n", rg, data);
            ci_sndl.ring[rg] = data;
            if ((rg & 1) && (data & 0x8000))            /* high word, CIQBA owned? */
                sim_activate_abs (&ci_unit[0], 20);     /* new entry to process */
            break;
            }
        if (rg >= CI_SEND_HI_RING) {
            rg = rg - CI_SEND_HI_RING;
//            sim_debug (DBG_REG, &ci_dev, "ci_wr: ci_sndh[%d] = %04X\n", rg, data);
            ci_sndh.ring[rg] = data;
            if ((rg & 1) && (data & 0x8000))            /* high word, CIQBA owned? */
                sim_activate_abs (&ci_unit[0], 20);     /* new entry to process */
            }
            break;
        }
return SCPE_OK;
}

int32 ci_vtb_rd (int32 pa)
{
int32 rg;
int32 data = 0;
uint16 *nvr = (uint16 *) ci_unit[1].filebuf;

if ((pa >= VTB_NVR_BASE) && (pa < VTB_NVR_BASE+VTB_NVR_SIZE)) { /* NovRam */
    rg = (pa >> 1) & 0xF;
    data = nvr[rg];
    }
sim_debug (DBG_REG, &ci_dev, "ci_vtb_rd: %08X = %04X at %08X\n", pa, data, fault_PC);
return data;
}

void ci_vtb_wr (int32 pa, int32 data)
{
int32 rg;
uint16 *nvr = (uint16 *) ci_unit[1].filebuf;

sim_debug (DBG_REG, &ci_dev, "ci_vtb_wr: %08X = %04X at %08X\n", pa, data, fault_PC);
if ((pa >= VTB_NVR_BASE) && (pa < VTB_NVR_BASE+VTB_NVR_SIZE)) { /* NovRam */
    rg = (pa >> 1) & 0xF;
    nvr[rg] = data;
    ci_unit[1].hwmark = (rg << 1);
    }
}

int32 ci_mem_rd (int32 pa)
{
int32 rg = pa & 0x7FFC;
return (ciqba_pra0_bin[rg] |
       (ciqba_pra0_bin[rg+1] << 8) |
       (ciqba_pra0_bin[rg+2] << 16) |
       (ciqba_pra0_bin[rg+3] << 24));
}

t_stat ci_read_packet (CI_PKT *pkt, size_t length)
{
uint32 i;
memset (pkt->data, 0, PPD_TYPE);
sim_debug (DBG_REG, &ci_dev, "ci_read_packet (%d bytes):\n", length);
if (Map_ReadB (pkt->addr + PPD_TYPE, length - PPD_TYPE, &pkt->data[PPD_TYPE])) {
    sim_debug (DBG_REG, &ci_dev, "read failed\n");
    return SCPE_EOF;  // Need own status codes
    }
for (i = 0; i < length; i++) {
    if ((i % 4) == 0)
        sim_debug (DBG_REG, &ci_dev, "\n");
    sim_debug (DBG_REG, &ci_dev, "%02X ", pkt->data[i ^ 3]);
    }
sim_debug (DBG_REG, &ci_dev, "\n");
return SCPE_OK;
}

t_stat ci_write_packet (CI_PKT *pkt, size_t length)
{
uint32 i;
sim_debug (DBG_REG, &ci_dev, "ci_write_packet (%d bytes):\n", length);
if (Map_WriteB (pkt->addr + PPD_TYPE, length - PPD_TYPE, &pkt->data[PPD_TYPE])) {
    sim_debug (DBG_REG, &ci_dev, "write failed\n");
    return SCPE_EOF;
    }
for (i = 0; i < length; i++) {
    if ((i % 4) == 0)
        sim_debug (DBG_REG, &ci_dev, "\n");
    sim_debug (DBG_REG, &ci_dev, "%02X ", pkt->data[i ^ 3]);
    }
sim_debug (DBG_REG, &ci_dev, "\n");
return SCPE_OK;
}

t_stat ci_dequeue (CI_QUEUE *queue, CI_PKT *pkt, t_bool keep)
{
uint32 entry, i;

i = queue->ptr;
entry = queue->ring[i*2] | (queue->ring[i*2+1] << 16);  // May need a macro?
if (entry & RENT_CIQBA_OWNED) {
    pkt->addr = entry & RENT_ADDR;
    pkt->size = (entry & RENT_LENGTH) >> RENT_V_LENGTH;
    pkt->size = (pkt->size << 2);
    if (entry & RENT_PACKET_TYPE)
        pkt->type = 1;
    else
        pkt->type = 0;
    sim_debug (DBG_REG, &ci_dev, "ci_dequeue: entry = %d, addr = %X, size = %X\n", i, pkt->addr, pkt->size);
    if (keep)
        entry = RENT_CIQBA_OWNED;
    else
        entry = entry & RENT_ADDR;
    queue->ring[i*2] = entry & 0xFFFF;  // Also may need a macro
    queue->ring[i*2+1] = (entry >> 16) & 0xFFFF;
    if (!keep) {
        queue->ptr++;
        if (queue->ptr == queue->entries)
            queue->ptr = 0;
        }
    return SCPE_OK;
    }
return SCPE_EOF;  // May need our own status codes (like sim_tape, sim_disk)
}

t_stat ci_enqueue (CI_QUEUE *queue, CI_PKT *pkt, t_bool internal)
{
uint32 entry, i;
uint32 size;

i = queue->ptr;
entry = queue->ring[i*2] | (queue->ring[i*2+1] << 16);  // May need a macro?
if (entry == RENT_CIQBA_OWNED) {                    /* owned, empty? */
    sim_debug (DBG_REG, &ci_dev, "ci_enqueue: entry = %d, addr = %X, size = %X\n", i, pkt->addr, pkt->size);
    entry = pkt->addr & RENT_ADDR;
    size = pkt->size >> 2;
    entry = entry | (size << RENT_V_LENGTH);
    if (!internal)
        entry = entry | RENT_PACKET_TYPE;
    queue->ring[i*2] = entry & 0xFFFF;  // Also may need a macro
    queue->ring[i*2+1] = (entry >> 16) & 0xFFFF;
    queue->ptr++;
    if (queue->ptr == queue->entries)
        queue->ptr = 0;
    return SCPE_OK;
    }
sim_debug (DBG_REG, &ci_dev, "ci_enqueue failed\n");
return SCPE_EOF;  // May need our own status codes (like sim_tape, sim_disk)
}

t_bool ci_can_enq (CI_QUEUE *queue)
{
uint32 entry;
entry = queue->ring[queue->ptr*2] | (queue->ring[queue->ptr*2+1] << 16);  // May need a macro?
if (entry == RENT_CIQBA_OWNED)                          /* owned, empty? */
    return TRUE;
return FALSE;
}

t_bool ci_can_deq (CI_QUEUE *queue)
{
uint32 entry;
entry = queue->ring[queue->ptr*2] | (queue->ring[queue->ptr*2+1] << 16);  // May need a macro?
if (entry & RENT_CIQBA_OWNED)
    return TRUE;
return FALSE;
}

void ci_fmt_receive (CI_PKT *pkt)
{
uint32 swflags = pkt->data[PPD_SWFLAG];
uint32 port = pkt->data[PPD_PORT];
uint32 opc = pkt->data[PPD_OPC] & ~OPC_M_RECV;
uint32 flags = pkt->data[PPD_FLAGS];
uint32 i;

if ((opc != OPC_SNDDG) && (opc != OPC_SNDMSG)) {
    pkt->length += 2;
    for (i = pkt->length - 1; i >= PPD_MTYPE; i--)
        pkt->data[i] = pkt->data[i-2];
    }

swflags = swflags & 0x86;
if (flags & PPD_LP)
    swflags = swflags | CIPPD_LAST;
swflags |= ((flags << 3) & 0x30); // Add received path
pkt->data[CIPPD_LEN_HI] = (pkt->length >> 8) & 0xFF;
pkt->data[CIPPD_LEN_LO] = pkt->length & 0xFF;
pkt->data[CIPPD_DEST] = ci_unit[0].ci_node;
pkt->data[CIPPD_NOT_DEST] = ~ci_unit[0].ci_node;
pkt->data[CIPPD_SRC] = port;
pkt->data[CIPPD_OPC] = opc;
pkt->data[CIPPD_SEQ_CNTRL] = swflags;
if (pkt->length < 0x12)
    pkt->length = 0x12;
}

void ci_fmt_send (CI_PKT *pkt)
{
uint32 i;

for (i = PPD_LENGTH; i < pkt->length - 2; i++)
    pkt->data[i] = pkt->data[i+2];
pkt->length = pkt->length - 2;
}

t_stat ci_send_data (UNIT *uptr, CI_PKT *pkt)
{
CI_PKT *blk;


return SCPE_OK;
}

#define CIQBA_MAXDAT  512

t_stat ci_receive_data (UNIT *uptr, CI_PKT *pkt)
{
uint32 rbnam, rboff, sboff, rblen, xfrsz;
uint32 addr = 0;
CI_PKT blk;
t_bool last;
t_stat r;

last = (pkt->data[PPD_FLAGS] & PPD_LP) ? TRUE : FALSE;  /* save last packet flag */
rbnam = CI_GET32 (pkt->data, PPD_RBNAM);                /* get buffer name */
rboff = CI_GET32 (pkt->data, PPD_RBOFF);                /* get buffer offset */
xfrsz = (pkt->length - CI_DATHDR);                      /* payload length */

if (!last)
    addr = ciqba_map_buffer (rbnam, (rboff + xfrsz));   /* try to map buffer */
if (addr != 0) {
    sim_printf ("Found map addr %X for buffer %08X\n", addr, rbnam);
    rblen = Map_WriteB (addr, xfrsz, &pkt->data[CI_DATHDR]); /* transfer block data directly */
    if (rblen)
        sim_printf ("Error transferring data to host memory\n");
    return SCPE_OK;
    }

pkt->data[PPD_SWFLAG] = CIPPD_BLOCK;                    /* block transfer */
pkt->data[PPD_FLAGS] &= ~PPD_LP;                        /* clear last packet flag */

blk.length = PPD_XFRSZ;                                 /* header + transaction ID */
memcpy (blk.data, pkt->data, PPD_XFRSZ);                /* copy header */
ci_fmt_receive (&blk);                                  /* convert to CIQBA format */

CI_PUT32 (blk.data, CIPPD_NAME, rbnam);
sboff = 0;
while (xfrsz) {
    rblen = (xfrsz > CIQBA_MAXDAT) ? CIQBA_MAXDAT : xfrsz;
    memcpy (&blk.data[CIPPD_Q22HAN], &pkt->data[(CI_DATHDR + sboff)], rblen);
    blk.length = rblen + 0x17;                          // FIXME: Why 0x17?
    blk.data[CIPPD_LEN_HI] = (blk.length >> 8) & 0xFF;
    blk.data[CIPPD_LEN_LO] = blk.length & 0xFF;
    blk.length = rblen + CIPPD_Q22HAN;                  /* set actual length */
    if ((xfrsz <= CIQBA_MAXDAT) && last)
        blk.data[CIPPD_SEQ_CNTRL] |= CIPPD_LAST;        /* last packet */
    CI_PUT32 (blk.data, CIPPD_OFFSET, rboff);
    r = ci_receive_int (&blk, FALSE);                   /* pass to system */
    if (r != SCPE_OK)
        sim_printf ("Failed to receive packet\n");
    xfrsz -= rblen;
    rboff += rblen;
    sboff += rblen;
    }
return SCPE_OK;
}

t_stat ci_send (CI_PKT *pkt)
{
UNIT *uptr = &ci_unit[0];

switch (pkt->data[PPD_OPC]) {                           /* opcodes handled by port */

    case OPC_SNDDAT:
    case OPC_RETDAT:
        return ci_send_data (uptr, pkt);

    case OPC_REQID:
    case OPC_RETID:
        ci_fmt_send (pkt);                              /* convert to CI format */
        break;
        }
return ci_send_ppd (uptr, pkt);
}

t_stat ci_dispose_int (CI_PKT *pkt, t_bool internal)
{
sim_debug (DBG_REG, &ci_dev, "ci_dispose\n");
ci_csr |= CSR_DISPOSE_ATTN;
pkt->size = 0;
return ci_enqueue (&ci_dsp, pkt, internal);
}

t_stat ci_dispose (CI_PKT *pkt)
{
return ci_dispose_int (pkt, FALSE);
}

t_stat ci_respond (CI_PKT *pkt)
{
sim_debug (DBG_REG, &ci_dev, "ci_respond\n");
ci_write_packet (pkt, PPD_HDR);
ci_csr |= CSR_DISPOSE_ATTN;
pkt->size = 0;
return ci_enqueue (&ci_dsp, pkt, FALSE);
}

t_stat ci_receive_int (CI_PKT *pkt, t_bool internal)
{
t_stat r;
sim_debug (DBG_REG, &ci_dev, "ci_receive\n");
r = ci_dequeue (&ci_rcv, pkt, TRUE);
if (r != SCPE_OK)
    sim_debug (DBG_REG, &ci_dev, "ci_dequeue failed\n");
ci_write_packet (pkt, pkt->length);
ci_csr |= CSR_RECEIVE_ATTN;
return ci_enqueue (&ci_rcv, pkt, internal);
}

t_stat ci_receive (CI_PKT *pkt)
{
UNIT *uptr = &ci_unit[0];

switch (pkt->data[PPD_OPC]) {                           /* opcodes handled by port */
    case OPC_SNDDATREC:
    case OPC_DATREC:
        return ci_receive_data (uptr, pkt);
        }
ci_fmt_receive (pkt);
return ci_receive_int (pkt, FALSE);
}

t_stat ciqba_map (CI_PKT *pkt)
{
uint32 addr, len, bnam, i;

addr = CI_GET32 (pkt->data, CIPPD_ADDR);
len = CI_GET32 (pkt->data, CIPPD_LENGTH);
bnam = CI_GET32 (pkt->data, CIPPD_NAME);

ci_map[0].addr = addr;                                  /* setup map entry */
ci_map[0].length = len;
ci_map[0].bnam = bnam;
#if 0
for (i = 0; i < MAX_BMAP; i++) {                        /* find free map entry */
    if (ci_map[i].addr == 0) {
        ci_map[i].addr = addr;                          /* setup map entry */
        ci_map[i].length = len;
        ci_map[i].bnam = bnam;
        return SCPE_OK;
        }
    }
sim_printf ("CIQBA buffer maps exhausted\n");
return SCPE_IERR;
#endif
}

uint32 ciqba_map_buffer (uint32 bnam, uint32 off)
{
uint32 i;

for (i = 0; i < MAX_BMAP; i++) {
    if (ci_map[i].bnam == bnam) {                       /* buffer name found? */
        if (off > ci_map[i].length)                     /* buffer length exceeded? */
            return 0;                                   /* can't map */
        return ci_map[i].addr;                          /* yes, return QBus address */
        }
    }
return 0;                                               /* no, unmapped transfer */
}

t_stat ciqba_msg (CI_PKT *pkt)
{
uint8 port;
t_stat r;

switch (pkt->data[PPD_OPC]) {
    case OPC_QUERY:
        sim_debug (DBG_REG, &ci_dev, "CIQBA Host Query\n");
        ci_dispose_int (pkt, TRUE);
        pkt->data[PPD_OPC] = OPC_QUERY_RSP;
        ci_receive_int (pkt, TRUE);                     /* FIXME: check for space in ring */
        break;

    case OPC_INIT:
        sim_debug (DBG_REG, &ci_dev, "CIQBA Host Init\n");
        ci_dispose_int (pkt, TRUE);
        break;

    case OPC_PROMISC_EN:
        sim_debug (DBG_REG, &ci_dev, "CIQBA Promisc Enable\n");
        ci_dispose_int (pkt, TRUE);
        break;

    case OPC_PROMISC_DIS:
        sim_debug (DBG_REG, &ci_dev, "CIQBA Promisc Disable\n");
        ci_dispose_int (pkt, TRUE);
        break;

    case OPC_RESET:
        sim_debug (DBG_REG, &ci_dev, "CIQBA Reset\n");
        port = pkt->data[PPD_PORT];
        if (!ci_vc_open[port]) {
            sim_debug (DBG_REG, &ci_dev, "CIQBA Opening VC to %d\n", port);
            r = ci_open_vc (&ci_unit[0], port);
            }
        else {
            sim_debug (DBG_REG, &ci_dev, "CIQBA Closing VC to %d\n", port);
            r = ci_close_vc (&ci_unit[0], port);
            }
        if (r == SCPE_OK) {
            sim_debug (DBG_REG, &ci_dev, "CIQBA VC status: OK\n");
            ci_vc_open[port] = !ci_vc_open[port];
            }
        else
            sim_debug (DBG_REG, &ci_dev, "CIQBA VC status: ERROR\n");
        ci_dispose_int (pkt, TRUE);
        break;

    case OPC_MAP:
        sim_debug (DBG_REG, &ci_dev, "CIQBA Map\n");
        ciqba_map (pkt);
        ci_dispose_int (pkt, TRUE);
        break;

    case OPC_UNMAP:
        sim_debug (DBG_REG, &ci_dev, "CIQBA Unmap\n");
        ci_dispose_int (pkt, TRUE);
        break;

    default:
        sim_debug (DBG_REG, &ci_dev, "CIQBA Unknown Message\n");
        break;
        }
return SCPE_OK;
}

t_stat ci_svc_queue (CI_QUEUE *queue, uint32 *processed)
{
CI_PKT pkt;
t_stat r = SCPE_OK;

while (ci_can_enq (&ci_dsp)) {
    if (ci_dequeue (queue, &pkt, FALSE) != SCPE_OK)
        break;
    *processed = *processed + 1;
    r = ci_read_packet (&pkt, pkt.size);
    if (r != SCPE_OK) {
        sim_debug (DBG_REG, &ci_dev, "Read packet failed\n");
        break;
        }
    pkt.length = pkt.size;
    sim_debug (DBG_REG, &ci_dev, "Processing packet: type = %d\n", pkt.type);
    // FIXME: Handle status from ci_send_ppd
    if (pkt.type == 1) {
        r = ci_send (&pkt);
        if (pkt.data[PPD_FLAGS] & PPD_RSP)              /* response requested? */
            r = ci_respond (&pkt);                      /* driver wants it back */
        else
            r = ci_dispose (&pkt);                      /* dispose of packet */
        }
    else
        r = ciqba_msg (&pkt);
    if (r != SCPE_OK) {
        sim_debug (DBG_REG, &ci_dev, "Dispose packet failed\n");
        break;
        }
    }
return r;
}

t_stat ciqba_svc (UNIT *uptr)
{
CI_PKT pkt;
t_stat r;
uint32 processed = 0;

sim_debug (DBG_REG, &ci_dev, "CIQBA Poll\n");
r = ci_svc_queue (&ci_sndh, &processed);                /* service high priority queue */
if (r != SCPE_OK)
    return r;
if (processed > 0)                                      /* any processed? */
    ci_csr |= CSR_SEND_HI_AVAIL;
processed = 0;
r = ci_svc_queue (&ci_sndl, &processed);                /* service low priority queue */
if (r != SCPE_OK)
    return r;
if (processed > 0)                                      /* any processed? */
    ci_csr |= CSR_SEND_LO_AVAIL;
while (ci_can_deq (&ci_rcv)) {                          /* receive ring available? */
    r = ci_receive_ppd (uptr, &pkt);                    /* get next packet */
// TODO: handle errors from ci_receive
    if (r != SCPE_OK)
        break;
    r = ci_receive (&pkt);                              /* process packet */
    if (r != SCPE_OK)
        return r;
    };
if ((ci_csr & CSR_ATTN) && (ci_ccr & CCR_ENABLE_INT))   /* any interrupt, enabled? */
    SET_INT (CI);
return ci_svc (uptr);
}

char *ci_sym (uint32 addr)
{
uint32 i = 0;
addr = addr - PQ_BASE;
while (pq_syms[i].symbol != NULL) {
    if (addr < pq_syms[i].value) {
        sprintf (ci_sym_text, "%s+%X", pq_syms[i-1].symbol, (addr - pq_syms[i-1].value));
        return ci_sym_text;
        }
    i++;
    }
return "";
}

int32 ci_inta (void)
{
DIB *dibp = (DIB *)ci_dev.ctxt;
return dibp->vec;
}

t_stat ciqba_attach (UNIT *uptr, CONST char *cptr)
{
if (uptr == &ci_unit[0])
    return ci_attach (uptr, cptr);
else
    return attach_unit (uptr, cptr);
}

t_stat ciqba_detach (UNIT *uptr)
{
if (uptr == &ci_unit[0])
    return ci_detach (uptr);
else
    return detach_unit (uptr);
}

/* Software reset */

t_stat ci_sw_reset (DEVICE *dptr)
{
int32 i;

ci_ccr = 0;
ci_csr = 0;
ci_rcv.ptr = 0;                                         /* reset queue pointers */
ci_dsp.ptr = 0;
ci_sndl.ptr = 0;
ci_sndh.ptr = 0;
for (i = 0; i < 32; i++)                                /* all VCs closed */
    ci_vc_open[i] = FALSE;
for (i = 0; i < MAX_BMAP; i++) {                        /* clear buffer map */
    ci_map[i].addr = 0;
    ci_map[i].length = 0;
    ci_map[i].bnam = 0;
    }
CLR_INT (CI);
return ci_port_reset (dptr);
}

/* Reset CI adapter */

t_stat ci_reset (DEVICE *dptr)
{
DIB *dibp = (DIB *)ci_dev.ctxt;
dibp->vec = 0;
ci_vtba = 0;
ci_vtbd = 0;
ci_sw_reset (dptr);
return auto_config (0, 0);                              /* run autoconfig */
}
