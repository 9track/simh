/* vax_ciqba.c: QBus Computer Interconnect adapter

   Copyright (c) 2018, Matt Burke

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

#define IOBA_CI              (IOPAGEBASE + 0xE00)
#define IOLN_CI              0x80

typedef struct {
    uint32 entries;
    uint16 *ring;
} CI_QUEUE;

#define NEW_QUEUE(n,s)      uint16 n##_d[(s * 2)]; \
                            CI_QUEUE n = { s, &n##_d[0] }

uint16 ci_csr = 0;                                      /* status register */
uint32 ci_vtba = 0;                                     /* VTB address */
uint32 ci_vtbd = 0;                                     /* VTB data */
uint16 ci_nvr[0x20] = { 0 };
NEW_QUEUE (ci_rcv, CIS_RECEIVE_RING_ENTRIES);
NEW_QUEUE (ci_dsp, CIS_DISPOSE_RING_ENTRIES);
NEW_QUEUE (ci_sndl, CIS_SEND_LO_RING_ENTRIES);
NEW_QUEUE (ci_sndh, CIS_SEND_HI_RING_ENTRIES);

t_stat ci_rd (int32 *data, int32 PA, int32 access);
t_stat ci_wr (int32 data, int32 PA, int32 access);
int32 ci_vtb_rd (int32 pa);
void ci_vtb_wr (int32 pa, int32 data);
t_stat ci_svc_queue (CI_QUEUE *queue);
t_stat ci_reset (DEVICE *dptr);
int32 ci_inta (void);


/* CIQBA adapter data structures

   ci_dev       CI device descriptors
   ci_unit      CI unit
   ci_reg       CI register list
*/

DIB ci_dib = {
    IOBA_CI, IOLN_CI, &ci_rd, &ci_wr,
    1, IVCL (CI), 0, { &ci_inta }
    };

UNIT ci_unit = { UDATA (&ci_svc, UNIT_IDLE|UNIT_FIX|UNIT_ATTABLE|UNIT_BUFABLE|UNIT_MUSTBUF, VTB_NVR_SIZE) };

REG ci_reg[] = {
    { NULL }
    };

MTAB ci_mod[] = {
//    { MTAB_XTD|MTAB_VDV, 0, "NODE", "NODE",
//        &ci_set_node, &ci_show_node },
    { MTAB_XTD|MTAB_VDV|MTAB_VALR, 004, "ADDRESS", "ADDRESS",
        &set_addr, &show_addr, NULL, "Bus address" },
    { MTAB_XTD|MTAB_VDV|MTAB_VALR, 0, "VECTOR", "VECTOR",
        &set_vec,  &show_vec,  NULL, "Interrupt vector" },
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
    { 0 }
    };

DEVICE ci_dev = {
    "CI", &ci_unit, ci_reg, ci_mod,
    1, DEV_RDX, 20, 1, DEV_RDX, 16,
    NULL, NULL, &ci_reset,
    NULL, NULL, NULL,
    &ci_dib, DEV_QBUS | DEV_DEBUG, 0,
    ci_debug, 0, 0
    };


t_stat ci_rd (int32 *data, int32 PA, int32 access)
{
int32 rg = (PA >> 1) &  0x3F;

*data = 0;
switch (rg) {

    case CI_CONTROL_REG:
        break;

    case CI_STATUS_REG:
        *data = ci_csr;
        break;

    case CI_INTERRUPT_VEC:
        break;

    case CI_FIRMWARE_VER:
        break;

    case CI_PORT_NUM:
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
            sim_debug (DBG_REG, &ci_dev, "ci_rd: ci_rcv[%d] = %04X\n", rg, *data);
            break;
            }
        if (rg >= CI_DISPOSE_RING) {
            rg = rg - CI_DISPOSE_RING;
            *data = ci_dsp.ring[rg];
            sim_debug (DBG_REG, &ci_dev, "ci_rd: ci_dsp[%d] = %04X\n", rg, *data);
            break;
            }
        if (rg >= CI_SEND_LO_RING) {
            rg = rg - CI_SEND_LO_RING;
            *data = ci_sndl.ring[rg];
            sim_debug (DBG_REG, &ci_dev, "ci_rd: ci_sndl[%d] = %04X\n", rg, *data);
            break;
            }
        if (rg >= CI_SEND_HI_RING) {
            rg = rg - CI_SEND_HI_RING;
            *data = ci_sndh.ring[rg];
            sim_debug (DBG_REG, &ci_dev, "ci_rd: ci_sndh[%d] = %04X\n", rg, *data);
            }
            break;
        }
sim_debug (DBG_REG, &ci_dev, "ci_rd: %08X = %04X at %08X\n", PA, *data, fault_PC);
}

t_stat ci_wr (int32 data, int32 PA, int32 access)
{
int32 rg = (PA >> 1) &  0x3F;
uint32 entry, addr, length, i, j;
uint8 pkt[1024];

sim_debug (DBG_REG, &ci_dev, "ci_wr: %08X = %04X at %08X\n", PA, data, fault_PC);
    
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
        else if (data & CCR_INITIALIZE)
            ci_csr |= CSR_INITIALIZED;
        else if (data & CCR_START)
            ci_csr |= CSR_RUNNING;
        else if (data & CCR_RUNDOWN)
            ci_csr &= ~CSR_RUNNING;
        else if (data & CCR_HALT) {
            ci_csr &= ~CSR_INITIALIZED;
            ci_csr &= ~CSR_RUNNING;
            }
        else if (data & CCR_SEND_HI_ATTN) {
            sim_debug (DBG_REG, &ci_dev, "ci_wr: CCR_SEND_HI_ATTN\n");
            ci_svc_queue (&ci_sndh);
            }
        else if (data & CCR_SEND_LO_ATTN) {
            sim_debug (DBG_REG, &ci_dev, "ci_wr: CCR_SEND_LO_ATTN\n");
            ci_svc_queue (&ci_sndl);
            }
        break;

    case CI_STATUS_REG:
        break;

    case CI_INTERRUPT_VEC:
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
            sim_debug (DBG_REG, &ci_dev, "ci_wr: ci_rcv[%d] = %04X\n", rg, data);
            ci_rcv.ring[rg] = data;
            break;
            }
        if (rg >= CI_DISPOSE_RING) {
            rg = rg - CI_DISPOSE_RING;
            sim_debug (DBG_REG, &ci_dev, "ci_wr: ci_dsp[%d] = %04X\n", rg, data);
            ci_dsp.ring[rg] = data;
            break;
            }
        if (rg >= CI_SEND_LO_RING) {
            rg = rg - CI_SEND_LO_RING;
            sim_debug (DBG_REG, &ci_dev, "ci_wr: ci_sndl[%d] = %04X\n", rg, data);
            ci_sndl.ring[rg] = data;
            break;
            }
        if (rg >= CI_SEND_HI_RING) {
            rg = rg - CI_SEND_HI_RING;
            sim_debug (DBG_REG, &ci_dev, "ci_wr: ci_sndh[%d] = %04X\n", rg, data);
            ci_sndh.ring[rg] = data;
            }
            break;
        }
return SCPE_OK;
}

int32 ci_vtb_rd (int32 pa)
{
int32 rg;
int32 data = 0;
uint16 *nvr = (uint16 *) ci_unit.filebuf;

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
uint16 *nvr = (uint16 *) ci_unit.filebuf;

sim_debug (DBG_REG, &ci_dev, "ci_vtb_wr: %08X = %04X at %08X\n", pa, data, fault_PC);
if ((pa >= VTB_NVR_BASE) && (pa < VTB_NVR_BASE+VTB_NVR_SIZE)) { /* NovRam */
    rg = (pa >> 1) & 0xF;
    nvr[rg] = data;
    ci_unit.hwmark = (rg << 1);
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
if (Map_ReadB (pkt->addr, length, &pkt->data[0]))
    return SCPE_EOF;  // Need own status codes
return SCPE_OK;
}

t_stat ci_write_packet (CI_PKT *pkt, size_t length)
{
if (Map_WriteB (pkt->addr, length, &pkt->data[0]))
    return SCPE_EOF;
return SCPE_OK;
}

t_stat ci_send_data (int32 opcode, uint8 *buffer, int32 port, int32 path)
{
return SCPE_OK;
}

t_stat ci_receive_data (int32 opcode, int32 src, uint8 *buffer)
{
return SCPE_OK;
}

t_stat ci_dequeue (CI_QUEUE *queue, CI_PKT *pkt)
{
uint32 entry, i;

for (i = 0; i < queue->entries; i++) {
    entry = queue->ring[i*2] | (queue->ring[i*2+1] << 16);  // May need a macro?
    sim_debug (DBG_REG, &ci_dev, "ci_dequeue: entry[%d] = %08X\n", i, entry);
    if (entry & RENT_CIQBA_OWNED) {
        pkt->addr = entry & RENT_ADDR;
        pkt->length = (entry & RENT_LENGTH) >> RENT_V_LENGTH;
        pkt->length = (pkt->length << 2);
        sim_debug (DBG_REG, &ci_dev, "ci_dequeue: addr = %X, length = %X\n", pkt->addr, pkt->length);
        entry = entry & RENT_ADDR;
//        entry = 0;
        queue->ring[i*2] = entry & 0xFFFF;  // Also may need a macro
        queue->ring[i*2+1] = (entry >> 16) & 0xFFFF;
        return SCPE_OK;
        }
    }
return SCPE_EOF;  // May need our own status codes (like sim_tape, sim_disk)
}

t_stat ci_enqueue (CI_QUEUE *queue, CI_PKT *pkt)
{
uint32 entry, i;
uint32 length;

for (i = 0; i < queue->entries; i++) {
    entry = queue->ring[i*2] | (queue->ring[i*2+1] << 16);  // May need a macro?
    sim_debug (DBG_REG, &ci_dev, "ci_enqueue: entry[%d] = %08X\n", i, entry);
    if (entry == RENT_CIQBA_OWNED) {                    /* owned, empty? */
        sim_debug (DBG_REG, &ci_dev, "ci_enqueue: addr = %X, length = %X\n", pkt->addr, pkt->length);
        entry = pkt->addr & RENT_ADDR;
//        length = pkt->length >> 2;
//        entry = entry | (length << RENT_V_LENGTH);
        queue->ring[i*2] = entry & 0xFFFF;  // Also may need a macro
        queue->ring[i*2+1] = (entry >> 16) & 0xFFFF;
        return SCPE_OK;
        }
    }
return SCPE_EOF;  // May need our own status codes (like sim_tape, sim_disk)
}

t_stat ci_put_dfq (CI_PKT *pkt)
{
// Put to dispose queue
return ci_enqueue (&ci_dsp, pkt);
}

t_stat ci_put_mfq (CI_PKT *pkt)
{
// Put to dispose queue
return ci_enqueue (&ci_dsp, pkt);
}

t_stat ci_put_rsq (CI_PKT *pkt)
{
// Put to receive queue
return ci_enqueue (&ci_rcv, pkt);
}

t_stat ci_get_dfq (CI_PKT *pkt)
{
// Get from receive queue?
return ci_dequeue (&ci_rcv, pkt);
}

t_stat ci_get_mfq (CI_PKT *pkt)
{
// Get from receive queue?
return ci_dequeue (&ci_rcv, pkt);
}

t_stat ci_svc_queue (CI_QUEUE *queue)
{
CI_PKT pkt;
t_stat r;
while (ci_dequeue (queue, &pkt) == SCPE_OK) {
    r = ci_read_packet (&pkt, pkt.length);
    if (r != SCPE_OK) {
        sim_debug (DBG_REG, &ci_dev, "Read packet failed\n");
        break;
        }
    sim_debug (DBG_REG, &ci_dev, "Processing packet\n");
    r = ci_ppd (&pkt);
    if (r != SCPE_OK) {
        sim_debug (DBG_REG, &ci_dev, "Process packet failed\n");
        break;
        }
    }
return r;
}

int32 ci_inta (void)
{
return 0;
}

/* Reset CI adapter */

t_stat ci_reset (DEVICE *dptr)
{
ci_csr = 0;
ci_vtba = 0;
ci_vtbd = 0;
CLR_INT (CI);
ci_port_reset (dptr);
return SCPE_OK;
}
