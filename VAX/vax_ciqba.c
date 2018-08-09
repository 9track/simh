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
#define CIS_OP_COMPLETE      0x8000

/* Ring entry */

#define RENT_ADDR_LO         0x0000FFFF
#define RENT_ADDR_HI         0x003F0000
#define RENT_XMIT_FAIL       0x00400000
#define RENT_LENGTH          0x3FC00000
#define RENT_PACKET_TYPE     0x40000000
#define RENT_CIQBA_OWNED     0x80000000

#define IOBA_CI              (IOPAGEBASE + 0xE00)
#define IOLN_CI              0x80

uint32 ci_vtba = 0;
uint16 ci_vtbs = 0;

t_stat ci_rd (int32 *data, int32 PA, int32 access);
t_stat ci_wr (int32 data, int32 PA, int32 access);
int32 ci_vtb_rd (void);
void ci_vtb_wr (void);
t_stat ci_svc (UNIT *uptr);
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

UNIT ci_unit = { UDATA (&ci_svc, UNIT_IDLE|UNIT_ATTABLE, 0) };

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

/* Debugging Bitmaps */

#define DBG_REG         0x0001                          /* register activity */

DEBTAB ci_debug[] = {
    { "REG",    DBG_REG },
    { 0 }
};

DEVICE ci_dev = {
    "CI", &ci_unit, ci_reg, ci_mod,
    1, 0, 0, 0, 0, 0,
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
        break;

    case CI_INTERRUPT_VEC:
        break;

    case CI_FIRMWARE_VER:
        break;

    case CI_PORT_NUM:
        break;

    case CI_MEM_ADDR_LO:
        break;

    case CI_MEM_ADDR_HI:
        ci_vtbs &= ~CIS_OP_COMPLETE;
        break;

    case CI_MEM_DATA:
        break;

    default:
        if (rg > CI_RECEIVE_RING)
            break;

        if (rg > CI_DISPOSE_RING)
            break;

        if (rg > CI_SEND_LO_RING)
            break;

        if (rg > CI_SEND_HI_RING)
            break;
        }
sim_debug (DBG_REG, &ci_dev, "ci_rd: %08X = %04X at %08X\n", PA, *data, fault_PC);
}

t_stat ci_wr (int32 data, int32 PA, int32 access)
{
int32 rg = (PA >> 1) &  0x3F;

sim_debug (DBG_REG, &ci_dev, "ci_wr: %08X = %04X at %08X\n", PA, data, fault_PC);
    
switch (rg) {

    case CI_CONTROL_REG:
        if (data & CCR_READ_MEMORY)
            ci_vtb_rd ();
        else if (data & CCR_WRITE_MEMORY)
            ci_vtb_wr ();
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
        break;

    default:
        if (rg > CI_RECEIVE_RING)
            break;

        if (rg > CI_DISPOSE_RING)
            break;

        if (rg > CI_SEND_LO_RING)
            break;

        if (rg > CI_SEND_HI_RING)
            break;
        }
return SCPE_OK;
}

int32 ci_vtb_rd ()
{
ci_vtbs |= CIS_OP_COMPLETE;
sim_debug (DBG_REG, &ci_dev, "ci_vtb_rd: %08X = %04X at %08X\n", ci_vtba, 0, fault_PC);
return 0;
}

void ci_vtb_wr ()
{
sim_debug (DBG_REG, &ci_dev, "ci_vtb_wr: %08X = %04X at %08X\n", ci_vtba, 0, fault_PC);
ci_vtbs |= CIS_OP_COMPLETE;
}

int32 ci_mem_rd (int32 pa)
{
int32 rg = pa & 0x7FFC;
return (ciqba_pra0_bin[rg] |
       (ciqba_pra0_bin[rg+1] << 8) |
       (ciqba_pra0_bin[rg+2] << 16) |
       (ciqba_pra0_bin[rg+3] << 24));
}

t_stat ci_svc (UNIT *uptr)
{
return SCPE_OK;
}

int32 ci_inta (void)
{
return 0;
}

/* Reset CI adapter */

t_stat ci_reset (DEVICE *dptr)
{
ci_vtba = 0;
ci_vtbs = 0;
CLR_INT (CI);
return SCPE_OK;
}
