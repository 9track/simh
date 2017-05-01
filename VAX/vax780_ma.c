/* vax780_ma.c: MA780 memory controller

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

   This module contains the VAX 11/780 system-specific registers and devices.

   ma         MA780 memory controller

   Related documents:

        EK-VAXV2-HB-002 - VAX Maintenenace Handbook (VAX-11/780) (pp. 235 - 253)
*/

#include "vax_defs.h"
#include "sim_ipc.h"

/* Port Configuration Register */

#define PCR_OF          0x0
#define PCR_SPF         0x80000000                      /* SBI parity fault */
#define PCR_SWSF        0x40000000                      /* SBI write sequence fault */
#define PCR_SISF        0x10000000                      /* SBI interlock sequence fault */
#define PCR_SMXF        0x08000000                      /* SBI multi transmit fault? */
#define PCR_PXDF        0x04000000                      /* port transmit dur? fault */
#define PCR_PDN         0x00800000                      /* port power down */
#define PCR_PUP         0x00400000                      /* port power up */
#define PCR_CODE        0x00000040                      /* adapter code */
#define PCR_PORT        0x00000003                      /* port number */

/* Port Interface Control Register */

#define PIFCR_OF        0x1
#define PIFCR_BPFI      0x80000000                      /* BDI parity fault on input */
#define PIFCR_BPFO      0x40000000                      /* BDI parity fault on output */
#define PIFCR_ILOB      0x20000000                      /* IVDT? lost on BDI */
#define PIFCR_RPE       0x10000000                      /* RAM parity error */
#define PIFCR_IANR      0x08000000                      /* IVDT? ack not received */
#define PIFCR_RO        0x04000000                      /* RAM ov? */
#define PIFCR_MR        0x02000000                      /* mark request */
#define PIFCR_MT        0x01000000                      /* mark timeout */
#define PIFCR_MII       0x00800000                      /* mark interlock ip? */
#define PIFCR_V_RC      16                              /* RAM count */
#define PIFCR_M_RC      0xF
#define PIFCR_IBPI      0x00000040                      /* input BDI parity invert? */
#define PIFCR_OBPI      0x00000020                      /* output BDI parity invert? */
#define PIFCR_IRA       0x00000010                      /* inhibit RAM arbitration */
#define PIFCR_PEIE      0x00000002                      /* parity error interrupt enable */
#define PIFCR_MIE       0x00000001                      /* master interrupt enable */
#define PIFCR_W1C       0xFF000000
#define PIFCR_WR        0x00000073

/* Port Controller Status Register */

#define PCSR_OF         0x2

/* Port Invalidation Control Register */

#define PIVCR_OF        0x3
#define PIVCR_CDI       0x0000FFFF                      /* cache device ident */
#define PIVCR_V_AS      16                              /* array size */
#define PIVCR_V_SA      20                              /* starting address */
#define PIVCR_CFB       0x80000000                      /* cache forced bit */

/* Array Error Register */

#define AERR_OF         0x4

/* Configuration Status Register 0 */

#define CSR0_OF         0x5

/* Configuration Status Register 1 */

#define CSR1_OF         0x6

/* Maintenance Control Register */

#define MCR_OF          0x7
#define MCR_V_MN        14                              /* multiport number */
#define MCR_M_MN        0x3

/* Interport Interrupt Request Register */

#define IIRQ_OF         0x8
#define IIRQ_V_CTL      16                              /* outgoing interrupts */
#define IIRQ_V_STS      0                               /* incoming interrupts */
#define IIRQ_M_BLOCK    0xF                             /* port interrupt block */
#define IIRQ_W1C        0x0000FFFF

/* Interport Interrupt Enable Register */

#define IIEN_OF         0x9
#define IIEN_V_CTL      16                              /* incoming interrupts? */
#define IIEN_V_STS      0                               /* outgoing interrupts? */

#define MAXMASIZE       (1u << 21)                      /* max memory size */
#define INITMASIZE      (1u << 20)                      /* initial memory size */

#define UNIT_V_MSIZE    (UNIT_V_UF + 0)                 /* dummy */
#define UNIT_MSIZE      (1u << UNIT_V_MSIZE)

uint32 ma_port[MA_NUM] = { 0 };                         /* local port number */
uint32 ma_pifcr[MA_NUM] = { 0 };                        /* port intf ctl reg */
uint32 ma_pivcr[MA_NUM] = { 0 };                        /* port inv ctl reg */
uint32 ma_iirq[MA_NUM] = { 0 };                         /* interport int req */
uint32 ma_iien[MA_NUM] = { 0 };                         /* interport int en */

extern uint32 nexus_req[NEXUS_HLVL];
extern uint32 nexusM[NEXUS_NUM];
extern int32 tmxr_poll;                                 /* calibrated delay */

t_stat ma_reset (DEVICE *dptr);
t_stat ma_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr);
const char *ma_description (DEVICE *dptr);
t_stat ma_set_enable (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ma_set_size (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ma_set_port (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ma_show_port (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat ma_attach (UNIT *uptr, CONST char *cptr);
t_stat ma_detach (UNIT *uptr);
t_stat ma_rdreg (int32 *val, int32 pa, int32 mode);
t_stat ma_wrreg (int32 val, int32 pa, int32 mode);
t_stat ma_svc (UNIT *uptr);
void ma_set_int (uint32 mctl);
void ma_clr_int (uint32 mctl);

/* MA data structures

   ma_dev    MA device descriptor
   ma_unit   MA unit
   ma_reg    MA register list
*/

DIB ma0_dib[] = { TR_MA0, 0, &ma_rdreg, &ma_wrreg, 0, NVCL (MA0) };

UNIT ma0_unit = { UDATA (&ma_svc, UNIT_FIX+UNIT_ATTABLE, INITMASIZE) };

REG ma0_reg[] = {
    { HRDATA (PORT, ma_port[0], 32) },
    { HRDATA (PIFCR, ma_pifcr[0], 32) },
    { HRDATA (PIVCR, ma_pivcr[0], 32) },
    { HRDATA (IIRQ, ma_iirq[0], 32) },
    { HRDATA (IIEN, ma_iien[0], 32) },
    { NULL }
    };

DIB ma1_dib[] = { TR_MA1, 0, &ma_rdreg, &ma_wrreg, 0, NVCL (MA1) };

UNIT ma1_unit = { UDATA (&ma_svc, UNIT_FIX+UNIT_ATTABLE, INITMASIZE) };

REG ma1_reg[] = {
    { HRDATA (PORT, ma_port[1], 32) },
    { HRDATA (PIFCR, ma_pifcr[1], 32) },
    { HRDATA (PIVCR, ma_pivcr[1], 32) },
    { HRDATA (IIRQ, ma_iirq[1], 32) },
    { HRDATA (IIEN, ma_iien[1], 32) },
    { NULL }
    };

DIB ma2_dib[] = { TR_MA2, 0, &ma_rdreg, &ma_wrreg, 0, NVCL (MA2) };

UNIT ma2_unit = { UDATA (&ma_svc, UNIT_FIX+UNIT_ATTABLE, INITMASIZE) };

REG ma2_reg[] = {
    { HRDATA (PORT, ma_port[2], 32) },
    { HRDATA (PIFCR, ma_pifcr[2], 32) },
    { HRDATA (PIVCR, ma_pivcr[2], 32) },
    { HRDATA (IIRQ, ma_iirq[2], 32) },
    { HRDATA (IIEN, ma_iien[2], 32) },
    { NULL }
    };

DIB ma3_dib[] = { TR_MA3, 0, &ma_rdreg, &ma_wrreg, 0, NVCL (MA3) };

UNIT ma3_unit = { UDATA (&ma_svc, UNIT_FIX+UNIT_ATTABLE, INITMASIZE) };

REG ma3_reg[] = {
    { HRDATA (PORT, ma_port[3], 32) },
    { HRDATA (PIFCR, ma_pifcr[3], 32) },
    { HRDATA (PIVCR, ma_pivcr[3], 32) },
    { HRDATA (IIRQ, ma_iirq[3], 32) },
    { HRDATA (IIEN, ma_iien[3], 32) },
    { NULL }
    };

MTAB ma_mod[] = {
    { MTAB_XTD|MTAB_VDV, TR_MA0, "NEXUS", NULL,
      NULL, &show_nexus },
    { MTAB_XTD|MTAB_VDV, 0, "PORT", "PORT",
      &ma_set_port, &ma_show_port, NULL, "Set/Show the port number" },
    { MTAB_XTD|MTAB_VDV, 1, NULL, "ENABLE",
        &ma_set_enable, NULL, NULL, "Enable MA780" },
    { MTAB_XTD|MTAB_VDV, 0, NULL, "DISABLE",
        &ma_set_enable, NULL, NULL, "Disable MA780" },
    { UNIT_MSIZE, (1u << 18), NULL, "256K",
      &ma_set_size, NULL, NULL, "Set Memory to 256K bytes" },
    { UNIT_MSIZE, (1u << 19), NULL, "512K",
      &ma_set_size, NULL, NULL, "Set Memory to 512K bytes" },
    { UNIT_MSIZE, (1u << 19) + (1u << 18), NULL, "768K",
      &ma_set_size, NULL, NULL, "Set Memory to 768K bytes" },
    { UNIT_MSIZE, (1u << 20), NULL, "1M",
      &ma_set_size, NULL, NULL, "Set Memory to 1M bytes" },
    { UNIT_MSIZE, (1u << 20) + (1u << 18), NULL, "1280K",
      &ma_set_size, NULL, NULL, "Set Memory to 1280K bytes" },
    { UNIT_MSIZE, (1u << 20) + (1u << 19), NULL, "1536K",
      &ma_set_size, NULL, NULL, "Set Memory to 1536K bytes" },
    { UNIT_MSIZE, (1u << 20) + (1u << 19) + (1u << 18), NULL, "1792K",
      &ma_set_size, NULL, NULL, "Set Memory to 1792K bytes" },
    { UNIT_MSIZE, (1u << 21), NULL, "2M",
      &ma_set_size, NULL, NULL, "Set Memory to 2M bytes" },
//    { MTAB_XTD|MTAB_VDV|MTAB_NMO, 0, "MEMORY", NULL, NULL, &ma_show_memory, NULL, "Display memory configuration" }
    { 0 }
    };

/* Debugging Bitmaps */

#define DBG_REG         0x0001                          /* register activity */
#define DBG_INT         0x0002                          /* interrupts */

DEBTAB ma_deb[] = {
    { "REG", DBG_REG },
    { "INT", DBG_INT },
    { NULL, 0 }
    };

DEVICE ma_dev[] = {
    {
    "MA0", &ma0_unit, ma0_reg, ma_mod,
    1, 16, 16, 1, 16, 8,
    NULL, NULL, &ma_reset,
    NULL, &ma_attach, &ma_detach,
    &ma0_dib, DEV_NEXUS | DEV_SHM | DEV_DEBUG | DEV_DIS | DEV_DISABLE, 0,
    ma_deb, &ma_set_size, NULL, &ma_help, NULL, NULL,
    &ma_description
    },
    {
    "MA1", &ma1_unit, ma1_reg, ma_mod,
    1, 16, 16, 1, 16, 8,
    NULL, NULL, &ma_reset,
    NULL, &ma_attach, &ma_detach,
    &ma1_dib, DEV_NEXUS | DEV_SHM | DEV_DEBUG | DEV_DIS | DEV_DISABLE, 0,
    ma_deb, &ma_set_size, NULL, &ma_help, NULL, NULL,
    &ma_description
    },
    {
    "MA2", &ma2_unit, ma2_reg, ma_mod,
    1, 16, 16, 1, 16, 8,
    NULL, NULL, &ma_reset,
    NULL, &ma_attach, &ma_detach,
    &ma2_dib, DEV_NEXUS | DEV_SHM | DEV_DEBUG | DEV_DIS | DEV_DISABLE, 0,
    ma_deb, &ma_set_size, NULL, &ma_help, NULL, NULL,
    &ma_description
    },
    {
    "MA3", &ma3_unit, ma3_reg, ma_mod,
    1, 16, 16, 1, 16, 8,
    NULL, NULL, &ma_reset,
    NULL, &ma_attach, &ma_detach,
    &ma3_dib, DEV_NEXUS | DEV_SHM | DEV_DEBUG | DEV_DIS | DEV_DISABLE, 0,
    ma_deb, &ma_set_size, NULL, &ma_help, NULL, NULL,
    &ma_description
    },
    };

const char *ma_regnames[] = {
    "PCR",
    "PIFCR",
    "PCSR",
    "PIVCR",
    "AERR",
    "CSR0",
    "CSR1",
    "MCR",
    "IIRQ",
    "IIEN"
    };

/* MA780 register read */

t_stat ma_rdreg (int32 *val, int32 pa, int32 lnt)
{
int32 mctl, ofs;
DEVICE *dptr;

if ((pa & 3) || (lnt != L_LONG)) {                      /* unaligned or not lw? */
    sim_printf (">>MA: invalid adapter read mask, pa = %X, lnt = %d\r\n", pa, lnt);
    sbi_set_errcnf ();                                  /* err confirmation */
    return SCPE_OK;
    }
mctl = NEXUS_GETNEX (pa) - TR_MA0;                      /* get mctl num */
ofs = NEXUS_GETOFS (pa);                                /* get offset */
dptr = &ma_dev[mctl];                                   /* get device */

switch (ofs) {

    case PCR_OF:                                        /* PCR */
        *val = (PCR_CODE | ma_port[mctl]);
        break;

    case PIFCR_OF:                                      /* PIFCR */
        *val = ma_pifcr[mctl];
        break;

    case PCSR_OF:                                       /* PCSR */
        *val = 0;
        break;

    case PIVCR_OF:                                      /* PIVCR */
        *val = ma_pivcr[mctl];
        break;

    case AERR_OF:                                       /* AERR */
        *val = 0;
        break;

    case CSR0_OF:                                       /* CSR0 */
        *val = 0;
        break;

    case CSR1_OF:                                       /* CSR1 */
        *val = 0;
        break;

    case MCR_OF:                                        /* MCR */
        *val = ((mctl & MCR_M_MN) << MCR_V_MN);
        break;

    case IIRQ_OF:                                       /* IIRQ */
        *val = ma_iirq[mctl];
        break;

    case IIEN_OF:                                       /* IIEN */
        *val = ma_iien[mctl];
        break;

    default:
        return SCPE_NXM;
        }

sim_debug (DBG_REG, dptr, "ma_rd(%s) data=0x%04X at %08X\n", ma_regnames[ofs], *val, fault_PC);
return SCPE_OK;
}

/* MA780 register write */

t_stat ma_wrreg (int32 val, int32 pa, int32 lnt)
{
int32 mctl, ofs;
uint32 sc, i;
DEVICE *dptr;
t_stat r;

if ((pa & 3) || (lnt != L_LONG)) {                      /* unaligned or not lw? */
    sim_printf (">>MA: invalid adapter write mask, pa = %X, lnt = %d\r\n", pa, lnt);
    sbi_set_errcnf ();                                  /* err confirmation */
    return SCPE_OK;
    }
mctl = NEXUS_GETNEX (pa) - TR_MA0;                      /* get mctl num */
ofs = NEXUS_GETOFS (pa);                                /* get offset */
dptr = &ma_dev[mctl];                                   /* get device */

sim_debug (DBG_REG, dptr, "ma_wr(%s) data=0x%04X at %08X\n", ma_regnames[ofs], val, fault_PC);
switch (ofs) {

    case PCR_OF:                                        /* PCR */
        break;

    case PIFCR_OF:                                      /* PIFCR */
        ma_pifcr[mctl] = (ma_pifcr[mctl] & ~PIFCR_WR) | (val & PIFCR_WR);
        break;

    case PCSR_OF:                                       /* PCSR */
        break;

    case PIVCR_OF:                                      /* PIVCR */
        break;

    case AERR_OF:                                       /* AERR */
        break;

    case CSR0_OF:                                       /* CSR0 */
        break;

    case CSR1_OF:                                       /* CSR1 */
        break;

    case MCR_OF:                                        /* MCR */
        break;

    case IIRQ_OF:                                       /* IIRQ */
        sc = (ma_port[mctl] << 2) + IIRQ_V_CTL;
        val = (val >> sc) & IIRQ_M_BLOCK;               /* get port int block */
        for (i = 0; i <= PCR_PORT; i++) {
            if (val & 1) {                              /* send int to this port? */
                sim_debug (DBG_INT, dptr,
                   "Sending interrupt to remote port %d\n", i);
                r = ipc_send_int (&dptr->units[0], i);
                if (r != SCPE_OK)
                    ABORT (r);
                }
            val = val >> 1;
            }
        break;

    case IIEN_OF:                                       /* IIEN */
        ma_iien[mctl] = val;
        break;

    default:
        return SCPE_NXM;
        }

return SCPE_OK;
}

t_stat ma_svc (UNIT *uptr)
{
DEVICE *dptr = find_dev_from_unit (uptr);
uint32 mctl = (uint32)(dptr - &ma_dev[0]);
uint32 lp = (ma_port[mctl] << 2);                       /* local port block */
uint32 i;

if (ipc_poll_int (uptr)) {
    for (i = 0; i < PCR_PORT; i++) {
        if ((uptr->buf >> i) & 1) {
            sim_debug (DBG_INT, dptr, "Interrupt from remote port %d\n", i);
            ma_iirq[mctl] = ma_iirq[mctl] | (1u << (IIRQ_V_STS + lp + i));
            }
        }
    if ((ma_iirq[mctl] & (ma_iien[mctl] >> IIEN_V_CTL)) &&
        (ma_pifcr[mctl] & PIFCR_MIE)) {                 /* unmasked ints? */
        ma_set_int (mctl);
        }
    }
sim_clock_coschedule (uptr, tmxr_poll);                 /* reactivate */
return SCPE_OK;
}

/* MA780 reset */

t_stat ma_reset (DEVICE *dptr)
{
uint32 mctl = (uint32)(dptr - &ma_dev[0]);
DIB *dibp = (DIB *)dptr->ctxt;
UNIT *uptr = (UNIT *)&dptr->units[0];
uint32 as;

sim_cancel (uptr);                                      /* stop poll */
ma_pifcr[mctl] = 0;
as = (((uint32)uptr->capac) >> 18) - 1;                 /* array size */
ma_pivcr[mctl] = (nexusM[dibp->ba] << PIVCR_V_SA) | (as << PIVCR_V_AS);
ma_iirq[mctl] = 0;
ma_iien[mctl] = 0;

if (uptr->filebuf == NULL) {                            /* first time init? */
    uptr->filebuf = (void *)calloc ((size_t)(uptr->capac >> 2), sizeof (uint32));
                                                        /* alloc local mem */
    if (uptr->filebuf == NULL)
        return SCPE_MEM;
    }
sim_activate_abs (uptr, tmxr_poll);
return SCPE_OK;
}

void ma_set_int (uint32 mctl)
{
DIB *dibp;

if (mctl >= MA_NUM)
    return;
dibp = (DIB *) ma_dev[mctl].ctxt;
if (dibp) {
    nexus_req[dibp->vloc >> 5] |= (1u << (dibp->vloc & 0x1F));
    sim_debug (DBG_INT, &ma_dev[mctl], "ma_set_int(0x%X)\n", dibp->vloc);
    }
return;
}

void ma_clr_int (uint32 mctl)
{
DIB *dibp;

if (mctl >= MBA_NUM)
    return;
dibp = (DIB *) ma_dev[mctl].ctxt;
if (dibp) {
    nexus_req[dibp->vloc >> 5] &= ~(1u << (dibp->vloc & 0x1F));
    sim_debug (DBG_INT, &ma_dev[mctl], "ma_clr_int(0x%X)\n", dibp->vloc);
    }
return;
}

/* Attach routine */

t_stat ma_attach (UNIT *uptr, CONST char *cptr)
{
t_stat r;
void *buf;

if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;

buf = uptr->filebuf;                                    /* save local mem */
r = ipc_attach (uptr, cptr);
if (r != SCPE_OK) {
    uptr->filebuf = buf;                                /* back to local mem */
    return r;
    }
free (buf);                                             /* free local mem */
uptr->flags |= UNIT_ATT;
reset_all (0);                                          /* reset everything */
return SCPE_OK;
}

t_stat ma_detach (UNIT *uptr)
{
if (!(uptr->flags & UNIT_ATT))                          /* attached? */
    return SCPE_OK;

ipc_detach (uptr);
uptr->flags &= ~UNIT_ATT;
uptr->filebuf = (void *)calloc ((size_t)(uptr->capac >> 2), sizeof (uint32));
                                                        /* alloc local mem */
if (uptr->filebuf == NULL)
    return SCPE_MEM;
reset_all (0);                                          /* reset everything */
return SCPE_OK;
}

t_stat ma_set_enable (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
DEVICE *dptr = find_dev_from_unit (uptr);

if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;

if (val)
    dptr->flags = dptr->flags & ~DEV_DIS;
else
    dptr->flags = dptr->flags | DEV_DIS;
reset_all (0);                                          /* reset everything */
return SCPE_OK;
}

/* Memory allocation */

t_stat ma_set_size (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
if ((val <= 0) || (val > MAXMASIZE))
    return SCPE_ARG;
uptr->capac = val;
free (uptr->filebuf);
uptr->filebuf = (void *)calloc ((size_t)(uptr->capac >> 2), sizeof (uint32));
                                                        /* realloc local mem */
if (uptr->filebuf == NULL)
    return SCPE_MEM;
reset_all (0);                                          /* reset everything */
return SCPE_OK;
}

t_stat ma_set_port (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
DEVICE *dptr = find_dev_from_unit (uptr);
uint32 mctl = (uint32)(dptr - &ma_dev[0]);
t_stat r;
uint32 newport;

newport = (uint32) get_uint (cptr, 10, PCR_PORT, &r);   /* port is range 0..3 */
if (r != SCPE_OK)
    return r;

ma_port[mctl] = newport;
ipc_set_node (uptr, ma_port[mctl]);
return SCPE_OK;
}

t_stat ma_show_port (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
DEVICE *dptr = find_dev_from_unit (uptr);
uint32 mctl = (uint32)(dptr - &ma_dev[0]);
fprintf (st, "port=%d", ma_port[mctl]);
return SCPE_OK;
}

t_stat ma_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr)
{
fprintf (st, "Multiport Memory Adapter (MA)\n\n");
fprintf (st, "The Multiport Memory Adapter (MA) simulates the MA780.\n");
return SCPE_OK;
}

const char *ma_description (DEVICE *dptr)
{
return "Multiport memory adapter";
}
