/* vax_cibci.c: VAXBI Computer Interconnect adapter

   Copyright (c) 2020, Matt Burke

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

   ci             CIBCI CI adapter
*/

#include "vax_defs.h"
#include "vax_ci.h"
#include "vax_ci_dec.h"

/* CIBCI Registers */

#define CNFGR_OF        0x100
#define PMCSR_OF        0x110
#define MADR_OF         0x114
#define MDATR_OF        0x118
#define BCAR_OF         0x124
#define BCMR_OF         0x128
#define DMAF_OF         0x12C

#define CNFGR_BCMR      0x0000FFFF
#define CNFGR_CMD       0x000F0000                      /* corrected read data */
#define CNFGR_CIPA      0x00800000                      /* no CIPA */
#define CNFGR_DCLO      0x01000000                      /* CIPA DC low */
#define CNFGR_DIAG      0x02000000                      /* enable diagnostic mode */
#define CNFGR_PUP       0x04000000                      /* power up */
#define CNFGR_PDN       0x08000000                      /* power down */
#define CNFGR_BBE       0x10000000                      /* VAXBI BSY error */
#define CNFGR_BIPE      0x20000000                      /* VAXBI parity error */ 
#define CNFGR_BAPE      0x40000000                      /* BICA parity error */
#define CNFGR_CPPE      0x80000000                      /* CIPA bus parity error */
#define CNFGR_RW        (CNFGR_DIAG)

#define PMCSR_MI        0x00000001                      /* maintenance initialise */
#define PMCSR_MTD       0x00000002                      /* maintenance timer disable */
#define PMCSR_MIE       0x00000004                      /* maintenance interrupt enable */
#define PMCSR_MIF       0x00000008                      /* maintenance interrupt flag */
#define PMCSR_WP        0x00000010                      /* wrong parity */
#define PMCSR_RSVD      0x00000020                      /* reserved */
#define PMCSR_PSA       0x00000040                      /* programmable starting addr */
#define PMCSR_UI        0x00000080                      /* uninitialised state */
#define PMCSR_TBPE      0x00000100                      /* transmit buffer parity error */
#define PMCSR_OPE       0x00000200                      /* output parity error */
#define PMCSR_IPE       0x00000400                      /* input parity error */
#define PMCSR_TMPE      0x00000800                      /* transmit multiple parity error */
#define PMCSR_RBPE      0x00001000                      /* receive buffer parity error */
#define PMCSR_LSPE      0x00002000                      /* local store parity error */
#define PMCSR_CSPE      0x00004000                      /* control store parity error */
#define PMCSR_PE        0x00008000                      /* parity error */
#define PMCSR_RD        0x0000FFFE
#define PMCSR_RW        (PMCSR_PSA | PMCSR_WP | PMCSR_MIE | \
                         PMCSR_MTD | PMCSR_RSVD)
#define PMCSR_W1C       (PMCSR_PE | PMCSR_CSPE | PMCSR_LSPE | \
                         PMCSR_RBPE | PMCSR_TMPE | PMCSR_IPE | \
                         PMCSR_OPE | PMCSR_TBPE | PMCSR_MIF)

#define BCAR_ADDR       0x0FFFFFFF                      /* DMA address */
#define BCAR_V_ADDR     2                               /* offset for read only */
#define BCAR_CMD        0xF0000000                      /* SBI command */
#define BCAR_LEN        0xE0000000                      /* transfer length */

#define BCMR_MSK0       0x0000000F                      /* first longword mask */
#define BCMR_MSK1       0x000000F0                      /* second longword mask */
#define BCMR_MSK        (BCMR_MSK0 | BCMR_MSK1)
#define BCMR_V_MSK      2                               /* offset for read only */
#define BCMR_CMD        0x00003C00                      /* SBI command */
#define BCMR_LEN        0x0001C000                      /* transfer length */

#define PSR_MTE         0x80000000                      /* maintenance error */

#define MADR_ADDR       0x1FFF

/* Standard Port Registers */

#define PSR_OF          0x900
#define PQBBR_OF        0x904
#define PCQ0CR_OF       0x908
#define PCQ1CR_OF       0x90C
#define PCQ2CR_OF       0x910
#define PCQ3CR_OF       0x914
#define PSRCR_OF        0x918
#define PECR_OF         0x91C
#define PDCR_OF         0x920
#define PICR_OF         0x924
#define PDFQCR_OF       0x928
#define PMFQCR_OF       0x92C
#define PMTCR_OF        0x930
#define PMTECR_OF       0x934
#define PFAR_OF         0x938
#define PESR_OF         0x93C
#define PPR_OF          0x940

/* CIBCI Port Parameters */

#define RPORT_TYPE      0x80000002
#define CODE_REV        0x00700040
#define FUNC_MASK       0xFFFF0F00
#define INT_BUF_LEN     0x3F9

/* CIBCI Port States */

#define PORT_UCODERUN   1                               /* microcode running */

BIIC ci_biic;                                           /* BIIC standard registers */
uint32 ci_cnfgr;                                        /* configuration reg */
uint32 ci_pmcsr;                                        /* port maintenance csr */
uint32 ci_madr;                                         /* mainteneance addr reg */
uint32 ci_mdatr[8192];                                  /* maintenanace data reg */
uint32 ci_bcar;                                         /* BICA address register */
uint32 ci_bcmr;                                         /* BICA command/byte mask register */
uint32 ci_dmaf[4];                                      /* DMA register file */
uint32 ci_local_store[1024];

extern uint32 nexus_req[NEXUS_HLVL];

t_stat ci_reset (DEVICE *dptr);
t_stat ci_rdreg (int32 *val, int32 pa, int32 mode);
t_stat ci_wrreg (int32 val, int32 pa, int32 lnt);
void ci_set_int (void);
void ci_clr_int (void);

/* CIBCI adapter data structures

   ci_dev     CI device descriptors
   ci_unit    CI unit
   ci_reg     CI register list
*/

DIB ci_dib = { TR_CI, 0, &ci_rdreg, &ci_wrreg, 0, NVCL (CI) };

UNIT ci_unit = { UDATA (&ci_dec_svc, UNIT_IDLE|UNIT_ATTABLE, 0) };

REG ci_reg[] = {
    { HRDATA (CNFGR, ci_cnfgr, 32) },
    { HRDATA (PMCSR, ci_pmcsr, 32) },
    { HRDATA (MADR, ci_madr, 32) },
    { HRDATA (MDATR, ci_mdatr, 32) },
    { FLDATA (NEXINT, nexus_req[IPL_CI], TR_CI) },
    { NULL }
    };

MTAB ci_mod[] = {
    { MTAB_XTD|MTAB_VDV, TR_CI, "NEXUS", NULL,
      NULL, &show_nexus },
    { MTAB_XTD|MTAB_VDV, 0, "NODE", "NODE",
      &ci_set_node, &ci_show_node },
    { MTAB_XTD|MTAB_VDV, 0, "GROUP", "GROUP",
      &ci_set_group, &ci_show_group },
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
    "CI", &ci_unit, ci_reg, ci_mod,
    1, 0, 0, 0, 0, 0,
    NULL, NULL, &ci_reset,
    NULL, &ci_attach, &ci_detach,
    &ci_dib, DEV_NEXUS | DEV_DEBUG | DEV_DISABLE | DEV_DIS | DEV_CI, 0,
    ci_debug, 0, 0
    };

REGMAP ci_regmap[] = {
    { PSR_OF, CI_PSR },
    { PQBBR_OF, CI_PQBBR },
    { PCQ0CR_OF, CI_PCQ0CR },
    { PCQ1CR_OF, CI_PCQ1CR },
    { PCQ2CR_OF, CI_PCQ2CR },
    { PCQ3CR_OF, CI_PCQ3CR },
    { PSRCR_OF, CI_PSRCR },
    { PECR_OF, CI_PECR },
    { PDCR_OF, CI_PDCR },
    { PICR_OF, CI_PICR },
    { PDFQCR_OF, CI_PDFQCR },
    { PMFQCR_OF, CI_PMFQCR },
    { PMTCR_OF, CI_PMTCR },
    { PMTCR_OF, CI_PMTCR },
    { PMTECR_OF, CI_PMTECR },
    { PFAR_OF, CI_PFAR },
    { PESR_OF, CI_PESR },
    { PPR_OF, CI_PPR },
    { 0, 0 }
    };

/* Read CIBCI adapter register */

t_stat ci_rdreg (int32 *val, int32 pa, int32 lnt)
{
int32 rg = (pa & 0xFFF);
UNIT *uptr = &ci_unit;
REGMAP *p;

if ((rg >= 0x800) && (uptr->ci_state < PORT_UCODERUN)) { /* microcode not running? */
    *val = ci_local_store[(rg - 0x800)];
    return SCPE_OK;
    }
for (p = &ci_regmap[0]; p->offset != 0; p++) {          /* check for port register */
    if (p->offset == rg)                                /* mapped? */
        return ci_dec_rd (uptr, val, p->rg, lnt);
    }
switch (rg) {                                           /* CIBCI specific registers */

    case BI_DTYPE:
        *val = DTYPE_CIBCI;
        sim_debug (DBG_REG, &ci_dev, "DTYPE rd: %08X\n", *val);
        break;

    case BI_CSR:
        *val = ci_biic.csr & BICSR_RD;
        sim_debug (DBG_REG, &ci_dev, "VAXBICSR rd: %08X\n", *val);
        break;

    case BI_BER:
        *val = ci_biic.ber & BIBER_RD;
        sim_debug (DBG_REG, &ci_dev, "BER rd: %08X\n", *val);
        break;

    case BI_EICR:
        *val = ci_biic.eicr & BIECR_RD;
        sim_debug (DBG_REG, &ci_dev, "EINTCSR rd: %08X\n", *val);
        break;

    case BI_IDEST:
        *val = ci_biic.idest & BIID_RD;
        sim_debug (DBG_REG, &ci_dev, "INTRDES rd: %08X\n", *val);
        break;

    case BI_IMSK:
        *val = ci_biic.imsk & BIIMR_RD;
        sim_debug (DBG_REG, &ci_dev, "IPIMR rd: %08X\n", *val);
        break;

    case BI_BCIC:
        *val = ci_biic.bcic & BIBCI_RD;
        sim_debug (DBG_REG, &ci_dev, "BCICR rd: %08X\n", *val);
        break;

    case BI_UIIC:
        *val = ci_biic.uiic;
        sim_debug (DBG_REG, &ci_dev, "UICR rd: %08X\n", *val);
        break;

    case CNFGR_OF:
        *val = ci_cnfgr | (ci_bcmr & CNFGR_BCMR);
        sim_debug (DBG_REG, &ci_dev, "CNFGR rd: %08x\n", *val);
        break;

    case PMCSR_OF:
        *val = ci_pmcsr & PMCSR_RD;
        if (uptr->ci_state == PORT_UNINIT)
            *val |= PMCSR_UI;
        if ((ci_pmcsr & 0x7F00) > 0)                /* any error bits set? */
            *val |= PMCSR_PE;                       /* yes, set parity error */
        sim_debug (DBG_REG, &ci_dev, "PMCSR rd: %08x\n", *val);
        break;

    case MADR_OF:
        *val = ci_madr & MADR_ADDR;
        sim_debug (DBG_REG, &ci_dev, "MADR rd: %08x\n", *val);
        break;

    case MDATR_OF:
        *val = ci_mdatr[ci_madr];
        sim_debug (DBG_REG, &ci_dev, "MDATR rd: %08x\n", *val);
        break;

    case BCAR_OF:
        *val = ci_bcar;
        sim_debug (DBG_REG, &ci_dev, "BCAR rd: %08x\n", *val);
        break;

    case BCMR_OF:
        *val = ci_bcmr;
        sim_debug (DBG_REG, &ci_dev, "BCMR rd: %08x\n", *val);
        break;

    case DMAF_OF + 0x0:
    case DMAF_OF + 0x4:
    case DMAF_OF + 0x8:
    case DMAF_OF + 0xC:
        *val = ci_dmaf[(rg - DMAF_OF) >> 2];
        break;

    default:
        sim_debug (DBG_WRN, &ci_dev, "defaulting on read: %X\n", rg);
        }

return SCPE_OK;
}

/* Write CIBCI adapter register */

t_stat ci_wrreg (int32 val, int32 pa, int32 lnt)
{
int32 rg = (pa & 0xFFF);
UNIT *uptr = &ci_unit;
REGMAP *p;

if ((rg >= 0x800) && (uptr->ci_state < PORT_UCODERUN)) { /* microcode not running? */
    ci_local_store[(rg - 0x800)] = val;
    return SCPE_OK;
    }
for (p = &ci_regmap[0]; p->offset != 0; p++) {          /* check for port register */
    if (p->offset == rg)                                /* mapped? */
        return ci_dec_wr (uptr, val, p->rg, lnt);
    }
switch (rg) {                                           /* case on type */

    case BI_CSR:
        sim_debug (DBG_REG, &ci_dev, "VAXBICSR wr: %08X\n", val);
        if (val & BICSR_RST) {
            ci_reset (&ci_dev);                         /* reset adapter */
            break;
            }
        ci_biic.csr = (ci_biic.csr & ~BICSR_RW) | (val & BICSR_RW);
        break;

    case BI_BER:
        sim_debug (DBG_REG, &ci_dev, "BER wr: %08X\n", val);
        ci_biic.ber = ci_biic.ber & ~(val & BIBER_W1C);
        break;

    case BI_EICR:
        sim_debug (DBG_REG, &ci_dev, "EINTCSR wr: %08X\n", val);
        ci_biic.eicr = (ci_biic.eicr & ~BIECR_RW) | (val & BIECR_RW);
        ci_biic.eicr = ci_biic.eicr & ~(val & BIECR_W1C);
        break;

    case BI_IDEST:
        sim_debug (DBG_REG, &ci_dev, "INTRDES wr: %08X\n", val);
        ci_biic.idest = val & BIID_RW;
        break;

    case BI_IMSK:
        sim_debug (DBG_REG, &ci_dev, "IPIMR wr: %08X\n", val);
        ci_biic.imsk = val & BIIMR_RW;
        break;

    case BI_BCIC:
        sim_debug (DBG_REG, &ci_dev, "BCICR wr: %08X\n", val);
        ci_biic.bcic = val & BIBCI_RW;
        break;

    case BI_UIIC:
        sim_debug (DBG_REG, &ci_dev, "UICR wr: %08X\n", val);
        break;

    case CNFGR_OF:
        sim_debug (DBG_REG, &ci_dev, "CNFGR wr: %08X\n", val);
        ci_cnfgr = (ci_cnfgr & ~CNFGR_RW) | (val & CNFGR_RW); /* Set RW bits */
        break;

    case PMCSR_OF:
        sim_debug (DBG_REG, &ci_dev, "PMCSR wr: %08X\n", val);
        if (val & PMCSR_MI) {                           /* Maintenance Initialise */
            ci_reset (&ci_dev);
            break;
            }
        ci_pmcsr &= ~(val & PMCSR_W1C);                 /* Clear W1C bits */
        ci_pmcsr = (ci_pmcsr & ~PMCSR_RW) | (val & PMCSR_RW); /* Set RW bits */

        if (val & PMCSR_MIF)                            /* interrupt W1C */
            ci_clr_int();

        if ((val & PMCSR_PSA) && (uptr->ci_state < PORT_UCODERUN)) {
            ci_set_state (uptr, PORT_UCODERUN);         /* Start microcode */
            }
        break;

    case MADR_OF:
        sim_debug (DBG_REG, &ci_dev, "MADR wr: %08X\n", val);
        ci_madr = (val & 0x1FFF);
        break;

    case MDATR_OF:
        sim_debug (DBG_REG, &ci_dev, "MDATR wr: %08X\n", val);
        ci_mdatr[ci_madr] = val;
        break;

    case BCAR_OF:
        sim_debug (DBG_REG, &ci_dev, "BCAR wr: %08X\n", val);
        ci_bcar = (val & BCAR_ADDR) << BCAR_V_ADDR;
        break;

    case BCMR_OF:
        sim_debug (DBG_REG, &ci_dev, "BCMR wr: %08X\n", val);
        ci_bcmr = (val & BCMR_MSK) << BCMR_V_MSK;
        break;

    case DMAF_OF + 0x0:
    case DMAF_OF + 0x4:
    case DMAF_OF + 0x8:
    case DMAF_OF + 0xC:
        ci_dmaf[(rg - DMAF_OF) >> 2] = val;
        break;

    default:
        sim_debug (DBG_WRN, &ci_dev, "defaulting on write: %X\n", rg);
        }

return SCPE_OK;
}

void ci_set_int ()
{
ci_pmcsr |= PMCSR_MIF;
if (ci_pmcsr & PMCSR_MIE)
    SET_NEXUS_INT (CI);
return;
}

void ci_clr_int ()
{
ci_pmcsr &= ~PMCSR_MIF;
CLR_NEXUS_INT (CI);
return;
}

/* Reset CIBCI adapter */

t_stat ci_reset (DEVICE *dptr)
{
int32 i;

ci_biic.csr = (1u << BICSR_V_IF) | BICSR_STS | (TR_CI & BICSR_NODE);
ci_biic.ber = 0;
ci_biic.eicr = 0;
ci_biic.idest = 0;
ci_biic.imsk = 0;
ci_biic.uiic = BIICR_EXV;
ci_cnfgr = 0;
ci_pmcsr = PMCSR_UI;
ci_madr = 0;
ci_bcar = 0;
ci_bcmr = 0;
for (i = 0; i < 4; i++)
    ci_dmaf[i] = 0;
for (i = 0; i < 8192; i++)
    ci_mdatr[i] = 0;
ci_dec_reset (dptr);
return SCPE_OK;
}
