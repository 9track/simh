/* vaxbi_ci.c: VAXBI Computer Interconnect adapter

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

   ci             CIBCA/CIBCI CI adapter
*/

#include "vax_defs.h"
#include "vax_ci.h"
#include "vax_ci_dec.h"

/* CIBCA Registers */

#define CNFGR_OF        0x0
#define PMCSR_OF        0x401                           /* port maintenanace CSR */
#define MADR_OF         0x402                           /* maintenanace address */
#define MDATR_OF        0x403                           /* maintenanace data */
#define EADR_OF         0x410                           /* EEPROM address */

#define DTYPE_R01
#define DTYPE_R02
#define DTYPE_R03
#define DTYPE_R04
#define DTYPE_BCA       0x00400000                      /* CIBCA-BA */
#define DTYPE_EPRM      0x00800000                      /* EEPROM size (0 = 8k, 1 = 32k) */
#define DTYPE_R20       0x01000000                      /* revision 20 */
#define DTYPE_R21       0x02000000                      /* revision 21 */
#define DTYPE_R22       0x04000000                      /* revision 22 */
#define DTYPE_R23       0x08000000                      /* revision 23 */
#define DTYPE_R24       0x10000000                      /* revision 24 */
#define DTYPE_P4K       0x40000000                      /* packet buffer size 4k */
#define DTYPE_FDX       0x80000000                      /* full duplex */

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
#define PMCSR_RW        (PMCSR_PSA | PMCSR_WP | PMCSR_MIE | \
                         PMCSR_MTD | PMCSR_RSVD)
#define PMCSR_W1C       (PMCSR_PE | PMCSR_CSPE | PMCSR_LSPE | \
                         PMCSR_RBPE | PMCSR_TMPE | PMCSR_IPE | \
                         PMCSR_OPE | PMCSR_TBPE | PMCSR_MIF)

#define MADR_ADDR       0x1FFF

/* Standard Port Registers */

#define PQBBR_OF        BI_GPR0
#define PFAR_OF         BI_GPR1
#define PPR_OF          BI_GPR2
#define PESR_OF         BI_GPR3
#define PSR_OF          0x400
#define PCQ0CR_OF       0x404
#define PCQ1CR_OF       0x405
#define PCQ2CR_OF       0x406
#define PCQ3CR_OF       0x407
#define PSRCR_OF        0x408
#define PECR_OF         0x409
#define PDCR_OF         0x40A
#define PICR_OF         0x40B
#define PDFQCR_OF       0x40C
#define PMFQCR_OF       0x40D
#define PMTCR_OF        0x40E
#define PMTECR_OF       0x40F

/* CIBCA Port Parameters */

#define RPORT_TYPE      0x80000002
#define CODE_REV        0x00700040
#define FUNC_MASK       0xFFFF0F00
#define INT_BUF_LEN     0x3F9

/* CIBCA Port States */

#define PORT_UCODERUN   1                               /* microcode running */

BIIC uba_biic;                                          /* BIIC standard registers */
uint32 ci_pmcsr;                                        /* port maintenance csr */
uint32 ci_madr;                                         /* mainteneance addr reg */
uint32 ci_mdatr[8192];                                  /* maintenanace data reg */
uint32 ci_local_store[2048];

extern uint32 nexus_req[NEXUS_HLVL];

t_stat ci_reset (DEVICE *dptr);
t_stat ci_rdreg (int32 *val, int32 pa, int32 mode);
t_stat ci_wrreg (int32 val, int32 pa, int32 lnt);
void ci_set_int (void);
void ci_clr_int (void);

/* CIBCA adapter data structures

   ci_dev     CI device descriptors
   ci_unit    CI unit
   ci_reg     CI register list
*/

DIB ci_dib = { TR_CI, 0, &ci_rdreg, &ci_wrreg, 0, NVCL (CI) };

UNIT ci_unit = { UDATA (&ci_svc, UNIT_IDLE|UNIT_ATTABLE, 0) };

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
    { 0 }
    };

DEVICE ci_dev = {
    "CI", &ci_unit, ci_reg, ci_mod,
    1, 0, 0, 0, 0, 0,
    NULL, NULL, &ci_reset,
    NULL, &ci_attach, &ci_detach,
    &ci_dib, DEV_NEXUS | DEV_DEBUG | DEV_DISABLE | DEV_DIS, 0,
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

/* Read CIBCA adapter register */

t_stat ci_rdreg (int32 *val, int32 pa, int32 lnt)
{
int32 rg;
REGMAP *p;

rg = NEXUS_GETOFS (pa);                                 /* get offset */

if ((rg >= 0x400) && (ci_state < PORT_UCODERUN)) {      /* microcode not running? */
    *val = ci_local_store[(rg - 0x400)];
    return SCPE_OK;
    }
for (p = &ci_regmap[0]; p->offset != 0; p++) {          /* check for port register */
    if (p->offset == rg)                                /* mapped? */
        return ci_dec_rd (val, p->rg, lnt);
    }
switch (rg) {                                           /* CIBCA specific registers */

    case BI_DTYPE:
        *val = DTYPE_CIBCA;
        break;

    case BI_CSR:
        *val = ci_biic.csr & BICSR_RD;
        break;

    case BI_BER:
        *val = ci_biic.ber & BIBER_RD;
        break;

    case BI_EICR:
        *val = ci_biic.eicr & BIECR_RD;
        break;

    case BI_IDEST:
        *val = ci_biic.idest & BIID_RD;
        break;

    case BI_BCIC:
        *val = ci_biic.bcic;
        break;

    case BI_UIIC:
        *val = ci_biic.uiic;
        break;

    case PMCSR_OF:
        if (ci_state > PORT_UNINIT)
            ci_pmcsr = ci_pmcsr & ~PMCSR_UI;
        else
            ci_pmcsr = ci_pmcsr | PMCSR_UI;
        *val = (ci_pmcsr & 0xFFFE);                     /* TODO: Need defined mask */
        sim_debug (DBG_REG, &ci_dev, "PMCSR rd: %08x\n", *val);
        break;

    case MADR_OF:
        *val = (ci_madr & MADR_ADDR);
        break;

    case MDATR_OF:
        *val = ci_mdatr[ci_madr];
        break;

    case EADR_OF:
        *val = 0;
        break;

    default:
        sim_debug (DBG_WRN, &ci_dev, "defaulting on read: %X\n", rg);
        }

return SCPE_OK;
}

/* Write CIBCA adapter register */

t_stat ci_wrreg (int32 val, int32 pa, int32 lnt)
{
int32 rg;
int32 src_ipa, src_ipp;
t_stat r;
UNIT *uptr = &ci_unit;
REGMAP *p;

rg = NEXUS_GETOFS (pa);                                 /* get offset */

if ((rg >= 0x400) && (ci_state < PORT_UCODERUN)) {      /* microcode not running? */
    ci_local_store[(rg - 0x400)] = val;
    return SCPE_OK;
    }
for (p = &ci_regmap[0]; p->offset != 0; p++) {          /* check for port register */
    if (p->offset == rg)                                /* mapped? */
        return ci_dec_wr (val, p->rg, lnt);
    }
switch (rg) {                                           /* case on type */

    case BI_CSR:
        if (val & BICSR_RST) {
            ci_reset (&ci_dev);                         /* reset adapter */
            break;
            }
        ci_biic.csr = (ci_biic.csr & ~BICSR_RW) | (val & BICSR_RW);
        break;

    case BI_BER:
        ci_biic.ber = ci_biic.ber & ~(val & BIBER_W1C);
        break;

    case BI_EICR:
        ci_biic.eicr = (ci_biic.eicr & ~BIECR_RW) | (val & BIECR_RW);
        ci_biic.eicr = ci_biic.eicr & ~(val & BIECR_W1C);
        break;

    case BI_IDEST:
        ci_biic.idest = val & BIID_RW;
        break;

    case BI_UIIC:
        break;

    case PMCSR_OF:
        sim_debug (DBG_REG, &ci_dev, "PMCSR wr: %08X\n", val);
        if (val & PMCSR_MI) {                           /* Maintenance Initialise */
            ci_reset (&ci_dev);
            break;
            }
        ci_pmcsr &= ~(val & PMCSR_W1C);                 /* Clear W1C bits */
        ci_pmcsr = (ci_pmcsr & ~PMCSR_RW) | (val & PMCSR_RW); /* Set RW bits */

        if (ci_state > PORT_UNINIT)
            ci_pmcsr = ci_pmcsr & ~PMCSR_UI;
        else
            ci_pmcsr = ci_pmcsr | PMCSR_UI;

        if ((ci_pmcsr & 0x7F00) > 0)
            ci_pmcsr |= PMCSR_PE;                       /* Parity Error bit is composite of other parity error bits */
        else ci_pmcsr &= ~PMCSR_PE;

        if (val & PMCSR_MIF)                            /* interrupt W1C */
            ci_clr_int();

        if ((val & PMCSR_PSA) && (ci_state < PORT_UCODERUN)) {
            ci_set_state (PORT_UCODERUN);               /* Start microcode */
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

/* Reset CIBCA adapter */

t_stat ci_reset (DEVICE *dptr)
{
int32 i;
t_stat r;

ci_biic.csr = (1u << BICSR_V_IF) | BICSR_STS | (TR_CI & BICSR_NODE);
ci_biic.ber = 0;
ci_biic.eicr = 0;
ci_biic.idest = 0;
ci_biic.uiic = BIICR_EXV;
ci_pmcsr = PMCSR_UI;
ci_madr = 0;
for (i = 0; i < 8192; i++)
    ci_mdatr[i] = 0;
ci_port_reset (dptr);
ci_dec_reset (dptr);
return SCPE_OK;
}
