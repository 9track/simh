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

#define PMCSR_OF        0x401                           /* port maintenanace CSR */
#define MADR_OF         0x402                           /* maintenanace address */
#define MDATR_OF        0x403                           /* maintenanace data */
#define EADR_OF         0x410                           /* EEPROM address */

#define DTYPE_BCA       0x00400000                      /* CIBCA-BA */
#define DTYPE_EPRM      0x00800000                      /* EEPROM size (0 = 8k, 1 = 32k) */
#define DTYPE_R20       0x01000000                      /* revision 20 */
#define DTYPE_R21       0x02000000                      /* revision 21 */
#define DTYPE_R22       0x04000000                      /* revision 22 */
#define DTYPE_R23       0x08000000                      /* revision 23 */
#define DTYPE_R24       0x10000000                      /* revision 24 */
#define DTYPE_P4K       0x40000000                      /* packet buffer size 4k */
#define DTYPE_FDX       0x80000000                      /* full duplex */

/* Port maintenance control/status register */

#define PMCSR_START     0x00000001                      /* start */
#define PMCSR_MTD       0x00000002                      /* maintenance timer disable */
#define PMCSR_WP        0x00000010                      /* wrong parity */
#define PMCSR_MIE       0x00000020                      /* maintenance interrupt enable */
#define PMCSR_HALT      0x00000080                      /* halt sequencer */
#define PMCSR_BTO       0x00000100                      /* BI bus timeout */
#define PMCSR_IIPE      0x00000200                      /* II bus parity error */
#define PMCSR_MBIE      0x00000400                      /* map BCI/BI error */
#define PMCSR_CPE       0x00000800                      /* CILP parity error */
#define PMCSR_XMPE      0x00001000                      /* transmit buffer parity error */
#define PMCSR_IBPE      0x00002000                      /* internal bus parity error */
#define PMCSR_CSPE      0x00004000                      /* control store parity error */
#define PMCSR_BCIPE     0x00008000                      /* BCI parity error */
#define PMCSR_RW        (PMCSR_HALT | PMCSR_MIE | PMCSR_WP | \
                         PMCSR_MTD)

/* Port status register - CIBCA specific bits */

#define PSR_MISC        0x00000080                      /* miscellaneous error */
#define PSR_MTE         0x00000100                      /* maintenance error */
#define PSR_MIF         0x00000200                      /* maintenance interrupt */
#define PSR_UNI         0x00000400                      /* uninitialised */
#define PSR_NRSPE       0x00008000                      /* no response error */

#define MADR_ADDR       0x1FFF
#define MADR_M_WORD     0xC000
#define MADR_V_WORD     14

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

BIIC ci_biic;                                           /* BIIC standard registers */
uint32 ci_pmcsr;                                        /* port maintenance csr */
uint32 ci_psr_bi;                                       /* port status reg (CIBCA specific) */
uint32 ci_madr;                                         /* mainteneance addr reg */
uint32 ci_mdatr[0x4000];                                /* maintenanace data reg */
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

UNIT ci_unit = { UDATA (&ci_dec_svc, UNIT_IDLE|UNIT_ATTABLE, 0) };

REG ci_reg[] = {
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

/* Read CIBCA adapter register */

t_stat ci_rdreg (int32 *val, int32 pa, int32 lnt)
{
int32 rg;
UNIT *uptr = &ci_unit;
REGMAP *p;
t_stat r;

rg = NEXUS_GETOFS (pa);                                 /* get offset */

if (rg >= 0x700) {
    sim_printf ("VCD read: %X at %08X\n", pa, fault_PC);
    *val = 0;
    return SCPE_OK;
    }
if ((rg >= 0x414) && (uptr->ci_state < PORT_UCODERUN)) { /* microcode not running? */
    *val = ci_local_store[(rg - 0x400)];
    sim_debug (DBG_REG, &ci_dev, "LOCAL STORE rd: %X = %08X\n", pa, *val);
    return SCPE_OK;
    }
for (p = &ci_regmap[0]; p->offset != 0; p++) {          /* check for port register */
    if (p->offset == rg) {                              /* mapped? */
        r = ci_dec_rd (uptr, val, p->rg, lnt);
        if (r != SCPE_OK)
            return r;
        if (p->rg == CI_PSR) {
            *val = *val | ci_psr_bi;
            if (uptr->ci_state <= PORT_UCODERUN)
                *val = *val | PSR_UNI;
            }
        return SCPE_OK;
        }
    }
switch (rg) {                                           /* CIBCA specific registers */

    case BI_DTYPE:
        *val = DTYPE_CIBCA;
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

    case BI_BCIC:
        *val = ci_biic.bcic;
        sim_debug (DBG_REG, &ci_dev, "BCICSR rd: %08X\n", *val);
        break;

    case BI_UIIC:
        *val = ci_biic.uiic;
        sim_debug (DBG_REG, &ci_dev, "UINTRCSR rd: %08X\n", *val);
        break;

    case PMCSR_OF:
        *val = ci_pmcsr;
        sim_debug (DBG_REG, &ci_dev, "PMCSR rd: %08X\n", *val);
        break;

    case MADR_OF:
        *val = (ci_madr >> 2) & MADR_ADDR;
        *val = *val | ((ci_madr << MADR_V_WORD) & MADR_M_WORD);
        sim_debug (DBG_REG, &ci_dev, "MADR rd: %08X\n", *val);
        break;

    case MDATR_OF:
        *val = ci_mdatr[ci_madr];
        sim_debug (DBG_REG, &ci_dev, "MDATR rd: %04X = %08X\n", ci_madr, *val);
        break;

    case EADR_OF:
        *val = 0;
        sim_debug (DBG_REG, &ci_dev, "EEPROM rd: %08X\n", *val);
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
UNIT *uptr = &ci_unit;
REGMAP *p;

rg = NEXUS_GETOFS (pa);                                 /* get offset */

if (rg >= 0x700) {
    sim_printf ("VCD write: %X = %X at %08X\n", pa, val, fault_PC);
    return SCPE_OK;
    }
if ((rg >= 0x414) && (uptr->ci_state < PORT_UCODERUN)) { /* microcode not running? */
        sim_debug (DBG_REG, &ci_dev, "LOCAL STORE wr: %X = %08X\n", pa, val);
    ci_local_store[(rg - 0x400)] = val;
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

    case BI_BCIC:
        sim_debug (DBG_REG, &ci_dev, "BCICSR wr: %08X\n", val);
        ci_biic.bcic = (ci_biic.bcic & ~BICSR_RW) | (val & BICSR_RW);
        break;

    case BI_UIIC:
        sim_debug (DBG_REG, &ci_dev, "UINTRCSR wr: %08X\n", val);
        break;

    case PMCSR_OF:
        sim_debug (DBG_REG, &ci_dev, "PMCSR wr: %08X\n", val);
        if (val & PMCSR_START) {
            ci_reset (&ci_dev);
            ci_set_state (uptr, PORT_UCODERUN);         /* start microcode */
            break;
            }
        else if (val & PMCSR_HALT)
            ci_set_state (uptr, PORT_UNINIT);           /* stop microcode */
        ci_pmcsr = (ci_pmcsr & ~PMCSR_RW) | (val & PMCSR_RW); /* set RW bits */
        break;

    case MADR_OF:
        sim_debug (DBG_REG, &ci_dev, "MADR wr: %08X\n", val);
        ci_madr = ((val & MADR_ADDR) << 2);
        sim_debug (DBG_REG, &ci_dev, "    -> %08X\n", ci_madr);
        ci_madr = ci_madr | ((val & MADR_M_WORD) >> MADR_V_WORD);
        sim_debug (DBG_REG, &ci_dev, "    -> %08X\n", ci_madr);
        break;

    case MDATR_OF:
        sim_debug (DBG_REG, &ci_dev, "MDATR wr: %04X = %08X\n", ci_madr, val);
        ci_mdatr[ci_madr++] = val;
        if ((ci_madr & 0x3) == 0x3)
            ci_madr++;
        ci_madr = ci_madr & 0xFFFF;
        break;

    default:
        sim_debug (DBG_WRN, &ci_dev, "defaulting on write: %X\n", rg);
        }
return SCPE_OK;
}

void ci_set_int ()
{
ci_psr_bi |= PSR_MIF;
if (ci_pmcsr & PMCSR_MIE)
    SET_NEXUS_INT (CI);
return;
}

void ci_clr_int ()
{
ci_psr_bi &= ~PSR_MIF;
CLR_NEXUS_INT (CI);
return;
}

/* Reset CIBCA adapter */

t_stat ci_reset (DEVICE *dptr)
{
int32 i;

ci_biic.csr = (1u << BICSR_V_IF) | BICSR_STS | (TR_CI & BICSR_NODE);
ci_biic.ber = 0;
ci_biic.eicr = 0;
ci_biic.idest = 0;
ci_biic.uiic = BIICR_EXV;
ci_pmcsr = 0;
ci_psr_bi = 0;
ci_madr = 0;
ci_dec_reset (dptr);
return SCPE_OK;
}
