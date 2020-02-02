/* vax_shac.c: SHAC adapter

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

   sh             SHAC adapter
*/

#include "vax_defs.h"
#include "vax_ci.h"
#include "vax_ci_dec.h"

/* SHAC Registers */

#define SSWCR_OF        0xC
#define SSHMA_OF        0x11

/* Standard Port Registers */

#define PQBBR_OF        0x12
#define PSR_OF          0x13
#define PESR_OF         0x14
#define PFAR_OF         0x15
#define PPR_OF          0x16
#define PMCSR_OF        0x17
#define PCQ0CR_OF       0x20
#define PCQ1CR_OF       0x21
#define PCQ2CR_OF       0x22
#define PCQ3CR_OF       0x23
#define PDFQCR_OF       0x24
#define PMFQCR_OF       0x25
#define PSRCR_OF        0x26
#define PECR_OF         0x27
#define PDCR_OF         0x28
#define PICR_OF         0x29
#define PMTCR_OF        0x2A
#define PMTECR_OF       0x2B

/* SHAC Port Parameters */

#define RPORT_TYPE      0x80000022
#define CODE_REV        0x03060D22
#define FUNC_MASK       0xFFFF0D00
#define INT_BUF_LEN     0x1010

uint32 sh_shma;
int32 shac_int = 0;

int32 shac_rd (int32 pa);
void shac_wr (int32 pa, int32 val, int32 lnt);


/* SHAC adapter data structures

   shac_dev     SHAC device descriptors
   shac_unit    SHAC unit
   shac_reg     SHAC register list
*/

UNIT shac_unit = { UDATA (&shac_svc, UNIT_IDLE|UNIT_ATTABLE, 0) };

REG shac_reg[] = {
    { NULL }
    };

MTAB shac_mod[] = {
    { MTAB_XTD|MTAB_VDV, 0, "NODE", "NODE",
      &ci_set_node, &ci_show_node },
    { MTAB_XTD|MTAB_VDV, 0, "GROUP", "GROUP",
      &ci_set_group, &ci_show_group },
    { 0 }
    };

DEBTAB shac_debug[] = {
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

DEVICE shac_dev = {
    "SHAC", &shac_unit, shac_reg, shac_mod,
    1, 0, 0, 0, 0, 0,
    NULL, NULL, &ci_reset,
    NULL, &ci_attach, &ci_detach,
    NULL, DEV_DEBUG | DEV_CI, 0,
    ci_debug, 0, 0
    };

REGMAP shac_regmap = {
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
    { PMDQCR_OF, CI_PMFQCR },
    { PMTCR_OF, CI_PMTCR },
    { PMTCR_OF, CI_PMTCR },
    { PMTECR_OF, CI_PMTECR },
    { PFAR_OF, CI_PFAR },
    { PESR_OF, CI_PESR },
    { PPR_OF, CI_PPR },
    { 0, 0 }
    };

int32 shac_rd (int32 pa)
{
int32 rg = (pa >> 2) &  0x3F;
int32 val;
REGMAP *p;

for (p = &shac_regmap[0]; p->addr != 0; p++) {          /* check for port register */
    if (p->addr == rg) {                                /* mapped? */
        val = ci_dec_rd (&shac_unit, val, p->rg, lnt);
        if ((p->rg == CI_PQBBR) && (val == 0))          /* PQBBR and not set? */
            break;                                      /* yes, return version instead */
        else
            return val;
        }
    }
switch (rg) {

    case SSWCR_OF:
        sim_debug (DBG_REG, &shac_dev, "SSWCR rd: %08X(f) at %08X\n", 0, fault_PC);
        return 0;

    case SSHMA_OF:
        sim_debug (DBG_REG, &shac_dev, "SSHMA rd: %08X at %08X\n", sh_shma, fault_PC);
        return sh_shma;

    case PQBBR_OF:
        sim_debug (DBG_REG, &shac_dev, "PQBBR(ver) rd: %08X at %08X\n", 0x03060022);
        return 0x03060022;                              /* SHAC version */

    default:
        printf ("SHAC: Unknown address (read) %08X\n", pa);
        }
}

void shac_wr (int32 pa, int32 val, int32 lnt)
{
int32 rg = (pa >> 2) &  0x3F;
REGMAP *p;

for (p = &shac_regmap[0]; p->addr != 0; p++) {          /* check for port register */
    if (p->addr == rg)                                  /* mapped? */
        return ci_dec_wr (&shac_unit, val, p->rg, lnt);
    }
switch (rg) {

    case SSWCR_OF:
        sim_debug (DBG_REG, &shac_dev, "SSWCR wr: %08X at %08X\n", val, fault_PC);
        if (val & 0xFFFFFFFF) {                         /* Maintenance Initialise */
            shac_reset (&shac_dev);
            break;
        }
        break;

    case SSHMA_OF:
        sim_debug (DBG_REG, &shac_dev, "SSHMA wr: %08X at %08X\n", val, fault_PC);
        sh_shma = val & 0x3FFFFFF0;
        break;

    default:
        printf ("SHAC: Unknown address (write) %08X\n", pa);
        }
}

void ci_set_int ()
{
shac_int = 1;
return;
}

void ci_clr_int ()
{
shac_int = 0;
return;
}

/* Reset SHAC adapter */

t_stat shac_reset (DEVICE *dptr)
{
ci_ppr |= ((0x1010 & PPR_M_IBLEN) << PPR_V_IBLEN);  /* SHAC: set internal buffer length */
sh_shma = 0;
shac_int = 0;
ci_dec_reset (dptr);
return SCPE_OK;
}
