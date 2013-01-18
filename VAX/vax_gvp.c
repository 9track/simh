/* vax_gvp.c - Generic VAX Port Memory Management

   Copyright (c) 2017, Matt Burke
   This module incorporates code from SimH, Copyright (c) 1998-2008, Robert M Supnik

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
   THE AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name(s) of the author(s) shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from the author(s).
*/

#include "vax_defs.h"
#include "vax_gvp.h"

/* TLB fill

   This routine fills the TLB after a tag or access mismatch, or
   on a write if pte<m> = 0.  It fills the TLB and returns the
   pte to the caller.  On an error, it aborts directly to the
   fault handler in the CPU.

   If called from map (VAX PROBEx), the error status is returned
   to the caller, and no fault occurs.
*/

TLBENT gvp_fill (GVPMMU *mmu, uint32 va, int32 lnt)
{
int32 ptidx = (((uint32) va) >> 7) & ~03;
int32 tlbpte, ptead, pte, tbi, vpn;
static TLBENT zero_pte = { 0, 0 };

if (va & VA_S0) {                                       /* system space? */
    if (ptidx >= mmu->slr) {                            /* system */
        mmu->status = SCPE_NXM;
        return zero_pte;
        }
    ptead = (mmu->sbr + ptidx) & PAMASK;
    }
else {
    mmu->status = SCPE_NXM;
    return zero_pte;
    }
pte = GVP_ReadL (mmu, ptead);                           /* read pte */
if (mmu->status) return zero_pte;
if ((pte & PTE_V) == 0) {                               /* more checks needed? */
    if ((pte & PTE_M) != 0) {                           /* yes, check valid */
        mmu->status = SCPE_NXM;
        return zero_pte;
        }
    if (PTE_GETSW(pte) == 2) {                          /* check for global pt index */
        ptidx = (pte & PTE_PFN) << 2;
        if (ptidx > mmu->glr) {
            mmu->status = SCPE_NXM;
            return zero_pte;
            }
        ptead = mmu->gbr + ptidx;
        if ((ptead & VA_S0) == 0) {
            mmu->status = SCPE_NXM;                     /* gpte must be sys */
            return zero_pte;
            }
        ptidx = ((uint32) ptead) >> 7;                  /* xlate like sys */
        if (ptidx >= mmu->slr) {
            mmu->status = SCPE_NXM;
            return zero_pte;
            }
        pte = GVP_ReadL (mmu, (mmu->sbr + ptidx) & PAMASK); /* get system pte */
        if (mmu->status) return zero_pte;
        if ((pte & PTE_V) == 0) {                       /* more checks needed? */
            if (((pte & PTE_M) != 0) ||
                (PTE_GETSW(pte) == 2))                  /* yes, check valid */
                mmu->status = SCPE_NXM;
                return zero_pte;
            }
        }
    }
tlbpte = ((pte << VA_N_OFF) & TLB_PFN);                 /* set addr */
vpn = VA_GETVPN (va);
tbi = VA_GETTBI (vpn);
mmu->tlb[tbi].tag = vpn;                                /* store tlb ent */
mmu->tlb[tbi].pte = tlbpte;
return mmu->tlb[tbi];
}

/* Zap whole tb */

void gvp_zap_tb (GVPMMU *mmu)
{
size_t i;

mmu->status = SCPE_OK;
for (i = 0; i < VA_TBSIZE; i++)
    mmu->tlb[i].tag = mmu->tlb[i].pte = -1;
return;
}

/* Zap single tb entry corresponding to va */

void gvp_zap_tb_ent (GVPMMU *mmu, uint32 va)
{
int32 tbi = VA_GETTBI (VA_GETVPN (va));

mmu->status = SCPE_OK;
if (va & VA_S0)
    mmu->tlb[tbi].tag = mmu->tlb[tbi].pte = -1;
else {
    mmu->fva = va;
    mmu->status = SCPE_NXM;
    }
return;
}

/* TLB reset */

t_stat gvp_tlb_reset (GVPMMU *mmu)
{
size_t i;

mmu->status = SCPE_OK;
for (i = 0; i < VA_TBSIZE; i++)
    mmu->tlb[i].tag = mmu->tlb[i].pte = -1;
return SCPE_OK;
}

/* inserts entry on specified queue
   if queue is busy returns 0 */

int32 gvp_insqti (GVPMMU *mmu, int32 h, int32 d)
{
int32 a, c, t;

if ((h == d) || ((h | d) & 07)) {                       /* h, d quad align? */
    // TODO: error                                      /* These should be datastructure errors */
    return 0;                                           /* reported by the port */
    }
a = GVP_Read (mmu, h, L_LONG);                          /* a <- (h), wchk */
if (a == 0) {
    GVP_Write (mmu, h, a | 1, L_LONG);                  /* acquire interlock */
    a = a + h;                                          /* abs addr of a */
    if (GVP_Test (mmu, a) < 0)
        GVP_Write (mmu, h, a - h, L_LONG);              /* wtst a, rls if err */
    GVP_Write (mmu, a + 4, d - a, L_LONG);              /* (a+4) <- d-a, flt ok */
    GVP_Write (mmu, d, a - d, L_LONG);                  /* (d) <- a-d */
    GVP_Write (mmu, d + 4, h - d, L_LONG);              /* (d+4) <- h-d */
    GVP_Write (mmu, h, d - h, L_LONG);                  /* (h) <- d-h, rls int */
    return 2;                                           /* success - queue was empty */
    }
else {
    if (a & 06) {                                       /* chk quad align */
        // TODO: error
        return 0;
        }
    if (a & 01) {                                       /* busy, cc = 0001 */
        // TODO: interlock error
        return 0;
        }
    GVP_Write (mmu, h, a | 1, L_LONG);                  /* acquire interlock */
    c = GVP_Read (mmu, h + 4, L_LONG) + h;              /* c <- (h+4) + h */
    if (c & 07) {                                       /* c quad aligned? */
        GVP_Write (mmu, h, a, L_LONG);                  /* release interlock */
        // TODO: error                                  /* fault */
        return 0;
        }
    if (GVP_Test (mmu, c) < 0)
        GVP_Write (mmu, h, a, L_LONG);                  /* wtst c, rls if err */
    GVP_Write (mmu, c, d - c, L_LONG);                  /* (c) <- d-c, flt ok */
    GVP_Write (mmu, d, h - d, L_LONG);                  /* (d) <- h-d */
    GVP_Write (mmu, d + 4, c - d, L_LONG);              /* (d+4) <- c-d */
    GVP_Write (mmu, h + 4, d - h, L_LONG);              /* (h+4) <- d-h */
    GVP_Write (mmu, h, a, L_LONG);                      /* release interlock */
    return 1;                                           /* success - queue was not empty */
    }
}

/* returns pointer to queue entry
   if queue is busy returns 0 */

int32 gvp_remqhi (GVPMMU *mmu, int32 h)
{
int32 ar, a, b, t;
if (h & 07) {                                           /* h quad aligned? */
    // TODO: error
    return 0;
    }
ar = GVP_Read (mmu, h, L_LONG);                         /* ar <- (h) */
if (ar & 06) {                                          /* a quad aligned? */
    // TODO: error
    return 0;
    }
if (ar & 01) {                                          /* busy, cc = 0011 */
    // TODO: interlock error
    return 0;
    }
a = ar + h;                                             /* abs addr of a */
if (ar) {                                               /* queue not empty? */
    GVP_Write (mmu, h, ar | 1, L_LONG);                 /* acquire interlock */
    if (GVP_Test (mmu, a) < 0)                          /* read tst a */
         GVP_Write (mmu, h, ar, L_LONG);                /* release if error */
    b = GVP_Read (mmu, a, L_LONG) + a;                  /* b <- (a)+a, flt ok */
    if (b & 07) {                                       /* b quad aligned? */
        GVP_Write (mmu, h, ar, L_LONG);                 /* release interlock */
        // TODO: interlock error                        /* fault */
        return 0;
        }
    if (GVP_Test (mmu, b) < 0)                          /* write test b */
        GVP_Write (mmu, h, ar, L_LONG);                 /* release if err */
    GVP_Write (mmu, b + 4, h - b, L_LONG);              /* (b+4) <- h-b, flt ok */
    GVP_Write (mmu, h, b - h, L_LONG);                  /* (h) <- b-h, rls int */
    }
return a;                                               /* return result */
}
