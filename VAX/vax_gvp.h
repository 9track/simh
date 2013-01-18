/* vax_gvp.h - Generic VAX Port Memory Management

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

#ifndef VAX_GVP_H_
#define VAX_GVP_H_ 1

#include "vax_defs.h"

typedef struct {
    TLBENT      tlb[VA_TBSIZE];
    uint32      sbr;                                    /* system page table base */
    uint32      slr;                                    /* system page table length */
    uint32      gbr;                                    /* global page table base */
    uint32      glr;                                    /* global page table length */
    uint32      fva;                                    /* failing address */
    t_stat      status;                                 /* read/write status */
    } GVPMMU;

extern void gvp_zap_tb (GVPMMU *mmu);
extern void gvp_zap_tb_ent (GVPMMU *mmu, uint32 va);
extern t_stat gvp_tlb_reset (GVPMMU *mmu);
extern TLBENT gvp_fill (GVPMMU *mmu, uint32 va, int32 lnt);
static SIM_INLINE int32 GVP_ReadB (GVPMMU *mmu, uint32 pa);
static SIM_INLINE int32 GVP_ReadW (GVPMMU *mmu, uint32 pa);
static SIM_INLINE int32 GVP_ReadL (GVPMMU *mmu, uint32 pa);
static SIM_INLINE int32 GVP_ReadU (GVPMMU *mmu, uint32 pa, int32 lnt);
static SIM_INLINE void GVP_WriteB (GVPMMU *mmu, uint32 pa, int32 val);
static SIM_INLINE void GVP_WriteW (GVPMMU *mmu, uint32 pa, int32 val);
static SIM_INLINE void GVP_WriteL (GVPMMU *mmu, uint32 pa, int32 val);
static SIM_INLINE void GVP_WriteU (GVPMMU *mmu, uint32 pa, int32 val, int32 lnt);

int32 gvp_insqti (GVPMMU *mmu, int32 h, int32 d);
int32 gvp_remqhi (GVPMMU *mmu, int32 h);

/* Read and write virtual

   These routines logically fall into three phases:

   1.   Look up the virtual address in the translation buffer, calling
        the fill routine on a tag mismatch.  The fill routine handles
        all errors.  If the resulting physical address is aligned,
        do an aligned physical read or write.
   2.   Test for unaligned across page boundaries.  If cross page, look
        up the physical address of the second page.  If not cross page,
        the second physical address is the same as the first.
   3.   Using the two physical addresses, do an unaligned read or
        write, with three cases: unaligned long, unaligned word within
        a longword, unaligned word crossing a longword boundary.

   Note that these routines do not handle quad or octa references.
*/

/* Read virtual

   Inputs:
        va      =       virtual address
        lnt     =       length code (BWL)
   Output:
        returned data, right justified in 32b longword
*/

static SIM_INLINE int32 GVP_Read (GVPMMU *mmu, uint32 va, int32 lnt)
{
int32 vpn, off, tbi, pa;
int32 pa1, bo, sc, wl, wh;
TLBENT xpte;

mmu->fva = va;
vpn = VA_GETVPN (va);                                   /* get vpn, offset */
off = VA_GETOFF (va);
tbi = VA_GETTBI (vpn);
xpte = mmu->tlb[tbi];                                   /* access tlb */
if (xpte.tag != vpn) {
    xpte = gvp_fill (mmu, va, lnt);                     /* fill if needed */
    if (mmu->status) return 0;
    }
pa = (xpte.pte & TLB_PFN) | off;                        /* get phys addr */
if ((pa & (lnt - 1)) == 0) {                            /* aligned? */
    if (lnt >= L_LONG)                                  /* long, quad? */
        return GVP_ReadL (mmu, pa);
    if (lnt == L_WORD)                                  /* word? */
        return GVP_ReadW (mmu, pa);
    return GVP_ReadB (mmu, pa);                         /* byte */
    }
if ((off + lnt) > VA_PAGSIZE) {                         /* cross page? */
    vpn = VA_GETVPN (va + lnt);                         /* vpn 2nd page */
    tbi = VA_GETTBI (vpn);
    xpte = mmu->tlb[tbi];                               /* access tlb */
    if (xpte.tag != vpn) {
        xpte = gvp_fill (mmu, va + lnt, lnt);            /* fill if needed */
        if (mmu->status) return 0;
        }
    pa1 = ((xpte.pte & TLB_PFN) | VA_GETOFF (va + 4)) & ~03;
    }
else pa1 = ((pa + 4) & PAMASK) & ~03;                   /* not cross page */
bo = pa & 3;
if (lnt >= L_LONG) {                                    /* lw unaligned? */
    sc = bo << 3;
    wl = GVP_ReadU (mmu, pa, L_LONG - bo);              /* read both fragments */
    if (mmu->status) return 0;
    wh = GVP_ReadU (mmu, pa1, bo);                      /* extract */
    if (mmu->status) return 0;
    return ((wl | (wh << (32 - sc))) & LMASK);
    }
else if (bo == 1)                                       /* read within lw */
    return GVP_ReadU (mmu, pa, L_WORD);
else {
    wl = GVP_ReadU (mmu, pa, L_BYTE);                   /* word cross lw */
    if (mmu->status) return 0;
    wh = GVP_ReadU (mmu, pa1, L_BYTE);                  /* read, extract */
    if (mmu->status) return 0;
    return (wl | (wh << 8));
    }
}

/* Write virtual

   Inputs:
        va      =       virtual address
        val     =       data to be written, right justified in 32b lw
        lnt     =       length code (BWL)
   Output:
        none
*/

static SIM_INLINE void GVP_Write (GVPMMU *mmu, uint32 va, int32 val, int32 lnt)
{
int32 vpn, off, tbi, pa;
int32 pa1, bo, sc, wl, wh;
TLBENT xpte;

mmu->fva = va;
vpn = VA_GETVPN (va);
off = VA_GETOFF (va);
tbi = VA_GETTBI (vpn);
xpte = mmu->tlb[tbi];                                   /* access tlb */
if (xpte.tag != vpn) {
    xpte = gvp_fill (mmu, va, lnt);
    if (mmu->status) return;
    }
pa = (xpte.pte & TLB_PFN) | off;
if ((pa & (lnt - 1)) == 0) {                            /* aligned? */
    if (lnt >= L_LONG)                                  /* long, quad? */
        GVP_WriteL (mmu, pa, val);
    else if (lnt == L_WORD)                             /* word? */
        GVP_WriteW (mmu, pa, val);
    else GVP_WriteB (mmu, pa, val);                     /* byte */
    return;
    }
if ((off + lnt) > VA_PAGSIZE) {
    vpn = VA_GETVPN (va + 4);
    tbi = VA_GETTBI (vpn);
    xpte = mmu->tlb[tbi];                               /* access tlb */
    if (xpte.tag != vpn) {
        xpte = gvp_fill (mmu, va + lnt, lnt);
        if (mmu->status) return;
        }
    pa1 = ((xpte.pte & TLB_PFN) | VA_GETOFF (va + 4)) & ~03;
    }
else pa1 = ((pa + 4) & PAMASK) & ~03;
bo = pa & 3;
if (lnt >= L_LONG) {
    sc = bo << 3;
    GVP_WriteU (mmu, pa, val & insert[L_LONG - bo], L_LONG - bo);
    GVP_WriteU (mmu, pa1, (val >> (32 - sc)) & insert[bo], bo);
    }
else if (bo == 1)                                       /* write within lw */
    GVP_WriteU (mmu, pa, val & WMASK, L_WORD);
else {                                                  /* word cross lw */
    GVP_WriteU (mmu, pa, val & BMASK, L_BYTE);
    GVP_WriteU (mmu, pa1, (val >> 8) & BMASK, L_BYTE);
    }
return;
}

/* Test access to a byte (VAX PROBEx) */

static SIM_INLINE int32 GVP_Test (GVPMMU *mmu, uint32 va)
{
int32 vpn, off, tbi;
TLBENT xpte;

vpn = VA_GETVPN (va);                                   /* get vpn, off */
off = VA_GETOFF (va);
tbi = VA_GETTBI (vpn);
xpte = mmu->tlb[tbi];                                   /* access tlb */
if (xpte.tag == vpn)                                    /* TB hit? */
    return (xpte.pte & TLB_PFN) | off;
xpte = gvp_fill (mmu, va, L_BYTE);                      /* fill TB */
if (mmu->status)
    return -1;
else
    return (xpte.pte & TLB_PFN) | off;
return va & PAMASK;                                     /* ret phys addr */
}

/* Read aligned physical (in virtual context, unless indicated)

   Inputs:
        pa      =       physical address, naturally aligned
   Output:
        returned data, right justified in 32b longword
*/

static SIM_INLINE int32 GVP_ReadB (GVPMMU *mmu, uint32 pa)
{
int32 dat;

if (ADDR_IS_MEM (pa)) {
    mmu->status = SCPE_OK;
    dat = M[pa >> 2];
    }
else {
    mmu->status = SCPE_NXM;
    return 0;
    }
return ((dat >> ((pa & 3) << 3)) & BMASK);
}

static SIM_INLINE int32 GVP_ReadW (GVPMMU *mmu, uint32 pa)
{
int32 dat;

if (ADDR_IS_MEM (pa)) {
    mmu->status = SCPE_OK;
    dat = M[pa >> 2];
    }
else {
    mmu->status = SCPE_NXM;
    return 0;
    }
return ((dat >> ((pa & 2)? 16: 0)) & WMASK);
}

static SIM_INLINE int32 GVP_ReadL (GVPMMU *mmu, uint32 pa)
{
if (ADDR_IS_MEM (pa)) {
    mmu->status = SCPE_OK;
    return M[pa >> 2];
    }
else {
    mmu->status = SCPE_NXM;
    return 0;
    }
}

/* Read unaligned physical (in virtual context)

   Inputs:
        pa      =       physical address
        lnt     =       length in bytes (1, 2, or 3)
   Output:
        returned data
*/

static SIM_INLINE int32 GVP_ReadU (GVPMMU *mmu, uint32 pa, int32 lnt)
{
int32 dat;
int32 sc = (pa & 3) << 3;
if (ADDR_IS_MEM (pa)) {
    dat = M[pa >> 2];
    return ((dat >> sc) & insert[lnt]);
    }
else {
    mmu->status = SCPE_NXM;
    return 0;
    }
}

/* Write aligned physical (in virtual context, unless indicated)

   Inputs:
        pa      =       physical address, naturally aligned
        val     =       data to be written, right justified in 32b longword
   Output:
        none
*/

static SIM_INLINE void GVP_WriteB (GVPMMU *mmu, uint32 pa, int32 val)
{
if (ADDR_IS_MEM (pa)) {
    mmu->status = SCPE_OK;
    int32 id = pa >> 2;
    int32 sc = (pa & 3) << 3;
    int32 mask = 0xFF << sc;
    M[id] = (M[id] & ~mask) | (val << sc);
    }
else mmu->status = SCPE_OK;
return;
}

static SIM_INLINE void GVP_WriteW (GVPMMU *mmu, uint32 pa, int32 val)
{
if (ADDR_IS_MEM (pa)) {
    mmu->status = SCPE_OK;
    int32 id = pa >> 2;
    M[id] = (pa & 2)? (M[id] & 0xFFFF) | (val << 16):
        (M[id] & ~0xFFFF) | val;
    }
else mmu->status = SCPE_OK;
return;
}

static SIM_INLINE void GVP_WriteL (GVPMMU *mmu, uint32 pa, int32 val)
{
if (ADDR_IS_MEM (pa)) {
    mmu->status = SCPE_OK;
    M[pa >> 2] = val;
    }
else mmu->status = SCPE_OK;
return;
}

/* Write unaligned physical (in virtual context)

   Inputs:
        pa      =       physical address
        val     =       data to be written, right justified in 32b longword
        lnt     =       length (1, 2, or 3 bytes)
   Output:
        none
*/

static SIM_INLINE void GVP_WriteU (GVPMMU *mmu, uint32 pa, int32 val, int32 lnt)
{
if (ADDR_IS_MEM (pa)) {
    int32 bo = pa & 3;
    int32 sc = bo << 3;
    M[pa >> 2] = (M[pa >> 2] & ~(insert[lnt] << sc)) | ((val & insert[lnt]) << sc);
    }
else mmu->status = SCPE_OK;
return;
}

#endif /* VAX_GVP_H_ */
