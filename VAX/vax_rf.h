/* vax_rf.h: MSCP disk controller simulator

   Copyright (c) 2001-2008, Robert M Supnik
   Derived from work by Stephen F. Shirron

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
   ROBERT M SUPNIK BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the name of Robert M Supnik shall not be
   used in advertising or otherwise to promote the sale, use or other dealings
   in this Software without prior written authorization from Robert M Supnik.

   30-Aug-02    RMS     Added TMSCP support
*/

#ifndef VAX_RF_H_
#define VAX_RF_H_ 0

#define RF_NPKTS        32                              /* # packets (pwr of 2) */
#define RF_M_NPKTS      (RF_NPKTS - 1)                  /* mask */
#define RF_PKT_SIZE_W   32                              /* payload size (wds) */
#define RF_PKT_SIZE     (RF_PKT_SIZE_W * sizeof (int16))

struct rfpkt {
    uint16      link;                                   /* link to next */
    uint16      d[RF_PKT_SIZE_W];                       /* data */
    };

typedef struct {
    uint32              cid;                            /* connection id */
    uint32              wait;                           /* ctrl number - ctlr */
    uint32              sa;                             /* status, addr - uq */
    uint32              saw;                            /* written data - uq */
    uint32              s1dat;                          /* S1 data - uq */
    uint32              comm;                           /* comm region - uq */
    uint32              csta;                           /* ctrl state - conn? */
    uint16              perr;                           /* last error */
    uint16              cflgs;                          /* ctrl flags - conn */
    uint32              irq;                            /* intr request - uq */
    uint32              prgi;                           /* purge int - uq */
    uint32              pip;                            /* poll in progress - uq */
    uint16              freq;                           /* free list - uq */
    uint16              rspq;                           /* resp list - uq */
    uint32              pbsy;                           /* #busy pkts - conn? */
    uint32              credits;                        /* credits - uq */
    uint32              hat;                            /* host timer - conn? */
    uint32              htmo;                           /* host timeout - conn? */
    uint32              ctype;                          /* controller type - uq/hsc */
    struct uq_ring      cq;                             /* cmd ring - uq */
    struct uq_ring      rq;                             /* rsp ring - uq */
    struct rfpkt        pak[RF_NPKTS];                  /* packet queue - conn */
    uint16              max_plug;                       /* highest unit plug number - ctlr */
    } MSC;

t_bool rf_mscp (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_deqf (MSC *cp, uint16 *pkt);
void rf_enqh (MSC *cp, uint16 *lh, uint16 pkt);
t_stat hsc_mscp_done (MSC *cp, uint16 rf_pkt);

extern MSC rf_ctx;

#endif
