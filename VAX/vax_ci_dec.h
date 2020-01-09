/* vax_ci_dec.h: Computer Interconnect adapter (DEC Common)

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
*/

#ifndef _VAX_CI_DEC_H_
#define _VAX_CI_DEC_H_

/* CI Registers */

#define CI_PSR          0                               /* port status */
#define CI_PQBBR        1                               /* port queue block base */
#define CI_PCQ0CR       2                               /* port cmd queue 0 control */
#define CI_PCQ1CR       3                               /* port cmd queue 1 control */
#define CI_PCQ2CR       4                               /* port cmd queue 2 control */
#define CI_PCQ3CR       5                               /* port cmd queue 3 control */
#define CI_PSRCR        6                               /* FIXME - description */
#define CI_PECR         7                               /* FIXME - description */
#define CI_PDCR         8                               /* FIXME - description */
#define CI_PICR         9                               /* FIXME - description */
#define CI_PDFQCR       10                              /* port datagram free queue control */
#define CI_PMFQCR       11                              /* port message free queue control */
#define CI_PMTCR        12                              /* FIXME - description */
#define CI_PMTECR       13                              /* FIXME - description */
#define CI_PFAR         14                              /* port failing address */
#define CI_PESR         15                              /* FIXME - description */
#define CI_PPR          16                              /* port parameter */

typedef struct {                                        /* register mappings */
    uint32      offset;                                 /* memory offset */
    uint32      rg;                                     /* register index */
} REGMAP;

extern UNIT ci_unit;

t_stat ci_dec_rd (int32 *val, int32 rg, int32 lnt);
t_stat ci_dec_wr (int32 val, int32 rg, int32 lnt);
t_stat ci_dec_svc (UNIT *uptr);
t_stat ci_dec_reset (DEVICE *dptr);

#endif
