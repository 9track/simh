/* vax_ci.h: Computer Interconnect adapter

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
*/

#ifndef _VAX_CI_H_
#define _VAX_CI_H_

/* Debug Streams */

#define DBG_REG         0x0001                          /* registers */
#define DBG_WRN         0x0002                          /* warnings */
#define DBG_REQID       0x0004                          /* request IDs/reponses */
#define DBG_SCSDG       0x0008                          /* SCS datagrams */
#define DBG_SCSMSG      0x0010                          /* SCS Messages */
#define DBG_PPDDG       0x0020                          /* PPD Datagrams */
#define DBG_BLKTF       0x0040                          /* block transfer requests/responses/confirms */
#define DBG_LCMD        0x0080                          /* local commands */
#define DBG_CONN        0x0100                          /* connections */
#define DBG_TRC         0x0200                          /* trace */

/* Port States */

#define PORT_UNINIT     0                               /* Uninitialised */
#define PORT_INIT       10                              /* Initialised */
#define PORT_ENABLED    20                              /* Enabled */

#define CI_MAX_NODES   16

#define CI_GET16(p,w)   (((uint16) p[w]) | \
                        (((uint16) p[(w)+1]) << 8))
#define CI_GET32(p,w)   (((uint32) p[w]) | \
                        (((uint32) p[(w)+1]) << 8) | \
                        (((uint32) p[(w)+2]) << 16) | \
                        (((uint32) p[(w)+3]) << 24))

#define CI_PUT16(p,w,x) p[w] = (x) & 0xFF; \
                        p[(w)+1] = ((x) >> 8) & 0xFF
#define CI_PUT32(p,w,x) p[w] = (x) & 0xFF; \
                        p[(w)+1] = ((x) >> 8) & 0xFF; \
                        p[(w)+2] = ((x) >> 16) & 0xFF; \
                        p[(w)+3] = ((x) >> 24) & 0xFF

#define CI_MAXFR        1024                            /* max xfer */

typedef struct {
    uint32 addr;
    size_t length;
    uint8 data[CI_MAXFR];
} CI_PKT;

extern uint32 ci_state;
extern uint32 ci_node;

void ci_set_state (uint32 state);
t_stat ci_ppd (CI_PKT *pkt);
t_stat ci_show_node (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat ci_set_node (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ci_show_tcp (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat ci_set_tcp (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ci_attach (UNIT *uptr, CONST char *cptr);
t_stat ci_detach (UNIT *uptr);
t_stat ci_svc (UNIT *uptr);
t_stat ci_port_reset (DEVICE *dptr);

#endif
