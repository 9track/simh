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

typedef struct {                                        /* register mappings */
    uint32      offset;                                 /* low addr */
    uint32      rg;                                     /* high addr */
} REGMAP;

#define VC_OPEN(x)     (x & 0x8000)
#define DG_INHIBIT(x)  (x & 0x1000)
#define CI_MAX_NODES   16

#define GET_UINT32(buf,off)          ((uint32)((buf[off+3] << 24) | (buf[off+2] << 16) | (buf[off+1] << 8) | (buf[off] & 0xff)))
#define GET_INT32(buf,off)           ((int32)((buf[off+3] << 24) | (buf[off+2] << 16) | (buf[off+1] << 8) | (buf[off] & 0xff)))
#define GET_UINT16(buf,off)          ((uint16)((buf[off+1] << 8) | (buf[off] & 0xff)))
#define GET_INT16(buf,off)           ((int16)((buf[off+1] << 8) | (buf[off] & 0xff)))

#define CI_READB_NET(buffer,offset)  (buffer[offset])
#define CI_READB_MEM(addr,offset)    GVP_Read (&ci_mmu, addr + offset, L_BYTE)
#define CI_READB(x,offset,source)    CI_READB_##source(x,offset)

#define CI_READW_NET(buffer,offset)  (((buffer[offset+1] & 0xff) << 8) | buffer[offset])
#define CI_READW_MEM(addr,offset)    GVP_Read (&ci_mmu, addr + offset, L_WORD)
#define CI_READW(x,offset,source)    CI_READB_##source(x,offset)

#define CI_READL_NET(buffer,offset)  (((buffer[offset+1] & 0xff) << 24) | ((buffer[offset+1] & 0xff) << 16) | ((buffer[offset+1] & 0xff) << 8) | buffer[offset])
#define CI_READL_MEM(addr,offset)    GVP_Read (&ci_mmu, addr + offset, L_LONG)
#define CI_READL(x,offset,source)    CI_READB_##source(x,offset)

/* GVP header 'get' macros */
#define CI_GET_ST_SIZE(x,source)     CI_READW(x,0x8,source)
#define CI_GET_ST_TYPE(x,source)     CI_READB(x,0xa,source)
#define CI_GET_PORT(x,source)        CI_READB(x,0xc,source)
#define CI_GET_STATUS(x,source)      CI_READB(x,0xd,source)
#define CI_GET_OPCODE(x,source)      CI_READB(x,0xe,source)
#define CI_GET_CM_FLAGS(x,source)    CI_READB(x,0xf,source)

/* MSG/DG 'get' macros */
#define CI_GET_MSG_SIZE(x,source)    CI_READW(x,0x10,source)
#define CI_GET_PPD_TYPE(x,source)    CI_READW(x,0x12,source)

/* SETCKT 'get' macros */
#define CI_GET_VCD_MASK(x,source)    CI_READW(x,0x10,source)
#define CI_GET_VCD_VAL(x,source)     CI_READW(x,0x14,source)

/* MSG 'get' macros */
#define CI_GET_MSG_TYPE(x,source)    CI_READW(x,0x14,source)


#define CI_WRITEB_NET(buffer,offset,val)   buffer[offset] = val
#define CI_WRITEB_MEM(addr,offset,val)     GVP_Write (&ci_mmu, addr + offset, val, L_BYTE)
#define CI_WRITEB(x,offset,val,source)     CI_WRITEB_##source(x,offset,val)

#define CI_WRITEW_NET(buffer,offset,val)  buffer[offset+1] = (val >> 8) & 0xff; \
                                          buffer[offset]   = val & 0xff
#define CI_WRITEW_MEM(addr,offset,val)    GVP_Write (&ci_mmu, addr + offset, val, L_WORD)
#define CI_WRITEW(x,offset,val,source)    CI_WRITEB_##source(x,offset,val)

#define CI_WRITEL_NET(buffer,offset,val)  buffer[offset+3] = (char) ((val >> 24) & 0xff); \
                                          buffer[offset+2] = (char) ((val >> 16) & 0xff); \
                                          buffer[offset+1] = (char) ((val >> 8)  & 0xff); \
                                          buffer[offset]   = (char) (val & 0xff)
#define CI_WRITEL_MEM(addr,offset,val)    GVP_Write (&ci_mmu, addr + offset, val, L_LONG)
#define CI_WRITEL(x,offset,val,source)    CI_WRITEB_##source(x,offset,val)

/* GVP header 'set' macros */
#define CI_SET_PORT(x,val,source)        CI_WRITEB(x,0xc,val,source)
#define CI_SET_STATUS(x,val,source)      CI_WRITEB(x,0xd,val,source)
#define CI_SET_OPCODE(x,val,source)      CI_WRITEB(x,0xe,val,source)
#define CI_SET_CM_FLAGS(x,val,source)    CI_WRITEB(x,0xf,val,source)

/* SETCKT 'set' macros */
#define CI_SET_VCD_IVAL(x,val,source)    CI_WRITEL(x,0x18,val,source)

/* IDREC 'set' macros */
#define CI_SET_RP_TYPE(x,val,source)    CI_WRITEL(x,0x18,val,source)
#define CI_SET_RP_REV(x,val,source)     CI_WRITEB(x,0x1C,val,source)
#define CI_SET_RP_FUNC(x,val,source)    CI_WRITEL(x,0x20,val,source)
#define CI_SET_RST_PORT(x,val,source)   CI_WRITEB(x,0x24,val,source)
#define CI_SET_RP_STATE(x,val,source)   CI_WRITEB(x,0x18,val,source)

uint32 ci_state;

t_stat ci_rdport (int32 *val, int32 rg, int32 lnt);
t_stat ci_wrport (int32 val, int32 rg, int32 lnt);
void ci_set_state (uint32 state);
t_stat ci_show_node (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat ci_set_node (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ci_show_tcp (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat ci_set_tcp (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat ci_attach (UNIT *uptr, CONST char *cptr);
t_stat ci_detach (UNIT *uptr);
t_stat ci_svc (UNIT *uptr);
t_stat ci_port_reset (DEVICE *dptr);
