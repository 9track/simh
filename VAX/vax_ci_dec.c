/* vax_ci_dec.c: Computer Interconnect adapter (DEC Common)

   Copyright (c) 2018, Matt Burke

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

#include "vax_defs.h"
#include "vax_ci.h"
#include "vax_gvp.h"

/* Port Status */

#define PSR_V_RQA       0                               /* request queue available */
#define PSR_V_MFQE      1                               /* message free queue empty */
#define PSR_V_PDC       2                               /* port disable complete */
#define PSR_V_PIC       3                               /* port initialisation complete */
#define PSR_V_DSE       4                               /* data structure error */
#define PSR_V_MSE       5                               /* memory system error */
#define PSR_V_MTE       6                               /* maintenance timer expiration  */
#define PSR_V_ME        31                              /* maintenance error */

#define PSR_RQA         (1u << PSR_V_RQA)
#define PSR_MFQE        (1u << PSR_V_MFQE)
#define PSR_PDC         (1u << PSR_V_PDC)
#define PSR_PIC         (1u << PSR_V_PIC)
#define PSR_DSE         (1u << PSR_V_DSE)
#define PSR_MSE         (1u << PSR_V_MSE)
#define PSR_MTE         (1u << PSR_V_MTE)
#define PSR_ME          (1u << PSR_V_ME)

/* Port Queue Block Base */

#define PQBB_V_ADDR     9                               /* port queue block base */
#define PQBB_M_ADDR     0x1FFFFF

#define PQBB_ADDR       (PQBB_M_ADDR << PQBB_V_ADDR)

/* Port Parameter */

#define PPR_M_NODE      0xF                             /* ci node number */
#define PPR_V_IBLEN     16                              /* internal buffer length */
#define PPR_M_IBLEN     0x1FFF
#define PPR_V_CS        31                              /* cluster size */

#define PPR_NODE        (PPR_M_NODE)
#define PPR_IBLEN       (PPR_M_IBLEN << PPR_V_IBLEN)
#define PPR_CS          (1u << PPR_V_CS)

/* PQB Offsets */

#define PQB_CMD0_OF     0x0                             /* Command queue 0 (low priority) */
#define PQB_CMD1_OF     0x8                             /* Command queue 1 (high priority) */
#define PQB_CMD2_OF     0x10                            /* Command queue 2 */
#define PQB_CMD3_OF     0x18                            /* Command queue 3 */
#define PQB_RESP_OF     0x20                            /* Response queue */
#define PQB_DFQA_OF     0x28                            /* Datagram free queue Address */
#define PQB_MFQA_OF     0x2C                            /* Message free queue Address */
#define PQB_DFQL_OF     0x30                            /* Datagram free queue entry length */
#define PQB_DFML_OF     0x34                            /* Message free queue entry length */
#define PQB_PQBBVA_OF   0x38                            /* VA of port queue block base */
#define PQB_BDTVA_OF    0x3C                            /* VA of buffer descriptor table */
#define PQB_BDTLEN_OF   0x40                            /* # entries in buffer descriptor table */
#define PQB_SBR_OF      0x44                            /* system page table PA */
#define PQB_SLR_OF      0x48                            /* system page table size */
#define PQB_GPT_OF      0x4C                            /* global page table VA */
#define PQB_GLR_OF      0x50                            /* global page table size */

/* Queue Names */

#define Q_CMD0          0                               /* Command queue 0 (low priority) */
#define Q_CMD1          1                               /* Command queue 1 (high priority) */
#define Q_CMD2          2                               /* Command queue 2 */
#define Q_CMD3          4                               /* Command queue 3 */
#define Q_RSP           5                               /* Response queue */
#define Q_DF            6                               /* Datagram free queue */
#define Q_MF            7                               /* Message free queue */

/* Generic VAXport buffer descriptor */

#define BD_OFF_OF       0x0                             /* byte offset into buffer page */
#define BD_KEY_OF       0x2                             /* sequence number */
#define BD_SIZE_OF      0x4                             /* buffer size */
#define BD_PTE_OF       0x8                             /* VA of buffer PTE */
#define BD_NEXT_OF      0xc                             /* next free buffer descriptor */
#define BD_LEN          0x10                            /* buffer descriptor length */

/* Macros */

#define CI_ReadL(x)     GVP_ReadL (&ci_mmu, x)

uint32 ci_psr;                                          /* port status reg */
uint32 ci_pqbb_pa;                                      /* port queue base pa */
uint32 ci_pqbb_va;                                      /* port queue base va */
uint32 ci_bdt_va;                                       /* buffer descriptor table va */
uint32 ci_bdt_len;                                      /* buffer descriptor table entries */
uint32 ci_spt_base;                                     /* buffer descriptor table entries */
uint32 ci_pfar;                                         /* port failing addr */
uint32 ci_pesr;                                         /* port error status */
uint32 ci_ppr;                                          /* port parameter reg */
GVPMMU ci_mmu;                                          /* memory management unit */

void ci_readb (uint32 pte_ba, int32 bc, int32 offset, uint8 *buf);
void ci_writeb (uint32 pte_ba, int32 bc, int32 offset, uint8 *buf);
void ci_read_pqb (uint32 pa);

t_stat ci_port_command (int32 queue)
{
int32 status, msg_type;
t_stat r;
uint32 i, ent_addr, opcode, port_num, dg_type, cmd_flags, ci_queue_base, reply_addr;
uint32 ci_path;
uint16 msg_size;
UNIT *uptr = &ci_unit;

for ( ;; ) {
    ent_addr = gvp_remqhi (&ci_mmu, ci_pqbb_va + queue);

    if (ent_addr == 0)
        return SCPE_OK;
    if (ent_addr == (ci_pqbb_va + queue))
        return SCPE_OK;

    if (ent_addr & 0x3)
        printf ("Command queue packet not LW aligned\n");
    }
}

/* Read CI port register */

t_stat ci_rdport (int32 *val, int32 rg, int32 lnt)
{
switch (rg) {

    case CI_PSR:
        *val = ci_psr;
        sim_debug (DBG_REG, &ci_dev, "PSR rd: %08X\n", *val);
        break;

    case CI_PQBBR:
        *val = ci_pqbb_pa;
        sim_debug (DBG_REG, &ci_dev, "PQBBR rd: %08X\n", *val);
        break;

    case CI_PCQ0CR:
    case CI_PCQ1CR:
    case CI_PCQ2CR:
    case CI_PCQ3CR:
    case CI_PSRCR:
    case CI_PECR:
    case CI_PDCR:
    case CI_PICR:
    case CI_PDFQCR:
    case CI_PMFQCR:
    case CI_PMTCR:
    case CI_PMTECR:
        *val = 0;
        sim_debug (DBG_REG, &ci_dev, "other rd: %08X\n", *val);
        break;

    case CI_PFAR:
        *val = ci_pfar;
        sim_debug (DBG_REG, &ci_dev, "PFAR rd: %08X\n", *val);
        break;

    case CI_PESR:
        *val = ci_pesr;
        sim_debug (DBG_REG, &ci_dev, "PESR rd: %08X\n", *val);
        break;

    case CI_PPR:
        *val = ci_ppr | (ci_node & PPR_NODE);
        sim_debug (DBG_REG, &ci_dev, "PPR rd: %08X\n", *val);
        break;

    default:
        return SCPE_NXM;
        }

return SCPE_OK;
}

/* Write CI port register */

t_stat ci_wrport (int32 val, int32 rg, int32 lnt)
{
switch (rg) {

    case CI_PSR:
        sim_debug (DBG_WRN, &ci_dev, "PSR wr!\n");
        break;

    case CI_PQBBR:
        sim_debug (DBG_REG, &ci_dev, "PQBBR wr: %08X\n", val);
        ci_pqbb_pa = val & PQBB_ADDR;
        ci_read_pqb (ci_pqbb_pa);
        break;

    case CI_PCQ0CR:
        sim_debug (DBG_TRC, &ci_dev, "port command queue 0 control\n");
        if (val & 1)
            return ci_port_command (PQB_CMD0_OF);

    case CI_PCQ1CR:
        sim_debug (DBG_TRC, &ci_dev, "port command queue 1 control\n");
        if (val & 1)
            return ci_port_command (PQB_CMD1_OF);

    case CI_PCQ2CR:
        sim_debug (DBG_WRN, &ci_dev, "port command queue 2 control\n");
        if (val & 1)
            return ci_port_command (PQB_CMD2_OF);

    case CI_PCQ3CR:
        sim_debug (DBG_WRN, &ci_dev, "port command queue 3 control\n");
        if (val & 1)
            return ci_port_command (PQB_CMD3_OF);

    case CI_PSRCR:
        sim_debug (DBG_TRC, &ci_dev, "port status release control\n");
        if (val & 1) {
            ci_psr = 0;
            ci_clr_int ();
        }
        break;

    case CI_PECR:
        sim_debug (DBG_TRC, &ci_dev, "port enable\n");
        if (val & 1) {
            if (ci_state >= PORT_INIT)
                ci_set_state (PORT_ENABLED);
            }
        break;

    case CI_PDCR:
        sim_debug (DBG_TRC, &ci_dev, "port disable\n");
        if (val & 1) {
            if (ci_state >= PORT_INIT)
                ci_set_state (PORT_INIT);
            ci_psr |= PSR_PDC;
            ci_set_int ();
        }
        break;

    case CI_PICR:
        sim_debug (DBG_TRC, &ci_dev, "port init\n");
        if (val & 1) {
            if (ci_state >= PORT_UNINIT)
                ci_set_state (PORT_INIT);
            ci_psr |= PSR_PIC;
            ci_set_int ();
        }
        break;

    case CI_PDFQCR:
        sim_debug (DBG_TRC, &ci_dev, "port datagram free queue\n");
        break;

    case CI_PMFQCR:
        sim_debug (DBG_TRC, &ci_dev, "port message free queue\n");
        break;

    case CI_PMTCR:
        sim_debug (DBG_REG, &ci_dev, "PMTCR wr: %08X\n", val);
        break;

    case CI_PMTECR:
        sim_debug (DBG_REG, &ci_dev, "PMTECR wr: %08X\n", val);
        break;

    case CI_PFAR:
        sim_debug (DBG_WRN, &ci_dev, "PFAR wr!\n");
        break;

    case CI_PESR:
        sim_debug (DBG_WRN, &ci_dev, "PESR wr!\n");
        break;

    case CI_PPR:
        sim_debug (DBG_WRN, &ci_dev, "PPR wr!\n");
        break;

    default:
        return SCPE_NXM;
        }
return SCPE_OK;
}

/* Read parameters from port queue block */

void ci_read_pqb (uint32 pa)
{
//ci_dg_size = CI_ReadL (pa + PQB_DFQL_OF);               /* packet sizes */
//ci_msg_size = CI_ReadL (pa + PQB_DFML_OF);
ci_pqbb_va = CI_ReadL (pa + PQB_PQBBVA_OF);
ci_bdt_va = CI_ReadL (pa + PQB_BDTVA_OF);               /* buffer descriptors */
ci_bdt_len = CI_ReadL (pa + PQB_BDTLEN_OF);
ci_mmu.sbr = CI_ReadL (pa + PQB_SBR_OF);                /* memory management */
ci_mmu.sbr = (ci_mmu.sbr - 0x1000000) & ~03;            /* VA<31> >> 7 */
ci_mmu.slr = CI_ReadL (pa + PQB_SLR_OF);
ci_mmu.slr = (ci_mmu.slr << 2) + 0x1000000;             /* VA<31> >> 7 */
ci_mmu.gbr = CI_ReadL (pa + PQB_GPT_OF);
ci_mmu.glr = CI_ReadL (pa + PQB_GLR_OF);
ci_mmu.glr = (ci_mmu.glr << 2);
}

t_stat ci_send_data (int32 opcode, uint8 *buffer, int32 port, int32 path)
{
uint32 data_len, total_data_len, start_offset, msg_size;
uint16 bdt_offset, page_offset;
uint32 data_offset, data_pte;
int32 r;

total_data_len = GET_INT32 (buffer, 0x18);
bdt_offset = GET_INT16 (buffer, 0x1c);                  /* Get BDT table offset */
data_offset = GET_INT32 (buffer, 0x20);                 /* Get data starting offset */
data_pte = GVP_Read (&ci_mmu, ci_bdt_va + (bdt_offset * BD_LEN) + BD_PTE_OF, L_LONG); /* Get PTE addr */
page_offset = GVP_Read (&ci_mmu, ci_bdt_va + (bdt_offset * BD_LEN), L_WORD) & 0x1FF;

buffer[0xe] = opcode;                                   /* Set opcode */
msg_size = 0x2c;

r = ci_send_packet (buffer, msg_size, opcode, port, path);
if (r != SCPE_OK)
    return r;

start_offset = 0;
while (total_data_len > 0) {
    if (total_data_len > (CI_NET_BUF_SIZE - 0xc))
        data_len = (CI_NET_BUF_SIZE - 0xc);
    else
        data_len = total_data_len;

    sim_debug (DBG_BLKTF, &ci_dev, "==>   SNDDAT, data_len: %d, time: %d\n", data_len, sim_grtime ());

    ci_readb (data_pte, data_len, page_offset + data_offset + start_offset, &net_blk_buf[0xc]);
    r = ci_send_packet (net_blk_buf, (data_len + 0xc), opcode, port, path);
    total_data_len -= data_len;
    start_offset += data_len;
    }
return SCPE_OK;
}

t_stat ci_receive_data (int32 opcode, int32 src, uint8 *buffer)
{
uint32 data_len, total_data_len, start_offset;
uint16 bdt_offset, page_offset;
uint32 data_offset, data_pte;

if (blktf_table[src].incomplete == 0) {
    //sim_debug (DBG_BLKTF, &ci_dev, "<== RECDAT, src: %d, path: %s\n", src, ci_path_names[ci_path]);

    blktf_table[src].opcode = opcode;
    blktf_table[src].total_data_len = GET_INT32 (buffer, 0x18);
    //sim_debug (DBG_BLKTF, &ci_dev, "data_len: %d\n", blktf_table[src].total_data_len);
    bdt_offset = GET_INT16 (buffer, 0x24);
    blktf_table[src].data_offset = GET_INT32 (buffer, 0x28);
    blktf_table[src].data_pte = GVP_Read (&ci_mmu, ci_bdt_va + (bdt_offset * BD_LEN) + BD_PTE_OF, L_LONG);
    blktf_table[src].page_offset = GVP_Read (&ci_mmu, ci_bdt_va + (bdt_offset * BD_LEN), L_WORD) & 0x1FF;
    blktf_table[src].start_offset = 0;
    blktf_table[src].incomplete = 1;

    memcpy (blktf_table[src].conf_buf, buffer, 0x2c);
    if (opcode == OPC_SNDDAT) {
        blktf_table[src].conf_buf[0xe] = OPC_CNFREC;    /* Set opcode */
        blktf_table[src].conf_buf[0xf] = blktf_table[src].conf_buf[0xf] & ~0x1; /* Clear RSP flag */
        }
    return SCPE_INCOMP;
    }

if (blktf_table[src].total_data_len > (CI_NET_BUF_SIZE - 0xc))
    data_len = (CI_NET_BUF_SIZE - 0xc);
else
    data_len = blktf_table[src].total_data_len;

ci_writeb (blktf_table[src].data_pte, data_len, blktf_table[src].page_offset + blktf_table[src].data_offset
               + blktf_table[src].start_offset, &buffer[0xc]);
blktf_table[src].total_data_len -= data_len;
blktf_table[src].start_offset += data_len;
if (blktf_table[src].total_data_len > 0)
    return SCPE_INCOMP;

blktf_table[src].incomplete = 0;

return SCPE_OK;
}

void ci_read_packet (int32 addr, int32 size, uint8 *buf)
{
int32 i;

// Skip header (length = 0xc)
for (i = 0xc; i < size; i++)
    buf[i] = GVP_Read (&ci_mmu, (addr + i), L_BYTE);
}

void ci_write_packet (int32 addr, int32 size, uint8 *buf)
{
int32 i;

// Skip header (length = 0xc)
for (i = 0xc; i < size; i++)
    GVP_Write (&ci_mmu, (addr + i), buf[i], L_BYTE);
}

// TODO: Move these to GVP?

void ci_readb (uint32 pte_ba, int32 bc, int32 offset, uint8 *buf)
{
int32 i;
uint32 dat, pte_va, pte, pa;;

if ((offset | bc) & 03) {                               /* check alignment */
    for (i = offset; i < (offset + bc); i++, buf++) {   /* by bytes */
        pte_va = pte_ba + ((i >> 7) & ~0x3);
        pte = GVP_Read (&ci_mmu, pte_va, L_LONG);
        pa = ((pte & 0x1fffff) << 9) | VA_GETOFF(i);
        *buf = ReadB (pa);
        }
    }
else {
    for (i = offset; i < (offset + bc); i = i + 4, buf++) { /* by longwords */
        pte_va = pte_ba + ((i >> 7) & ~0x3);
        pte = GVP_Read (&ci_mmu, pte_va, L_LONG);
        pa = ((pte & 0x1fffff) << 9) | VA_GETOFF(i);
        dat = ReadL (pa);                               /* get lw */
        *buf++ = dat & BMASK;                           /* low 8b */
        *buf++ = (dat >> 8) & BMASK;                    /* next 8b */
        *buf++ = (dat >> 16) & BMASK;                   /* next 8b */
        *buf = (dat >> 24) & BMASK;
        }
    }
}

void ci_writeb (uint32 pte_ba, int32 bc, int32 offset, uint8 *buf)
{
int32 i;
uint32 dat, pte_va, pte, pa;

if ((offset | bc) & 03) {                               /* check alignment */
    // TODO: This should only need to translate PTE when going cross page
    for (i = offset; i < (offset + bc); i++, buf++) {   /* by bytes */
        pte_va = pte_ba + ((i >> 7) & ~0x3);
        pte = GVP_Read (&ci_mmu, pte_va, L_LONG);
        pa = ((pte & 0x1fffff) << 9) | VA_GETOFF(i);
        WriteB (pa, *buf);
        }
    }
else {
    for (i = offset; i < (offset + bc); i = i + 4, buf++) { /* by longwords */
        pte_va = pte_ba + ((i >> 7) & ~0x3);
        pte = GVP_Read (&ci_mmu, pte_va, L_LONG);
        pa = ((pte & 0x1fffff) << 9) | VA_GETOFF(i);
        dat = (uint32) *buf++;                          /* get low 8b */
        dat = dat | (((uint32) *buf++) << 8);           /* merge next 8b */
        dat = dat | (((uint32) *buf++) << 16);          /* merge next 8b */
        dat = dat | (((uint32) *buf) << 24);            /* merge hi 8b */
        WriteL (pa, dat);                               /* store lw */
        }
    }
}

void put_rsp_queue (int32 addr)
{
if (gvp_insqti (&ci_mmu, ci_pqbb_va + PQB_RESP_OF, addr) > 1) {
    ci_psr |= PSR_RQA;
    ci_set_int ();
    }
}

void put_dgf_queue (int32 addr)
{
uint32 hdr;

hdr = GVP_Read (&ci_mmu, ci_pqbb_va + PQB_DFQA_OF, L_LONG);
gvp_insqti (&ci_mmu, hdr, addr);
}

int32 get_dgf_queue (void)
{
uint32 hdr;

hdr = GVP_Read (&ci_mmu, ci_pqbb_va + PQB_DFQA_OF, L_LONG);
return gvp_remqhi (&ci_mmu, hdr);
}


void put_msf_queue (int32 addr)
{
uint32 hdr;

hdr = GVP_Read (&ci_mmu, ci_pqbb_va + PQB_MFQA_OF, L_LONG);
gvp_insqti (&ci_mmu, hdr, addr);
}

int32 get_msf_queue (void)
{
uint32 hdr;

hdr = GVP_Read (&ci_mmu, ci_pqbb_va + PQB_MFQA_OF, L_LONG);
return gvp_remqhi (&ci_mmu, hdr);
}

/* Reset CI adapter */

t_stat ci_port_reset (DEVICE *dptr)
{
ci_psr = 0;
ci_pesr = 0;
ci_pfar = 0;
ci_pqbb_pa = 0;
ci_pqbb_va = 0;
ci_bdt_va = 0;
ci_bdt_len = 0;
ci_ppr |= ((0x3f9 & PPR_M_IBLEN) << PPR_V_IBLEN);       /* TODO: set internal buffer length */
gvp_tlb_reset (&ci_mmu);                                /* reset MMU */
return SCPE_OK;
}
