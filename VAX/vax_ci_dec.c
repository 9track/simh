/* vax_ci_dec.c: Computer Interconnect adapter (DEC Common)

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

#include "vax_defs.h"
#include "vax_ci.h"
#include "vax_ci_dec.h"
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

/* Block transfer constants */

#define CI_DATHDR       0x2C
#define CI_MAXDAT       (CI_MAXFR - CI_DATHDR)

/* Macros */

#define CI_ReadL(x)     GVP_ReadL (&ci_mmu, x)

uint32 ci_psr;                                          /* port status reg */
uint32 ci_max_dg;                                       /* max datagram size */
uint32 ci_max_msg;                                      /* max message size */
uint32 ci_pqbb_pa;                                      /* port queue base pa */
uint32 ci_pqbb_va;                                      /* port queue base va */
uint32 ci_bdt_va;                                       /* buffer descriptor table va */
uint32 ci_bdt_len;                                      /* buffer descriptor table entries */
uint32 ci_spt_base;                                     /* buffer descriptor table entries */
uint32 ci_pfar;                                         /* port failing addr */
uint32 ci_pesr;                                         /* port error status */
uint32 ci_ppr;                                          /* port parameter reg */
GVPMMU ci_mmu;                                          /* memory management unit */

extern int32 tmxr_poll;

extern void ci_set_int (void);
extern void ci_clr_int (void);

void ci_read_packet (CI_PKT *pkt, size_t length);
void ci_write_packet (CI_PKT *pkt, size_t length);
void ci_readb (uint32 pte_ba, int32 bc, int32 offset, uint8 *buf);
void ci_writeb (uint32 pte_ba, int32 bc, int32 offset, uint8 *buf);
void ci_read_pqb (uint32 pa);
t_stat ci_receive_int (CI_PKT *pkt);
t_stat ci_receive (CI_PKT *pkt);
t_stat ci_send (CI_PKT *pkt);
t_stat ci_respond (CI_PKT *pkt);
t_stat ci_dispose (CI_PKT *pkt);

t_stat ci_port_command (int32 queue)
{
uint32 ent_addr;
int16 length;
CI_PKT pkt;
t_stat r;

for ( ;; ) {
    ent_addr = gvp_remqhi (&ci_mmu, ci_pqbb_va + queue);

    if (ent_addr == 0)
        break;
    if (ent_addr == (ci_pqbb_va + queue))
        break;

    if (ent_addr & 0x3)
        sim_printf ("CI: command queue packet not LW aligned\n");

    pkt.addr = ent_addr;
    length = GVP_Read (&ci_mmu, ent_addr + PPD_SIZE, L_WORD);
    if (length == PPD_DG)                               /* type is dg? */
        length = ci_max_dg;                             /* use max size */
    else if (length == PPD_MSG)                         /* type is msg? */
        length = ci_max_msg;                            /* use max size */
    else if (length < 0) {
        ent_addr = ent_addr + length;                   /* offset to network header */
        length = GVP_Read (&ci_mmu, ent_addr + PPD_SIZE, L_WORD) + length;
        }
    pkt.length = length;
    pkt.port = ci_unit.CI_NODE;
    ci_read_packet (&pkt, pkt.length);
    r = ci_send (&pkt);
    if (r != SCPE_OK)
        break;
    if (pkt.data[PPD_FLAGS] & PPD_RSP)                  /* response requested? */
        r = ci_respond (&pkt);                          /* driver wants it back */
    else
        r = ci_dispose (&pkt);                          /* dispose of packet */
    if (r != SCPE_OK)
        break;
    }
return SCPE_OK;
}

/* Read CI port register */

t_stat ci_dec_rd (int32 *val, int32 rg, int32 lnt)
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
        *val = ci_ppr | (ci_unit.CI_NODE & PPR_NODE);
        sim_debug (DBG_REG, &ci_dev, "PPR rd: %08X\n", *val);
        break;

    default:
        return SCPE_NXM;
        }

return SCPE_OK;
}

/* Write CI port register */

t_stat ci_dec_wr (int32 val, int32 rg, int32 lnt)
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
ci_max_dg = CI_ReadL (pa + PQB_DFQL_OF);                /* packet sizes */
ci_max_msg = CI_ReadL (pa + PQB_DFML_OF);
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

t_stat ci_send_data (CI_PKT *pkt)
{
uint32 data_len, total_data_len;
uint16 snd_name, page_offset;
uint32 snd_offset, rec_offset, data_pte;
t_stat r;

total_data_len = CI_GET32 (pkt->data, 0x18);
snd_name = CI_GET16 (pkt->data, 0x1C);                  /* Get BDT table offset */
snd_offset = CI_GET32 (pkt->data, 0x20);                /* Get data starting offset */
rec_offset = CI_GET32 (pkt->data, 0x28);                /* Get data starting offset */
data_pte = GVP_Read (&ci_mmu, ci_bdt_va + (snd_name * BD_LEN) + BD_PTE_OF, L_LONG); /* Get PTE addr */
page_offset = GVP_Read (&ci_mmu, ci_bdt_va + (snd_name * BD_LEN), L_WORD) & 0x1FF;

while (total_data_len > 0) {
    if (total_data_len > CI_MAXDAT)
        data_len = CI_MAXDAT;
    else
        data_len = total_data_len;
    if (pkt->data[PPD_OPC] == OPC_REQDATREC)
        pkt->data[PPD_OPC] = OPC_RETDAT;
    ci_readb (data_pte, data_len, page_offset + snd_offset, &pkt->data[CI_DATHDR]);
    CI_PUT32 (pkt->data, 0x18, total_data_len);         /* update packet */
    CI_PUT32 (pkt->data, 0x28, rec_offset);
    pkt->length = data_len + CI_DATHDR;
    r = ci_send_ppd (pkt);
    if (r != SCPE_OK)
        return r;
    total_data_len -= data_len;
    snd_offset += data_len;
    rec_offset += data_len;
    }
return SCPE_OK;
}

t_stat ci_receive_data (CI_PKT *pkt)
{
uint32 data_len, total_data_len;
uint16 rec_name, page_offset;
uint32 rec_offset, data_pte;
t_stat r;

total_data_len = CI_GET32 (pkt->data, 0x18);
rec_name = CI_GET16 (pkt->data, 0x24);                  /* Get BDT table offset */
rec_offset = CI_GET32 (pkt->data, 0x28);                /* Get data starting offset */
data_pte = GVP_Read (&ci_mmu, ci_bdt_va + (rec_name * BD_LEN) + BD_PTE_OF, L_LONG); /* Get PTE addr */
page_offset = GVP_Read (&ci_mmu, ci_bdt_va + (rec_name * BD_LEN), L_WORD) & 0x1FF;

if (total_data_len > CI_MAXDAT)
    data_len = CI_MAXDAT;
else
    data_len = total_data_len;

ci_writeb (data_pte, data_len, page_offset + rec_offset, &pkt->data[CI_DATHDR]);

total_data_len -= data_len;

if (total_data_len == 0) {
    pkt->length = CI_DATHDR;
    if (pkt->data[PPD_OPC] == OPC_SNDDATREC) {
        pkt->data[PPD_OPC] = OPC_RETCNF;
        return ci_send_ppd (pkt);
        }
    else                                                /* DATREC */
        return ci_receive_int (pkt);                    /* pass to system */
    }
return SCPE_OK;
}

t_stat ci_request_id (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
pkt->data[PPD_STATUS] = 0;                              /* status: OK */
pkt->data[PPD_OPC] = OPC_RETID;                         /* set opcode */
if (ci_check_vc (pkt->port, port)) {                    /* don't send ID once VC is open? */
    CI_PUT32 (pkt->data, 0x10, 1);                      /* transaction ID low (MBZ to trigger START) */
    }
else {
    CI_PUT32 (pkt->data, 0x10, 0);                      /* transaction ID low (MBZ to trigger START) */
    }
CI_PUT32 (pkt->data, 0x14, 0);                          /* transaction ID high (MBZ to trigger START) */
#if 0                                                   /* SHAC */
CI_PUT32 (pkt->data, 0x18, 0x80000022);                 /* remote port type */
CI_PUT32 (pkt->data, 0x1C, 0x03060D22);                 /* code revision */
CI_PUT32 (pkt->data, 0x20, 0xFFFF0D00);                 /* function mask */
#else                                                   /* CI780, CI750 */
CI_PUT32 (pkt->data, 0x18, 0x80000002);                 /* remote port type */
CI_PUT32 (pkt->data, 0x1C, 0x00700040);                 /* code revision */
CI_PUT32 (pkt->data, 0x20, 0xFFFF0F00);                 /* function mask */
#endif
pkt->data[0x24] = 0x0;                                  /* resetting port */
// TODO: should also respond when disabled?
pkt->data[0x25] = 0x4;                                  /* port state (maint = 0, state = enabled) */
pkt->length = 0x26;  // TODO: check length, add #define
return ci_send_ppd (pkt);
}

/* Read a packet from memory */

void ci_read_packet (CI_PKT *pkt, size_t length)
{
int32 i;

sim_debug (DBG_TRC, &ci_dev, "ci_read_packet:  addr: %08X, pkt->length: %d, length: %d\n", pkt->addr, pkt->length, length);
for (i = PPD_SIZE; i < length; i++)                     /* skip the queue pointers */
    pkt->data[i] = GVP_Read (&ci_mmu, (pkt->addr + i), L_BYTE);
}

/* Write a packet to memory */

void ci_write_packet (CI_PKT *pkt, size_t length)
{
int32 i;

sim_debug (DBG_TRC, &ci_dev, "ci_write_packet: addr: %08X, pkt->length: %d, length: %d\n", pkt->addr, pkt->length, length);
for (i = PPD_PORT; i < length; i++)                     /* don't overwrite the header */
    GVP_Write (&ci_mmu, (pkt->addr + i), pkt->data[i], L_BYTE);
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
        pa = ((pte & 0x1fffff) << 9) | VA_GETOFF(i);  // TODO: Check that 0x1fffff is correct
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

/* Put a packet on the response queue */

t_stat ci_put_rsq (CI_PKT *pkt)
{
if (gvp_insqti (&ci_mmu, ci_pqbb_va + PQB_RESP_OF, pkt->addr) > 1) {
    ci_psr |= PSR_RQA;
    ci_set_int ();
    }
return SCPE_OK;
}

/* Put a packet on the datagram free queue */

t_stat ci_put_dfq (CI_PKT *pkt)
{
uint32 hdr;

hdr = GVP_Read (&ci_mmu, ci_pqbb_va + PQB_DFQA_OF, L_LONG);
gvp_insqti (&ci_mmu, hdr, pkt->addr);
return SCPE_OK;
}

/* Get a packet from the response queue */

t_stat ci_get_dfq (CI_PKT *pkt)
{
uint32 hdr;

hdr = GVP_Read (&ci_mmu, ci_pqbb_va + PQB_DFQA_OF, L_LONG);
pkt->addr = gvp_remqhi (&ci_mmu, hdr);
return SCPE_OK;
}

/* Put a packet on the message free queue */

t_stat ci_put_mfq (CI_PKT *pkt)
{
uint32 hdr;

hdr = GVP_Read (&ci_mmu, ci_pqbb_va + PQB_MFQA_OF, L_LONG);
gvp_insqti (&ci_mmu, hdr, pkt->addr);
return SCPE_OK;
}

/* Get a packet from the datagram free queue */

t_stat ci_get_mfq (CI_PKT *pkt)
{
uint32 hdr;

hdr = GVP_Read (&ci_mmu, ci_pqbb_va + PQB_MFQA_OF, L_LONG);
pkt->addr = gvp_remqhi (&ci_mmu, hdr);
return SCPE_OK;
}

/* Dispose of an outgoing packet that does not need a response */

t_stat ci_dispose (CI_PKT *pkt)
{
if (pkt->addr != 0) {
    if (pkt->data[PPD_TYPE] == DYN_SCSMSG)
        return ci_put_mfq (pkt);
    else
        return ci_put_dfq (pkt);
    }
return SCPE_IERR;
}

/* Respond to an outgoing packet */

t_stat ci_respond (CI_PKT *pkt)
{
if (pkt->addr != 0) {
    ci_write_packet (pkt, 0x10); // TODO: move header size definitions to vax_ci.h
    return ci_put_rsq (pkt);
    }
return SCPE_IERR;
}

/* New incoming packet received (internal version) */

t_stat ci_receive_int (CI_PKT *pkt)
{
t_stat r;
if (pkt->addr == 0) {
    if (pkt->data[PPD_TYPE] == DYN_SCSMSG)
        r = ci_get_mfq (pkt);
    else
        r = ci_get_dfq (pkt);
    if (r != SCPE_OK)
        return r;
    }
ci_write_packet (pkt, pkt->length);
return ci_put_rsq (pkt);
}

/* New incoming packet received */

t_stat ci_receive (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];

switch (pkt->data[PPD_OPC]) {                           /* opcodes handled by port */
    case OPC_IDREC:
        if (ci_check_vc (pkt->port, port))              /* VC open? */
            return SCPE_OK;                             /* discard */
        break;

    case OPC_REQDATREC:
        return ci_send_data (pkt);

    case OPC_SNDDATREC:
    case OPC_DATREC:
        return ci_receive_data (pkt);

    case OPC_REQREC:
        return ci_request_id (pkt);
        }
return ci_receive_int (pkt);
}

t_stat ci_send (CI_PKT *pkt)
{
switch (pkt->data[PPD_OPC]) {                           /* opcodes handled by port */
    case OPC_SNDDAT:
        return ci_send_data (pkt);
        }
return ci_send_ppd (pkt);
}

t_stat ci_dec_svc (UNIT *uptr)
{
t_stat r;
CI_PKT pkt;

do {
    pkt.port = ci_unit.CI_NODE;
    r = ci_receive_ppd (&pkt);
    if (r == SCPE_OK)
        r = ci_receive (&pkt);
} while (r == SCPE_OK);
// TODO: handle errors from ci_receive
ci_svc (uptr);
sim_clock_coschedule (uptr, tmxr_poll);
return SCPE_OK;
}

/* Reset CI adapter */

t_stat ci_dec_reset (DEVICE *dptr)
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