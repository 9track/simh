/* vax_hsc.c: Hierarchical Storage Controller

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

   hsc            HSC50/HSC70
*/

#include "vax_defs.h"
#include "pdp11_uqssp.h"
#include "vax_ci.h"
#include "vax_rf.h"

/* Limits */

#define MAX_CONN        5                               /* max connections */
#define MAX_BUFF        (CI_MAX_NODES)                  /* max buffer descriptors */

#define HSC_TIMER       1

/* SYSAP descriptor */

typedef struct {
    char   *name;                                       /* SYSAP name */
    uint32 inicr;                                       /* initial credits */
    uint32 mincr;                                       /* min credits */
    t_stat (*msgrec)(uint32 conid, CI_PKT *pkt);
} SYSAP;

/* Buffer descriptor */

typedef struct {
    uint8 *buf;                                         /* buffer pointer */
    size_t xfrsz;                                       /* buffer length */
    uint32 boff;                                        /* buffer offset */
    UNIT *uptr;                                         /* buffer owner */
} BDT;

/* SCS Connection descriptor */

typedef struct {
    uint32 port;                                        /* remote port */
    uint32 conid;                                       /* remote conid */
    uint32 credits;                                     /* current credits */
    SYSAP *sysap;                                       /* SYSAP descriptor */
} CONN;

/* Virtual circuit states */

#define STATE_CLOSED    0                               /* VC closed */
#define STATE_IDREC     1                               /* remote port ID received */
#define STATE_STSENT    2                               /* START sent */
#define STATE_STRECV    3                               /* START received */
#define STATE_VCOPEN    4                               /* VC open */

uint32 hsc_sysid = 1;
char hsc_name[6];
uint32 hsc_state[CI_MAX_NODES];
uint32 hsc_path;
CONN hsc_conn[MAX_CONN];
BDT hsc_bdt[MAX_BUFF];

extern int32 tmxr_poll;

t_stat hsc_scsdir (uint32 conid, CI_PKT *pkt);
t_stat hsc_mscp (uint32 conid, CI_PKT *pkt);
t_stat hsc_reqid (CI_PKT *pkt);
t_stat hsc_start (CI_PKT *pkt, uint32 dg_type);
t_stat hsc_stack (CI_PKT *pkt);
t_stat hsc_show_sysid (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat hsc_set_sysid (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat hsc_tmrsvc (UNIT *uptr);
t_stat hsc_svc (UNIT *uptr);
t_stat hsc_reset (DEVICE *dptr);

/* HSC data structures

   hsc_dev    HSC device descriptors
   hsc_unit   HSC unit
   hsc_reg    HSC register list
*/

SYSAP hsc_sysap[] = {
    { "SCS$DIRECTORY", 1, 0, hsc_scsdir },
    { "MSCP$DISK", 10, 1, hsc_mscp },
    { 0 }
    };

UNIT hsc_unit[] = {
    { UDATA (&hsc_svc, UNIT_IDLE, 0) },
    { UDATA (&hsc_tmrsvc, UNIT_IDLE|UNIT_DIS, 0) }
    };

REG hsc_reg[] = {
    { NULL }
    };

MTAB hsc_mod[] = {
    { MTAB_XTD|MTAB_VDV, 0, "NODE", "NODE",
      &ci_set_node, &ci_show_node },
    { MTAB_XTD|MTAB_VDV, 0, "SYSID", "SYSID",
      &hsc_set_sysid, &hsc_show_sysid },
    { 0 }
    };

DEBTAB hsc_debug[] = {
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

DEVICE hsc_dev = {
    "HSC", hsc_unit, hsc_reg, hsc_mod,
    2, 0, 0, 0, 0, 0,
    NULL, NULL, &hsc_reset,
    NULL, NULL, NULL,
    NULL, DEV_DEBUG | DEV_DISABLE | DEV_DIS | DEV_CI, 0,
    hsc_debug, 0, 0
    };

/*
CI_NODE ci_node = {
    RPORT_TYPE,
    CODE_REV,
    FUNC_MASK,
    &ci_receive
    };
*/

/* Find a connection with the given id */

CONN *hsc_getconn (uint32 conid)
{
if (conid < MAX_CONN) {                                 /* valid? */
    if (hsc_conn[conid].sysap)                          /* connected? */
        return &hsc_conn[conid];
    }
return NULL;
}

void hsc_fail_vc (uint32 port)
{
uint32 i;

for (i = 0; i < MAX_CONN; i++) {
    if (hsc_conn[i].port == port)
        memset (&hsc_conn[i], 0, sizeof (CONN));
    }
ci_close_vc (&hsc_unit[0], port);
}

/* Find a SYSAP with the given name */

SYSAP* hsc_getsysap (char *name)
{
SYSAP *sysap;

for (sysap = &hsc_sysap[0]; sysap->name; sysap++) {
    if (strncmp (name, sysap->name, strlen (sysap->name)) == 0)
        break;                                          /* name matches */
    }
if (sysap->name)
    return sysap;                                       /* SYSAP found */
else
    return NULL;                                        /* not found */
}

/* SYSAP - SCS$DIRECTORY */

t_stat hsc_scsdir (uint32 conid, CI_PKT *pkt)
{
uint16 credits = CI_GET16 (pkt->data, SCS_CREDIT);
CONN *conn = hsc_getconn (conid);
SYSAP *sysap = hsc_getsysap (&pkt->data[SCS_APPL + 0x4]);
uint32 srccon = CI_GET32 (pkt->data, SCS_SRCCON);

conn->credits += credits;
pkt->length = 0x44;
pkt->data[PPD_OPC] = OPC_SNDMSG;
CI_PUT16 (pkt->data, PPD_LENGTH, (pkt->length - PPD_MSGHDR));
CI_PUT16 (pkt->data, SCS_CREDIT, 1);
conn->credits--;
CI_PUT32 (pkt->data, SCS_DSTCON, srccon);
CI_PUT32 (pkt->data, SCS_SRCCON, conid);
if (sysap) {
    CI_PUT16 (pkt->data, SCS_APPL, 1);                  /* status OK */
    sprintf (&pkt->data[SCS_APPL + 0x14], "%-16s", sysap->name);
    }
else {
    CI_PUT16 (pkt->data, SCS_APPL, 0x20A4);             /* no such object */
    }
return ci_send_ppd (&hsc_unit[0], pkt);                 /* send response */
}

/* SYSAP - MSCP$DISK */

t_stat hsc_mscp (uint32 conid, CI_PKT *pkt)
{
uint16 credits = CI_GET16 (pkt->data, SCS_CREDIT);
CONN *conn = hsc_getconn (conid);
uint16 rf_pkt;
MSC *cp = &rf_ctx;

conn->credits += credits;
rf_pkt = 0;
if (!rf_deqf (cp, &rf_pkt))                             /* get cmd pkt */
    return SCPE_IERR;
cp->hat = 0;                                            /* dsbl hst timer */
cp->cid = conid;
memcpy (cp->pak[rf_pkt].d, &pkt->data[SCS_APPL + UQ_HDR_OFF], RF_PKT_SIZE);
rf_mscp (cp, rf_pkt, TRUE);
return SCPE_OK;
}

t_stat hsc_mscp_done (MSC *cp, uint16 rf_pkt)
{
CONN *conn = hsc_getconn (cp->cid);
CI_PKT pkt;

rf_enqh (cp, &cp->freq, rf_pkt);                        /* pkt is free */
cp->pbsy = cp->pbsy - 1;                                /* decr busy cnt */
if (cp->pbsy == 0)                                      /* idle? strt hst tmr */
    cp->hat = cp->htmo;
if (conn == NULL)
    return SCPE_OK;
pkt.length = (SCS_APPL + cp->pak[rf_pkt].d[UQ_HLNT]);
pkt.data[PPD_PORT] = conn->port;                        /* dest port */
pkt.data[PPD_STATUS] = 0;                               /* status */
pkt.data[PPD_OPC] = OPC_SNDMSG;                         /* opcode */
pkt.data[PPD_FLAGS] = 0;                                /* flags */
CI_PUT16 (pkt.data, PPD_LENGTH, (pkt.length - PPD_DGHDR));
CI_PUT16 (pkt.data, PPD_MTYPE, DG_SCSMSG);
CI_PUT16 (pkt.data, PPD_STYPE, SCS_APPMSG);
CI_PUT16 (pkt.data, SCS_CREDIT, 1);                     /* credit extension */
conn->credits--;
memcpy (&pkt.data[SCS_APPL + UQ_HDR_OFF], cp->pak[rf_pkt].d, RF_PKT_SIZE); /* application message */
CI_PUT32 (pkt.data, SCS_DSTCON, conn->conid);           /* destination connection ID */
CI_PUT32 (pkt.data, SCS_SRCCON, cp->cid);               /* source connection ID */
return ci_send_ppd (&hsc_unit[0], &pkt);
}

/* SCS connection request */

t_stat hsc_conreq (CI_PKT *pkt)
{
SYSAP *sysap = hsc_getsysap (&pkt->data[SCS_DSTPROC]);
uint32 srccon = CI_GET32 (pkt->data, SCS_SRCCON);
uint16 credits = CI_GET16 (pkt->data, SCS_CREDIT);
uint16 stat = SCS_STNORM;
uint32 conid = 0;
uint32 i;

pkt->length = 0x28;                                     /* build CONRSP */
pkt->data[PPD_OPC] = OPC_SNDMSG;
CI_PUT16 (pkt->data, PPD_LENGTH, (pkt->length - PPD_MSGHDR));
CI_PUT16 (pkt->data, PPD_STYPE, SCS_CONRSP);
CI_PUT16 (pkt->data, SCS_CREDIT, 0);
CI_PUT32 (pkt->data, SCS_DSTCON, srccon);
if (!sysap)                                             /* SYSAP not found? */
    stat = SCS_STNOMAT;                                 /* no matching listener */
else {
    for (i = 1; i < MAX_CONN; i++) {
        if (hsc_conn[i].sysap == NULL) {                /* free connection? */
            hsc_conn[i].sysap = sysap;                  /* init connection */
            hsc_conn[i].port = pkt->data[PPD_PORT];
            hsc_conn[i].conid = srccon;
            hsc_conn[i].credits = credits;
            conid = i;
            break;
            }
        }
    if (!conid)
        stat = SCS_STNORS;                              /* no resources */
    }
CI_PUT32 (pkt->data, SCS_SRCCON, conid);
CI_PUT16 (pkt->data, SCS_STATUS, stat);
ci_send_ppd (&hsc_unit[0], pkt);                        /* send CONRSP */
if (stat == SCS_STNORM) {
    pkt->length = 0x54;                                 /* build ACCREQ */
    CI_PUT16 (pkt->data, PPD_LENGTH, (pkt->length - PPD_MSGHDR));
    pkt->data[PPD_STYPE] = SCS_ACCREQ;
    CI_PUT16 (pkt->data, SCS_CREDIT, sysap->inicr);     /* initial credits */
    CI_PUT32 (pkt->data, SCS_SRCCON, conid);
    CI_PUT16 (pkt->data, SCS_MINCR, sysap->mincr);      /* min credits */
    strncpy (&pkt->data[SCS_DSTPROC], &pkt->data[SCS_SRCPROC], 16);
    sprintf (&pkt->data[SCS_SRCPROC], "%-16s", sysap->name);
    sprintf (&pkt->data[SCS_CONDAT], "%-16s", "");
    ci_send_ppd (&hsc_unit[0], pkt);                    /* send ACCREQ */
    }
return SCPE_OK;
}

/* SCS accept response */

t_stat hsc_accrsp (CI_PKT *pkt)
{
return SCPE_OK;
}

/* SCS reject response */

t_stat hsc_rejrsp (CI_PKT *pkt)
{
return SCPE_OK;
}

/* SCS disconnect request */

t_stat hsc_disreq (CI_PKT *pkt)
{
uint32 conid = CI_GET32 (pkt->data, SCS_DSTCON);
uint32 srccon = CI_GET32 (pkt->data, SCS_SRCCON);
CONN *conn = hsc_getconn (conid);

pkt->length = 0x20;
pkt->data[PPD_OPC] = OPC_SNDMSG;
CI_PUT16 (pkt->data, PPD_LENGTH, (pkt->length - PPD_MSGHDR));
CI_PUT16 (pkt->data, PPD_STYPE, SCS_DISRSP);
CI_PUT16 (pkt->data, SCS_CREDIT, 0);
CI_PUT32 (pkt->data, SCS_DSTCON, srccon);
CI_PUT32 (pkt->data, SCS_SRCCON, 0);
if (conn->sysap) {                                      /* connection found? */
    CI_PUT32 (pkt->data, SCS_SRCCON, conid);
    conn->port = 0;
    conn->conid = 0;
    conn->sysap = NULL;
    }
ci_send_ppd (&hsc_unit[0], pkt);                        /* send DISRSP */
return SCPE_OK;
}

/* SCS credit request */

t_stat hsc_crdreq (CI_PKT *pkt)
{
uint16 credits = CI_GET16 (pkt->data, SCS_CREDIT);
uint32 conid = CI_GET32 (pkt->data, SCS_DSTCON);
uint32 srccon = CI_GET32 (pkt->data, SCS_SRCCON);
CONN *conn = hsc_getconn (conid);

pkt->length = 0x20;
pkt->data[PPD_OPC] = OPC_SNDMSG;
CI_PUT16 (pkt->data, PPD_LENGTH, (pkt->length - PPD_MSGHDR));
CI_PUT16 (pkt->data, PPD_STYPE, SCS_CRRSP);
CI_PUT32 (pkt->data, SCS_DSTCON, srccon);
CI_PUT32 (pkt->data, SCS_SRCCON, 0);
if (conn->sysap) {                                      /* connection found? */
    CI_PUT32 (pkt->data, SCS_SRCCON, conid);
    if ((conn->credits - credits) < conn->sysap->mincr) {
        credits = conn->credits - conn->sysap->mincr;
        conn->credits = conn->sysap->mincr;
        }
    else
        conn->credits = conn->credits - credits;
    }
else {
    CI_PUT16 (pkt->data, SCS_CREDIT, 0);
    }
ci_send_ppd (&hsc_unit[0], pkt);                        /* send CRRSP */
return SCPE_OK;
}

/* SCS application message */

t_stat hsc_appmsg (CI_PKT *pkt)
{
uint32 conid = CI_GET32 (pkt->data, SCS_DSTCON);
CONN *conn = hsc_getconn (conid);

if (conn)
    return conn->sysap->msgrec (conid, pkt);
return SCPE_IERR;
}

/* SCS packet received */

t_stat hsc_scsrec (CI_PKT *pkt)
{
uint32 msg_size = CI_GET16 (pkt->data, PPD_LENGTH) + PPD_MSGHDR;
uint32 msg_type = CI_GET16 (pkt->data, PPD_STYPE);

switch (msg_type) {
    
    case SCS_CONREQ:                                    /* connect request */
        return hsc_conreq (pkt);

    case SCS_ACCRSP:                                    /* accept response */
        return hsc_accrsp (pkt);

    case SCS_REJRSP:                                    /* reject respone */
        return hsc_rejrsp (pkt);

    case SCS_DISREQ:                                    /* disconnect request */
        return hsc_disreq (pkt);

    case SCS_CRREQ:                                     /* credit request */
        return hsc_crdreq (pkt);

    case SCS_APPMSG:                                    /* application message */
    case SCS_APPDG:                                     /* application datagram */
        return hsc_appmsg (pkt);

    default:
        break;
        }
}

/* PPD datagram or SCS message received */

t_stat hsc_dgrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 dg_type = CI_GET16 (pkt->data, PPD_MTYPE);

switch (dg_type) {

    case DG_START:
        if (hsc_state[port] < STATE_STRECV)
            hsc_state[port] = STATE_STRECV;
        if (hsc_state[port] == STATE_VCOPEN) {          /* VC already open? */
            hsc_state[port] = STATE_IDREC;
            hsc_fail_vc (port);                         /* close VC and start again */
            }
        if (hsc_state[port] >= STATE_IDREC)             /* got remote port ID? */
            return hsc_start (pkt, DG_STACK);           /* send STACK */
        else
            return hsc_reqid (pkt);                     /* request remote port ID */
        break;

    case DG_STACK:
        if (hsc_state[port] < STATE_STSENT)
            return SCPE_OK;
        if (hsc_state[port] < STATE_VCOPEN) {
            hsc_state[port] = STATE_VCOPEN;             /* open VC */
            ci_open_vc (&hsc_unit[0], port);
            }
        return hsc_stack (pkt);                         /* send ACK */

    case DG_ACK:
        if (hsc_state[port] < STATE_STSENT)
            return SCPE_OK;
        if (hsc_state[port] < STATE_VCOPEN) {
            hsc_state[port] = STATE_VCOPEN;             /* open VC */
            ci_open_vc (&hsc_unit[0], port);
            }
        break;

    case DG_SCSMSG:
        return hsc_scsrec (pkt);

    case DG_HOSTSHUT:
        hsc_state[port] = STATE_CLOSED;                 /* close VC */
        hsc_fail_vc (port);
        break;

    default:
        break;
        }
return SCPE_OK;
}

/* Process request for port ID */

t_stat hsc_reqrec (CI_PKT *pkt)
{
pkt->data[PPD_STATUS] = 0;                              /* status: OK */
pkt->data[PPD_OPC] = OPC_RETID;                         /* set opcode */
if (ci_check_vc (&hsc_unit[0], pkt->data[PPD_PORT])) {  /* don't send ID once VC is open? */
    CI_PUT32 (pkt->data, 0x10, 1);                      /* transaction ID low (MBZ to trigger START) */
    }
else {
    CI_PUT32 (pkt->data, 0x10, 0);                      /* transaction ID low (MBZ to trigger START) */
    }
CI_PUT32 (pkt->data, 0x14, 0);                          /* transaction ID high (MBZ to trigger START) */
CI_PUT32 (pkt->data, 0x18, (CI_DUALPATH | CI_HSC));     /* remote port type */
CI_PUT32 (pkt->data, 0x1C, 0x03060D22);                 /* code revision */
CI_PUT32 (pkt->data, 0x20, 0xFFFF0D00);                 /* function mask */
pkt->data[0x24] = hsc_unit[0].ci_node;                  /* resetting port */
// TODO: should also respond when disabled?
pkt->data[0x25] = 0x4;                                  /* port state (maint = 0, state = enabled) */
pkt->length = 0x26;  // TODO: check length, add #define
return ci_send_ppd (&hsc_unit[0], pkt);
}

/* Request remote port ID */

t_stat hsc_reqid (CI_PKT *pkt)
{
pkt->length = PPD_HDR;
pkt->data[PPD_STATUS] = 0;
pkt->data[PPD_OPC] = OPC_REQID;
return ci_send_ppd (&hsc_unit[0], pkt);
}

/* Process START datagram */

t_stat hsc_start (CI_PKT *pkt, uint32 dg_type)
{
/* send START */
pkt->length = (PPD_TIME + 8); // TODO: add #define
pkt->data[PPD_OPC] = OPC_SNDDG;
CI_PUT16 (pkt->data, PPD_MTYPE, dg_type);
CI_PUT16 (pkt->data, PPD_LENGTH, (pkt->length - PPD_DGHDR));
CI_PUT32 (pkt->data, PPD_SYSID, hsc_sysid);             /* sending system ID */
CI_PUT16 (pkt->data, PPD_SYSID + 4, 0x0800);
pkt->data[PPD_PROTO] = PPD_BASE;
#if 0
CI_PUT16 (pkt->data, PPD_MAXDG, 0x40);                  /* max datagram size */
CI_PUT16 (pkt->data, PPD_MAXMSG, 0x40);                 /* max message size */
sprintf (&pkt->data[PPD_SWTYPE], "RFXX");               /* software type */
sprintf (&pkt->data[PPD_SWVER], "V103");                /* software version */
//pkt->data[PPD_SWIN]                                     /* software incarnation */
sprintf (&pkt->data[PPD_HWTYPE], "RF71");               /* hardware type */
sprintf (&pkt->data[PPD_HWVER], "PCB-5/ECO-00");        /* hardware version */
#else
CI_PUT16 (pkt->data, PPD_MAXDG, 0x3E);                  /* max datagram size */
CI_PUT16 (pkt->data, PPD_MAXMSG, 0x42);                 /* max message size */
sprintf (&pkt->data[PPD_SWTYPE], "HSC ");               /* software type */
sprintf (&pkt->data[PPD_SWVER], "V600");                /* software version */
CI_PUT32 (pkt->data, PPD_SWIN + 0, 0);                  /* software incarnation */
CI_PUT32 (pkt->data, PPD_SWIN + 4, 0);
sprintf (&pkt->data[PPD_HWTYPE], "HS70");               /* hardware type */
CI_PUT16 (pkt->data, PPD_HWVER + 0, 0x0222);            /* hardware version */
CI_PUT16 (pkt->data, PPD_HWVER + 2, 0x0222);
CI_PUT16 (pkt->data, PPD_HWVER + 4, 0x0222);
CI_PUT16 (pkt->data, PPD_HWVER + 6, 0x0227);
CI_PUT16 (pkt->data, PPD_HWVER + 8, 0x0222);
CI_PUT16 (pkt->data, PPD_HWVER + 10, 0x0222);
#endif
sprintf (&pkt->data[PPD_NODE], "SIMH    ");             /* node name */
memset (&pkt->data[PPD_TIME], 0, 8);                    /* current time */
return ci_send_ppd (&hsc_unit[0], pkt);
}

/* Process STACK datagram */

t_stat hsc_stack (CI_PKT *pkt)
{
pkt->length = PPD_MTYPE + 2;
pkt->data[PPD_OPC] = OPC_SNDDG;
pkt->data[PPD_STATUS] = 0;
pkt->data[PPD_FLAGS] = 0;
CI_PUT16 (pkt->data, PPD_MTYPE, DG_ACK);
CI_PUT16 (pkt->data, PPD_LENGTH, (pkt->length - PPD_DGHDR));
return ci_send_ppd (&hsc_unit[0], pkt);
}

/* Send block data */

int32 hsc_snddat (uint32 *bd, uint8 *buf, size_t xfrsz, UNIT *uptr)
{
CI_PKT pkt;
BDT *bdt = NULL;
CONN *conn = hsc_getconn (bd[2]);
uint32 len, i, rboff;
t_stat r = SCPE_OK;

for (i = 0; i < MAX_BUFF; i++) {                        /* find existing buffer descriptor */
    if (hsc_bdt[i].buf == buf) {
        bdt = &hsc_bdt[i];
        break;
        }
    }
if (bdt == NULL) {
    for (i = 0; i < MAX_BUFF; i++) {                    /* find free buffer descriptor */
        if (hsc_bdt[i].xfrsz == 0) {
            bdt = &hsc_bdt[i];
            bdt->buf = buf;                             /* setup buffer */
            bdt->boff = 0;
            bdt->xfrsz = xfrsz;
            bdt->uptr = uptr;
            break;
            }
        }
    }
if (bdt == NULL)                                        /* none available? */
    return xfrsz;                                       /* error */

if (bdt->boff == bdt->xfrsz) {                          /* transfer done? */
    bdt->buf = NULL;                                    /* remove buffer */
    bdt->boff = 0;
    bdt->xfrsz = 0;
    bdt->uptr = NULL;
    return 0;                                           /* done */
    }

pkt.data[PPD_PORT] = conn->port;                        /* dest port */
pkt.data[PPD_STATUS] = 0;                               /* status */
pkt.data[PPD_OPC] = OPC_SNDDAT;                         /* opcode */
// TODO: Need to set flags (multiple, last packet)
pkt.data[PPD_FLAGS] = 0;                                /* flags */
CI_PUT32 (pkt.data, PPD_LCONID, bd[2]);                 /* local connection ID */
CI_PUT32 (pkt.data, PPD_RSPID, i);                      /* local response ID */
CI_PUT32 (pkt.data, PPD_SBNAM, i);
CI_PUT32 (pkt.data, PPD_RBNAM, bd[1]);

rboff = bd[0] + bdt->boff;
xfrsz = xfrsz - bdt->boff;
while (xfrsz) {
    len = (xfrsz > CI_MAXDAT) ? CI_MAXDAT : xfrsz;
    pkt.length = len + CI_DATHDR;
    CI_PUT32 (pkt.data, PPD_XFRSZ, xfrsz);
    CI_PUT32 (pkt.data, PPD_SBOFF, bdt->boff);
    CI_PUT32 (pkt.data, PPD_RBOFF, rboff);
    if (xfrsz <= CI_MAXDAT)
        pkt.data[PPD_FLAGS] |= PPD_LP;                  /* last packet */
    memcpy (&pkt.data[CI_DATHDR], &bdt->buf[bdt->boff], len);
    r = ci_send_ppd (&hsc_unit[0], &pkt);
    if (r != SCPE_OK) {
        sim_printf ("HSC stalled during SNDDAT operation\n");
        sim_activate (bdt->uptr, tmxr_poll);            /* retry */
        return -1;                                      /* FIXME: handle errors other than buffer full */
        }
    bdt->boff += len;
    rboff += len;
    xfrsz -= len;
    }
return -1;                                              /* I/O in progress */
}

/* Request block data */

int32 hsc_reqdat (uint32 *bd, uint8 *buf, size_t xfrsz, UNIT *uptr)
{
CI_PKT pkt;
BDT *bdt = NULL;
CONN *conn = hsc_getconn (bd[2]);
uint32 i;
t_stat r;

for (i = 0; i < MAX_BUFF; i++) {                        /* find existing buffer descriptor */
    if (hsc_bdt[i].buf == buf) {
        bdt = &hsc_bdt[i];
        break;
        }
    }
if (bdt == NULL) {
    for (i = 0; i < MAX_BUFF; i++) {                    /* find free buffer descriptor */
        if (hsc_bdt[i].xfrsz == 0) {
            bdt = &hsc_bdt[i];
            bdt->buf = buf;                             /* setup buffer */
            bdt->boff = 0;
            bdt->xfrsz = xfrsz;
            bdt->uptr = uptr;
            break;
            }
        }
    }
if (bdt == NULL)                                        /* none available? */
    return xfrsz;

if (bdt->boff == bdt->xfrsz) {                          /* transfer done? */
    bdt->buf = NULL;                                    /* remove buffer */
    bdt->boff = 0;
    bdt->xfrsz = 0;
    bdt->uptr = NULL;
    return 0;                                           /* done */
    }

pkt.length = CI_DATHDR;
pkt.data[PPD_PORT] = conn->port;                        /* dest port */
pkt.data[PPD_STATUS] = 0;                               /* status */
pkt.data[PPD_OPC] = OPC_REQDAT;                         /* opcode */
// TODO: Need to set flags (multiple, last packet)
pkt.data[PPD_FLAGS] = 0;                                /* flags */
CI_PUT32 (pkt.data, PPD_LCONID, bd[2]);                 /* local connection ID */
CI_PUT32 (pkt.data, PPD_RSPID, i);                      /* local response ID */
CI_PUT32 (pkt.data, PPD_XFRSZ, xfrsz);
CI_PUT32 (pkt.data, PPD_SBNAM, bd[1]);
CI_PUT32 (pkt.data, PPD_SBOFF, bd[0]);
CI_PUT32 (pkt.data, PPD_RBNAM, i);
CI_PUT32 (pkt.data, PPD_RBOFF, 0);
r = ci_send_ppd (&hsc_unit[0], &pkt);
if (r != SCPE_OK) {
    sim_printf ("HSC stalled during REQDAT operation\n");
    sim_activate (bdt->uptr, tmxr_poll);                /* retry */
    return -1;                                          /* FIXME: handle errors other than buffer full */
    }
return -1;                                              /* I/O in progress */
}

/* Process received block data */

t_stat hsc_datrec (CI_PKT *pkt)
{
uint32 xfrsz = CI_GET32 (pkt->data, PPD_XFRSZ);
uint32 bnam = CI_GET16 (pkt->data, PPD_RBNAM);
uint32 boff = CI_GET32 (pkt->data, PPD_RBOFF);
uint32 len = (xfrsz > CI_MAXDAT) ? CI_MAXDAT : xfrsz;
BDT *bdt;

if (bnam < MAX_BUFF) {                                  /* valid name? */
    bdt = &hsc_bdt[bnam];
    if ((bdt->buf) && (boff < bdt->xfrsz)) {            /* valid buffer? */
        memcpy (&bdt->buf[boff], &pkt->data[CI_DATHDR], len);
        bdt->boff += len;
        }
    }

if (pkt->data[PPD_FLAGS] & PPD_LP) {                    /* last packet? */
//FIXME: Should only send RETCNF for SNDDATREC?
#if 0
    pkt->length = CI_DATHDR;
    pkt->data[PPD_OPC] = OPC_RETCNF;                    /* send confirmation */
    ci_send_ppd (&hsc_unit[0], pkt);
#endif
    sim_activate (bdt->uptr, 0);                        /* xfer done, reactivate */
    }
return SCPE_OK;
}

/* Process confirmation of block data */

t_stat hsc_cnfrec (CI_PKT *pkt)
{
uint32 bnam = CI_GET32 (pkt->data, PPD_RSPID);
BDT *bdt;

if (bnam < MAX_BUFF) {                                  /* valid name? */
    bdt = &hsc_bdt[bnam];
    sim_activate (bdt->uptr, 0);                        /* xfer done, reactivate */
    }
return SCPE_OK;
}

t_stat hsc_idrec (CI_PKT *pkt)
{
uint32 conid, rspid, port;

port = pkt->data[PPD_PORT];
conid = CI_GET32 (pkt->data, PPD_LCONID);
rspid = CI_GET32 (pkt->data, PPD_RSPID);
if (hsc_state[port] < STATE_IDREC)
    hsc_state[port] = STATE_IDREC;
if ((conid | rspid) == 0) {                             /* transaction ID = 0? */
    if (ci_check_vc (&hsc_unit[0], port)) {             /* VC open? */
        hsc_state[port] = STATE_IDREC;
        hsc_fail_vc (port);                             /* VC failed */
        }
    if (hsc_state[port] < STATE_STSENT)  
        hsc_state[port] = STATE_STSENT;
    return hsc_start (pkt, DG_START);               /* send START */
    }
return SCPE_OK;
}

/* Process received PPD */

t_stat hsc_ppd (CI_PKT *pkt)
{
uint32 opcode = pkt->data[PPD_OPC];
uint32 port = pkt->data[PPD_PORT];

switch (opcode) {

    case OPC_DGREC:
        return hsc_dgrec (pkt);

    case OPC_MSGREC:
        return hsc_dgrec (pkt);

    case OPC_REQREC:
        return hsc_reqrec (pkt);
        
    case OPC_IDREC:
        return hsc_idrec (pkt);

    case OPC_DATREC:
        return hsc_datrec (pkt);

    case OPC_CNFREC:
        return hsc_cnfrec (pkt);
        }
}

/* Clock service (roughly once per second) */

t_stat hsc_tmrsvc (UNIT *uptr)
{
CI_PKT pkt;
uint32 i;

for (i = 0; i < CI_MAX_NODES; i++) {
    if (i == hsc_unit[0].ci_node)                       /* local node? */
        continue;                                       /* yes, skip */
    pkt.data[PPD_PORT] = i;
    pkt.data[PPD_FLAGS] = (hsc_path << PPD_V_PS);
    hsc_reqid (&pkt);
    }
if (hsc_path == PPD_PS0)                                /* switch path */
    hsc_path = PPD_PS1;
else
    hsc_path = PPD_PS0;
sim_activate_after (uptr, 10000000);                    /* reactivate */
return SCPE_OK;
}

t_stat hsc_svc (UNIT *uptr)
{
t_stat r;
CI_PKT pkt;

do {
    r = ci_receive_ppd (uptr, &pkt);
    if (r == SCPE_OK)
        r = hsc_ppd (&pkt);
} while (r == SCPE_OK);
// TODO: handle errors from hsc_ppd
return ci_svc (uptr);
}

t_stat hsc_show_sysid (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
fprintf (st, "sysid=%d", hsc_sysid);
return SCPE_OK;
}

t_stat hsc_set_sysid (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
int32 r;
uint32 newid;

newid = (uint32) get_uint (cptr, 10, 0xFFFFFFFF, &r);
if (r != SCPE_OK)
    return r;
hsc_sysid = newid;
return SCPE_OK;
}

/* Reset controller */

t_stat hsc_reset (DEVICE *dptr)
{
uint32 i;

for (i = 0; i < CI_MAX_NODES; i++)
    hsc_state[i] = STATE_CLOSED;
hsc_path = PPD_PS0;
ci_port_reset (dptr);
if (dptr->flags & DEV_DIS)
    return SCPE_OK;
sim_activate_after (dptr->units + HSC_TIMER, 10000000);
sim_clock_coschedule (&hsc_unit[0], tmxr_poll);
return SCPE_OK;
}
