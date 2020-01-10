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

/* CI780 Port States */

#define MAX_CONN        5                               /* max connections */

typedef struct {
    char   *name;                                       /* SYSAP name */
    uint32 inicr;                                       /* initial credits */
    uint32 mincr;                                       /* min credits */
    t_stat (*msgrec)(uint32 conid, CI_PKT *pkt);
} SYSAP;

typedef struct {
    uint32 port;                                        /* remote port */
    uint32 conid;                                       /* remote conid */
    uint32 credits;                                     /* current credits */
    SYSAP *sysap;                                       /* SYSAP descriptor */
} CONN;

uint32 hsc_sysid = 1;
char hsc_name[6];
uint32 ci_cnfgr;                                        /* configuration reg */
uint32 ci_pmcsr;                                        /* port maintenance csr */
uint32 ci_madr;                                         /* mainteneance addr reg */
uint32 ci_mdatr[8192];                                  /* maintenanace data reg */
uint32 ci_local_store[1024];
CONN hsc_conn[MAX_CONN];

extern int32 tmxr_poll;

t_stat hsc_scsdir (uint32 conid, CI_PKT *pkt);
t_stat hsc_mscp (uint32 conid, CI_PKT *pkt);
t_stat hsc_start (CI_PKT *pkt, uint32 dg_type);
t_stat hsc_show_sysid (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat hsc_set_sysid (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
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

UNIT hsc_unit = { UDATA (&hsc_svc, UNIT_IDLE, 0) };

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
    { 0 }
    };

DEVICE hsc_dev = {
    "HSC", &hsc_unit, hsc_reg, hsc_mod,
    1, 0, 0, 0, 0, 0,
    NULL, NULL, &hsc_reset,
    NULL, NULL, NULL,
    NULL, DEV_DEBUG | DEV_DISABLE | DEV_DIS, 0,
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
return ci_send_ppd (pkt);                               /* send response */
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

pkt.port = hsc_unit.CI_NODE;                            /* source port */
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
rf_enqh (cp, &cp->freq, rf_pkt);                        /* pkt is free */
cp->pbsy = cp->pbsy - 1;                                /* decr busy cnt */
if (cp->pbsy == 0)                                      /* idle? strt hst tmr */
    cp->hat = cp->htmo;
return ci_send_ppd (&pkt);
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
CI_PUT32 (pkt->data, SCS_SRCCON, 0);
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
CI_PUT16 (pkt->data, SCS_STATUS, stat);
ci_send_ppd (pkt);                                      /* send CONRSP */
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
    ci_send_ppd (pkt);                                  /* send ACCREQ */
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
ci_send_ppd (pkt);                                      /* send DISRSP */
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
ci_send_ppd (pkt);                                      /* send CRRSP */
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

t_stat hsc_dgrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
uint32 dg_type = CI_GET16 (pkt->data, PPD_MTYPE);

switch (dg_type) {

    case DG_START:
        return hsc_start (pkt, DG_STACK);

    case DG_STACK:
        /* send ACK */
        /* send SETCKT */
        break;

    case DG_ACK:
        /* send SETCKT */
        break;

    case DG_SCSMSG:
        return hsc_scsrec (pkt);

    case DG_HOSTSHUT:
        /* send SETCKT? */
        break;

    default:
        break;
        }

}

t_stat hsc_reqrec (CI_PKT *pkt)
{
uint32 port = pkt->data[PPD_PORT];
pkt->data[PPD_STATUS] = 0;                              /* status: OK */
pkt->data[PPD_OPC] = OPC_RETID;                         /* set opcode */
//if (ci_check_vc (pkt->port, port)) {                    /* don't send ID once VC is open? */
//    CI_PUT32 (pkt->data, 0x10, 1);                      /* transaction ID low (MBZ to trigger START) */
//    }
//else {
    CI_PUT32 (pkt->data, 0x10, 0);                      /* transaction ID low (MBZ to trigger START) */
//    }
CI_PUT32 (pkt->data, 0x14, 0);                          /* transaction ID high (MBZ to trigger START) */
CI_PUT32 (pkt->data, 0x18, (CI_DUALPATH | CI_HSC));     /* remote port type */
CI_PUT32 (pkt->data, 0x1C, 0x03060D22);                 /* code revision */
CI_PUT32 (pkt->data, 0x20, 0xFFFF0D00);                 /* function mask */
pkt->data[0x24] = 0x0;                                  /* resetting port */
// TODO: should also respond when disabled?
pkt->data[0x25] = 0x4;                                  /* port state (maint = 0, state = enabled) */
pkt->length = 0x26;  // TODO: check length, add #define
return ci_send_ppd (pkt);
}

t_stat hsc_start (CI_PKT *pkt, uint32 dg_type)
{
/* send START */
pkt->length = (PPD_TIME + 8); // TODO: add #define
pkt->data[PPD_OPC] = OPC_SNDDG;
pkt->data[PPD_MTYPE] = dg_type;
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
return ci_send_ppd (pkt);
}

t_stat hsc_ppd (CI_PKT *pkt)
{
uint32 opcode = pkt->data[PPD_OPC];
switch (opcode) {

    case OPC_DGREC:
        return hsc_dgrec (pkt);

    case OPC_MSGREC:
        return hsc_dgrec (pkt);

    case OPC_REQREC:
        return hsc_reqrec (pkt);
        
    case OPC_IDREC:
        return hsc_start (pkt, DG_START);
        }
}

t_stat hsc_svc (UNIT *uptr)
{
t_stat r;
CI_PKT pkt;

do {
    pkt.port = uptr->CI_NODE;
    r = ci_receive_ppd (&pkt);
    if (r == SCPE_OK)
        r = hsc_ppd (&pkt);
} while (r == SCPE_OK);
// TODO: handle errors from hsc_ppd
//ci_svc (uptr);
sim_clock_coschedule (uptr, tmxr_poll);
return SCPE_OK;
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
//if (dptr->flags & DEV_DIS)
//    ci_register (port_no, NULL);
//else
//    ci_register (port_no, &ci_node);
ci_port_reset (dptr);
sim_clock_coschedule (&hsc_unit, tmxr_poll);
return SCPE_OK;
}
