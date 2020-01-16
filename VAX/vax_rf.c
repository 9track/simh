/* vax_rf.c: MSCP disk controller simulator

   Copyright (c) 2002-2013, Robert M Supnik
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

   rf           MSCP disk controller

   23-Oct-13    RMS     Revised for new boot setup routine
   09-Dec-12    MB      Added support for changing controller type.
   24-Oct-12    MB      Added mapped transfers for VAX
   29-Jan-11    HUH     Added RC25, RCF25 and RA80 disks
                        Not all disk parameters set yet
                        "KLESI" MSCP controller (3) / port (1) types for RC25 
                        not yet implemented
                        
                        Remarks on the RC25 disk drives: 
                        In "real" life the RC25 drives exist in pairs only,
                        one RC25 (removable) and one RCF25 (fixed) in one housing.
                        The removable platter has always got an even drive number 
                        (e.g. "0"), the fixed platter has always got the next (odd) 
                        drive number (e.g. "1"). These two rules are not enforced 
                        by the disk drive simulation.
   07-Mar-11    MP      Added working behaviors for removable device types.
                        This allows physical CDROM's to come online and be 
                        ejected.
   02-Mar-11    MP      Fixed missing information from save/restore which
                        caused operations to not complete correctly after 
                        a restore until the OS reset the controller.
   02-Feb-11    MP      Added Autosize support to rf_attach
   28-Jan-11    MP      Adopted use of sim_disk disk I/O library
                         - added support for the multiple formats sim_disk
                           provides (SimH, RAW, and VHD)
                         - adjusted to potentially leverage asynch I/O when 
                           available
                         - Added differing detailed debug output via sim_debug
   14-Jan-09    JH      Added support for RD32 disc drive
   18-Jun-07    RMS     Added UNIT_IDLE flag to timer thread
   31-Oct-05    RMS     Fixed address width for large files
   16-Aug-05    RMS     Fixed C++ declaration and cast problems
   22-Jul-05    RMS     Fixed warning from Solaris C (Doug Gwyn)
   17-Jan-05    RMS     Added more RA and RD disks
   31-Oct-04    RMS     Added -L switch (LBNs) to RAUSER size specification
   01-Oct-04    RMS     Revised Unibus interface
                        Changed to identify as UDA50 in Unibus configurations
                        Changed width to be 16b in all configurations
                        Changed default timing for VAX
   24-Jul-04    RMS     VAX controllers luns start with 0 (Andreas Cejna)
   05-Feb-04    RMS     Revised for file I/O library
   25-Jan-04    RMS     Revised for device debug support
   12-Jan-04    RMS     Fixed bug in interrupt control (Tom Evans)
   07-Oct-03    RMS     Fixed problem with multiple RAUSER drives
   17-Sep-03    RMS     Fixed MB to LBN conversion to be more accurate
   11-Jul-03    RMS     Fixed bug in user disk size (Chaskiel M Grundman)
   19-May-03    RMS     Revised for new conditional compilation scheme
   25-Apr-03    RMS     Revised for extended file support
   14-Mar-03    RMS     Fixed variable size interaction with save/restore
   27-Feb-03    RMS     Added user-defined drive support
   26-Feb-03    RMS     Fixed bug in vector calculation for VAXen
   22-Feb-03    RMS     Fixed ordering bug in queue process
   12-Oct-02    RMS     Added multicontroller support
   29-Sep-02    RMS     Changed addressing to 18b in Unibus mode
                        Added variable address support to bootstrap
                        Added vector display support
                        Fixed status code in HBE error log
                        Consolidated MSCP/TMSCP header file
                        New data structures
   16-Aug-02    RMS     Removed unused variables (David Hittner)
   04-May-02    RMS     Fixed bug in polling loop for queued operations
   26-Mar-02    RMS     Fixed bug, reset routine cleared UF_WPH
   09-Mar-02    RMS     Adjusted delays for M+ timing bugs
   04-Mar-02    RMS     Added delays to initialization for M+, RSTS/E
   16-Feb-02    RMS     Fixed bugs in host timeout logic, boot
   26-Jan-02    RMS     Revised bootstrap to conform to M9312
   06-Jan-02    RMS     Revised enable/disable support
   30-Dec-01    RMS     Revised show routines
   19-Dec-01    RMS     Added bigger drives
   17-Dec-01    RMS     Added queue process
*/

#if defined (VM_PDP10)                                  /* PDP10 version */
#error "RQDX3 not supported on PDP-10!"

#elif defined (VM_VAX)                                  /* VAX version */
#include "vax_defs.h"
#define RF_QTIME        100
#define RF_XTIME        200
#define OLDPC           fault_PC
extern int32 fault_PC;

#else                                                   /* PDP-11 version */
#include "pdp11_defs.h"
#define RF_QTIME        200
#define RF_XTIME        500
#define OLDPC           MMR2
extern int32 MMR2;
#endif

#if !defined (RF_NUMCT)
#define RF_NUMCT        1
#elif (RF_NUMCT > 4)
#error "Assertion failure: RF_NUMCT exceeds 4"
#endif

#include "pdp11_uqssp.h"
#include "pdp11_mscp.h"
#include "sim_disk.h"
#include "vax_rf.h"

#define UF_MSK          (UF_CMR|UF_CMW)                 /* settable flags */

#define RF_SH_MAX       24                              /* max display wds */
#define RF_SH_PPL       8                               /* wds per line */
#define RF_SH_DPL       4                               /* desc per line */
#define RF_SH_RI        001                             /* show rings */
#define RF_SH_FR        002                             /* show free q */
#define RF_SH_RS        004                             /* show resp q */
#define RF_SH_UN        010                             /* show unit q's */
#define RF_SH_ALL       017                             /* show all */

#define RF_CLASS        1                               /* RF class */
#define RFU_UQPM        6                               /* UB port model */
#define RFQ_UQPM        19                              /* QB port model */
#define RF_UQPM         (UNIBUS? RFU_UQPM: RFQ_UQPM)
#define RFU_MODEL       6                               /* UB MSCP ctrl model (UDA50A) */
#define RFQ_MODEL       19                              /* QB MSCP ctrl model (RQDX3) */
#define RF_MODEL        (UNIBUS? RFU_MODEL: RFQ_MODEL)
#define RF_HVER         1                               /* hardware version */
#define RF_SVER         3                               /* software version */
#define RF_DHTMO        60                              /* def host timeout */
#define RF_DCTMO        120                             /* def ctrl timeout */
#define RF_NUMDR        4                               /* # drives */
#define RF_NUMBY        512                             /* bytes per block */
#define RF_MAXFR        (1 << 16)                       /* max xfer */
#define RF_MAPXFER      (1u << 31)                      /* mapped xfer */
#define RF_M_PFN        0x1FFFFF                        /* map entry PFN */

#define UNIT_V_ONL      (DKUF_V_UF + 0)                 /* online */
#define UNIT_V_WLK      (DKUF_V_UF + 1)                 /* hwre write lock */
#define UNIT_V_ATP      (DKUF_V_UF + 2)                 /* attn pending */
#define UNIT_V_DTYPE    (DKUF_V_UF + 3)                 /* drive type */
#define UNIT_M_DTYPE    0x1F
#define UNIT_V_NOAUTO   (DKUF_V_UF + 8)                 /* noautosize */
#define UNIT_ONL        (1 << UNIT_V_ONL)
#define UNIT_WLK        (1 << UNIT_V_WLK)
#define UNIT_ATP        (1 << UNIT_V_ATP)
#define UNIT_NOAUTO     (1 << UNIT_V_NOAUTO)
#define UNIT_DTYPE      (UNIT_M_DTYPE << UNIT_V_DTYPE)
#define GET_DTYPE(x)    (((x) >> UNIT_V_DTYPE) & UNIT_M_DTYPE)
#define cpkt            us9                             /* current packet */
#define pktq            us10                            /* packet queue */
#define uf              buf                             /* settable unit flags */
#define cnum            wait                            /* controller index */
#define unit_plug       u4                              /* drive unit plug value */
#define io_status       u5                              /* io status from callback */
#define io_complete     u6                              /* io completion flag */
#define rfxb            filebuf                         /* xfer buffer */
#define UNIT_WPRT       (UNIT_WLK | UNIT_RO)            /* write prot */
#define RF_RMV(u)       ((drv_tab[GET_DTYPE (u->flags)].flgs & RFDF_RMV)? \
                        UF_RMV: 0)
#define RF_WPH(u)       (((drv_tab[GET_DTYPE (u->flags)].flgs & RFDF_RO) || \
                        (u->flags & UNIT_WPRT) || sim_disk_wrp (u))? UF_WPH: 0)

#define CST_S1          0                               /* init stage 1 */
#define CST_S1_WR       1                               /* stage 1 wrap */
#define CST_S2          2                               /* init stage 2 */
#define CST_S3          3                               /* init stage 3 */
#define CST_S3_PPA      4                               /* stage 3 sa wait */
#define CST_S3_PPB      5                               /* stage 3 ip wait */
#define CST_S4          6                               /* stage 4 */
#define CST_UP          7                               /* online */
#define CST_DEAD        8                               /* fatal error */

#define ERR             0                               /* must be SCPE_OK! */
#define OK              1

#define RF_TIMER        (RF_NUMDR)
#define RF_QUEUE        (RF_TIMER + 1)

#define RW_BNL          (RW_BD0L)                       /* buff name */
#define RW_BNH          (RW_BD0H)
#define RW_BOL          (RW_BD1L)                       /* byte offset */
#define RW_BOH          (RW_BD1H)
#define RW_CNL          (RW_BD2L)                       /* conn id */
#define RW_CNH          (RW_BD2H)


/* Internal packet management.  The real RQDX3 manages its packets as true
   linked lists.  However, use of actual addresses in structures won't work
   with save/restore.  Accordingly, the packets are an arrayed structure,
   and links are actually subscripts.  To minimize complexity, packet[0]
   is not used (0 = end of list), and the number of packets must be a power
   of two.
*/

#if 0
#define RF_NPKTS        32                              /* # packets (pwr of 2) */
#define RF_M_NPKTS      (RF_NPKTS - 1)                  /* mask */
#define RF_PKT_SIZE_W   32                              /* payload size (wds) */
#define RF_PKT_SIZE     (RF_PKT_SIZE_W * sizeof (int16))

struct rfpkt {
    uint16      link;                                   /* link to next */
    uint16      d[RF_PKT_SIZE_W];                       /* data */
    };
#endif

/* Packet payload extraction and insertion; cp defines controller */

#define GETP(p,w,f)     ((cp->pak[p].d[w] >> w##_V_##f) & w##_M_##f)
#define GETP32(p,w)     (((uint32) cp->pak[p].d[w]) | \
                        (((uint32) cp->pak[p].d[(w)+1]) << 16))
#define PUTP32(p,w,x)   cp->pak[p].d[w] = (x) & 0xFFFF; \
                        cp->pak[p].d[(w)+1] = ((x) >> 16) & 0xFFFF

/* Disk formats.  An RQDX3 disk consists of the following regions:

   XBNs         Extended blocks - contain information about disk format,
                also holds track being reformatted during bad block repl.
                Size = sectors/track + 1, replicated 3 times.
   DBNs         Diagnostic blocks - used by diagnostics.  Sized to pad
                out the XBNs to a cylinder boundary.
   LBNs         Logical blocks - contain user information.
   RCT          Replacement control table - first block contains status,
                second contains data from block being replaced, remaining
                contain information about replaced bad blocks.
                Size = RBNs/128 + 3, replicated 4-8 times.
   RBNs         Replacement blocks - used to replace bad blocks.

   The simulator does not need to perform bad block replacement; the
   information below is for simulating RCT reads, if required.

   Note that an RA drive has a different order: LBNs, RCT, XBN, DBN;
   the RBNs are spare blocks at the end of every track.
*/

#define RCT_OVHD        2                               /* #ovhd blks */
#define RCT_ENTB        128                             /* entries/blk */
#define RCT_END         0x80000000                      /* marks RCT end */

/* The RQDX3 supports multiple disk drive types (x = not implemented):

   type sec     surf    cyl     tpg     gpc     RCT     LBNs
        
   RX50 10      1       80      5       16      -       800
   RX33 15      2       80      2       1       -       2400
   RD51 18      4       306     4       1       36*4    21600
   RD31 17      4       615     4       1       3*8     41560
   RD52 17      8       512     8       1       4*8     60480
   RD32 17      6       820     6       1       4*8     83204
x  RD33 17      7       1170    ?       ?       ?       138565
   RD53 17      8       1024    8       1       5*8     138672
   RD54 17      15      1225    15      1       7*8     311200

   The simulator also supports larger drives that only existed
   on SDI controllers.

   RA60 42(+1)  6       1600    6       1       1008    400176
x  RA70 33(+1)  11      1507+   11      1       ?       547041
   RA80 31      14      546      ?      ?       ?       237212
   RA81 51(+1)  14      1258    14      1       2856    891072
   RA82 57(+1)  15      1435    15      1       3420    1216665
   RA71 51(+1)  14      1921    14      1       1428    1367310         
   RA72 51(+1)  20      1921    20      1       2040    1953300
   RA90 69(+1)  13      2656    13      1       1794    2376153
   RA92 73(+1)  13      3101    13      1       949     2940951
x  RA73 70(+1)  21      2667+   21      1       ?       3920490

   LESI attached RC25 disks (one removable, one fixed)
   type  sec     surf    cyl     tpg     gpc     RCT     LBNs
   RC25  31      2        821    ?       ?       ?       50902
   RCF25 31      2        821    ?       ?       ?       50902

   Each drive can be a different type.  The drive field in the
   unit flags specified the drive type and thus, indirectly,
   the drive size.
*/

#define RFDF_RMV        01                              /* removable */
#define RFDF_RO         02                              /* read only */
#define RFDF_SDI        04                              /* SDI drive */

#define RX50_DTYPE      0
#define RX50_SECT       10
#define RX50_SURF       1
#define RX50_CYL        80
#define RX50_TPG        5
#define RX50_GPC        16
#define RX50_XBN        0
#define RX50_DBN        0
#define RX50_LBN        800
#define RX50_RCTS       0
#define RX50_RCTC       0
#define RX50_RBN        0
#define RX50_MOD        7
#define RX50_MED        0x25658032
#define RX50_FLGS       RFDF_RMV

#define RX33_DTYPE      1
#define RX33_SECT       15
#define RX33_SURF       2
#define RX33_CYL        80
#define RX33_TPG        2
#define RX33_GPC        1
#define RX33_XBN        0
#define RX33_DBN        0
#define RX33_LBN        2400
#define RX33_RCTS       0
#define RX33_RCTC       0
#define RX33_RBN        0
#define RX33_MOD        10
#define RX33_MED        0x25658021
#define RX33_FLGS       RFDF_RMV

#define RD51_DTYPE      2
#define RD51_SECT       18
#define RD51_SURF       4
#define RD51_CYL        306
#define RD51_TPG        4
#define RD51_GPC        1
#define RD51_XBN        57
#define RD51_DBN        87
#define RD51_LBN        21600
#define RD51_RCTS       36
#define RD51_RCTC       4
#define RD51_RBN        144
#define RD51_MOD        6
#define RD51_MED        0x25644033
#define RD51_FLGS       0

#define RD31_DTYPE      3
#define RD31_SECT       17
#define RD31_SURF       4
#define RD31_CYL        615                             /* last unused */
#define RD31_TPG        RD31_SURF
#define RD31_GPC        1
#define RD31_XBN        54
#define RD31_DBN        14
#define RD31_LBN        41560
#define RD31_RCTS       3
#define RD31_RCTC       8
#define RD31_RBN        100
#define RD31_MOD        12
#define RD31_MED        0x2564401F
#define RD31_FLGS       0

#define RD52_DTYPE      4                               /* Quantum params */
#define RD52_SECT       17
#define RD52_SURF       8
#define RD52_CYL        512
#define RD52_TPG        RD52_SURF
#define RD52_GPC        1
#define RD52_XBN        54
#define RD52_DBN        82
#define RD52_LBN        60480
#define RD52_RCTS       4
#define RD52_RCTC       8
#define RD52_RBN        168
#define RD52_MOD        8
#define RD52_MED        0x25644034
#define RD52_FLGS       0

#define RD53_DTYPE      5
#define RD53_SECT       17
#define RD53_SURF       8
#define RD53_CYL        1024                            /* last unused */
#define RD53_TPG        RD53_SURF
#define RD53_GPC        1
#define RD53_XBN        54
#define RD53_DBN        82
#define RD53_LBN        138672
#define RD53_RCTS       5
#define RD53_RCTC       8
#define RD53_RBN        280
#define RD53_MOD        9
#define RD53_MED        0x25644035
#define RD53_FLGS       0

#define RD54_DTYPE      6
#define RD54_SECT       17
#define RD54_SURF       15
#define RD54_CYL        1225                            /* last unused */
#define RD54_TPG        RD54_SURF
#define RD54_GPC        1
#define RD54_XBN        54
#define RD54_DBN        201
#define RD54_LBN        311200
#define RD54_RCTS       7
#define RD54_RCTC       8
#define RD54_RBN        609
#define RD54_MOD        13
#define RD54_MED        0x25644036
#define RD54_FLGS       0

#define RA82_DTYPE      7                               /* SDI drive */
#define RA82_SECT       57                              /* +1 spare/track */
#define RA82_SURF       15
#define RA82_CYL        1435                            /* 0-1422 user */
#define RA82_TPG        RA82_SURF
#define RA82_GPC        1
#define RA82_XBN        3480                            /* cyl 1427-1430 */
#define RA82_DBN        3480                            /* cyl 1431-1434 */
#define RA82_LBN        1216665                         /* 57*15*1423 */
#define RA82_RCTS       3420                            /* cyl 1423-1426 */
#define RA82_RCTC       1
#define RA82_RBN        21345                           /* 1 *15*1423 */
#define RA82_MOD        11
#define RA82_MED        0x25641052
#define RA82_FLGS       RFDF_SDI

#define RRD40_DTYPE     8
#define RRD40_SECT      128
#define RRD40_SURF      1
#define RRD40_CYL       10400
#define RRD40_TPG       RRD40_SURF
#define RRD40_GPC       1
#define RRD40_XBN       0
#define RRD40_DBN       0
#define RRD40_LBN       1331200
#define RRD40_RCTS      0
#define RRD40_RCTC      0
#define RRD40_RBN       0
#define RRD40_MOD       26
#define RRD40_MED       0x25652228
#define RRD40_FLGS      (RFDF_RMV | RFDF_RO)

#define RA72_DTYPE      9                               /* SDI drive */
#define RA72_SECT       51                              /* +1 spare/trk */
#define RA72_SURF       20
#define RA72_CYL        1921                            /* 0-1914 user */
#define RA72_TPG        RA72_SURF
#define RA72_GPC        1
#define RA72_XBN        2080                            /* cyl 1917-1918? */
#define RA72_DBN        2080                            /* cyl 1920-1921? */
#define RA72_LBN        1953300                         /* 51*20*1915 */
#define RA72_RCTS       2040                            /* cyl 1915-1916? */
#define RA72_RCTC       1
#define RA72_RBN        38300                           /* 1 *20*1915 */
#define RA72_MOD        37
#define RA72_MED        0x25641048
#define RA72_FLGS       RFDF_SDI

#define RA90_DTYPE      10                              /* SDI drive */
#define RA90_SECT       69                              /* +1 spare/trk */
#define RA90_SURF       13
#define RA90_CYL        2656                            /* 0-2648 user */
#define RA90_TPG        RA90_SURF
#define RA90_GPC        1
#define RA90_XBN        1820                            /* cyl 2651-2652? */
#define RA90_DBN        1820                            /* cyl 2653-2654? */
#define RA90_LBN        2376153                         /* 69*13*2649 */
#define RA90_RCTS       1794                            /* cyl 2649-2650? */
#define RA90_RCTC       1
#define RA90_RBN        34437                           /* 1 *13*2649 */
#define RA90_MOD        19
#define RA90_MED        0x2564105A
#define RA90_FLGS       RFDF_SDI

#define RA92_DTYPE      11                              /* SDI drive */
#define RA92_SECT       73                              /* +1 spare/trk */
#define RA92_SURF       13
#define RA92_CYL        3101                            /* 0-3098 user */
#define RA92_TPG        RA92_SURF
#define RA92_GPC        1
#define RA92_XBN        174                             /* cyl 3100? */
#define RA92_DBN        788
#define RA92_LBN        2940951                         /* 73*13*3099 */
#define RA92_RCTS       949                             /* cyl 3099? */
#define RA92_RCTC       1
#define RA92_RBN        40287                           /* 1 *13*3099 */
#define RA92_MOD        29
#define RA92_MED        0x2564105C
#define RA92_FLGS       RFDF_SDI

#define RA8U_DTYPE      12                              /* user defined */
#define RA8U_SECT       57                              /* from RA82 */
#define RA8U_SURF       15
#define RA8U_CYL        1435                            /* from RA82 */
#define RA8U_TPG        RA8U_SURF
#define RA8U_GPC        1
#define RA8U_XBN        0
#define RA8U_DBN        0
#define RA8U_LBN        1216665                         /* from RA82 */
#define RA8U_RCTS       400
#define RA8U_RCTC       8
#define RA8U_RBN        21345
#define RA8U_MOD        11                              /* RA82 */
#define RA8U_MED        0x25641052                      /* RA82 */
#define RA8U_FLGS       RFDF_SDI
#define RA8U_MINC       10000                           /* min cap LBNs */
#define RA8U_MAXC       4194303                         /* max cap LBNs */
#define RA8U_EMAXC      2147483647                      /* ext max cap */

#define RA60_DTYPE      13                              /* SDI drive */
#define RA60_SECT       42                              /* +1 spare/track */
#define RA60_SURF       6
#define RA60_CYL        1600                            /* 0-1587 user */
#define RA60_TPG        RA60_SURF
#define RA60_GPC        1
#define RA60_XBN        1032                            /* cyl 1592-1595 */
#define RA60_DBN        1032                            /* cyl 1596-1599 */
#define RA60_LBN        400176                          /* 42*6*1588 */
#define RA60_RCTS       1008                            /* cyl 1588-1591 */
#define RA60_RCTC       1
#define RA60_RBN        9528                            /* 1 *6*1588 */
#define RA60_MOD        4
#define RA60_MED        0x22A4103C
#define RA60_FLGS       (RFDF_RMV | RFDF_SDI)

#define RA81_DTYPE      14                              /* SDI drive */
#define RA81_SECT       51                              /* +1 spare/track */
#define RA81_SURF       14
#define RA81_CYL        1258                            /* 0-1247 user */
#define RA81_TPG        RA81_SURF
#define RA81_GPC        1
#define RA81_XBN        2436                            /* cyl 1252-1254? */
#define RA81_DBN        2436                            /* cyl 1255-1256? */
#define RA81_LBN        891072                          /* 51*14*1248 */
#define RA81_RCTS       2856                            /* cyl 1248-1251? */
#define RA81_RCTC       1
#define RA81_RBN        17472                           /* 1 *14*1248 */
#define RA81_MOD        5
#define RA81_MED        0x25641051
#define RA81_FLGS       RFDF_SDI

#define RA71_DTYPE      15                              /* SDI drive */
#define RA71_SECT       51                              /* +1 spare/track */
#define RA71_SURF       14
#define RA71_CYL        1921                            /* 0-1914 user */
#define RA71_TPG        RA71_SURF
#define RA71_GPC        1
#define RA71_XBN        1456                            /* cyl 1917-1918? */
#define RA71_DBN        1456                            /* cyl 1919-1920? */
#define RA71_LBN        1367310                         /* 51*14*1915 */
#define RA71_RCTS       1428                            /* cyl 1915-1916? */
#define RA71_RCTC       1
#define RA71_RBN        26810                           /* 1 *14*1915 */
#define RA71_MOD        40
#define RA71_MED        0x25641047
#define RA71_FLGS       RFDF_SDI

#define RD32_DTYPE      16
#define RD32_SECT       17
#define RD32_SURF       6
#define RD32_CYL        820
#define RD32_TPG        RD32_SURF
#define RD32_GPC        1
#define RD32_XBN        54
#define RD32_DBN        48
#define RD32_LBN        83236
#define RD32_RCTS       4
#define RD32_RCTC       8
#define RD32_RBN        200
#define RD32_MOD        15
#define RD32_MED        0x25644020
#define RD32_FLGS       0

#define RC25_DTYPE      17                              /*  */
#define RC25_SECT       50                              /*  */
#define RC25_SURF       8
#define RC25_CYL        1260                            /*  */
#define RC25_TPG        RC25_SURF
#define RC25_GPC        1
#define RC25_XBN        0                               /*  */
#define RC25_DBN        0                               /*  */
#define RC25_LBN        50902                           /* ? 50*8*1260 ? */
#define RC25_RCTS       0                               /*  */
#define RC25_RCTC       1
#define RC25_RBN        0                               /*  */
#define RC25_MOD        3
#define RC25_MED        0x20643019
#define RC25_FLGS       RFDF_RMV

#define RCF25_DTYPE     18                              /*  */
#define RCF25_SECT      50                              /*  */
#define RCF25_SURF      8
#define RCF25_CYL       1260                            /*  */
#define RCF25_TPG       RCF25_SURF
#define RCF25_GPC       1
#define RCF25_XBN       0                               /*  */
#define RCF25_DBN       0                               /*  */
#define RCF25_LBN       50902                           /* ? 50*8*1260 ? */
#define RCF25_RCTS      0                               /*  */
#define RCF25_RCTC      1
#define RCF25_RBN       0                               /*  */
#define RCF25_MOD       3
#define RCF25_MED       0x20643319
#define RCF25_FLGS      0

#define RA80_DTYPE      19                              /* SDI drive */
#define RA80_SECT       31                              /* +1 spare/track */
#define RA80_SURF       14
#define RA80_CYL        546                             /*  */
#define RA80_TPG        RA80_SURF
#define RA80_GPC        1
#define RA80_XBN        0                               /*  */
#define RA80_DBN        0                               /*  */
#define RA80_LBN        237212                          /* 31*14*546 */
#define RA80_RCTS       0                               /*  */
#define RA80_RCTC       1
#define RA80_RBN        0                               /*  */
#define RA80_MOD        1
#define RA80_MED        0x25641050
#define RA80_FLGS       RFDF_SDI

// [RLA]   Most of these RA70 parameters came from doing a DUSTAT on a real
// [RLA] RA70 drive.  The remainder are just educated guesses...
#define RA70_DTYPE      20              /* SDI drive */
#define RA70_SECT       33              /* +1 spare/track */
#define RA70_SURF       11              /* tracks/cylinder */
#define RA70_CYL        1507            /* 0-1506 user */
#define RA70_TPG        RA70_SURF
#define RA70_GPC        1
#define RA70_XBN        0               /* ??? */
#define RA70_DBN        0               /* ??? */
#define RA70_LBN        547041          /* 33*11*1507 */
#define RA70_RCTS       198             /* Size of the RCT */
#define RA70_RCTC       7               /* Number of RCT copies */
#define RA70_RBN        16577           /* 1*11*1507 */
#define RA70_MOD        0               /* ??? */
#define RA70_MED        0x25641046      /* RA70 MEDIA ID */
#define RA70_FLGS       RFDF_SDI

// [RLA] Likewise for the RA73 ...
#define RA73_DTYPE      21              /* SDI drive */
#define RA73_SECT       70              /* +1 spare/track */
#define RA73_SURF       21              /* tracks/cylinder */
#define RA73_CYL        2667            /* 0-2666 user */
#define RA73_TPG        RA73_SURF
#define RA73_GPC        1
#define RA73_XBN        0               /* ??? */
#define RA73_DBN        0               /* ??? */
#define RA73_LBN        3920490         /* 70*21*2667 */
#define RA73_RCTS       198             /* Size of the RCT ??????*/
#define RA73_RCTC       7               /* Number of RCT copies */
#define RA73_RBN        56007           /* 1*21*2667 */
#define RA73_MOD        0               /* ??? */
#define RA73_MED        0x25641049      /* RA73 MEDIA ID */
#define RA73_FLGS       RFDF_SDI

/* Controller parameters */

#define DEFAULT_CTYPE   0

// AFAIK the UNIBUS KLESI and QBUS KLESI used the same controller type ...
#define KLESI_CTYPE     1               // RC25 controller (UNIBUS and QBUS both)
#define KLESI_UQPM      1
#define KLESI_MODEL     1

#define RUX50_CTYPE     2               // UNIBUS RX50-only controller
#define RUX50_UQPM      2
#define RUX50_MODEL     2

#define UDA50_CTYPE     3               // UNIBUS SDI (RAxx) controller
#define UDA50_UQPM      6
#define UDA50_MODEL     6

#define RQDX3_CTYPE     4               // QBUS RX50/RDxx controller
#define RQDX3_UQPM      19
#define RQDX3_MODEL     19

#define KDA50_CTYPE     5               // QBUS SDI (RAxx) controller
#define KDA50_UQPM      13
#define KDA50_MODEL     13

#define KRQ50_CTYPE     6               // QBUS RRD40/50 CDROM controller
#define KRQ50_UQPM      16
#define KRQ50_MODEL     16

#define KRU50_CTYPE     7               // UNIBUS RRD40/50 CDROM controller
#define KRU50_UQPM      26
#define KRU50_MODEL     26

struct drvtyp {
    uint16      sect;                                   /* sectors */
    int32       surf;                                   /* surfaces */
    int32       cyl;                                    /* cylinders */
    uint16      tpg;                                    /* trk/grp */
    uint16      gpc;                                    /* grp/cyl */
    int32       xbn;                                    /* XBN size */
    int32       dbn;                                    /* DBN size */
    uint32      lbn;                                    /* LBN size */
    uint16      rcts;                                   /* RCT size */
    int32       rctc;                                   /* RCT copies */
    int32       rbn;                                    /* RBNs */
    uint16      mod;                                    /* MSCP model */
    int32       MediaId;                                /* MSCP media */
    int32       flgs;                                   /* flags */
    const char  *name;                                  /* name */
    };

/*

MediaId

Is defined in the MSCP Basic Disk Functions Manual, page 4-37 to 4-38:

The media type identifier is a 32-bit number, and it's coded like this:
The high 25 bits are 5 characters, each coded with 5 bits. The low 7 
bits is a binary coded 2 digits.

Looking at it, you have:
D0,D1,A0,A1,A2,N

For an RA81, it would be:

D0,D1 is the preferred device type name for the unit. In our case, 
that would be "DU".
A0,A1,A2 is the name of the media used on the unit. In our case "RA".
N is the value of the two decimal digits, so 81 for this example.

And for letters, the coding is that A=1, B=2 and so on. 0 means the 
character is not used.

So, again, for an RA81, we would get:

Decimal Values:        4,    21,    18,     1,     0,      81
Hex Values:            4,    15,    12,     1,     0,      51
Binary Values:     00100, 10101, 10010, 00001, 00000, 1010001
Hex 4 bit Nibbles:    2     5     6     4   1     0     5   1

The 32bit value of RA81_MED is 0x25641051

 */

#define RF_DRV(d) \
  { d##_SECT, d##_SURF, d##_CYL,  d##_TPG, \
    d##_GPC,  d##_XBN,  d##_DBN,  d##_LBN, \
    d##_RCTS, d##_RCTC, d##_RBN,  d##_MOD, \
    d##_MED, d##_FLGS, #d }
#define RF_SIZE(d)      d##_LBN

static struct drvtyp drv_tab[] = {
    RF_DRV (RX50),
    RF_DRV (RX33),
    RF_DRV (RD51),
    RF_DRV (RD31),
    RF_DRV (RD52),
    RF_DRV (RD53),
    RF_DRV (RD54),
    RF_DRV (RA82),
    RF_DRV (RRD40),
    RF_DRV (RA72),
    RF_DRV (RA90),
    RF_DRV (RA92),
    RF_DRV (RA8U),
    RF_DRV (RA60),
    RF_DRV (RA81),
    RF_DRV (RA71),
    RF_DRV (RD32),
    RF_DRV (RC25),
    RF_DRV (RCF25),
    RF_DRV (RA80),
    RF_DRV (RA70),
    RF_DRV (RA73),
    { 0 }
    };

struct ctlrtyp {
    uint32      uqpm;                                   /* port model */
    uint16      model;                                  /* controller model */
    const char  *name;                                  /* name */
    };

#define RF_CTLR(d) \
    { d##_UQPM, d##_MODEL, #d }

static struct ctlrtyp ctlr_tab[] = {
    { 0, 0, "DEFAULT" },
    RF_CTLR (KLESI),
    RF_CTLR (RUX50),
    RF_CTLR (UDA50),
    RF_CTLR (RQDX3),
    RF_CTLR (KDA50),
    RF_CTLR (KRQ50),
    RF_CTLR (KRU50),
    { 0 }
    };

int32 rf_itime = 450;                                   /* init time, except */
int32 rf_itime4 = 10;                                   /* stage 4 */
int32 rf_qtime = RF_QTIME;                              /* queue time */
int32 rf_xtime = RF_XTIME;                              /* transfer time */

#if NEW
typedef struct {
    uint32              cnum;                           /* ctrl number - ctlr */
    uint16              max_plug;                       /* highest unit plug number - ctlr */
    } CTLR;

typedef struct {
    uint32              sa;                             /* status, addr - uq */
    uint32              saw;                            /* written data - uq */
    uint32              s1dat;                          /* S1 data - uq */
    uint32              comm;                           /* comm region - uq */
    uint16              perr;                           /* last error */
    uint32              irq;                            /* intr request - uq */
    uint32              prgi;                           /* purge int - uq */
    uint32              pip;                            /* poll in progress - uq */
    uint16              freq;                           /* free list - uq */
    uint16              rspq;                           /* resp list - uq */
    uint32              credits;                        /* credits - uq */
    uint32              ctype;                          /* controller type - uq/hsc */
    struct uq_ring      cq;                             /* cmd ring - uq */
    struct uq_ring      rq;                             /* rsp ring - uq */
    } UQSSP;

typedef struct {
    void                *ctx;                           /* UQSSP or HSC */
    CTLR                *ctl;                           /* ctrl params */
    uint32              csta;                           /* ctrl state - conn? */
    uint16              cflgs;                          /* ctrl flags - conn */
    uint32              pbsy;                           /* #busy pkts - conn? */
    uint32              hat;                            /* host timer - conn? */
    uint32              htmo;                           /* host timeout - conn? */
    struct rfpkt        pak[RF_NPKTS];                  /* packet queue - conn */
    } MSC;
#endif

#if 0
typedef struct {
    // connection id
    // or pointer to HSC/UQSSP context?
    uint32              cnum;                           /* ctrl number - ctlr */
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
#endif

/* debugging bitmaps */
#define DBG_TRC  0x0001                                 /* trace routine calls */
#define DBG_INI  0x0002                                 /* display setup/init sequence info */
#define DBG_REG  0x0004                                 /* trace read/write registers */
#define DBG_REQ  0x0008                                 /* display transfer requests */
#define DBG_DSK  0x0010                                 /* display sim_disk activities */
#define DBG_DAT  0x0020                                 /* display transfer data */

DEBTAB rf_debug[] = {
  {"TRACE",  DBG_TRC, "trace routine calls"},
  {"INIT",   DBG_INI, "display setup/init sequence info"},
  {"REG",    DBG_REG, "trace read/write registers"},
  {"REQ",    DBG_REQ, "display transfer requests"},
  {"DISK",   DBG_DSK, "display sim_disk activities"},
  {"DATA",   DBG_DAT, "display transfer data"},
  {0}
};

static const char *rf_cmdname[] = {
    "",                                                 /*  0 */
    "ABO",                                              /*  1 b: abort */
    "GCS",                                              /*  2 b: get command status */
    "GUS",                                              /*  3 b: get unit status */
    "SCC",                                              /*  4 b: set controller char */
    "","","",                                           /*  5-7 */
    "AVL",                                              /*  8 b: available */
    "ONL",                                              /*  9 b: online */
    "SUC",                                              /* 10 b: set unit char */
    "DAP",                                              /* 11 b: det acc paths - nop */
    "","","","",                                        /* 12-15 */
    "ACC",                                              /* 16 b: access */
    "CCD",                                              /* 17 d: compare - nop */
    "ERS",                                              /* 18 b: erase */
    "FLU",                                              /* 19 d: flush - nop */
    "","",                                              /* 20-21 */
    "ERG",                                              /* 22 t: erase gap */
    "","","","","","","","","",                         /* 23-31 */
    "CMP",                                              /* 32 b: compare */
    "RD",                                               /* 33 b: read */
    "WR",                                               /* 34 b: write */
    "",                                                 /* 35 */
    "WTM",                                              /* 36 t: write tape mark */
    "POS",                                              /* 37 t: reposition */
    "","","","","","","","","",                         /* 38-46 */
    "FMT",                                              /* 47 d: format */
    "","","","","","","","","","","","","","","","",    /* 48-63 */
    "AVA",                                              /* 64 b: unit now avail */
    };

t_stat rf_svc (UNIT *uptr);
t_stat rf_tmrsvc (UNIT *uptr);
t_stat rf_quesvc (UNIT *uptr);
t_stat rf_reset (DEVICE *dptr);
t_stat rf_attach (UNIT *uptr, CONST char *cptr);
t_stat rf_detach (UNIT *uptr);
t_stat rf_set_wlk (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat rf_set_type (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat rf_set_ctype (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat rf_set_plug (UNIT *uptr, int32 val, CONST char *cptr, void *desc);
t_stat rf_show_plug (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat rf_show_type (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat rf_show_ctype (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat rf_show_wlk (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat rf_show_unitq (FILE *st, UNIT *uptr, int32 val, CONST void *desc);
t_stat rf_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr);
const char *rf_description (DEVICE *dptr);

t_bool rf_step4 (MSC *cp);
t_bool rf_mscp (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_abo (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_avl (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_fmt (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_gcs (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_gus (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_onl (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_rw (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_scc (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_suc (MSC *cp, uint16 pkt, t_bool q);
t_bool rf_plf (MSC *cp, uint16 err);
t_bool rf_dte (MSC *cp, UNIT *uptr, uint16 err);
t_bool rf_hbe (MSC *cp, UNIT *uptr);
t_bool rf_una (MSC *cp, uint16 un);
t_bool rf_deqf (MSC *cp, uint16 *pkt);
uint16 rf_deqh (MSC *cp, uint16 *lh);
void rf_enqh (MSC *cp, uint16 *lh, uint16 pkt);
void rf_enqt (MSC *cp, uint16 *lh, uint16 pkt);
t_bool rf_getpkt (MSC *cp, uint16 *pkt);
t_bool rf_putpkt (MSC *cp, uint16 pkt, t_bool qt);
t_bool rf_getdesc (MSC *cp, struct uq_ring *ring, uint32 *desc);
t_bool rf_putdesc (MSC *cp, struct uq_ring *ring, uint32 desc);
uint16 rf_rw_valid (MSC *cp, uint16 pkt, UNIT *uptr, uint16 cmd);
t_bool rf_rw_end (MSC *cp, UNIT *uptr, uint16 flg, uint16 sts);
void rf_putr (MSC *cp, uint16 pkt, uint16 cmd, uint16 flg,
    uint16 sts, uint16 lnt, uint16 typ);
void rf_putr_unit (MSC *cp, uint16 pkt, UNIT *uptr, uint16 lu, t_bool all);
void rf_setf_unit (MSC *cp, uint16 pkt, UNIT *uptr);
void rf_init_int (MSC *cp);
void rf_ring_int (MSC *cp, struct uq_ring *ring);
t_bool rf_fatal (MSC *cp, uint16 err);
UNIT *rf_getucb (MSC *cp, uint16 lu);

/* RF data structures

   rf_dev       RF device descriptor
   rf_unit      RF unit list
   rf_reg       RF register list
   rf_mod       RF modifier list
*/

MSC rf_ctx = { 0 };

UNIT rf_unit[] = {
    { UDATA (&rf_svc, UNIT_FIX+UNIT_ATTABLE+UNIT_DISABLE+UNIT_ROABLE+
            (RD54_DTYPE << UNIT_V_DTYPE), RF_SIZE (RD54)) },
    { UDATA (&rf_svc, UNIT_FIX+UNIT_ATTABLE+UNIT_DISABLE+UNIT_ROABLE+
            (RD54_DTYPE << UNIT_V_DTYPE), RF_SIZE (RD54)) },
    { UDATA (&rf_svc, UNIT_FIX+UNIT_ATTABLE+UNIT_DISABLE+UNIT_ROABLE+
            (RD54_DTYPE << UNIT_V_DTYPE), RF_SIZE (RD54)) },
    { UDATA (&rf_svc, UNIT_FIX+UNIT_ATTABLE+UNIT_DISABLE+UNIT_ROABLE+
            (RX50_DTYPE << UNIT_V_DTYPE), RF_SIZE (RX50)) },
    { UDATA (&rf_tmrsvc, UNIT_IDLE|UNIT_DIS, 0) },
    { UDATA (&rf_quesvc, UNIT_DIS, 0) }
    };

REG rf_reg[] = {
    { GRDATAD (SA,      rf_ctx.sa,      DEV_RDX, 16, 0, "status/address register") },
    { GRDATAD (SAW,     rf_ctx.saw,     DEV_RDX, 16, 0, "written data") },
    { GRDATAD (S1DAT,   rf_ctx.s1dat,   DEV_RDX, 16, 0, "step 1 init host data") },
    { GRDATAD (COMM,    rf_ctx.comm,    DEV_RDX, 22, 0, "comm region") },
    { GRDATAD (CQIOFF,  rf_ctx.cq.ioff, DEV_RDX, 32, 0, "command queue intr offset") },
    { GRDATAD (CQBA,    rf_ctx.cq.ba,   DEV_RDX, 22, 0, "command queue base address") },
    { GRDATAD (CQLNT,   rf_ctx.cq.lnt,  DEV_RDX, 32, 2, "command queue length"), REG_NZ },
    { GRDATAD (CQIDX,   rf_ctx.cq.idx,  DEV_RDX,  8, 2, "command queue index") },
    { GRDATAD (RFIOFF,  rf_ctx.rq.ioff, DEV_RDX, 32, 0, "request queue intr offset") },
    { GRDATAD (RFBA,    rf_ctx.rq.ba,   DEV_RDX, 22, 0, "request queue base address") },
    { GRDATAD (RFLNT,   rf_ctx.rq.lnt,  DEV_RDX, 32, 2, "request queue length"), REG_NZ },
    { GRDATAD (RFIDX,   rf_ctx.rq.idx,  DEV_RDX,  8, 2, "request queue index") },
    { DRDATAD (FREE,    rf_ctx.freq,                 5, "head of free packet list") },
    { DRDATAD (RESP,    rf_ctx.rspq,                 5, "head of response packet list") },
    { DRDATAD (PBSY,    rf_ctx.pbsy,                 5, "number of busy packets") },
    { GRDATAD (CFLGS,   rf_ctx.cflgs,   DEV_RDX, 16, 0, "controller flags") },
    { GRDATAD (CSTA,    rf_ctx.csta,    DEV_RDX,  4, 0, "controller state") },
    { GRDATAD (PERR,    rf_ctx.perr,    DEV_RDX,  9, 0, "port error number") },
    { DRDATAD (CRED,    rf_ctx.credits,              5, "host credits") },
    { DRDATAD (HAT,     rf_ctx.hat,                 17, "host available timer") },
    { DRDATAD (HTMO,    rf_ctx.htmo,                17, "host timeout value") },
    { FLDATA  (PRGI,    rf_ctx.prgi,                 0), REG_HIDDEN },
    { FLDATA  (PIP,     rf_ctx.pip,                  0), REG_HIDDEN },
    { FLDATA  (CTYPE,   rf_ctx.ctype,               32), REG_HIDDEN  },
    { DRDATAD (ITIME,   rf_itime,                   24, "init time delay, except stage 4"), PV_LEFT + REG_NZ },
    { DRDATAD (I4TIME,  rf_itime4,                  24, "init stage 4 delay"), PV_LEFT + REG_NZ },
    { DRDATAD (QTIME,   rf_qtime,                   24, "response time for 'immediate' packets"), PV_LEFT + REG_NZ },
    { DRDATAD (XTIME,   rf_xtime,                   24, "response time for data transfers"), PV_LEFT + REG_NZ },
    { BRDATAD (PKTS,    rf_ctx.pak,     DEV_RDX,    16, sizeof(rf_ctx.pak)/2, "packet buffers, 33W each, 32 entries") },
    { URDATAD (CPKT,    rf_unit[0].cpkt, 10, 5, 0, RF_NUMDR, 0, "current packet, units 0 to 3") },
    { URDATAD (UCNUM,   rf_unit[0].cnum, 10, 5, 0, RF_NUMDR, 0, "ctrl number, units 0 to 3") },
    { URDATAD (PKTQ,    rf_unit[0].pktq, 10, 5, 0, RF_NUMDR, 0, "packet queue, units 0 to 3") },
    { URDATAD (UFLG,    rf_unit[0].uf,  DEV_RDX, 16, 0, RF_NUMDR, 0, "unit flags, units 0 to 3") },
    { URDATA  (CAPAC,   rf_unit[0].capac, 10, T_ADDR_W, 0, RF_NUMDR, PV_LEFT | REG_HRO) },
    { URDATAD (PLUG,    rf_unit[0].unit_plug, 10, T_ADDR_W, 0, RF_NUMDR, PV_LEFT | REG_RO, "unit plug value, units 0 to 3") },
    { DRDATA  (DEVLBN,  drv_tab[RA8U_DTYPE].lbn, 22), REG_HRO },
    { NULL }
    };

MTAB rf_mod[] = {
    { UNIT_WLK,                 0,  NULL, "WRITEENABLED", 
        &rf_set_wlk, NULL, NULL, "Write enable disk drive" },
    { UNIT_WLK,          UNIT_WLK,  NULL, "LOCKED", 
        &rf_set_wlk, NULL, NULL, "Write lock disk drive"  },
    { MTAB_XTD|MTAB_VUN, 0, "WRITE", NULL,
      NULL, &rf_show_wlk, NULL,  "Display drive writelock status" },
    { MTAB_XTD|MTAB_VDV, RQDX3_CTYPE, NULL, "RQDX3",
      &rf_set_ctype, NULL, NULL, "Set RQDX3 (QBUS RX50/RDnn) Controller Type" },
    { MTAB_XTD|MTAB_VDV, UDA50_CTYPE, NULL, "UDA50",
      &rf_set_ctype, NULL, NULL, "Set UDA50 (UNIBUS SDI RAnn) Controller Type" },
    { MTAB_XTD|MTAB_VDV, KDA50_CTYPE, NULL, "KDA50",
      &rf_set_ctype, NULL, NULL, "Set KDA50 (QBUS SDI RAnn) Controller Type" },
    { MTAB_XTD|MTAB_VDV, KRQ50_CTYPE, NULL, "KRQ50",
      &rf_set_ctype, NULL, NULL, "Set KRQ50 (QBUS CDROM) Controller Type" },
    { MTAB_XTD|MTAB_VDV, KRU50_CTYPE, NULL, "KRU50",
      &rf_set_ctype, NULL, NULL, "Set KRU50 (UNIBUS CDROM) Controller Type" },
    { MTAB_XTD|MTAB_VDV, KLESI_CTYPE, NULL, "KLESI",
      &rf_set_ctype, NULL, NULL, "Set KLESI (RC25) Controller Type"  },
    { MTAB_XTD|MTAB_VDV, RUX50_CTYPE, NULL, "RUX50",
      &rf_set_ctype, NULL, NULL, "Set RUX50 (UNIBUS RX50) Controller Type" },
    { MTAB_XTD|MTAB_VUN|MTAB_NMO, 0, "UNITQ", NULL,
      NULL, &rf_show_unitq, NULL, "Display unit queue" },
    { MTAB_XTD|MTAB_VUN, RX50_DTYPE, NULL, "RX50",
      &rf_set_type, NULL, NULL, "Set RX50 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RX33_DTYPE, NULL, "RX33",
      &rf_set_type, NULL, NULL, "Set RX33 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RD31_DTYPE, NULL, "RD31",
      &rf_set_type, NULL, NULL, "Set RD31 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RD32_DTYPE, NULL, "RD32",
      &rf_set_type, NULL, NULL, "Set RD32 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RD51_DTYPE, NULL, "RD51",
      &rf_set_type, NULL, NULL, "Set RD51 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RD52_DTYPE, NULL, "RD52",
      &rf_set_type, NULL, NULL, "Set RD52 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RD53_DTYPE, NULL, "RD53",
      &rf_set_type, NULL, NULL, "Set RD53 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RD54_DTYPE, NULL, "RD54",
      &rf_set_type, NULL, NULL, "Set RD54 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA60_DTYPE, NULL, "RA60",
      &rf_set_type, NULL, NULL, "Set RA60 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA81_DTYPE, NULL, "RA81",
      &rf_set_type, NULL, NULL, "Set RA81 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA82_DTYPE, NULL, "RA82",
      &rf_set_type, NULL, NULL, "Set RA82 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RRD40_DTYPE, NULL, "RRD40",
      &rf_set_type, NULL, NULL, "Set RRD40 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RRD40_DTYPE, NULL, "CDROM",
      &rf_set_type, NULL, NULL, "Set CDROM Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA70_DTYPE, NULL, "RA70",
      &rf_set_type, NULL, NULL, "Set RA70 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA71_DTYPE, NULL, "RA71",
      &rf_set_type, NULL, NULL, "Set RA71 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA72_DTYPE, NULL, "RA72",
      &rf_set_type, NULL, NULL, "Set RA72 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA73_DTYPE, NULL, "RA73",
      &rf_set_type, NULL, NULL, "Set RA73 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA90_DTYPE, NULL, "RA90",
      &rf_set_type, NULL, NULL, "Set RA90 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA92_DTYPE, NULL, "RA92",
      &rf_set_type, NULL, NULL, "Set RA92 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RC25_DTYPE, NULL, "RC25",
      &rf_set_type, NULL, NULL, "Set RC25 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RCF25_DTYPE, NULL, "RCF25",
      &rf_set_type, NULL, NULL, "Set RCF25 Disk Type" },
    { MTAB_XTD|MTAB_VUN, RA80_DTYPE, NULL, "RA80",
      &rf_set_type, NULL, NULL, "Set RA80 Disk Type" },
    { MTAB_XTD|MTAB_VUN|MTAB_VALR, RA8U_DTYPE, NULL, "RAUSER=SizeInMB",
      &rf_set_type, NULL, NULL, "Set RAUSER Disk Type and its size" },
    { MTAB_XTD|MTAB_VUN, 0, "TYPE", NULL,
      NULL, &rf_show_type, NULL, "Display device type" },
    { MTAB_XTD|MTAB_VUN|MTAB_VALR, 0, "UNIT", "UNIT=val (0-65534)",
      &rf_set_plug, &rf_show_plug, NULL, "Set/Display Unit plug value" },
    { UNIT_NOAUTO, UNIT_NOAUTO, "noautosize", "NOAUTOSIZE", NULL, NULL, NULL, "Disables disk autosize on attach" },
    { UNIT_NOAUTO,           0, "autosize",   "AUTOSIZE",   NULL, NULL, NULL, "Enables disk autosize on attach" },
    { MTAB_XTD|MTAB_VUN|MTAB_VALR, 0, "FORMAT", "FORMAT={SIMH|VHD|RAW}",
      &sim_disk_set_fmt, &sim_disk_show_fmt, NULL, "Set/Display disk format" },
    { MTAB_XTD | MTAB_VDV, 0, "TYPE", NULL,
      NULL, &rf_show_ctype, NULL, "Display controller type" },
    { 0 }
    };

DEVICE rf_dev = {
    "RF", rf_unit, rf_reg, rf_mod,
    RF_NUMDR + 2, DEV_RDX, T_ADDR_W, 2, DEV_RDX, 16,
    NULL, NULL, &rf_reset,
    NULL, &rf_attach, &rf_detach,
    NULL, DEV_DISABLE | DEV_DEBUG | DEV_DISK | DEV_SECTORS | DEV_CI,
    0, rf_debug, NULL, NULL, &rf_help, NULL, NULL,
    &rf_description
    };

static DEVICE *rf_devmap[RF_NUMCT] = {
    &rf_dev
    };

static MSC *rf_ctxmap[RF_NUMCT] = {
    &rf_ctx
    };

/* Transition to step 4 - init communications region */

#if 0
t_bool rf_step4 (MSC *cp)
{
int32 i, lnt;
uint32 base;
uint16 zero[SA_COMM_MAX >> 1];

cp->rq.ioff = SA_COMM_RI;                               /* set intr offset */
cp->rq.ba = cp->comm;                                   /* set rsp q base */
cp->rq.lnt = SA_S1H_RQ (cp->s1dat) << 2;                /* get resp q len */
cp->cq.ioff = SA_COMM_CI;                               /* set intr offset */
cp->cq.ba = cp->comm + cp->rq.lnt;                      /* set cmd q base */
cp->cq.lnt = SA_S1H_CQ (cp->s1dat) << 2;                /* get cmd q len */
cp->cq.idx = cp->rq.idx = 0;                            /* clear q idx's */
if (cp->prgi)
    base = cp->comm + SA_COMM_QQ;
else base = cp->comm + SA_COMM_CI;
lnt = cp->comm + cp->cq.lnt + cp->rq.lnt - base;        /* comm lnt */
if (lnt > SA_COMM_MAX)                                  /* paranoia */
    lnt = SA_COMM_MAX;
for (i = 0; i < (lnt >> 1); i++)                        /* clr buffer */
    zero[i] = 0;
if (Map_WriteW (base, lnt, zero))                       /* zero comm area */
    return rf_fatal (cp, PE_QWE);                       /* error? */
cp->sa = SA_S4 |                                        /* send step 4 */
    (ctlr_tab[cp->ctype].uqpm << SA_S4C_V_MOD) |
    (RF_SVER << SA_S4C_V_VER);
cp->csta = CST_S4;                                      /* set step 4 */
rf_init_int (cp);                                       /* poke host */
return OK;
}
#endif

/* Queue service - invoked when any of the queues (host queue, unit
   queues, response queue) require servicing.  Also invoked during
   initialization to provide some delay to the next step.

   Process at most one item off each unit queue
   If the unit queues were empty, process at most one item off the host queue
   Process at most one item off the response queue

   If all queues are idle, terminate thread
*/

t_stat rf_quesvc (UNIT *uptr)
{
int32 i, cnid;
uint16 pkt = 0;
UNIT *nuptr;
MSC *cp = rf_ctxmap[uptr->cnum];
DEVICE *dptr = rf_devmap[uptr->cnum];
DIB *dibp = (DIB *) dptr->ctxt;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_quesvc\n");

#if 0
if (cp->csta < CST_UP) {                                /* still init? */

    sim_debug(DBG_INI, dptr, "CSTA=%d, SAW=0x%X\n", cp->csta, cp->saw);

    switch (cp->csta) {                                 /* controller state? */

    case CST_S1:                                        /* need S1 reply */
        if (cp->saw & SA_S1H_VL) {                      /* valid? */
            if (cp->saw & SA_S1H_WR) {                  /* wrap? */
                cp->sa = cp->saw;                       /* echo data */
                cp->csta = CST_S1_WR;                   /* endless loop */
                }
            else {
                cp->s1dat = cp->saw;                    /* save data */
                dibp->vec = (cp->s1dat & SA_S1H_VEC) << 2; /* get vector */
                cp->sa = SA_S2 | SA_S2C_PT | SA_S2C_EC (cp->s1dat);
                cp->csta = CST_S2;                      /* now in step 2 */
                rf_init_int (cp);                       /* intr if req */
                }
            }                                           /* end if valid */
        break;

    case CST_S1_WR:                                     /* wrap mode */
        cp->sa = cp->saw;                               /* echo data */
        break;

    case CST_S2:                                        /* need S2 reply */
        cp->comm = cp->saw & SA_S2H_CLO;                /* get low addr */
        cp->prgi = cp->saw & SA_S2H_PI;                 /* get purge int */
        cp->sa = SA_S3 | SA_S3C_EC (cp->s1dat);
        cp->csta = CST_S3;                              /* now in step 3 */
        rf_init_int (cp);                               /* intr if req */
        break;

    case CST_S3:                                        /* need S3 reply */
        cp->comm = ((cp->saw & SA_S3H_CHI) << 16) | cp->comm;
        if (cp->saw & SA_S3H_PP) {                      /* purge/poll test? */
            cp->sa = 0;                                 /* put 0 */
            cp->csta = CST_S3_PPA;                      /* wait for 0 write */
            }
        else rf_step4 (cp);                             /* send step 4 */
        break;

    case CST_S3_PPA:                                    /* need purge test */
        if (cp->saw)                                    /* data not zero? */
            rf_fatal (cp, PE_PPF);
        else cp->csta = CST_S3_PPB;                     /* wait for poll */
        break;

    case CST_S4:                                        /* need S4 reply */
        if (cp->saw & SA_S4H_GO) {                      /* go set? */
            sim_debug (DBG_REQ, dptr, "initialization complete\n");
            cp->csta = CST_UP;                          /* we're up */
            cp->sa = 0;                                 /* clear SA */
            sim_activate_after (dptr->units + RF_TIMER, 1000000);
            if ((cp->saw & SA_S4H_LF)
                && cp->perr) rf_plf (cp, cp->perr);
            cp->perr = 0;
            }
        break;
        }                                               /* end switch */  
                      
    return SCPE_OK;
    }                                                   /* end if */
#endif

for (i = 0; i < RF_NUMDR; i++) {                        /* chk unit q's */
    nuptr = dptr->units + i;                            /* ptr to unit */
    if (nuptr->cpkt || (nuptr->pktq == 0))
        continue;
    pkt = rf_deqh (cp, &nuptr->pktq);                   /* get top of q */
    if (!rf_mscp (cp, pkt, FALSE))                      /* process */
        return SCPE_OK;
    }
#if 0
if ((pkt == 0) && cp->pip) {                            /* polling? */
    if (!rf_getpkt (cp, &pkt))                          /* get host pkt */
        return SCPE_OK;
    if (pkt) {                                          /* got one? */
        sim_debug (DBG_REQ, dptr, "cmd=%04X(%3s), mod=%04X, unit=%d, bc=%04X%04X, ma=%04X%04X, lbn=%04X%04X\n", 
                cp->pak[pkt].d[CMD_OPC], rf_cmdname[cp->pak[pkt].d[CMD_OPC]&0x3f],
                cp->pak[pkt].d[CMD_MOD], cp->pak[pkt].d[CMD_UN],
                cp->pak[pkt].d[RW_BCH], cp->pak[pkt].d[RW_BCL],
                cp->pak[pkt].d[RW_BAH], cp->pak[pkt].d[RW_BAL],
                cp->pak[pkt].d[RW_LBNH], cp->pak[pkt].d[RW_LBNL]);
        if (GETP (pkt, UQ_HCTC, TYP) != UQ_TYP_SEQ)     /* seq packet? */
            return rf_fatal (cp, PE_PIE);               /* no, term thread */
        cnid = GETP (pkt, UQ_HCTC, CID);                /* get conn ID */
        if (cnid == UQ_CID_MSCP) {                      /* MSCP packet? */
            if (!rf_mscp (cp, pkt, TRUE))               /* proc, q non-seq */
                return SCPE_OK;
            }
        else if (cnid == UQ_CID_DUP) {                  /* DUP packet? */
            rf_putr (cp, pkt, OP_END, 0, ST_CMD | I_OPCD, RSP_LNT, UQ_TYP_SEQ);
            if (!rf_putpkt (cp, pkt, TRUE))             /* ill cmd */
                return SCPE_OK;
            }
        else return rf_fatal (cp, PE_ICI);              /* no, term thread */
        }                                               /* end if pkt */
    else cp->pip = 0;                                   /* discontinue poll */
    }                                                   /* end if pip */
#endif
#if 0
if (cp->rspq) {                                         /* resp q? */
    pkt = rf_deqh (cp, &cp->rspq);                      /* get top of q */
    if (!rf_putpkt (cp, pkt, FALSE))                    /* send to host */
        return SCPE_OK;
    sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_quesvc - rf_putpkt failed - 1\n");
    }                                                   /* end if resp q */
#endif
if (pkt)                                                /* more to do? */
    sim_activate (uptr, rf_qtime);
return SCPE_OK;                                         /* done */
}

/* Clock service (roughly once per second) */

t_stat rf_tmrsvc (UNIT *uptr)
{
uint16 i;
UNIT *nuptr;
MSC *cp = rf_ctxmap[uptr->cnum];
DEVICE *dptr = rf_devmap[uptr->cnum];

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_tmrsvc\n");
sim_activate_after (uptr, 1000000);                     /* reactivate */
for (i = 0; i < dptr->numunits - 2; i++) {                        /* poll */
    nuptr = dptr->units + i;
    if ((nuptr->flags & UNIT_ATP) &&                    /* ATN pending? */
        (nuptr->flags & UNIT_ATT) &&                    /* still online? */
        (cp->cflgs & CF_ATN)) {                         /* wanted? */
        if (!rf_una (cp, nuptr->unit_plug))
            return SCPE_OK;
        }
    nuptr->flags = nuptr->flags & ~UNIT_ATP;
    }
#if 0
if ((cp->hat > 0) && (--cp->hat == 0))                  /* host timeout? */
    rf_fatal (cp, PE_HAT);                              /* fatal err */ 
#endif
return SCPE_OK;
}

/* MSCP packet handling */

t_bool rf_mscp (MSC *cp, uint16 pkt, t_bool q)
{
uint16 sts, cmd = GETP (pkt, CMD_OPC, OPC);

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_mscp - %s\n", q? "Queue" : "No Queue");

switch (cmd) {

    case OP_ABO:                                        /* abort */
        return rf_abo (cp, pkt, q);

    case OP_AVL:                                        /* avail */
        return rf_avl (cp, pkt, q);

    case OP_FMT:                                        /* format */
        return rf_fmt (cp, pkt, q);

    case OP_GCS:                                        /* get cmd status */
        return rf_gcs (cp, pkt, q);

    case OP_GUS:                                        /* get unit status */
        return rf_gus (cp, pkt, q);

    case OP_ONL:                                        /* online */
        return rf_onl (cp, pkt, q);

    case OP_SCC:                                        /* set ctrl char */
        return rf_scc (cp, pkt, q);

    case OP_SUC:                                        /* set unit char */
        return rf_suc (cp, pkt, q);

    case OP_ACC:                                        /* access */
    case OP_CMP:                                        /* compare */
    case OP_ERS:                                        /* erase */
    case OP_RD:                                         /* read */
    case OP_WR:                                         /* write */
        return rf_rw (cp, pkt, q);

    case OP_CCD:                                        /* nops */
    case OP_DAP:
    case OP_FLU:
        cmd = cmd | OP_END;                             /* set end flag */
        sts = ST_SUC;                                   /* success */
        break;

    default:
        cmd = OP_END;                                   /* set end op */
        sts = ST_CMD | I_OPCD;                          /* ill op */
        break;
        }

rf_putr (cp, pkt, cmd, 0, sts, RSP_LNT, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}

/* Abort a command - 1st parameter is ref # of cmd to abort */

t_bool rf_abo (MSC *cp, uint16 pkt, t_bool q)
{
uint16 lu = cp->pak[pkt].d[CMD_UN];                     /* unit # */
uint16 cmd = GETP (pkt, CMD_OPC, OPC);                  /* opcode */
uint32 ref = GETP32 (pkt, ABO_REFL);                    /* cmd ref # */
uint16 tpkt, prv;
UNIT *uptr;
DEVICE *dptr = rf_devmap[cp->cnum];

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_abo\n");

tpkt = 0;                                               /* set no mtch */
if ((uptr = rf_getucb (cp, lu))) {                      /* get unit */
    if (uptr->cpkt &&                                   /* curr pkt? */
        (GETP32 (uptr->cpkt, CMD_REFL) == ref)) {       /* match ref? */
        tpkt = uptr->cpkt;                              /* save match */
        uptr->cpkt = 0;                                 /* gonzo */
        sim_cancel (uptr);                              /* cancel unit */
        sim_activate (dptr->units + RF_QUEUE, rf_qtime);
        }
    else if (uptr->pktq &&                              /* head of q? */
        (GETP32 (uptr->pktq, CMD_REFL) == ref)) {       /* match ref? */
        tpkt = uptr->pktq;                              /* save match */
        uptr->pktq = cp->pak[tpkt].link;                /* unlink */
        }
    else if ((prv = uptr->pktq)) {                      /* srch pkt q */
        while ((tpkt = cp->pak[prv].link)) {            /* walk list */
            if (GETP32 (tpkt, RSP_REFL) == ref) {       /* match? unlink */
                cp->pak[prv].link = cp->pak[tpkt].link;
                break;
                }
            prv = tpkt;                                 /* no match, next */
            }
        }
    if (tpkt) {                                         /* found target? */
        uint16 tcmd = GETP (tpkt, CMD_OPC, OPC);        /* get opcode */
        rf_putr (cp, tpkt, tcmd | OP_END, 0, ST_ABO, RSP_LNT, UQ_TYP_SEQ);
        if (!rf_putpkt (cp, tpkt, TRUE))
            return ERR;
        }
    }                                                   /* end if unit */
rf_putr (cp, pkt, cmd | OP_END, 0, ST_SUC, ABO_LNT, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}

/* Unit available - set unit status to available - defer if q'd cmds */

t_bool rf_avl (MSC *cp, uint16 pkt, t_bool q)
{
uint16 lu = cp->pak[pkt].d[CMD_UN];                     /* unit # */
uint16 cmd = GETP (pkt, CMD_OPC, OPC);                  /* opcode */
uint32 mdf = cp->pak[pkt].d[CMD_MOD];                   /* modifier */
uint16 sts;
UNIT *uptr;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_avl\n");

if ((uptr = rf_getucb (cp, lu))) {                      /* unit exist? */
    if (q && uptr->cpkt) {                              /* need to queue? */
        rf_enqt (cp, &uptr->pktq, pkt);                 /* do later */
        return OK;
        }
    uptr->flags = uptr->flags & ~UNIT_ONL;              /* not online */
    if ((mdf & MD_SPD) && RF_RMV (uptr))                /* unload of removable device */
        sim_disk_unload (uptr);
    uptr->uf = 0;                                       /* clr flags */
    sts = ST_SUC;                                       /* success */
    }
else sts = ST_OFL;                                      /* offline */
rf_putr (cp, pkt, cmd | OP_END, 0, sts, AVL_LNT, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}

/* Get command status - only interested in active xfr cmd */

t_bool rf_gcs (MSC *cp, uint16 pkt, t_bool q)
{
uint16 lu = cp->pak[pkt].d[CMD_UN];                     /* unit # */
uint16 cmd = GETP (pkt, CMD_OPC, OPC);                  /* opcode */
uint32 ref = GETP32 (pkt, GCS_REFL);                    /* ref # */
int32 tpkt;
UNIT *uptr;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_gcs\n");

if ((uptr = rf_getucb (cp, lu)) &&                      /* valid lu? */
    (tpkt = uptr->cpkt) &&                              /* queued pkt? */
    (GETP32 (tpkt, CMD_REFL) == ref) &&                 /* match ref? */
    (GETP (tpkt, CMD_OPC, OPC) >= OP_ACC)) {            /* rd/wr cmd? */
    cp->pak[pkt].d[GCS_STSL] = cp->pak[tpkt].d[RW_WBCL];
    cp->pak[pkt].d[GCS_STSH] = cp->pak[tpkt].d[RW_WBCH];
    }
else {
    cp->pak[pkt].d[GCS_STSL] = 0;                       /* return 0 */
    cp->pak[pkt].d[GCS_STSH] = 0;
    }
rf_putr (cp, pkt, cmd | OP_END, 0, ST_SUC, GCS_LNT, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}

/* Get unit status */

t_bool rf_gus (MSC *cp, uint16 pkt, t_bool q)
{
uint16 lu = cp->pak[pkt].d[CMD_UN];                     /* unit # */
uint16 cmd = GETP (pkt, CMD_OPC, OPC);                  /* opcode */
uint16 dtyp, sts, rbpar;
UNIT *uptr;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_gus\n");

if (cp->pak[pkt].d[CMD_MOD] & MD_NXU) {                 /* next unit? */
    if (lu > cp->max_plug) {                            /* beyond last unit plug? */
        lu = 0;                                         /* reset to 0 */
        cp->pak[pkt].d[RSP_UN] = lu;
        }
    }
if ((uptr = rf_getucb (cp, lu))) {                      /* unit exist? */
    if ((uptr->flags & UNIT_ATT) == 0)                  /* not attached? */
        sts = ST_OFL | SB_OFL_NV;                       /* offl no vol */
    else if (uptr->flags & UNIT_ONL)                    /* online */
        sts = ST_SUC;
    else sts = ST_AVL;                                  /* avail */
    rf_putr_unit (cp, pkt, uptr, lu, FALSE);            /* fill unit fields */
    dtyp = GET_DTYPE (uptr->flags);                     /* get drive type */
    if (drv_tab[dtyp].rcts)                             /* ctrl bad blk? */
        rbpar = 1;
    else rbpar = 0;                                     /* fill geom, bblk */
    cp->pak[pkt].d[GUS_TRK] = drv_tab[dtyp].sect;
    cp->pak[pkt].d[GUS_GRP] = drv_tab[dtyp].tpg;
    cp->pak[pkt].d[GUS_CYL] = drv_tab[dtyp].gpc;
    cp->pak[pkt].d[GUS_UVER] = 0;
    cp->pak[pkt].d[GUS_RCTS] = drv_tab[dtyp].rcts;
    cp->pak[pkt].d[GUS_RBSC] =
        (rbpar << GUS_RB_V_RBNS) | (rbpar << GUS_RB_V_RCTC);
    }
else sts = ST_OFL;                                      /* offline */
cp->pak[pkt].d[GUS_SHUN] = lu;                          /* shadowing */
cp->pak[pkt].d[GUS_SHST] = 0;
rf_putr (cp, pkt, cmd | OP_END, 0, sts, GUS_LNT_D, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}

/* Unit online - defer if q'd commands */

t_bool rf_onl (MSC *cp, uint16 pkt, t_bool q)
{
uint16 lu = cp->pak[pkt].d[CMD_UN];                     /* unit # */
uint16 cmd = GETP (pkt, CMD_OPC, OPC);                  /* opcode */
uint16 sts;
UNIT *uptr;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_onl\n");

if ((uptr = rf_getucb (cp, lu))) {                      /* unit exist? */
    if (q && uptr->cpkt) {                              /* need to queue? */
        rf_enqt (cp, &uptr->pktq, pkt);                 /* do later */
        return OK;
        }
    if ((uptr->flags & UNIT_ATT) == 0)                  /* not attached? */
        sts = ST_OFL | SB_OFL_NV;                       /* offl no vol */
    else if (uptr->flags & UNIT_ONL)                    /* already online? */
        sts = ST_SUC | SB_SUC_ON;
    else if (sim_disk_isavailable (uptr))
        {                                              /* mark online */
        sts = ST_SUC;
        uptr->flags = uptr->flags | UNIT_ONL;
        rf_setf_unit (cp, pkt, uptr);                   /* hack flags */
        }
    else
        sts = ST_OFL | SB_OFL_NV;                       /* offl no vol */
    rf_putr_unit (cp, pkt, uptr, lu, TRUE);             /* set fields */
    }
else sts = ST_OFL;                                      /* offline */
cp->pak[pkt].d[ONL_SHUN] = lu;                          /* shadowing */
cp->pak[pkt].d[ONL_SHST] = 0;
rf_putr (cp, pkt, cmd | OP_END, 0, sts, ONL_LNT, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}

/* Set controller characteristics */

t_bool rf_scc (MSC *cp, uint16 pkt, t_bool q)
{
uint16 sts, cmd;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_scc\n");

if (cp->pak[pkt].d[SCC_MSV]) {                          /* MSCP ver = 0? */
    sts = ST_CMD | I_VRSN;                              /* no, lose */
    cmd = 0;
    }
else {
    sts = ST_SUC;                                       /* success */
    cmd = GETP (pkt, CMD_OPC, OPC);                     /* get opcode */
    cp->cflgs = (cp->cflgs & CF_RPL) |                  /* hack ctrl flgs */
        cp->pak[pkt].d[SCC_CFL] | CF_MLH;
    if ((cp->htmo = cp->pak[pkt].d[SCC_TMO]))           /* set timeout */
        cp->htmo = cp->htmo + 2;                        /* if nz, round up */
    cp->pak[pkt].d[RSP_UN] = 5;
    cp->pak[pkt].d[SCC_CFL] = cp->cflgs;                /* return flags */
    cp->pak[pkt].d[SCC_TMO] = RF_DCTMO;                 /* ctrl timeout */
    cp->pak[pkt].d[SCC_VER] = (RF_HVER << SCC_VER_V_HVER) |
        (RF_SVER << SCC_VER_V_SVER);
    cp->pak[pkt].d[SCC_CIDA] = 0;                       /* ctrl ID */
    cp->pak[pkt].d[SCC_CIDB] = 0;
    cp->pak[pkt].d[SCC_CIDC] = 0;
    cp->pak[pkt].d[SCC_CIDD] = (RF_CLASS << SCC_CIDD_V_CLS) |
        (32 << SCC_CIDD_V_MOD);
#if 0
        (ctlr_tab[cp->ctype].model << SCC_CIDD_V_MOD);
#endif
    cp->pak[pkt].d[SCC_MBCL] = 0;                       /* max bc */
    cp->pak[pkt].d[SCC_MBCH] = 0;
    }
rf_putr (cp, pkt, cmd | OP_END, 0, sts, SCC_LNT, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}
    
/* Set unit characteristics - defer if q'd commands */

t_bool rf_suc (MSC *cp, uint16 pkt, t_bool q)
{
uint16 lu = cp->pak[pkt].d[CMD_UN];                     /* unit # */
uint16 cmd = GETP (pkt, CMD_OPC, OPC);                  /* opcode */
uint16 sts;
UNIT *uptr;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_suc\n");

if ((uptr = rf_getucb (cp, lu))) {                      /* unit exist? */
    if (q && uptr->cpkt) {                              /* need to queue? */
        rf_enqt (cp, &uptr->pktq, pkt);                 /* do later */
        return OK;
        }
    if ((uptr->flags & UNIT_ATT) == 0)                  /* not attached? */
        sts = ST_OFL | SB_OFL_NV;                       /* offl no vol */
    else {                                              /* avail or onl */
        sts = ST_SUC;
        rf_setf_unit (cp, pkt, uptr);                   /* hack flags */
        }
    rf_putr_unit (cp, pkt, uptr, lu, TRUE);             /* set fields */
    }
else sts = ST_OFL;                                      /* offline */
cp->pak[pkt].d[ONL_SHUN] = lu;                          /* shadowing */
cp->pak[pkt].d[ONL_SHST] = 0;
rf_putr (cp, pkt, cmd | OP_END, 0, sts, SUC_LNT, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}

/* Format command - floppies only */

t_bool rf_fmt (MSC *cp, uint16 pkt, t_bool q)
{
uint16 lu = cp->pak[pkt].d[CMD_UN];                     /* unit # */
uint16 cmd = GETP (pkt, CMD_OPC, OPC);                  /* opcode */
uint16 sts;
UNIT *uptr;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_fmt\n");

if ((uptr = rf_getucb (cp, lu))) {                      /* unit exist? */
    if (q && uptr->cpkt) {                              /* need to queue? */
        rf_enqt (cp, &uptr->pktq, pkt);                 /* do later */
        return OK;
        }
    if (GET_DTYPE (uptr->flags) != RX33_DTYPE)          /* RX33? */
        sts = ST_CMD | I_OPCD;                          /* no, err */
    else if ((cp->pak[pkt].d[FMT_IH] & 0100000) == 0)   /* magic bit set? */
        sts = ST_CMD | I_FMTI;                          /* no, err */
    else if ((uptr->flags & UNIT_ATT) == 0)             /* offline? */
        sts = ST_OFL | SB_OFL_NV;                       /* no vol */
    else if (uptr->flags & UNIT_ONL) {                  /* online? */
        uptr->flags = uptr->flags & ~UNIT_ONL;
        uptr->uf = 0;                                   /* clear flags */
        sts = ST_AVL | SB_AVL_INU;                      /* avail, in use */
        }
    else if (RF_WPH (uptr))                             /* write prot? */
        sts = ST_WPR | SB_WPR_HW;                       /* can't fmt */
    else sts = ST_SUC;                                  /*** for now ***/
    }
else sts = ST_OFL;                                      /* offline */
rf_putr (cp, pkt, cmd | OP_END, 0, sts, FMT_LNT, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}

/* Data transfer commands */

t_bool rf_rw (MSC *cp, uint16 pkt, t_bool q)
{
uint16 lu = cp->pak[pkt].d[CMD_UN];                     /* unit # */
uint16 cmd = GETP (pkt, CMD_OPC, OPC);                  /* opcode */
uint16 sts;
UNIT *uptr;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_rw(lu=%d, pkt=%d, queue=%s)\n", lu, pkt, q?"yes" : "no");

if ((uptr = rf_getucb (cp, lu))) {                      /* unit exist? */
    if (q && uptr->cpkt) {                              /* need to queue? */
        uint16 tpktq = uptr->pktq;

        sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_rw - queued\n");

        rf_enqt (cp, &tpktq, pkt);                      /* do later */
        uptr->pktq = tpktq;
        return OK;
        }
    sts = rf_rw_valid (cp, pkt, uptr, cmd);             /* validity checks */
    if (sts == 0) {                                     /* ok? */
        uptr->cpkt = pkt;                               /* op in progress */
        cp->pak[pkt].d[RW_WBD0L] = cp->pak[pkt].d[RW_BD0L];
        cp->pak[pkt].d[RW_WBD0H] = cp->pak[pkt].d[RW_BD0H];
        cp->pak[pkt].d[RW_WBD1L] = cp->pak[pkt].d[RW_BD1L];
        cp->pak[pkt].d[RW_WBD1H] = cp->pak[pkt].d[RW_BD1H];
        cp->pak[pkt].d[RW_WBD2L] = cp->pak[pkt].d[RW_BD2L];
        cp->pak[pkt].d[RW_WBD2H] = cp->pak[pkt].d[RW_BD2H];
        cp->pak[pkt].d[RW_WBCL] = cp->pak[pkt].d[RW_BCL];
        cp->pak[pkt].d[RW_WBCH] = cp->pak[pkt].d[RW_BCH];
        cp->pak[pkt].d[RW_WBLL] = cp->pak[pkt].d[RW_LBNL];
        cp->pak[pkt].d[RW_WBLH] = cp->pak[pkt].d[RW_LBNH];
        uptr->iostarttime = sim_grtime();
        sim_activate (uptr, 0);                         /* activate */
        sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_rw - started\n");
        return OK;                                      /* done */
        }
    }
else sts = ST_OFL;                                      /* offline */
cp->pak[pkt].d[RW_BCL] = cp->pak[pkt].d[RW_BCH] = 0;    /* bad packet */
rf_putr (cp, pkt, cmd | OP_END, 0, sts, RW_LNT_D, UQ_TYP_SEQ);
return rf_putpkt (cp, pkt, TRUE);
}

/* Validity checks */

uint16 rf_rw_valid (MSC *cp, uint16 pkt, UNIT *uptr, uint16 cmd)
{
uint32 dtyp = GET_DTYPE (uptr->flags);                  /* get drive type */
uint32 lbn = GETP32 (pkt, RW_LBNL);                     /* get lbn */
uint32 bc = GETP32 (pkt, RW_BCL);                       /* get byte cnt */
uint32 maxlbn = (uint32)uptr->capac;                    /* get max lbn */

if ((uptr->flags & UNIT_ATT) == 0)                      /* not attached? */
    return (ST_OFL | SB_OFL_NV);                        /* offl no vol */
if ((uptr->flags & UNIT_ONL) == 0)                      /* not online? */
    return ST_AVL;                                      /* only avail */
if ((cmd != OP_ACC) && (cmd != OP_ERS) &&               /* 'real' xfer */
    (cp->pak[pkt].d[RW_BAL] & 1))                       /* odd address? */
    return (ST_HST | SB_HST_OA);                        /* host buf odd */
if (bc & 1)                                             /* odd byte cnt? */
    return (ST_HST | SB_HST_OC);
if (bc & 0xF0000000)                                    /* 'reasonable' bc? */
    return (ST_CMD | I_BCNT);
// if (lbn & 0xF0000000) return (ST_CMD | I_LBN);       /* 'reasonable' lbn? */
if (lbn >= maxlbn) {                                    /* accessing RCT? */
    if (lbn >= (maxlbn + drv_tab[dtyp].rcts))           /* beyond copy 1? */
        return (ST_CMD | I_LBN);                        /* lbn err */
    if (bc != RF_NUMBY)                                 /* bc must be 512 */
        return (ST_CMD | I_BCNT);
    }
else if ((lbn + ((bc + (RF_NUMBY - 1)) / RF_NUMBY)) > maxlbn)
    return (ST_CMD | I_BCNT);                           /* spiral to RCT */
if ((cmd == OP_WR) || (cmd == OP_ERS)) {                /* write op? */
    if (lbn >= maxlbn)                                  /* accessing RCT? */
        return (ST_CMD | I_LBN);                        /* lbn err */
    if (uptr->uf & UF_WPS)                              /* swre wlk? */
        return (ST_WPR | SB_WPR_SW);
    if (RF_WPH (uptr))                                  /* hwre wlk? */
        return (ST_WPR | SB_WPR_HW);
    }
return 0;                                               /* success! */
}

/* I/O completion callback */

void rf_io_complete (UNIT *uptr, t_stat status)
{
MSC *cp = rf_ctxmap[uptr->cnum];

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_io_complete(status=%d)\n", status);

uptr->io_status = status;
uptr->io_complete = 1;
/* Reschedule for the appropriate delay */
sim_activate_notbefore (uptr, uptr->iostarttime+rf_xtime);
}

// TODO: Need to update ba in rq version

/* Read byte buffer from memory */

int32 rf_readb (uint32 *bd, int32 bc, uint8 *buf)
{
// REQDAT
return 0;
}

/* Read word buffer from memory */

int32 rf_readw (uint32 *bd, int32 bc, uint16 *buf, UNIT *uptr)
{
return hsc_reqdat (bd, (uint8*)buf, bc, uptr);
}

/* Write word buffer to memory */

int32 rf_writew (uint32 *bd, int32 bc, uint16 *buf, UNIT *uptr)
{
return hsc_snddat (bd, (uint8*)buf, bc, uptr);
}

/* Unit service for data transfer commands */

t_stat rf_svc (UNIT *uptr)
{
MSC *cp = rf_ctxmap[uptr->cnum];
uint32 i, tbc, abc, wwc;
int32 t;
uint32 err = 0;
int32 pkt = uptr->cpkt;                                 /* get packet */
uint32 cmd, ba, bc, bl, ma;
uint32 bd[3];

if ((cp == NULL) || (pkt == 0))                         /* what??? */
    return STOP_RQ;
cmd = GETP (pkt, CMD_OPC, OPC);                         /* get cmd */
//FIXME ba = GETP32 (pkt, RW_WBAL);                             /* buf addr */
bc = GETP32 (pkt, RW_WBCL);                             /* byte count */
bl = GETP32 (pkt, RW_WBLL);                             /* block addr */
//FIXME ma = GETP32 (pkt, RW_WMPL);                             /* block addr */

bd[0] = GETP32 (pkt, RW_WBD0L);
bd[1] = GETP32 (pkt, RW_WBD1L);
bd[2] = GETP32 (pkt, RW_WBD2L);

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_svc(%s,unit=%d, pkt=%d, cmd=%s, lbn=%0X, bc=%0x, phase=%s)\n",
           sim_uname (uptr), uptr->unit_plug, pkt, rf_cmdname[cp->pak[pkt].d[CMD_OPC]&0x3f], bl, bc,
           uptr->io_complete ? "bottom" : "top");

tbc = (bc > RF_MAXFR)? RF_MAXFR: bc;                    /* trim cnt to max */

if ((uptr->flags & UNIT_ATT) == 0) {                    /* not attached? */
    rf_rw_end (cp, uptr, 0, ST_OFL | SB_OFL_NV);        /* offl no vol */
    return SCPE_OK;
    }
if (bc == 0) {                                          /* no xfer? */
    rf_rw_end (cp, uptr, 0, ST_SUC);                    /* ok by me... */
    return SCPE_OK;
    }

if ((cmd == OP_ERS) || (cmd == OP_WR)) {                /* write op? */
    if (RF_WPH (uptr)) {
        rf_rw_end (cp, uptr, 0, ST_WPR | SB_WPR_HW);
        return SCPE_OK;
        }
    if (uptr->uf & UF_WPS) {
        rf_rw_end (cp, uptr, 0, ST_WPR | SB_WPR_SW);
        return SCPE_OK;
        }
    }

if (!uptr->io_complete) { /* Top End (I/O Initiation) Processing */
    if (cmd == OP_ERS) {                                /* erase? */
        wwc = ((tbc + (RF_NUMBY - 1)) & ~(RF_NUMBY - 1)) >> 1;
        memset (uptr->rfxb, 0, wwc * sizeof(uint16));   /* clr buf */
        sim_disk_data_trace(uptr, (uint8 *)uptr->rfxb, bl, wwc << 1, "sim_disk_wrsect-ERS", DBG_DAT & rf_devmap[cp->cnum]->dctrl, DBG_REQ);
        err = sim_disk_wrsect_a (uptr, bl, (uint8 *)uptr->rfxb, NULL, (wwc << 1) / RF_NUMBY, rf_io_complete);
        }

    else if (cmd == OP_WR) {                            /* write? */
        t = rf_readw (bd, tbc, (uint16 *)uptr->rfxb, uptr);   /* fetch buffer */
        if (t < 0) {
            // TODO: Wait until reactivated
            return SCPE_OK;
            }
        uptr->hwmark = (uint32)t;
        if ((abc = tbc - t)) {                          /* any xfer? */
            wwc = ((abc + (RF_NUMBY - 1)) & ~(RF_NUMBY - 1)) >> 1;
            for (i = (abc >> 1); i < wwc; i++)
                ((uint16 *)(uptr->rfxb))[i] = 0;
            sim_disk_data_trace(uptr, (uint8 *)uptr->rfxb, bl, wwc << 1, "sim_disk_wrsect-WR", DBG_DAT & rf_devmap[cp->cnum]->dctrl, DBG_REQ);
            err = sim_disk_wrsect_a (uptr, bl, (uint8 *)uptr->rfxb, NULL, (wwc << 1) / RF_NUMBY, rf_io_complete);
            }
        }

    else {  /* OP_RD & OP_CMP */
        err = sim_disk_rdsect_a (uptr, bl, (uint8 *)uptr->rfxb, NULL, (tbc + RF_NUMBY - 1) / RF_NUMBY, rf_io_complete);
        }                                               /* end else read */
    return SCPE_OK;                                     /* done for now until callback */    
    }
else { /* Bottom End (After I/O processing) */
    uptr->io_complete = 0;
    err = uptr->io_status;
    if (cmd == OP_ERS) {                                /* erase? */
        }

    else if (cmd == OP_WR) {                            /* write? */
        abc = tbc - uptr->hwmark;                       /* any xfer? */
        if (uptr->hwmark) {                             /* nxm? */
            //FIXME: Already updated in readw?
            PUTP32 (pkt, RW_WBCL, bc - abc);            /* adj bc */
//FIXME            PUTP32 (pkt, RW_WBAL, ba + abc);            /* adj ba */
            if (rf_hbe (cp, uptr))                      /* post err log */
                rf_rw_end (cp, uptr, EF_LOG, ST_HST | SB_HST_NXM);  
            return SCPE_OK;                             /* end else wr */
            }
        }

    else {
        sim_disk_data_trace(uptr, (uint8 *)uptr->rfxb, bl, tbc, "sim_disk_rdsect", DBG_DAT & rf_devmap[cp->cnum]->dctrl, DBG_REQ);
        if ((cmd == OP_RD) && !err) {                   /* read? */
            t = rf_writew (bd, tbc, (uint16 *)uptr->rfxb, uptr);
            if (t < 0) {
                uptr->io_complete = 1;
                return SCPE_OK;
                }
            else if (t) {                               /* store, nxm? */
                PUTP32 (pkt, RW_WBCL, bc - (tbc - t));  /* adj bc */
//FIXME                PUTP32 (pkt, RW_WBAL, ba + (tbc - t));  /* adj ba */
                if (rf_hbe (cp, uptr))                  /* post err log */
                    rf_rw_end (cp, uptr, EF_LOG, ST_HST | SB_HST_NXM);      
                return SCPE_OK;
                }
            }
        else if ((cmd == OP_CMP) && !err) {             /* compare? */
#if 0
            uint8 dby, mby;
            for (i = 0; i < tbc; i++) {                 /* loop */
                if (rf_readb (bd + i, 1, &mby)) {       /* fetch, nxm? */
                    PUTP32 (pkt, RW_WBCL, bc - i);      /* adj bc */
                    PUTP32 (pkt, RW_WBAL, bc - i);      /* adj ba */
                    if (rf_hbe (cp, uptr))              /* post err log */
                        rf_rw_end (cp, uptr, EF_LOG, ST_HST | SB_HST_NXM);
                    return SCPE_OK;
                    }
                dby = (((uint16 *)(uptr->rfxb))[i >> 1] >> ((i & 1)? 8: 0)) & 0xFF;
                if (mby != dby) {                       /* cmp err? */
                    PUTP32 (pkt, RW_WBCL, bc - i);      /* adj bc */
                    rf_rw_end (cp, uptr, 0, ST_CMP);    /* done */
                    return SCPE_OK;                     /* exit */
                    }                                   /* end if */
                }                                       /* end for */
#endif
            }                                           /* end else if */
        }                                               /* end else read */
    }                                                   /* end else bottom end */
if (err != 0) {                                         /* error? */
    if (rf_dte (cp, uptr, ST_DRV))                      /* post err log */
        rf_rw_end (cp, uptr, EF_LOG, ST_DRV);           /* if ok, report err */
    sim_disk_perror (uptr, "RF I/O error");
    sim_disk_clearerr (uptr);
    return SCPE_IOERR;
    }
//FIXME ba = ba + tbc;                                          /* incr bus addr */
bc = bc - tbc;                                          /* decr byte cnt */
bl = bl + ((tbc + (RF_NUMBY - 1)) / RF_NUMBY);          /* incr blk # */
//FIXME PUTP32 (pkt, RW_WBAL, ba);                              /* update pkt */
PUTP32 (pkt, RW_WBCL, bc);
PUTP32 (pkt, RW_WBLL, bl);
if (bc)                                                 /* more? resched */
    sim_activate (uptr, 0);
else rf_rw_end (cp, uptr, 0, ST_SUC);                   /* done! */
return SCPE_OK;
}

/* Transfer command complete */

t_bool rf_rw_end (MSC *cp, UNIT *uptr, uint16 flg, uint16 sts)
{
uint16 pkt = uptr->cpkt;                                /* packet */
uint16 cmd = GETP (pkt, CMD_OPC, OPC);                  /* get cmd */
uint32 bc = GETP32 (pkt, RW_BCL);                       /* init bc */
uint32 wbc = GETP32 (pkt, RW_WBCL);                     /* work bc */
DEVICE *dptr = rf_devmap[uptr->cnum];

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_rw_end\n");

uptr->cpkt = 0;                                         /* done */
PUTP32 (pkt, RW_BCL, bc - wbc);                         /* bytes processed */
cp->pak[pkt].d[RW_WBD0L] = 0;                            /* clear temps */
cp->pak[pkt].d[RW_WBD0H] = 0;
cp->pak[pkt].d[RW_WBD1L] = 0;
cp->pak[pkt].d[RW_WBD1H] = 0;
cp->pak[pkt].d[RW_WBD2L] = 0;
cp->pak[pkt].d[RW_WBD2H] = 0;
cp->pak[pkt].d[RW_WBCL] = 0;
cp->pak[pkt].d[RW_WBCH] = 0;
cp->pak[pkt].d[RW_WBLL] = 0;
cp->pak[pkt].d[RW_WBLH] = 0;
rf_putr (cp, pkt, cmd | OP_END, flg, sts, RW_LNT_D, UQ_TYP_SEQ); /* fill pkt */
if (!rf_putpkt (cp, pkt, TRUE))                         /* send pkt */
    return ERR;
if (uptr->pktq)                                         /* more to do? */
    sim_activate (dptr->units + RF_QUEUE, rf_qtime);    /* activate thread */
return OK;
}

/* Data transfer error log packet */

t_bool rf_dte (MSC *cp, UNIT *uptr, uint16 err)
{
uint16 pkt, tpkt;
uint16 lu, ccyl, csurf, csect;
uint32 dtyp, lbn, t;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_dte\n");

if ((cp->cflgs & CF_THS) == 0)                          /* logging? */
    return OK;
if (!rf_deqf (cp, &pkt))                                /* get log pkt */
    return ERR;
tpkt = uptr->cpkt;                                      /* rw pkt */
lu = cp->pak[tpkt].d[CMD_UN];                           /* unit # */
lbn = GETP32 (tpkt, RW_WBLL);                           /* recent LBN */
dtyp = GET_DTYPE (uptr->flags);                         /* drv type */
if (drv_tab[dtyp].flgs & RFDF_SDI)                      /* SDI? ovhd @ end */
    t = 0;
else t = (drv_tab[dtyp].xbn + drv_tab[dtyp].dbn) /      /* ovhd cylinders */
    (drv_tab[dtyp].sect * drv_tab[dtyp].surf);
ccyl = (uint16)(t + (lbn / drv_tab[dtyp].cyl));         /* curr real cyl */
t = lbn % drv_tab[dtyp].cyl;                            /* trk relative blk */
csurf = (uint16)(t / drv_tab[dtyp].surf);               /* curr surf */
csect = (uint16)(t % drv_tab[dtyp].surf);               /* curr sect */

cp->pak[pkt].d[ELP_REFL] = cp->pak[tpkt].d[CMD_REFL];   /* copy cmd ref */
cp->pak[pkt].d[ELP_REFH] = cp->pak[tpkt].d[CMD_REFH];
cp->pak[pkt].d[ELP_UN] = lu;                            /* copy unit */
cp->pak[pkt].d[ELP_SEQ] = 0;                            /* clr seq # */
cp->pak[pkt].d[DTE_CIDA] = 0;                           /* ctrl ID */
cp->pak[pkt].d[DTE_CIDB] = 0;
cp->pak[pkt].d[DTE_CIDC] = 0;
cp->pak[pkt].d[DTE_CIDD] = (RF_CLASS << DTE_CIDD_V_CLS) |
    (ctlr_tab[cp->ctype].model << DTE_CIDD_V_MOD);
cp->pak[pkt].d[DTE_VER] = (RF_HVER << DTE_VER_V_HVER) |
    (RF_SVER << DTE_VER_V_SVER);
cp->pak[pkt].d[DTE_MLUN] = lu;                          /* MLUN */
cp->pak[pkt].d[DTE_UIDA] = lu;                          /* unit ID */
cp->pak[pkt].d[DTE_UIDB] = 0;
cp->pak[pkt].d[DTE_UIDC] = 0;
cp->pak[pkt].d[DTE_UIDD] = (UID_DISK << DTE_UIDD_V_CLS) |
    (drv_tab[dtyp].mod << DTE_UIDD_V_MOD);
cp->pak[pkt].d[DTE_UVER] = 0;                           /* unit versn */
cp->pak[pkt].d[DTE_SCYL] = ccyl;                        /* cylinder */
cp->pak[pkt].d[DTE_VSNL] = 01234 + lu;                  /* vol ser # */
cp->pak[pkt].d[DTE_VSNH] = 0;
cp->pak[pkt].d[DTE_D1] = 0;
cp->pak[pkt].d[DTE_D2] = csect << DTE_D2_V_SECT;        /* geometry */
cp->pak[pkt].d[DTE_D3] = (ccyl << DTE_D3_V_CYL) |
    (csurf << DTE_D3_V_SURF);
rf_putr (cp, pkt, FM_SDE, LF_SNR, err, DTE_LNT, UQ_TYP_DAT);
return rf_putpkt (cp, pkt, TRUE);
}

/* Host bus error log packet */

t_bool rf_hbe (MSC *cp, UNIT *uptr)
{
uint16 pkt, tpkt;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_hbe\n");

if ((cp->cflgs & CF_THS) == 0)                          /* logging? */
    return OK;
if (!rf_deqf (cp, &pkt))                                /* get log pkt */
    return ERR;
tpkt = uptr->cpkt;                                      /* rw pkt */
cp->pak[pkt].d[ELP_REFL] = cp->pak[tpkt].d[CMD_REFL];   /* copy cmd ref */
cp->pak[pkt].d[ELP_REFH] = cp->pak[tpkt].d[CMD_REFH];
cp->pak[pkt].d[ELP_UN] = cp->pak[tpkt].d[CMD_UN];       /* copy unit */
cp->pak[pkt].d[ELP_SEQ] = 0;                            /* clr seq # */
cp->pak[pkt].d[HBE_CIDA] = 0;                           /* ctrl ID */
cp->pak[pkt].d[HBE_CIDB] = 0;
cp->pak[pkt].d[HBE_CIDC] = 0;
cp->pak[pkt].d[HBE_CIDD] = (RF_CLASS << DTE_CIDD_V_CLS) |
    (ctlr_tab[cp->ctype].model << DTE_CIDD_V_MOD);
cp->pak[pkt].d[HBE_VER] = (RF_HVER << HBE_VER_V_HVER) | /* versions */
    (RF_SVER << HBE_VER_V_SVER);
cp->pak[pkt].d[HBE_RSV] = 0;
cp->pak[pkt].d[HBE_BADL] = cp->pak[tpkt].d[RW_WBAL];    /* bad addr */
cp->pak[pkt].d[HBE_BADH] = cp->pak[tpkt].d[RW_WBAH];
rf_putr (cp, pkt, FM_BAD, LF_SNR, ST_HST | SB_HST_NXM, HBE_LNT, UQ_TYP_DAT);
return rf_putpkt (cp, pkt, TRUE);
}

/* Port last failure error log packet */

t_bool rf_plf (MSC *cp, uint16 err)
{
uint16 pkt;

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_plf\n");

if (!rf_deqf (cp, &pkt))                                /* get log pkt */
    return ERR;
cp->pak[pkt].d[ELP_REFL] = 0;                           /* ref = 0 */
cp->pak[pkt].d[ELP_REFH] = 0;
cp->pak[pkt].d[ELP_UN] = 0;                             /* no unit */
cp->pak[pkt].d[ELP_SEQ] = 0;                            /* no seq */
cp->pak[pkt].d[PLF_CIDA] = 0;                           /* cntl ID */
cp->pak[pkt].d[PLF_CIDB] = 0;
cp->pak[pkt].d[PLF_CIDC] = 0;
cp->pak[pkt].d[PLF_CIDD] = (RF_CLASS << PLF_CIDD_V_CLS) |
    (ctlr_tab[cp->ctype].model << PLF_CIDD_V_MOD);
cp->pak[pkt].d[PLF_VER] = (RF_SVER << PLF_VER_V_SVER) |
    (RF_HVER << PLF_VER_V_HVER);
cp->pak[pkt].d[PLF_ERR] = err;
rf_putr (cp, pkt, FM_CNT, LF_SNR, ST_CNT, PLF_LNT, UQ_TYP_DAT);
cp->pak[pkt].d[UQ_HCTC] |= (UQ_CID_DIAG << UQ_HCTC_V_CID);
return rf_putpkt (cp, pkt, TRUE);
}

/* Unit now available attention packet */

t_bool rf_una (MSC *cp, uint16 lu)
{
uint16 pkt;
UNIT *uptr = rf_getucb (cp, lu);

if (uptr == NULL)                                       /* huh? */
    return OK;
sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_una (%s. Unit=%d)\n", sim_uname (uptr), lu);
if (!rf_deqf (cp, &pkt))                                /* get log pkt */
    return ERR;
cp->pak[pkt].d[RSP_REFL] = 0;                           /* ref = 0 */
cp->pak[pkt].d[RSP_REFH] = 0;
cp->pak[pkt].d[RSP_UN] = lu;
cp->pak[pkt].d[RSP_RSV] = 0;
rf_putr_unit (cp, pkt, uptr, lu, FALSE);                /* fill unit fields */
rf_putr (cp, pkt, OP_AVA, 0, 0, UNA_LNT, UQ_TYP_SEQ);   /* fill std fields */
return rf_putpkt (cp, pkt, TRUE);
}

/* List handling

   rf_deqf      -       dequeue head of free list (fatal err if none)
   rf_deqh      -       dequeue head of list
   rf_enqh      -       enqueue at head of list
   rf_enqt      -       enqueue at tail of list
*/

t_bool rf_deqf (MSC *cp, uint16 *pkt)
{
*pkt = 0;
if (cp->freq == 0)                                      /* no free pkts?? */
    return rf_fatal (cp, PE_NSR);
cp->pbsy = cp->pbsy + 1;                                /* cnt busy pkts */
*pkt = cp->freq;                                        /* head of list */
cp->freq = cp->pak[cp->freq].link;                      /* next */
return OK;
}

uint16 rf_deqh (MSC *cp, uint16 *lh)
{
uint16 ptr = *lh;                                        /* head of list */

if (ptr)                                                /* next */
    *lh = cp->pak[ptr].link;
return ptr;
}

void rf_enqh (MSC *cp, uint16 *lh, uint16 pkt)
{
if (pkt == 0)                                           /* any pkt? */
    return;
cp->pak[pkt].link = *lh;                                /* link is old lh */
*lh = pkt;                                              /* pkt is new lh */
return;
}

void rf_enqt (MSC *cp, uint16 *lh, uint16 pkt)
{
if (pkt == 0)                                           /* any pkt? */
    return;
cp->pak[pkt].link = 0;                                  /* it will be tail */
if (*lh == 0)                                           /* if empty, enqh */
    *lh = pkt;
else {
    uint32 ptr = *lh;                                   /* chase to end */
    while (cp->pak[ptr].link)
        ptr = cp->pak[ptr].link;
    cp->pak[ptr].link = pkt;                            /* enq at tail */
    }
return;
}

/* Packet and descriptor handling */

/* Get packet from command ring */

#if 0
t_bool rf_getpkt (MSC *cp, uint16 *pkt)
{
uint32 addr, desc;

*pkt = 0;
if (!rf_getdesc (cp, &cp->cq, &desc))                   /* get cmd desc */
    return ERR;
if ((desc & UQ_DESC_OWN) == 0) {                        /* none */
    *pkt = 0;                                           /* pkt = 0 */
    return OK;                                          /* no error */
    }
if (!rf_deqf (cp, pkt))                                 /* get cmd pkt */
    return ERR;
cp->hat = 0;                                            /* dsbl hst timer */
addr = desc & UQ_ADDR;                                  /* get Q22 addr */
if (Map_ReadW (addr + UQ_HDR_OFF, RF_PKT_SIZE, cp->pak[*pkt].d))
    return rf_fatal (cp, PE_PRE);                       /* read pkt */
return rf_putdesc (cp, &cp->cq, desc);                  /* release desc */
}
#endif

/* Put packet to response ring - note the clever hack about credits.
   The controller sends all its credits to the host.  Thereafter, it
   supplies one credit for every response packet sent over.  Simple!
*/

t_bool rf_putpkt (MSC *cp, uint16 pkt, t_bool qt)
{
hsc_mscp_done (cp, pkt);
#if 0
uint32 addr, desc, lnt, cr;
DEVICE *dptr = rf_devmap[cp->cnum];

if (pkt == 0)                                           /* any packet? */
    return OK;
sim_debug (DBG_REQ, dptr, "rsp=%04X, sts=%04X\n", 
                           cp->pak[pkt].d[RSP_OPF], cp->pak[pkt].d[RSP_STS]);
if (!rf_getdesc (cp, &cp->rq, &desc))                   /* get rsp desc */
    return ERR;
// FIXME: If can't send to HSC then queue locally
if ((desc & UQ_DESC_OWN) == 0) {                        /* not valid? */
    if (qt)                                             /* normal? q tail */
        rf_enqt (cp, &cp->rspq, pkt);
    else
        rf_enqh (cp, &cp->rspq, pkt);                   /* resp q call */
    sim_activate (dptr->units + RF_QUEUE, rf_qtime);    /* activate q thrd */
    return OK;
    }
addr = desc & UQ_ADDR;                                  /* get Q22 addr */
lnt = cp->pak[pkt].d[UQ_HLNT] - UQ_HDR_OFF;             /* size, with hdr */
if ((GETP (pkt, UQ_HCTC, TYP) == UQ_TYP_SEQ) &&         /* seq packet? */
    (GETP (pkt, CMD_OPC, OPC) & OP_END)) {              /* end packet? */
    cr = (cp->credits >= 14)? 14: cp->credits;          /* max 14 credits */
    cp->credits = cp->credits - cr;                     /* decr credits */
    cp->pak[pkt].d[UQ_HCTC] |= ((cr + 1) << UQ_HCTC_V_CR);
    }
if (Map_WriteW (addr + UQ_HDR_OFF, lnt, cp->pak[pkt].d))
    return rf_fatal (cp, PE_PWE);                       /* write pkt */
rf_enqh (cp, &cp->freq, pkt);                           /* pkt is free */
cp->pbsy = cp->pbsy - 1;                                /* decr busy cnt */
if (cp->pbsy == 0)                                      /* idle? strt hst tmr */
    cp->hat = cp->htmo;
return rf_putdesc (cp, &cp->rq, desc);                  /* release desc */
#endif
return OK;
}

/* Get a descriptor from the host */

#if 0
t_bool rf_getdesc (MSC *cp, struct uq_ring *ring, uint32 *desc)
{
uint32 addr = ring->ba + ring->idx;
uint16 d[2];

*desc = 0;
if (Map_ReadW (addr, 4, d))                             /* fetch desc */
    return rf_fatal (cp, PE_QRE);                       /* err? dead */
*desc = ((uint32) d[0]) | (((uint32) d[1]) << 16);
return OK;
}

/* Return a descriptor to the host, clearing owner bit
   If rings transitions from "empty" to "not empty" or "full" to
   "not full", and interrupt bit was set, interrupt the host.
   Actually, test whether previous ring entry was owned by host.
*/

t_bool rf_putdesc (MSC *cp, struct uq_ring *ring, uint32 desc)
{
uint32 prvd, newd = (desc & ~UQ_DESC_OWN) | UQ_DESC_F;
uint32 prva, addr = ring->ba + ring->idx;
uint16 d[2];

d[0] = newd & 0xFFFF;                                   /* 32b to 16b */
d[1] = (newd >> 16) & 0xFFFF;
if (Map_WriteW (addr, 4, d))                            /* store desc */
    return rf_fatal (cp, PE_QWE);                       /* err? dead */
if (desc & UQ_DESC_F) {                                 /* was F set? */
    if (ring->lnt <= 4)                                 /* lnt = 1? intr */
        rf_ring_int (cp, ring);
    else {                                              /* prv desc */
        prva = ring->ba + ((ring->idx - 4) & (ring->lnt - 1));
        if (Map_ReadW (prva, 4, d))                     /* read prv */
            return rf_fatal (cp, PE_QRE);
        prvd = ((uint32) d[0]) | (((uint32) d[1]) << 16);
        if (prvd & UQ_DESC_OWN)
            rf_ring_int (cp, ring);
        }
    }
ring->idx = (ring->idx + 4) & (ring->lnt - 1);
return OK;
}
#endif

/* Get unit descriptor for logical unit */

UNIT *rf_getucb (MSC *cp, uint16 lu)
{
DEVICE *dptr = rf_devmap[cp->cnum];
uint32 i;

for (i = 0; i < dptr->numunits - 2; i++)
    if ((lu == dptr->units[i].unit_plug) &&
        !(dptr->units[i].flags & UNIT_DIS))
        return &dptr->units[i];
return NULL;
}

/* Hack unit flags */

void rf_setf_unit (MSC *cp, uint16 pkt, UNIT *uptr)
{
uptr->uf = cp->pak[pkt].d[ONL_UFL] & UF_MSK;            /* settable flags */
if ((cp->pak[pkt].d[CMD_MOD] & MD_SWP) &&               /* swre wrp enb? */
    (cp->pak[pkt].d[ONL_UFL] & UF_WPS))                 /* swre wrp on? */
    uptr->uf = uptr->uf | UF_WPS;                       /* simon says... */
return;
}

/* Unit response fields */

void rf_putr_unit (MSC *cp, uint16 pkt, UNIT *uptr, uint16 lu, t_bool all)
{
uint32 dtyp = GET_DTYPE (uptr->flags);                  /* get drive type */
uint32 maxlbn = (uint32)uptr->capac;                    /* get max lbn */

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_putr_unit\n");

cp->pak[pkt].d[ONL_MLUN] = lu;                          /* unit */
cp->pak[pkt].d[ONL_UFL] = (uint16)(uptr->uf | UF_RPL | RF_WPH (uptr) | RF_RMV (uptr));
cp->pak[pkt].d[ONL_RSVL] = 0;                           /* reserved */
cp->pak[pkt].d[ONL_RSVH] = 0;
cp->pak[pkt].d[ONL_UIDA] = lu;                          /* UID low */
cp->pak[pkt].d[ONL_UIDB] = 0;
cp->pak[pkt].d[ONL_UIDC] = 0;
cp->pak[pkt].d[ONL_UIDD] = (UID_DISK << ONL_UIDD_V_CLS) |
    (drv_tab[dtyp].mod << ONL_UIDD_V_MOD);              /* UID hi */
PUTP32 (pkt, ONL_MEDL, drv_tab[dtyp].MediaId);          /* media type */
if (all) {                                              /* if long form */
    PUTP32 (pkt, ONL_SIZL, maxlbn);                     /* user LBNs */
    cp->pak[pkt].d[ONL_VSNL] = 01234 + lu;              /* vol serial # */
    cp->pak[pkt].d[ONL_VSNH] = 0;
    }
return;
}

/* UQ_HDR and RSP_OP fields */

void rf_putr (MSC *cp, uint16 pkt, uint16 cmd, uint16 flg,
          uint16 sts, uint16 lnt, uint16 typ)
{
cp->pak[pkt].d[RSP_OPF] = (cmd << RSP_OPF_V_OPC) |      /* set cmd, flg */
    (flg << RSP_OPF_V_FLG);
cp->pak[pkt].d[RSP_STS] = sts;
cp->pak[pkt].d[UQ_HLNT] = lnt;                          /* length */
cp->pak[pkt].d[UQ_HCTC] = (typ << UQ_HCTC_V_TYP) |      /* type, cid */
    (UQ_CID_MSCP << UQ_HCTC_V_CID);                     /* clr credits */
return;
}

/* Post interrupt during init */

#if 0
void rf_init_int (MSC *cp)
{
return;
}
#endif

/* Post interrupt during putpkt - note that NXMs are ignored! */

#if 0
void rf_ring_int (MSC *cp, struct uq_ring *ring)
{
return;
}
#endif

/* Fatal error */

t_bool rf_fatal (MSC *cp, uint16 err)
{
DEVICE *dptr = rf_devmap[cp->cnum];

sim_debug (DBG_TRC, rf_devmap[cp->cnum], "rf_fatal\n");

sim_debug (DBG_REQ, dptr, "fatal err=%X\n", err);
rf_reset (rf_devmap[cp->cnum]);                         /* reset device */
cp->sa = SA_ER | err;                                   /* SA = dead code */
cp->csta = CST_DEAD;                                    /* state = dead */
cp->perr = err;                                         /* save error */
return ERR;
}

/* Set/clear hardware write lock */

t_stat rf_set_wlk (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
uint32 dtyp = GET_DTYPE (uptr->flags);                  /* get drive type */

if (drv_tab[dtyp].flgs & RFDF_RO)                       /* not on read only */
    return SCPE_NOFNC;
return SCPE_OK;
}

/* Show write lock status */

t_stat rf_show_wlk (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
uint32 dtyp = GET_DTYPE (uptr->flags);                  /* get drive type */

if (drv_tab[dtyp].flgs & RFDF_RO)
    fprintf (st, "read only");
else if (uptr->flags & UNIT_WPRT)
    fprintf (st, "write locked");
else fprintf (st, "write enabled");
return SCPE_OK;
}

/* Set unit type (and capacity if user defined) */

t_stat rf_set_type (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
uint32 cap;
uint32 max = sim_toffset_64? RA8U_EMAXC: RA8U_MAXC;
t_stat r;

if ((val < 0) || ((val != RA8U_DTYPE) && cptr))
    return SCPE_ARG;
if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;
if (cptr) {
    cap = (uint32) get_uint (cptr, 10, 0xFFFFFFFF, &r);
    if ((sim_switches & SWMASK ('L')) == 0)
        cap = cap * ((sim_switches & SWMASK ('B')) ? 2048 : 1954);
    if ((r != SCPE_OK) || (cap < RA8U_MINC) || (cap > max))
        return SCPE_ARG;
    drv_tab[val].lbn = cap;
    }
uptr->flags = (uptr->flags & ~UNIT_DTYPE) | (val << UNIT_V_DTYPE);
uptr->capac = (t_addr)drv_tab[val].lbn;
return SCPE_OK;
}

/* Show unit plug */

t_stat rf_show_plug (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
fprintf (st, "UNIT=%d", uptr->unit_plug);
return SCPE_OK;
}

/* Set unit plug */

t_stat rf_set_plug (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
MSC *cp = rf_ctxmap[uptr->cnum];
int32 plug;
uint32 i;
t_stat r;
DEVICE *dptr = find_dev_from_unit (uptr);

if (cptr == NULL)
    return sim_messagef (SCPE_ARG, "Must specify UNIT=value\n");
plug = (int32) get_uint (cptr, 10, 0xFFFFFFFF, &r);
if ((r != SCPE_OK) || (plug > 65534))
    return sim_messagef (SCPE_ARG, "Invalid Unit Plug Number: %s\n", cptr);
if (uptr->unit_plug == plug)
    return SCPE_OK;
for (i=0; i < dptr->numunits - 2; i++)
    if (dptr->units[i].unit_plug == plug)
        return sim_messagef (SCPE_ARG, "Unit Plug %d Already In Use on %s\n", plug, sim_uname (&dptr->units[i]));
uptr->unit_plug = plug;
return SCPE_OK;
}

/* Show unit type */

t_stat rf_show_type (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
fprintf (st, "%s", drv_tab[GET_DTYPE (uptr->flags)].name);
return SCPE_OK;
}

/* Set controller type */

t_stat rf_set_ctype (UNIT *uptr, int32 val, CONST char *cptr, void *desc)
{
MSC *cp = rf_ctxmap[uptr->cnum];

if (val < 0)
    return SCPE_ARG;
cp->ctype = val;
return SCPE_OK;
}

/* Show controller type */

t_stat rf_show_ctype (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
MSC *cp = rf_ctxmap[uptr->cnum];
fprintf (st, "%s", ctlr_tab[cp->ctype].name);
return SCPE_OK;
}

/* Device attach */

t_stat rf_attach (UNIT *uptr, CONST char *cptr)
{
MSC *cp = rf_ctxmap[uptr->cnum];
t_stat r;

r = sim_disk_attach (uptr, cptr, RF_NUMBY, sizeof (uint16), (uptr->flags & UNIT_NOAUTO), DBG_DSK, drv_tab[GET_DTYPE (uptr->flags)].name, 0, 0);
if (r != SCPE_OK)
    return r;

if ((cp->csta == CST_UP) && sim_disk_isavailable (uptr))
    uptr->flags = uptr->flags | UNIT_ATP;
return SCPE_OK;
}

/* Device detach */

t_stat rf_detach (UNIT *uptr)
{
t_stat r;

r = sim_disk_detach (uptr);                             /* detach unit */
if (r != SCPE_OK)
    return r;
uptr->flags = uptr->flags & ~(UNIT_ONL | UNIT_ATP);     /* clr onl, atn pend */
uptr->uf = 0;                                           /* clr unit flgs */
return SCPE_OK;
} 

/* Device reset */

t_stat rf_reset (DEVICE *dptr)
{
int32 i, j, cidx;
UNIT *uptr;
MSC *cp;
static t_bool plugs_inited = FALSE;
#if 0
DIB *dibp = (DIB *) dptr->ctxt;
#endif

sim_debug (DBG_TRC, dptr, "rf_reset\n");

for (i = 0, cidx = -1; i < RF_NUMCT; i++) {             /* find ctrl num */
    if (rf_devmap[i] == dptr)
        cidx = i;
    }
if (cidx < 0)                                           /* not found??? */
    return SCPE_IERR;
cp = rf_ctxmap[cidx];                                   /* get context */
cp->cnum = cidx;                                        /* init index */
if (cp->ctype == DEFAULT_CTYPE)
    cp->ctype = (UNIBUS? UDA50_CTYPE : RQDX3_CTYPE);

if (!plugs_inited ) {
    uint32 d, u = 0;
    char uname[16];

    sprintf (uname, "%s-TIMER", dptr->name);
    sim_set_uname (&dptr->units[4], uname);
    sprintf (uname, "%s-QUESVC", dptr->name);
    sim_set_uname (&dptr->units[5], uname);
    plugs_inited  = TRUE;
    for (i = 0; i < RF_NUMCT; i++) {
        for (d = 0; d < rf_devmap[i]->numunits - 2; d++) {
            rf_devmap[i]->units[d].unit_plug = 
#if defined (VM_VAX)
                d;          /* VAX default units */
#else           
                u;          /* PDP11 unique unit numbers */
#endif
            ++u;
            }
        }
    }

cp->csta = CST_S1;                                      /* init stage 1 */
cp->s1dat = 0;                                          /* no S1 data */
#if 0
dibp->vec = 0;                                          /* no vector */
#endif
cp->comm = 0;                                           /* no comm region */
if (UNIBUS)                                             /* Unibus? */
    cp->sa = SA_S1 | SA_S1C_DI | SA_S1C_MP;
else cp->sa = SA_S1 | SA_S1C_Q22 | SA_S1C_DI | SA_S1C_MP; /* init SA val */
cp->cflgs = CF_RPL;                                     /* ctrl flgs off */
cp->htmo = RF_DHTMO;                                    /* default timeout */
cp->hat = cp->htmo;                                     /* default timer */
cp->cq.ba = cp->cq.lnt = cp->cq.idx = 0;                /* clr cmd ring */
cp->rq.ba = cp->rq.lnt = cp->rq.idx = 0;                /* clr rsp ring */
cp->credits = (RF_NPKTS / 2) - 1;                       /* init credits */
cp->freq = 1;                                           /* init free list */
for (i = 0; i < RF_NPKTS; i++) {                        /* all pkts free */
    if (i)
        cp->pak[i].link = (i + 1) & RF_M_NPKTS;
    else cp->pak[i].link = 0;
    for (j = 0; j < RF_PKT_SIZE_W; j++)
        cp->pak[i].d[j] = 0;
    }
cp->rspq = 0;                                           /* no q'd rsp pkts */
cp->pbsy = 0;                                           /* all pkts free */
cp->pip = 0;                                            /* not polling */
for (i = 0; i < (RF_NUMDR + 2); i++) {                  /* init units */
    uptr = dptr->units + i;
    sim_cancel (uptr);                                  /* clr activity */
    sim_disk_reset (uptr);
    uptr->cnum = cidx;                                  /* set ctrl index */
    uptr->flags = uptr->flags & ~(UNIT_ONL | UNIT_ATP);
    uptr->uf = 0;                                       /* clr unit flags */
    uptr->cpkt = uptr->pktq = 0;                        /* clr pkt q's */
    uptr->rfxb = (uint16 *) realloc (uptr->rfxb, (RF_MAXFR >> 1) * sizeof (uint16));
    if (uptr->rfxb == NULL)
        return SCPE_MEM;
    }
for (i=cp->max_plug=0; i<RF_NUMDR; i++)
    if (dptr->units[i].unit_plug > cp->max_plug)
        cp->max_plug = (uint16)dptr->units[i].unit_plug;
return auto_config (0, 0);                              /* run autoconfig */
}

void rf_show_pkt (FILE *st, MSC *cp, int32 pkt)
{
int32 i, j;

fprintf (st, "packet %d\n", pkt);
for (i = 0; i < RF_SH_MAX; i = i + RF_SH_PPL) {
    fprintf (st, " %2d:", i);
    for (j = i; j < (i + RF_SH_PPL); j++)
#if defined (VM_PDP11)
    fprintf (st, " %06o", cp->pak[pkt].d[j]);
#else
    fprintf (st, " %04x", cp->pak[pkt].d[j]);
#endif
    fprintf (st, "\n");
    }
return;
}

t_stat rf_show_unitq (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
MSC *cp = rf_ctxmap[uptr->cnum];
DEVICE *dptr = rf_devmap[uptr->cnum];
int32 pkt, u;

u = (int32) (uptr - dptr->units);
if (cp->csta != CST_UP) {
    fprintf (st, "Controller is not initialized\n");
    return SCPE_OK;
    }
if ((uptr->flags & UNIT_ONL) == 0) {
    if (uptr->flags & UNIT_ATT)
        fprintf (st, "Unit %d is available\n", u);
    else fprintf (st, "Unit %d is offline\n", u);
    return SCPE_OK;
    }
if (uptr->cpkt) {
    fprintf (st, "Unit %d current ", u);
    rf_show_pkt (st, cp, uptr->cpkt);
    if ((pkt = uptr->pktq)) {
        do {
            fprintf (st, "Unit %d queued ", u);
            rf_show_pkt (st, cp, pkt);
            } while ((pkt = cp->pak[pkt].link));
        }
    }
else fprintf (st, "Unit %d queues are empty\n", u);
return SCPE_OK;
}

t_stat rf_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr)
{
fprintf (st, "UDA50 MSCP Disk Controller (%s)\n\n", dptr->name);
fprintf (st, "The simulator implements four MSCP disk controllers, RQ, RQB, RQC, RQD.\n");
fprintf (st, "Initially, RQB, RQC, and RQD are disabled.  Each RQ controller simulates\n");
fprintf (st, "an MSCP disk controller with four drives.  The MSCP controller type can be\n");
fprintf (st, "specified as one of RQDX3, UDA50, KDA50, KRQ50, KLESI or RUX50.  RQ options\n");
fprintf (st, "include the ability to set units write enabled or write locked, and to set\n");
fprintf (st, "the drive type to one of many disk types:\n");
fprint_set_help (st, dptr);
fprintf (st, "set RQn RAUSER{=n}        Set disk type to RA82 with n MB's\n");
fprintf (st, "                          (1MB is 1000000 bytes)\n");
fprintf (st, "set -L RQn RAUSER{=n}     Set disk type to RA82 with n LBN's\n\n");
fprintf (st, "set -B RQn RAUSER{=n}     Set disk type to RA82 with n MB's\n");
fprintf (st, "                          (1MB is 2048 512 byte sectors)\n\n");
fprintf (st, "The type options can be used only when a unit is not attached to a file.\n");
fprintf (st, "RAUSER is a \"user specified\" disk; the user can specify the size of the\n");
fprintf (st, "disk in either MB (1000000 bytes) or logical block numbers (LBN's, 512 bytes\n");
fprintf (st, "each), or binary MB (1024*1024 bytes).  The minimum size is 5MB; the maximum\n");
fprintf (st, "size is 2GB without extended file support, 1TB with extended file support.\n\n");
fprintf (st, "The %s controllers support the BOOT command.\n\n", dptr->name);
fprint_show_help (st, dptr);
fprint_reg_help (st, dptr);
fprintf (st, "\nWhile VMS is not timing sensitive, most of the BSD-derived operating systems\n");
fprintf (st, "(NetBSD, OpenBSD, etc) are.  The QTIME and XTIME parameters are set to values\n");
fprintf (st, "that allow these operating systems to run correctly.\n\n");
fprintf (st, "\nError handling is as follows:\n\n");
fprintf (st, "    error         processed as\n");
fprintf (st, "    not attached  disk not ready\n");
fprintf (st, "    end of file   assume rest of disk is zero\n");
fprintf (st, "    OS I/O error  report error and stop\n");
fprintf (st, "\nDisk drives on the %s device can be attacbed to simulated storage in the\n", dptr->name);
fprintf (st, "following ways:\n\n");
sim_disk_attach_help (st, dptr, uptr, flag, cptr);
return SCPE_OK;
}

const char *rf_description (DEVICE *dptr)
{
static char buf[80];

sprintf (buf, "%s MSCP disk controller", ctlr_tab[rf_ctxmap[dptr->units->cnum]->ctype].name);
return buf;
}
