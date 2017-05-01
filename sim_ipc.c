/* sim_ipc.c: Inter-process Communication

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

#include "sim_ipc.h"

#define IPC_MAXNODES    32                              /* max nodes */
#define IPC_OSDF_OF     0                               /* O/S defined offset */
#define IPC_OSDF_LN     1                               /* O/S defined length */
#define IPC_SIZE_OF     1                               /* mem size offset */
#define IPC_SIZE_LN     1                               /* mem size length */
#define IPC_IBLK_OF     2                               /* interrupt block offset */
#define IPC_IBLK_LN     IPC_MAXNODES                    /* interrupt block length */
#define IPC_DATA_LEN    (IPC_OSDF_LN + IPC_SIZE_LN + \
                         IPC_IBLK_LN)                   /* internal data length */

#if defined (WIN32)

#include <windows.h>

typedef struct {
    HANDLE file;
    LPVOID buf;
    HANDLE mutex;
} IPC_CTX;

/* Lock the shared memory for exclusive access */

t_stat ipc_lock (IPC_CTX *ctx)
{
DWORD result;

if (ctx->mutex == NULL)
    return SCPE_OK;

result = WaitForSingleObject (
            ctx->mutex,
            INFINITE);

if (result != WAIT_OBJECT_0)
    return sim_messagef (SCPE_IERR, "Error acquiring exclusive lock\n");
return SCPE_OK;
}

/* Release the shared memory lock */

t_stat ipc_unlock (IPC_CTX *ctx)
{
BOOL result;

if (ctx->mutex == NULL)
    return SCPE_OK;

result = ReleaseMutex (ctx->mutex);

if (result != TRUE)
    return sim_messagef (SCPE_IERR, "Error releasing exclusive lock\n");
return SCPE_OK;
}

/* Create or map a shared memory section */

t_stat ipc_map (IPC_CTX *ctx, CONST char *name, size_t len, uint32 **buf, t_bool *new)
{
char mname[32];
char sname[32];
t_stat r;

sprintf (&mname[0], "SimhLock%s\n", name);
sprintf (&sname[0], "SimhMem%s\n", name);

*new = FALSE;                                           /* assume existing section */

ctx->mutex = CreateMutex (
                 NULL,                                  /* default security */
                 FALSE,                                 /* not owned */
                 &mname[0]);                            /* mutex name */

if (ctx->mutex == NULL)                                 /* error? */
    return SCPE_MEM;

r = ipc_lock (ctx);                                     /* get exclusive lock */
if (r != SCPE_OK)
    return r;

ctx->file = OpenFileMapping (                           /* try to map to existing section */
                FILE_MAP_ALL_ACCESS,                    /* desired access */
                FALSE,                                  /* inherit handle */
                &sname[0]);                             /* section name */

if (ctx->file == NULL) {                                /* error? */
    ctx->file = CreateFileMapping (                     /* try to create new section */
                    INVALID_HANDLE_VALUE,               /* not file backed */
                    NULL,                               /* default security */
                    PAGE_READWRITE,                     /* protection */
                    0,                                  /* size high */
                    len,                                /* size low */
                    &sname[0]);                         /* section name */
    *new = TRUE;
    }

if (ctx->file == NULL) {                                /* error? */
    ipc_unlock (ctx);
    CloseHandle (ctx->mutex);
    return SCPE_MEM;
    }

ctx->buf = MapViewOfFile (                              /* map section */
               ctx->file,                               /* file object */
               FILE_MAP_ALL_ACCESS,                     /* desired access */
               0,                                       /* offset high */
               0,                                       /* offset low */
               0);                                      /* bytes to map (all) */

if (ctx->buf == NULL) {                                 /* error? */
    ipc_unlock (ctx);
    CloseHandle (ctx->file);
    CloseHandle (ctx->mutex);
    free (ctx);
    return SCPE_MEM;
    }

*buf = (uint32 *)ctx->buf;

if (*new)                                               /* new section? */
    memset (*buf, 0, len);                              /* clear */

r = ipc_unlock (ctx);                                   /* release exclusive lock */
if (r != SCPE_OK)
    return r;

return SCPE_OK;
}

/* Unmap (and possibly deallocate) a shared memory section */

t_stat ipc_unmap (IPC_CTX *ctx)
{
UnmapViewOfFile (ctx->buf);
ctx->buf = NULL;
CloseHandle (ctx->file);
ctx->file = NULL;
CloseHandle (ctx->mutex);
ctx->mutex = NULL;
return SCPE_OK;
}

#elif defined (VMS)

#include <descrip.h>
#include <efndef.h>
#include <lckdef.h>
#include <lksbdef.h>
#include <psldef.h>
#include <secdef.h>
#include <ssdef.h>
#include <starlet.h>

typedef struct {
    unsigned int start;
    unsigned int end;
} VA_RANGE;

typedef struct {
    VA_RANGE retadr;                                    /* section virt addr range */
    char lock[32];                                      /* lock name */
    struct dsc$descriptor_s lock_d;                     /* lock descriptor */
    lksb lksb;                                          /* lock status block */
} IPC_CTX;

/* Lock the shared memory for exclusive access */

t_stat ipc_lock (IPC_CTX *ctx)
{
unsigned int status;

status = sys$enqw (                                     /* convert to exclusive lock */
             EFN$C_ENF,                                 /* event flag */
             LCK$K_EXMODE,                              /* lock mode */
             &ctx->lksb,                                /* lock status block */
             LCK$M_CONVERT,                             /* flags */
             0,                                         /* resource name */
             0,                                         /* parent lock */
             0,                                         /* AST address */
             0,                                         /* AST param */
             0,                                         /* blocking AST */
             0,                                         /* access mode */
             0,                                         /* resource domain */
             0);                                        /* null arg */

if (status != SS$_NORMAL)
    return sim_messagef (SCPE_IERR, "Error acquiring exclusive lock\n");
return SCPE_OK;
}

/* Release the shared memory lock */

t_stat ipc_unlock (IPC_CTX *ctx)
{
unsigned int status;

status = sys$enqw (                                     /* convert to NULL lock */
             EFN$C_ENF,                                 /* event flag */
             LCK$K_NLMODE,                              /* lock mode */
             &ctx->lksb,                                /* lock status block */
             LCK$M_CONVERT,                             /* flags */
             0,                                         /* resource name */
             0,                                         /* parent lock */
             0,                                         /* AST address */
             0,                                         /* AST param */
             0,                                         /* blocking AST */
             0,                                         /* access mode */
             0,                                         /* resource domain */
             0);                                        /* null arg */

if (status != SS$_NORMAL)
    sim_messagef (SCPE_IERR, "Error releasing exclusive lock\n");
return SCPE_OK;
}

/* Create or map a shared memory section */

t_stat ipc_map (IPC_CTX *ctx, CONST char *name, size_t len, uint32 **buf, t_bool *new)
{
char *lname = strdup (name);                            /* local copy due to CONST */
$DESCRIPTOR (gsdnam, lname);
unsigned int status;
unsigned int pagcnt = ((len + 511) >> 9);               /* number of pagelets */
unsigned short prot = 0xFFCC;                           /* (S:RW, O:RW, G:, W:) */
VA_RANGE inadr = { 0, 0 };
t_stat r;

sprintf (&ctx->lock[0], "SimhLock%s\n", name);          /* setup resource name */
ctx->lock_d.dsc$w_length = strlen (ctx->lock);
ctx->lock_d.dsc$a_pointer = (void *)&ctx->lock;

status = sys$enqw (                                     /* create NULL lock */
             EFN$C_ENF,                                 /* event flag */
             LCK$K_NLMODE,                              /* lock mode */
             &ctx->lksb,                                /* lock status block */
             0,                                         /* flags */
             &ctx->lock_d,                              /* resource name */
             0,                                         /* parent lock */
             0,                                         /* AST address */
             0,                                         /* AST param */
             0,                                         /* blocking AST */
             0,                                         /* access mode */
             0,                                         /* resource domain */
             0);                                        /* null arg */

if (status != SS$_NORMAL)
    return sim_messagef (SCPE_IERR, "Unable to create lock\n");

*new = FALSE;                                           /* assume existing section */

r = ipc_lock (ctx);                                     /* get exclusive lock */
if (r != SCPE_OK)
    return r;

status = sys$mgblsc (                                   /* try to map section */
             &inadr,                                    /* input addr */
             &ctx->retadr,                              /* return addr */
             PSL$C_USER,                                /* access mode */
             SEC$M_EXPREG|SEC$M_WRT,                    /* flags */
             &gsdnam,                                   /* global section name */
             NULL,                                      /* ident */
             0);                                        /* relative page */

if (status != SS$_NORMAL) {                             /* may not exist */
    status = sys$crmpsc (                               /* try to create it */
                 &inadr,                                /* input addr */
                 &ctx->retadr,                          /* return addr */
                 PSL$C_USER,                            /* access mode */
                 SEC$M_EXPREG|SEC$M_GBL|SEC$M_PAGFIL|SEC$M_WRT, /* flags */
                 &gsdnam,                               /* global section name */
                 NULL,                                  /* ident */
                 0,                                     /* relative page */
                 0,                                     /* channel */
                 pagcnt,                                /* number of pagelets */
                 0,                                     /* virtual block number */
                 prot,                                  /* protection mask */
                 0);                                    /* page fault cluster */

    if (status == SS$_CREATED)                          /* success? */
        *new = TRUE;
    else if (status != SS$_NORMAL) {
        free (lname);
        return sim_messagef (SCPE_MEM, "Unable to create/map shared memory\n");
        }
    }

*buf = (uint32 *)ctx->retadr.start;

if (*new)                                               /* new section? */
    memset (*buf, 0, len);                              /* clear */

r = ipc_unlock (ctx);                                   /* release exclusive lock */
if (r != SCPE_OK)
    return r;

return SCPE_OK;
}

/* Unmap (and possibly deallocate) a shared memory section */

t_stat ipc_unmap (IPC_CTX *ctx)
{
unsigned int status;

status = sys$deltva (                                   /* unmap pages */
             &ctx->retadr,                              /* input addr */
             NULL,                                      /* return addr */
             0);                                        /* access mode */
if (status != SS$_NORMAL)
    return sim_messagef (SCPE_IERR, "Error unmapping shared memory\n");

status = sys$deq (                                      /* delete NULL lock */
             ctx->lksb.lksb$l_lkid,                     /* lock id */
             0,                                         /* value block */
             0,                                         /* access mode */
             0);                                        /* flags */
if (status != SS$_NORMAL)
    return sim_messagef (SCPE_IERR, "Error de-queueing lock\n");

return SCPE_OK;
}

#elif defined(__unix__)

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>

typedef struct {
    int file;
    void *buf;
    size_t len;
    sem_t *mutex;
    char sname[32];
    char mname[32];
} IPC_CTX;

/* Lock the shared memory for exclusive access */

t_stat ipc_lock (IPC_CTX *ctx)
{
if (sem_wait (ctx->mutex) < 0)
    return sim_messagef (SCPE_IERR, "Error acquiring exclusive lock\n");
return SCPE_OK;
}

/* Release the shared memory lock */

t_stat ipc_unlock (IPC_CTX *ctx)
{
if (sem_post (ctx->mutex) < 0)
    return sim_messagef (SCPE_IERR, "Error releasing exclusive lock\n");
return SCPE_OK;
}

/* Create or map a shared memory section */

t_stat ipc_map (IPC_CTX *ctx, CONST char *name, size_t len, uint32 **buf, t_bool *new)
{
struct stat st;
t_stat r;

sprintf (&ctx->sname[0], "/SimhMem%s", name);
sprintf (&ctx->mname[0], "/SimhLock%s", name);
ctx->len = len;

ctx->mutex = sem_open (&ctx->mname[0], O_CREAT, S_IRUSR | S_IWUSR, 1);
if (ctx->mutex == SEM_FAILED)
    return sim_messagef (SCPE_MEM, "Unable to create semaphore\n");

r = ipc_lock (ctx);                                     /* get exclusive lock */
if (r != SCPE_OK)
    return r;

ctx->file = shm_open (                                  /* try create new section */
                &ctx->sname[0],                         /* name */
                O_RDWR | O_CREAT | O_EXCL,              /* flags */
                S_IRUSR | S_IWUSR);                     /* mode */
if (ctx->file < 0) {                                    /* may already exist */
    if (errno == EEXIST) {
        ctx->file = shm_open (                          /* open existing section */
                        &ctx->sname[0],                 /* name */
                        O_RDWR,                         /* flags */
                        S_IRUSR | S_IWUSR);             /* mode */
        if (ctx->file < 0)
            return sim_messagef (SCPE_MEM, "Unable to create/open shared memory\n");
        else
            *new = FALSE;
        }
    else
        return sim_messagef (SCPE_MEM, "Unable to create/open shared memory\n");
    }
else
    *new = TRUE;

if (*new) {
    if (ftruncate (ctx->file, ctx->len) < 0) {          /* set size */
        ipc_unlock (ctx);
        shm_unlink (ctx->sname);
        sem_close (ctx->mutex);
        sem_unlink (ctx->mname);
        return sim_messagef (SCPE_MEM, "Error setting shared memory size\n");
        }
    }
else {
    if (fstat (ctx->file, &st) < 0) {                   /* get size */
        ipc_unlock (ctx);
        sem_close (ctx->mutex);
        return sim_messagef (SCPE_IERR, "Error getting shared memory size\n");
        }
    ctx->len = st.st_size;
    }

ctx->buf = mmap (                                       /* map memory */
               NULL,
               ctx->len,
               PROT_READ | PROT_WRITE,
               MAP_SHARED,
               ctx->file,
               0);
if (ctx->buf == MAP_FAILED) {
    ipc_unlock (ctx);
    sem_close (ctx->mutex);
    if (*new) {
        shm_unlink (ctx->sname);
        sem_unlink (ctx->mname);
        }
    return sim_messagef (SCPE_MEM, "Unable to map shared memory\n");
    }

*buf = (uint32 *)ctx->buf;

if (*new)
    memset (*buf, 0, ctx->len);

(*buf)[IPC_OSDF_OF]++;                                  /* increment reference count */

r = ipc_unlock (ctx);                                   /* release exclusive lock */
if (r != SCPE_OK)
    return r;

return SCPE_OK;
}

/* Unmap (and possibly deallocate) a shared memory section */

t_stat ipc_unmap (IPC_CTX *ctx)
{
uint32 *buf;
t_bool last = FALSE;
t_stat r;

buf = (uint32 *)ctx->buf;
if (buf == NULL)
    return sim_messagef (SCPE_IERR, "Shared memory buffer is NULL\n");

r = ipc_lock (ctx);                                     /* get exclusive lock */
if (r != SCPE_OK)
    return r;

buf[IPC_OSDF_OF]--;                                     /* decrement reference count */
if (buf[IPC_OSDF_OF] == 0)
    last = TRUE;

r = ipc_unlock (ctx);                                   /* release exclusive lock */
if (r != SCPE_OK)
    return r;

if (munmap (ctx->buf, ctx->len) < 0)
    return sim_messagef (SCPE_IERR, "Error un-mapping shared memory\n");
if (sem_close (ctx->mutex) < 0)
    return sim_messagef (SCPE_IERR, "Error closing semaphore\n");

if (last) {
    if (shm_unlink (ctx->sname) < 0)
        return sim_messagef (SCPE_IERR, "Error deleting shared memory\n");
    if (sem_unlink (ctx->mname) < 0)
        return sim_messagef (SCPE_IERR, "Error deleting semaphore\n");
    }
return SCPE_OK;
}

#else

/* Non-implemented versions */

typedef uint32 IPC_CTX;

t_stat ipc_lock (IPC_CTX *ctx)
{
return SCPE_NOFNC;
}

t_stat ipc_unlock (IPC_CTX *ctx)
{
return SCPE_NOFNC;
}

t_stat ipc_map (IPC_CTX *ctx, CONST char *name, size_t len, uint32 **buf, t_bool *new)
{
*buf = NULL;
return SCPE_NOFNC;
}

t_stat ipc_unmap (IPC_CTX *ctx)
{
return SCPE_NOFNC;
}

#endif

/* Attach to a shared memory section */

t_stat ipc_attach (UNIT *uptr, CONST char *cptr)
{
IPC_CTX *ctx;
size_t len;
uint32 *buf = NULL;
t_bool new;
t_stat r;

if (uptr->flags & UNIT_ATT)
    return SCPE_ALATT;

ctx = (IPC_CTX *) calloc (1, sizeof (IPC_CTX));         /* alloc context */
if (ctx == NULL)
    return SCPE_MEM;

len = (size_t)uptr->capac;                              /* size to allocate */
len = len + (IPC_DATA_LEN << 2);                        /* plus internal data */

r = ipc_map (ctx, cptr, len, &buf, &new);
if (r != SCPE_OK)
    return r;

if (new)
    buf[IPC_SIZE_OF] = (len - (IPC_DATA_LEN << 2));     /* save section size */
uptr->filename = (char *) calloc (CBUFSIZE, sizeof (char));/* alloc name buf */
if (uptr->filename == NULL){
    ipc_unmap (ctx);
    return SCPE_MEM;
    }
strncpy (uptr->filename, cptr, CBUFSIZE);               /* save name */
uptr->up8 = (void *)ctx;                                /* save context */
uptr->capac = (buf[IPC_SIZE_OF] - (IPC_DATA_LEN << 2)); /* update unit size */

uptr->filebuf = (void *)(buf + IPC_DATA_LEN);
return SCPE_OK;
}

/* Detach shared memory section */

t_stat ipc_detach (UNIT *uptr)
{
IPC_CTX *ctx;

if ((uptr->flags & UNIT_ATT) && (uptr->filebuf != NULL)) {
    ctx = (IPC_CTX *)uptr->up8;
    ipc_unmap (ctx);
    uptr->filebuf = NULL;
    free (uptr->filename);
    uptr->filename = NULL;
    free (ctx);
    uptr->up8 = NULL;
    }
return SCPE_OK;
}

/* Associate a unit with a shared memory node */

t_stat ipc_set_node (UNIT *uptr, uint32 node)
{
if (node > IPC_MAXNODES)                                /* node in range? */
    return SCPE_ARG;

uptr->pos = node;
return SCPE_OK;
}

/* Send an interrupt to another node */

t_stat ipc_send_int (UNIT *uptr, uint32 rnode)
{
IPC_CTX *ctx;
uint32 *buf;
t_stat r;

if ((uptr->flags & UNIT_ATT) == 0)                      /* attached? */
    return SCPE_OK;
if (uptr->filebuf == NULL)                              /* setup? */
    return SCPE_OK;
if (rnode > IPC_MAXNODES)                               /* node in range? */
    return SCPE_OK;

ctx = (IPC_CTX *)uptr->up8;
buf = (uint32 *)uptr->filebuf;                          /* get shared memory */
buf = buf - IPC_DATA_LEN + IPC_IBLK_OF;                 /* get interrupt block */
r = ipc_lock (ctx);                                     /* get exclusive lock */
if (r != SCPE_OK)                                       /* error? */
    return r;
buf[rnode] |= (1u << uptr->pos);                        /* send interrupt */
return ipc_unlock (ctx);                                /* release lock */
}

/* Check for interrupts from other nodes */

t_bool ipc_poll_int (UNIT *uptr)
{
uint32 *buf;

if ((uptr->flags & UNIT_ATT) == 0)                      /* attached */
    return FALSE;
if (uptr->filebuf == NULL)                              /* setup? */
    return FALSE;

buf = (uint32 *)uptr->filebuf;                          /* get shared memory */
buf = buf - IPC_DATA_LEN + IPC_IBLK_OF;                 /* get interrupt block */
if (buf[uptr->pos]) {                                   /* interrupt? */
    uptr->buf = buf[uptr->pos];                         /* save in unit buffer */
    buf[uptr->pos] = 0;                                 /* acknowledge interrupt */
    return TRUE;
    }
return FALSE;
}
