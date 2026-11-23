/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
/* In-memory XAR style override hook for ARM QNX NavigationBasic.
 *
 * NavigationBasic's StyleDB-Provider loads stock GvID5VBU1025 Day and Night
 * XAR files into its RW shared-memory pool (0x28000000..0x2c000000), which is
 * shared directly with untouched NavigationFully.
 *
 * Zero OEM files are modified on disk:
 *   1. semantic_xar_patch_shm_in_place() safely scans NavigationBasic's SHM
 *      pool via /proc/<pid>/as pread (never faulting on unmapped pages),
 *      validates all 5 stock row signatures, and patches the 84 override bytes
 *      directly in RAM for Day and Night XAR buffers loaded before OnLoop.
 *   2. semantic_xar_install_hooks() also redirects NavigationBasic's writable
 *      .got.plt entries for open/open64/fopen for any XAR opened lazily later. */

#include "semantic_xar_hook.h"
#include "semantic_xar_patch.h"

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#if !defined(__QNXNTO__) || !defined(__arm__)
#error semantic_xar_hook.c targets ARM QNX NavigationBasic
#endif

typedef struct {
    uint8_t  e_ident[16];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint32_t e_entry;
    uint32_t e_phoff;
    uint32_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;
} elf32_ehdr_t;

typedef struct {
    uint32_t p_type;
    uint32_t p_offset;
    uint32_t p_vaddr;
    uint32_t p_paddr;
    uint32_t p_filesz;
    uint32_t p_memsz;
    uint32_t p_flags;
    uint32_t p_align;
} elf32_phdr_t;

typedef struct {
    int32_t  d_tag;
    uint32_t d_val;
} elf32_dyn_t;

typedef struct {
    uint32_t r_offset;
    uint32_t r_info;
} elf32_rel_t;

typedef struct {
    uint32_t st_name;
    uint32_t st_value;
    uint32_t st_size;
    uint8_t  st_info;
    uint8_t  st_other;
    uint16_t st_shndx;
} elf32_sym_t;

enum {
    ELF_PT_DYNAMIC      = 2,
    ELF_DT_NULL         = 0,
    ELF_DT_PLTRELSZ     = 2,
    ELF_DT_STRTAB       = 5,
    ELF_DT_SYMTAB       = 6,
    ELF_DT_JMPREL       = 23,
    ELF_R_ARM_JUMP_SLOT = 22,
    NAV_BASIC_BASE      = 0x00100000u,
    XAR_MAX_FILE_BYTES  = 512u * 1024u,
    XAR_STOCK_BYTES     = 146828u,
    SHM_SCAN_BEGIN      = 0x28000000u,
    SHM_SCAN_END        = 0x2c000000u,
    SHM_CHUNK_BYTES     = 65536u
};

typedef FILE *(*fopen_fn_t)(const char *, const char *);
typedef int   (*open_fn_t)(const char *, int, ...);

static fopen_fn_t real_fopen;
static open_fn_t  real_open;
static open_fn_t  real_open64;
static pthread_mutex_t g_stage_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile int g_hooks_installed = 0;
static unsigned g_staged_mask = 0;

static int page_mapped(int fd, uint32_t page_addr, uint8_t buf[4096]) {
    if (lseek(fd, (off_t)page_addr, SEEK_SET) != (off_t)page_addr)
        return 0;
    return read(fd, buf, 4096u) == 4096;
}

static int range_mapped(int fd, uint32_t start, size_t len) {
    uint8_t tmp[4];
    for (uint32_t pg = start & ~4095u; pg < start + (uint32_t)len; pg += 4096u) {
        if (lseek(fd, (off_t)pg, SEEK_SET) != (off_t)pg)
            return 0;
        if (read(fd, tmp, 4u) != 4)
            return 0;
    }
    return 1;
}

unsigned semantic_xar_patch_shm_in_place(int as_fd) {
    int own_fd = -1;
    if (as_fd < 0) {
        char path[48];
        snprintf(path, sizeof(path), "/proc/%d/as", (int)getpid());
        own_fd = real_open ? real_open(path, O_RDONLY, 0) : open(path, O_RDONLY);
        if (own_fd < 0)
            return 0;
        as_fd = own_fd;
    }

    uint8_t *page = (uint8_t *)malloc(4096u);
    uint8_t *xar_copy = (uint8_t *)malloc(XAR_STOCK_BYTES);
    if (!page || !xar_copy) {
        free(page);
        free(xar_copy);
        if (own_fd >= 0) close(own_fd);
        return 0;
    }

    unsigned total_ready = 0;
    unsigned mapped_pages = 0;
    unsigned magic_hits = 0;
    uint32_t addr = SHM_SCAN_BEGIN;
    while (addr < SHM_SCAN_END && total_ready < 2u) {
        if (!page_mapped(as_fd, addr, page)) {
            /* Skip ahead 64KB across unmapped holes (SHM style pools are >=1MB aligned). */
            addr = (addr + 65536u) & ~65535u;
            continue;
        }
        ++mapped_pages;

        for (size_t off = 0; off + 4u <= 4096u && total_ready < 2u; off += 4u) {
            uint32_t w0;
            memcpy(&w0, page + off, 4u);
            if (w0 != 0x00786172u) /* "rax\0" */
                continue;
            if (off + 8u <= 4096u) {
                uint32_t w1;
                memcpy(&w1, page + off + 4u, 4u);
                if (w1 != 0x00010000u)
                    continue;
            }

            uint32_t cand = addr + (uint32_t)off;
            if (!range_mapped(as_fd, cand, XAR_STOCK_BYTES))
                continue;
            memcpy(xar_copy, (const void *)(uintptr_t)cand, XAR_STOCK_BYTES);
            if (memcmp(xar_copy, "rax\0\0\0\1\0", 8) != 0)
                continue;
            ++magic_hits;

            /* Validate on the copy first so we never touch non-matching memory. */
            int rc = semantic_xar_patch_buffer(xar_copy, XAR_STOCK_BYTES, SEMANTIC_XAR_FLAG_ALL);
            if (rc == SEMANTIC_XAR_ALREADY_PATCHED) {
                fprintf(stderr, "SEMANTIC_XAR live SHM XAR @ %#x already patched in RAM\n", cand);
                ++total_ready;
                continue;
            }
            if (rc == SEMANTIC_XAR_PATCHED) {
                uint8_t *live = (uint8_t *)(uintptr_t)cand;
                memcpy(live + 36720u, xar_copy + 36720u, 4u);
                memcpy(live + 84436u, xar_copy + 84436u, 40u);
                memcpy(live + 84496u, xar_copy + 84496u, 20u);
                memcpy(live + 84516u, xar_copy + 84516u, 20u);
                __sync_synchronize();
                fprintf(stderr, "SEMANTIC_XAR patched live SHM XAR @ %#x (84 bytes in RAM, zero disk edits)\n",
                        cand);
                ++total_ready;
            }
        }
        addr += 4096u;
    }

    fprintf(stderr, "SEMANTIC_XAR scan summary mapped_pages=%u magic_hits=%u ready=%u\n",
            mapped_pages, magic_hits, total_ready);

    free(page);
    free(xar_copy);
    if (own_fd >= 0) close(own_fd);
    return total_ready;
}

static unsigned target_mask_bit(const char *shm_path) {
    if (strstr(shm_path, "night") != NULL)
        return 2u;
    return 1u;
}

static int stage_patched_xar_in_shmem(const char *oem_path, const char *shm_path) {
    if (!real_open)
        return 0;

    unsigned bit = target_mask_bit(shm_path);
    struct stat existing;
    if ((g_staged_mask & bit) != 0u &&
        stat(shm_path, &existing) == 0 && existing.st_size > 0) {
        return 1;
    }

    int fd = real_open(oem_path, O_RDONLY, 0);
    if (fd < 0) {
        fprintf(stderr, "SEMANTIC_XAR stage failed (open oem) path=%s errno=%d\n", oem_path, errno);
        return 0;
    }

    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size <= 0 || (uint64_t)st.st_size > XAR_MAX_FILE_BYTES) {
        fprintf(stderr, "SEMANTIC_XAR stage failed (fstat) path=%s errno=%d size=%lld\n",
                oem_path, errno, (long long)st.st_size);
        close(fd);
        return 0;
    }

    size_t len = (size_t)st.st_size;
    uint8_t *buf = (uint8_t *)malloc(len);
    if (!buf) {
        close(fd);
        return 0;
    }

    size_t total = 0;
    while (total < len) {
        ssize_t n = read(fd, buf + total, len - total);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "SEMANTIC_XAR stage failed (read) path=%s errno=%d\n", oem_path, errno);
            free(buf);
            close(fd);
            return 0;
        }
        if (n == 0)
            break;
        total += (size_t)n;
    }
    close(fd);

    if (total != len) {
        fprintf(stderr, "SEMANTIC_XAR stage failed (short read %u/%u) path=%s\n",
                (unsigned)total, (unsigned)len, oem_path);
        free(buf);
        return 0;
    }

    int rc = semantic_xar_patch_buffer(buf, len, SEMANTIC_XAR_FLAG_ALL);
    if (rc < 0) {
        fprintf(stderr, "SEMANTIC_XAR stock signature mismatch for %s (%u bytes); using untouched OEM file\n",
                oem_path, (unsigned)len);
        free(buf);
        return 0;
    }

    /* QNX /dev/shmem (procnto shm manager) does not support rename(); write
     * directly under g_stage_lock after unlinking any stale object. */
    unlink(shm_path);
    int out_fd = real_open(shm_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (out_fd < 0) {
        fprintf(stderr, "SEMANTIC_XAR stage failed (open shm) path=%s errno=%d\n", shm_path, errno);
        free(buf);
        return 0;
    }

    size_t written = 0;
    while (written < len) {
        ssize_t n = write(out_fd, buf + written, len - written);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "SEMANTIC_XAR stage failed (write) path=%s errno=%d\n", shm_path, errno);
            close(out_fd);
            unlink(shm_path);
            free(buf);
            return 0;
        }
        written += (size_t)n;
    }
    int close_rc = close(out_fd);
    free(buf);

    if (close_rc != 0) {
        fprintf(stderr, "SEMANTIC_XAR stage failed (close) path=%s errno=%d\n", shm_path, errno);
        unlink(shm_path);
        return 0;
    }

    g_staged_mask |= bit;
    fprintf(stderr, "SEMANTIC_XAR staged in-memory override: %s -> %s (%u bytes, status=%d)\n",
            oem_path, shm_path, (unsigned)len, rc);
    return 1;
}

static const char *maybe_redirect_path(const char *path) {
    const char *shm_path = NULL;
    if (!semantic_xar_match_target(path, &shm_path))
        return path;

    pthread_mutex_lock(&g_stage_lock);
    int ok = stage_patched_xar_in_shmem(path, shm_path);
    pthread_mutex_unlock(&g_stage_lock);
    return ok ? shm_path : path;
}

static FILE *hook_fopen(const char *path, const char *mode) {
    const char *use_path = path;
    if (path && mode && mode[0] == 'r' && strchr(mode, '+') == NULL)
        use_path = maybe_redirect_path(path);
    FILE *fp = real_fopen(use_path, mode);
    if (!fp && use_path != path) {
        fprintf(stderr, "SEMANTIC_XAR fopen redirected shm failed path=%s errno=%d; falling back\n",
                use_path, errno);
        fp = real_fopen(path, mode);
    }
    return fp;
}

static int hook_open(const char *path, int oflag, ...) {
    mode_t mode = 0;
    if (oflag & O_CREAT) {
        va_list ap;
        va_start(ap, oflag);
        mode = (mode_t)va_arg(ap, unsigned int);
        va_end(ap);
    }
    const char *use_path = path;
    if (path && (oflag & O_ACCMODE) == O_RDONLY && (oflag & O_CREAT) == 0)
        use_path = maybe_redirect_path(path);
    int fd = (oflag & O_CREAT) ? real_open(use_path, oflag, mode) : real_open(use_path, oflag);
    if (fd < 0 && use_path != path) {
        fprintf(stderr, "SEMANTIC_XAR open redirected shm failed path=%s oflag=%#x errno=%d; falling back\n",
                use_path, oflag, errno);
        fd = (oflag & O_CREAT) ? real_open(path, oflag, mode) : real_open(path, oflag);
    }
    return fd;
}

static int hook_open64(const char *path, int oflag, ...) {
    mode_t mode = 0;
    if (oflag & O_CREAT) {
        va_list ap;
        va_start(ap, oflag);
        mode = (mode_t)va_arg(ap, unsigned int);
        va_end(ap);
    }
    const char *use_path = path;
    if (path && (oflag & O_ACCMODE) == O_RDONLY && (oflag & O_CREAT) == 0)
        use_path = maybe_redirect_path(path);
    int fd = (oflag & O_CREAT) ? real_open64(use_path, oflag, mode) : real_open64(use_path, oflag);
    if (fd < 0 && use_path != path) {
        fprintf(stderr, "SEMANTIC_XAR open64 redirected shm failed path=%s oflag=%#x errno=%d; falling back\n",
                use_path, oflag, errno);
        fd = (oflag & O_CREAT) ? real_open64(path, oflag, mode) : real_open64(path, oflag);
    }
    return fd;
}

void semantic_xar_install_hooks(void) {
    if (__sync_lock_test_and_set(&g_hooks_installed, 1) != 0)
        return;

    void *libc = dlopen("libc.so.3", RTLD_NOW | RTLD_LOCAL);
    if (!libc) {
        fprintf(stderr, "SEMANTIC_XAR dlopen(libc.so.3) failed\n");
        return;
    }
    real_fopen  = (fopen_fn_t)dlsym(libc, "fopen");
    real_open   = (open_fn_t)dlsym(libc, "open");
    real_open64 = (open_fn_t)dlsym(libc, "open64");
    if (!real_fopen || !real_open) {
        fprintf(stderr, "SEMANTIC_XAR dlsym(fopen/open) failed\n");
        return;
    }

    const elf32_ehdr_t *eh = (const elf32_ehdr_t *)(uintptr_t)NAV_BASIC_BASE;
    if (eh->e_ident[0] != 0x7fu || eh->e_ident[1] != 'E' ||
        eh->e_ident[2] != 'L'  || eh->e_ident[3] != 'F' ||
        eh->e_ident[4] != 1u   || eh->e_machine != 40u) {
        fprintf(stderr, "SEMANTIC_XAR unexpected main ELF header at %#x\n", NAV_BASIC_BASE);
        return;
    }

    const elf32_phdr_t *ph = (const elf32_phdr_t *)(uintptr_t)(NAV_BASIC_BASE + eh->e_phoff);
    const elf32_dyn_t *dyn = NULL;
    for (uint16_t i = 0; i < eh->e_phnum; ++i) {
        if (ph[i].p_type == ELF_PT_DYNAMIC) {
            dyn = (const elf32_dyn_t *)(uintptr_t)ph[i].p_vaddr;
            break;
        }
    }
    if (!dyn) {
        fprintf(stderr, "SEMANTIC_XAR PT_DYNAMIC not found\n");
        return;
    }

    const elf32_rel_t *jmprel = NULL;
    uint32_t pltrelsz = 0;
    const elf32_sym_t *symtab = NULL;
    const char *strtab = NULL;

    for (const elf32_dyn_t *d = dyn; d->d_tag != ELF_DT_NULL; ++d) {
        if (d->d_tag == ELF_DT_JMPREL)   jmprel   = (const elf32_rel_t *)(uintptr_t)d->d_val;
        if (d->d_tag == ELF_DT_PLTRELSZ) pltrelsz = d->d_val;
        if (d->d_tag == ELF_DT_SYMTAB)   symtab   = (const elf32_sym_t *)(uintptr_t)d->d_val;
        if (d->d_tag == ELF_DT_STRTAB)   strtab   = (const char *)(uintptr_t)d->d_val;
    }
    if (!jmprel || !pltrelsz || !symtab || !strtab) {
        fprintf(stderr, "SEMANTIC_XAR incomplete DT_JMPREL/SYMTAB/STRTAB\n");
        return;
    }

    unsigned hooked = 0;
    uint32_t count = pltrelsz / sizeof(elf32_rel_t);
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t rtype = jmprel[i].r_info & 0xffu;
        uint32_t rsym  = jmprel[i].r_info >> 8;
        if (rtype != ELF_R_ARM_JUMP_SLOT)
            continue;
        const char *name = strtab + symtab[rsym].st_name;
        void **slot = (void **)(uintptr_t)jmprel[i].r_offset;
        if (strcmp(name, "fopen") == 0) {
            __atomic_store_n(slot, (void *)(uintptr_t)hook_fopen, __ATOMIC_RELEASE);
            ++hooked;
        } else if (strcmp(name, "open") == 0) {
            __atomic_store_n(slot, (void *)(uintptr_t)hook_open, __ATOMIC_RELEASE);
            ++hooked;
        } else if (real_open64 && strcmp(name, "open64") == 0) {
            __atomic_store_n(slot, (void *)(uintptr_t)hook_open64, __ATOMIC_RELEASE);
            ++hooked;
        }
    }

    fprintf(stderr, "SEMANTIC_XAR installed %u in-memory NavigationBasic GOT hooks (zero disk edits)\n", hooked);
}

__attribute__((constructor))
static void semantic_xar_ctor(void) {
    semantic_xar_install_hooks();
}
