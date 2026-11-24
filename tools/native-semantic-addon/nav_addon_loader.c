/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
/* Generic OnLoop addon directory loader for BMW NBT Evo navigation processes.
 *
 * Wired once via --execute=OnLoop:/opt/aa/nav-addons/libnav_addon_loader.so in
 * Appstarter.ini (NavigationBasic) and NavigationEceUsa.cfg (NavNBTEvoController).
 * At startup it identifies the host process basename from /proc/self/exefile
 * and loads every regular *.so in /opt/aa/nav-addons/<ProcessName>.d/ in
 * sorted alphabetical order, forwarding registerCallbacks(administrator).
 *
 * Fail-open safety:
 *   - If /var/opt/aa/nav-addons-disabled exists, loads nothing and returns.
 *   - If the scan directory is missing or empty, loads nothing and returns.
 *   - A dlopen() failure on one addon logs an error and does not block others
 *     or prevent OEM navigation from starting normally. */

#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#ifdef __QNXNTO__
#include <devctl.h>
#include <sys/procfs.h>
#endif

#define NAV_ADDON_DEFAULT_ROOT     "/opt/aa/nav-addons"
#define NAV_ADDON_DISABLE_FLAG     "/var/opt/aa/nav-addons-disabled"
#define NAV_ADDON_MAX_COUNT        64u
#define NAV_ADDON_MAX_NAME         256u

typedef void (*nav_addon_register_fn_t)(void *administrator);

static int nav_addon_is_safe_token(const char *s) {
    if (!s || !s[0] || strcmp(s, ".") == 0 || strcmp(s, "..") == 0)
        return 0;
    for (const unsigned char *p = (const unsigned char *)s; *p; ++p) {
        int ok = (*p >= 'A' && *p <= 'Z') ||
                 (*p >= 'a' && *p <= 'z') ||
                 (*p >= '0' && *p <= '9') ||
                 *p == '_' || *p == '-' || *p == '.';
        if (!ok)
            return 0;
    }
    return 1;
}

static int nav_addon_extract_basename(const char *raw, char *out, size_t out_cap) {
    if (!raw || !out || out_cap < 2u)
        return 0;
    size_t len = strlen(raw);
    while (len > 0u && (raw[len - 1u] == '\n' || raw[len - 1u] == '\r' ||
                        raw[len - 1u] == ' '  || raw[len - 1u] == '\t')) {
        --len;
    }
    if (len == 0u)
        return 0;
    const char *start = raw;
    for (size_t i = len; i > 0u; --i) {
        if (raw[i - 1u] == '/') {
            start = raw + i;
            break;
        }
    }
    size_t base_len = (size_t)((raw + len) - start);
    if (base_len == 0u || base_len >= out_cap)
        return 0;
    memcpy(out, start, base_len);
    out[base_len] = '\0';
    return nav_addon_is_safe_token(out);
}

static int nav_addon_has_so_suffix(const char *name) {
    if (!name || name[0] == '.')
        return 0;
    size_t n = strlen(name);
    if (n <= 3u)
        return 0;
    return strcmp(name + (n - 3u), ".so") == 0;
}

static void nav_addon_sort_names(char names[][NAV_ADDON_MAX_NAME], size_t count) {
    char tmp[NAV_ADDON_MAX_NAME];
    for (size_t i = 1u; i < count; ++i) {
        memcpy(tmp, names[i], NAV_ADDON_MAX_NAME);
        size_t j = i;
        while (j > 0u && strcmp(names[j - 1u], tmp) > 0) {
            memcpy(names[j], names[j - 1u], NAV_ADDON_MAX_NAME);
            --j;
        }
        memcpy(names[j], tmp, NAV_ADDON_MAX_NAME);
    }
}

static size_t nav_addon_collect_dir(const char *dir_path,
                                    char names[][NAV_ADDON_MAX_NAME],
                                    size_t max_names) {
    if (!dir_path || !dir_path[0] || !names || max_names == 0u)
        return 0u;
    DIR *dp = opendir(dir_path);
    if (!dp)
        return 0u;

    size_t count = 0u;
    struct dirent *ent;
    while ((ent = readdir(dp)) != NULL) {
        const char *name = ent->d_name;
        if (!nav_addon_has_so_suffix(name) || !nav_addon_is_safe_token(name))
            continue;
        size_t nlen = strlen(name);
        if (nlen >= NAV_ADDON_MAX_NAME)
            continue;

        char full[512];
        int fn = snprintf(full, sizeof(full), "%s/%s", dir_path, name);
        if (fn <= 0 || (size_t)fn >= sizeof(full))
            continue;
        struct stat st;
        if (stat(full, &st) != 0 || !S_ISREG(st.st_mode))
            continue;
        if (count >= max_names)
            break;
        memcpy(names[count], name, nlen + 1u);
        ++count;
    }
    closedir(dp);

    if (count > 1u)
        nav_addon_sort_names(names, count);
    return count;
}

#ifndef NAV_ADDON_LOADER_HOST_TEST

static FILE *g_loader_log = NULL;

static void loader_log(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fflush(stderr);
    if (g_loader_log) {
        va_start(ap, fmt);
        vfprintf(g_loader_log, fmt, ap);
        va_end(ap);
        fflush(g_loader_log);
    }
}

static int read_exefile_path(char *buf, size_t cap) {
    if (!buf || cap < 2u)
        return 0;

    char pid_path[64];
#ifdef __QNXNTO__
    /* QNX 6.5.0 /proc/<pid> only exposes "as"; use DCMD_PROC_MAPDEBUG_BASE. */
    snprintf(pid_path, sizeof(pid_path), "/proc/%d/as", (int)getpid());
    int as_fd = open(pid_path, O_RDONLY);
    if (as_fd >= 0) {
        struct {
            procfs_debuginfo info;
            char buff[PATH_MAX];
        } map;
        memset(&map, 0, sizeof(map));
        int rc = devctl(as_fd, DCMD_PROC_MAPDEBUG_BASE, &map, sizeof(map), NULL);
        close(as_fd);
        if (rc == EOK && map.info.path[0] != '\0') {
            snprintf(buf, cap, "%s", map.info.path);
            return 1;
        }
    }
#endif

    const char *candidates[2];
    snprintf(pid_path, sizeof(pid_path), "/proc/%d/exefile", (int)getpid());
    candidates[0] = "/proc/self/exefile";
    candidates[1] = pid_path;

    for (unsigned i = 0u; i < 2u; ++i) {
        int fd = open(candidates[i], O_RDONLY);
        if (fd < 0)
            continue;
        ssize_t n = read(fd, buf, cap - 1u);
        close(fd);
        if (n > 0) {
            buf[n] = '\0';
            return 1;
        }
    }
    return 0;
}

__attribute__((visibility("default")))
void registerCallbacks(void *administrator) {
    static unsigned entered;
    if (__atomic_exchange_n(&entered, 1u, __ATOMIC_ACQ_REL))
        return;

    char log_path[96];
    snprintf(log_path, sizeof(log_path), "/dev/shmem/nav-addon-loader-%d.log", (int)getpid());
    g_loader_log = fopen(log_path, "w");

    if (access(NAV_ADDON_DISABLE_FLAG, F_OK) == 0) {
        loader_log("NAV_ADDON_LOADER disabled via %s\n", NAV_ADDON_DISABLE_FLAG);
        if (g_loader_log) { fclose(g_loader_log); g_loader_log = NULL; }
        return;
    }

    char exepath[PATH_MAX];
    char proc_name[128];
    if (!read_exefile_path(exepath, sizeof(exepath)) ||
        !nav_addon_extract_basename(exepath, proc_name, sizeof(proc_name))) {
        loader_log("NAV_ADDON_LOADER could not determine process basename; skipping\n");
        if (g_loader_log) { fclose(g_loader_log); g_loader_log = NULL; }
        return;
    }

    const char *root = getenv("NBT_NAV_ADDONS_DIR");
    if (!root || root[0] != '/')
        root = NAV_ADDON_DEFAULT_ROOT;

    char scan_dir[384];
    int dn = snprintf(scan_dir, sizeof(scan_dir), "%s/%s.d", root, proc_name);
    if (dn <= 0 || (size_t)dn >= sizeof(scan_dir)) {
        if (g_loader_log) { fclose(g_loader_log); g_loader_log = NULL; }
        return;
    }

    char names[NAV_ADDON_MAX_COUNT][NAV_ADDON_MAX_NAME];
    size_t count = nav_addon_collect_dir(scan_dir, names, NAV_ADDON_MAX_COUNT);
    if (count == 0u) {
        loader_log("NAV_ADDON_LOADER no addons in %s (exe=%s)\n", scan_dir, exepath);
        if (g_loader_log) { fclose(g_loader_log); g_loader_log = NULL; }
        return;
    }

    unsigned loaded = 0u;
    for (size_t i = 0u; i < count; ++i) {
        char full[512];
        int fn = snprintf(full, sizeof(full), "%s/%s", scan_dir, names[i]);
        if (fn <= 0 || (size_t)fn >= sizeof(full))
            continue;
        void *handle = dlopen(full, RTLD_NOW | RTLD_LOCAL);
        if (!handle) {
            const char *err = dlerror();
            loader_log("NAV_ADDON_LOADER dlopen failed path=%s err=%s\n",
                       full, err ? err : "unknown");
            continue;
        }
        nav_addon_register_fn_t reg =
            (nav_addon_register_fn_t)dlsym(handle, "registerCallbacks");
        if (reg) {
            reg(administrator);
            loader_log("NAV_ADDON_LOADER loaded %s (registerCallbacks invoked)\n", full);
        } else {
            loader_log("NAV_ADDON_LOADER loaded %s (constructor only)\n", full);
        }
        ++loaded;
    }
    loader_log("NAV_ADDON_LOADER ready process=%s exe=%s dir=%s loaded=%u/%u\n",
               proc_name, exepath, scan_dir, loaded, (unsigned)count);
    if (g_loader_log) { fclose(g_loader_log); g_loader_log = NULL; }
}

#endif /* !NAV_ADDON_LOADER_HOST_TEST */
