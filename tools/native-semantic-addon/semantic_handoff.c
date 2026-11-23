// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#include "semantic_handoff.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

const char *semantic_handoff_default_path(void) {
#ifdef __QNXNTO__
    return "/dev/shmem/nbt-semantic-handoff";
#else
    return "/tmp/nbt-semantic-handoff";
#endif
}

int semantic_handoff_write(const char *path, const struct semantic_handoff *rec) {
    if (!path || !*path || !rec)
        return 0;
    struct semantic_handoff copy = *rec;
    copy.magic = SEMANTIC_HANDOFF_MAGIC;
    copy.version = SEMANTIC_HANDOFF_VERSION;

#ifdef __QNXNTO__
    /* QNX /dev/shmem (procnto shm manager) does not support rename().
     * The handoff record is 32 bytes; a single write() updates it in RAM. */
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0)
        return 0;
    ssize_t w = write(fd, &copy, sizeof(copy));
    int rc = close(fd);
    return (w == (ssize_t)sizeof(copy) && rc == 0) ? 1 : 0;
#else
    char tmp[160];
    int n = snprintf(tmp, sizeof(tmp), "%s.tmp.%ld", path, (long)getpid());
    if (n <= 0 || (size_t)n >= sizeof(tmp))
        return 0;

    int fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0)
        return 0;
    ssize_t w = write(fd, &copy, sizeof(copy));
    int rc = close(fd);
    if (w != (ssize_t)sizeof(copy) || rc != 0) {
        unlink(tmp);
        return 0;
    }
    if (rename(tmp, path) != 0) {
        unlink(tmp);
        return 0;
    }
    return 1;
#endif
}

int semantic_handoff_read(const char *path, struct semantic_handoff *out) {
    if (!path || !*path || !out)
        return 0;
    int fd = open(path, O_RDONLY);
    if (fd < 0)
        return 0;
    struct semantic_handoff tmp;
    ssize_t r = read(fd, &tmp, sizeof(tmp));
    close(fd);
    if (r != (ssize_t)sizeof(tmp) ||
        tmp.magic != SEMANTIC_HANDOFF_MAGIC ||
        tmp.version != SEMANTIC_HANDOFF_VERSION)
        return 0;
    *out = tmp;
    return 1;
}
