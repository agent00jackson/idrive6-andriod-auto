/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
#include "semantic_minimap_shm.h"

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

static uint32_t fnv1a_32(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

const char *semantic_minimap_default_path(void) {
#ifdef __QNXNTO__
    return "/dev/shmem/aa-semantic-minimap";
#else
    return "/tmp/aa-semantic-minimap";
#endif
}

int semantic_minimap_shm_write(const char *path, uint64_t session, uint32_t seq,
                               const struct semantic_hud_minimap *map) {
    if (!map) return 0;
    if (!path) path = semantic_minimap_default_path();
    struct semantic_minimap_record rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = SEMANTIC_MINIMAP_SHM_MAGIC;
    rec.version = SEMANTIC_MINIMAP_SHM_VERSION;
    rec.sequence = seq;
    rec.session = session;
    rec.map = *map;
    rec.checksum = 0u;
    rec.checksum = fnv1a_32(&rec, sizeof(rec));

    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0) return 0;
    ssize_t wr = write(fd, &rec, sizeof(rec));
    close(fd);
    return wr == (ssize_t)sizeof(rec);
}

int semantic_minimap_shm_read(const char *path, struct semantic_minimap_record *out) {
    if (!out) return 0;
    if (!path) path = semantic_minimap_default_path();
    int fd = open(path, O_RDONLY);
    if (fd < 0) return 0;
    struct semantic_minimap_record rec;
    ssize_t rd = read(fd, &rec, sizeof(rec));
    close(fd);
    if (rd != (ssize_t)sizeof(rec)) return 0;
    if (rec.magic != SEMANTIC_MINIMAP_SHM_MAGIC || rec.version != SEMANTIC_MINIMAP_SHM_VERSION)
        return 0;
    uint32_t saved = rec.checksum;
    rec.checksum = 0u;
    if (fnv1a_32(&rec, sizeof(rec)) != saved)
        return 0;
    rec.checksum = saved;
    if (rec.map.poly_count > SEMANTIC_HUD_MAX_POLYS)
        return 0;
    *out = rec;
    return 1;
}

void semantic_minimap_shm_clear(const char *path) {
    if (!path) path = semantic_minimap_default_path();
    (void)unlink(path);
}
