/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
/* Atomic shared-memory handoff passing rendered HUD minimap polygons from
 * NavigationBasic (NDS lookup + widget renderer) to NavNBTEvoController. */
#ifndef NBT_SEMANTIC_MINIMAP_SHM_H
#define NBT_SEMANTIC_MINIMAP_SHM_H

#include <stdint.h>

#include "semantic_minimap_render.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SEMANTIC_MINIMAP_SHM_MAGIC   0x4D4D4150u /* 'MMAP' */
#define SEMANTIC_MINIMAP_SHM_VERSION 2u

struct semantic_minimap_record {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t checksum;
    uint64_t session;
    struct semantic_hud_minimap map;
};

const char *semantic_minimap_default_path(void);
int semantic_minimap_shm_write(const char *path, uint64_t session, uint32_t seq,
                               const struct semantic_hud_minimap *map);
int semantic_minimap_shm_read(const char *path, struct semantic_minimap_record *out);
void semantic_minimap_shm_clear(const char *path);

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_MINIMAP_SHM_H */
