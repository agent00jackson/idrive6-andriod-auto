// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#ifndef NBT_SEMANTIC_HANDOFF_H
#define NBT_SEMANTIC_HANDOFF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEMANTIC_HANDOFF_MAGIC 0x4e425348u /* "NBSH" */
#define SEMANTIC_HANDOFF_VERSION 1u

enum semantic_handoff_state {
    SH_IDLE = 0,
    SH_BASIC_ARMED = 1,       /* Basic armed resolver wrapper for session */
    SH_ROUTE_READY = 2,       /* Basic produced RouteId for Controller's tour */
    SH_GUIDANCE_ACTIVE = 3,   /* Controller attached RouteId + called StartGuidance */
    SH_CLEARED = 4,           /* Basic cleared guidance cleanly */
};

struct semantic_handoff {
    uint32_t magic;
    uint32_t version;
    uint32_t state;
    uint32_t route_id;
    uint32_t trip_handle;
    uint32_t reserved;
    uint64_t session;
};

const char *semantic_handoff_default_path(void);
int semantic_handoff_write(const char *path, const struct semantic_handoff *rec);
int semantic_handoff_read(const char *path, struct semantic_handoff *out);

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_HANDOFF_H */
