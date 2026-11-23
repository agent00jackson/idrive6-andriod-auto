/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
#ifndef NBT_SEMANTIC_XAR_PATCH_H
#define NBT_SEMANTIC_XAR_PATCH_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    SEMANTIC_XAR_FLAG_STREET            = 1u << 0,
    SEMANTIC_XAR_FLAG_LANE_FALLBACK     = 1u << 1,
    SEMANTIC_XAR_FLAG_LANE_6WB_VIEWPORT = 1u << 2,
    SEMANTIC_XAR_FLAG_FULL_SIZE_SYMBOLS = 1u << 3,
    SEMANTIC_XAR_FLAG_ALL = SEMANTIC_XAR_FLAG_STREET |
                            SEMANTIC_XAR_FLAG_LANE_6WB_VIEWPORT |
                            SEMANTIC_XAR_FLAG_FULL_SIZE_SYMBOLS
};

enum semantic_xar_status {
    SEMANTIC_XAR_PATCHED = 0,
    SEMANTIC_XAR_ALREADY_PATCHED = 1,
    SEMANTIC_XAR_UNSUPPORTED = -1
};

/* Match stock Day/Night GvID5VBU1025 style paths (refuses /dev/shmem/ to prevent recursion).
 * Returns 1 and writes the ephemeral /dev/shmem/ staging path to *out_shm_path on match, else 0. */
int semantic_xar_match_target(const char *path, const char **out_shm_path);

/* Apply verified in-memory style overrides to a stock GvID5VBU1025 XAR buffer.
 * Validates every target row signature before writing any byte; returns SEMANTIC_XAR_UNSUPPORTED
 * without modifying buf if any byte signature does not match. */
int semantic_xar_patch_buffer(uint8_t *buf, size_t len, unsigned flags);

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_XAR_PATCH_H */
