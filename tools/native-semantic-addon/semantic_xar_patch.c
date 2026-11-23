/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
#include "semantic_xar_patch.h"

#include <string.h>

enum {
    XAR_FULL_SIZE_SELECTOR_OFFSET = 36720u,
    XAR_COMPACT_ROWS_OFFSET       = 84436u,
    XAR_LANE_ROW_OFFSET           = 84496u,
    XAR_VIEW_ROW_OFFSET           = 84516u,
    XAR_FULL_SIZE_ROWS_OFFSET     = 84804u,
    XAR_MIN_REQUIRED_BYTES        = 84804u + 40u
};

static void put_le32(uint8_t *dst, uint32_t v) {
    dst[0] = (uint8_t)(v & 0xffu);
    dst[1] = (uint8_t)((v >> 8) & 0xffu);
    dst[2] = (uint8_t)((v >> 16) & 0xffu);
    dst[3] = (uint8_t)((v >> 24) & 0xffu);
}

static uint32_t get_le32(const uint8_t *src) {
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

static void pack_row5(uint8_t out[20], uint32_t a, uint32_t b, uint32_t c,
                      uint32_t d, uint32_t e) {
    put_le32(out + 0, a);
    put_le32(out + 4, b);
    put_le32(out + 8, c);
    put_le32(out + 12, d);
    put_le32(out + 16, e);
}

int semantic_xar_match_target(const char *path, const char **out_shm_path) {
    if (!path || !path[0])
        return 0;
    if (strncmp(path, "/dev/shmem/", 11) == 0)
        return 0;
    const char *base = strrchr(path, '/');
    base = base ? base + 1 : path;
    if (strcmp(base, "GvID5VBU1025_GuidingDayStandard.xar") == 0) {
        if (out_shm_path)
            *out_shm_path = "/dev/shmem/aa-semantic-style-day.xar";
        return 1;
    }
    if (strcmp(base, "GvID5VBU1025_GuidingNightStandard.xar") == 0) {
        if (out_shm_path)
            *out_shm_path = "/dev/shmem/aa-semantic-style-night.xar";
        return 1;
    }
    return 0;
}

int semantic_xar_patch_buffer(uint8_t *buf, size_t len, unsigned flags) {
    if (!buf || len < XAR_MIN_REQUIRED_BYTES)
        return SEMANTIC_XAR_UNSUPPORTED;

    uint8_t container_row[20], street_row[20];
    uint8_t lane_row[20], lane_fallback_row[20], lane_viewport_row[20];
    uint8_t compact_rows[40], full_size_rows[40];

    pack_row5(container_row,     0x00000000u, 0x0024011eu, 35u, 0x00000000u, 3302u);
    pack_row5(street_row,        0x00000000u, 0x0024011eu,  6u, 0x00000000u, 3193u);
    pack_row5(lane_row,          0x00000000u, 0x002000d0u, 28u, 0x00000000u, 3103u);
    pack_row5(lane_fallback_row, 0x00000000u, 0x002000d0u, 28u, 0x20000000u, 3103u);
    pack_row5(lane_viewport_row, 0x002d0000u, 0x00c500d0u, 28u, 0x20000000u, 3103u);

    pack_row5(compact_rows +  0, 0x00000000u, 0x007400b0u, 18u, 0x00000000u, 3201u);
    pack_row5(compact_rows + 20, 0x0000001du, 0x002000cdu, 20u, 0x00000000u, 1202u);

    pack_row5(full_size_rows +  0, 0x0000001du, 0x007400cdu, 20u, 0x00000000u, 1111u);
    pack_row5(full_size_rows + 20, 0x002200c1u, 0x007400cfu, 14u, 0x00000000u, 3105u);

    int need_street = 0, need_lane = 0, need_symbols = 0;

    /* Phase 1: strictly validate ALL requested rows before modifying any byte. */
    if (flags & SEMANTIC_XAR_FLAG_STREET) {
        if (memcmp(buf + XAR_VIEW_ROW_OFFSET, container_row, 20) == 0) {
            need_street = 1;
        } else if (memcmp(buf + XAR_VIEW_ROW_OFFSET, street_row, 20) != 0) {
            return SEMANTIC_XAR_UNSUPPORTED;
        }
    }

    const uint8_t *target_lane_row = NULL;
    if (flags & SEMANTIC_XAR_FLAG_LANE_6WB_VIEWPORT) {
        target_lane_row = lane_viewport_row;
    } else if (flags & SEMANTIC_XAR_FLAG_LANE_FALLBACK) {
        target_lane_row = lane_fallback_row;
    }
    if (target_lane_row) {
        if (memcmp(buf + XAR_LANE_ROW_OFFSET, target_lane_row, 20) == 0) {
            need_lane = 0;
        } else if (memcmp(buf + XAR_LANE_ROW_OFFSET, lane_row, 20) == 0 ||
                   memcmp(buf + XAR_LANE_ROW_OFFSET, lane_fallback_row, 20) == 0) {
            need_lane = 1;
        } else {
            return SEMANTIC_XAR_UNSUPPORTED;
        }
    }

    if (flags & SEMANTIC_XAR_FLAG_FULL_SIZE_SYMBOLS) {
        uint32_t sel = get_le32(buf + XAR_FULL_SIZE_SELECTOR_OFFSET);
        if (memcmp(buf + XAR_FULL_SIZE_ROWS_OFFSET, full_size_rows, 40) != 0)
            return SEMANTIC_XAR_UNSUPPORTED;
        if (memcmp(buf + XAR_COMPACT_ROWS_OFFSET, compact_rows, 40) == 0 && sel == 4u) {
            need_symbols = 1;
        } else if (memcmp(buf + XAR_COMPACT_ROWS_OFFSET, full_size_rows, 40) == 0 && sel == 12u) {
            need_symbols = 0;
        } else {
            return SEMANTIC_XAR_UNSUPPORTED;
        }
    }

    if (!need_street && !need_lane && !need_symbols)
        return SEMANTIC_XAR_ALREADY_PATCHED;

    /* Phase 2: apply validated in-memory overrides. */
    if (need_street)
        memcpy(buf + XAR_VIEW_ROW_OFFSET, street_row, 20);
    if (need_lane)
        memcpy(buf + XAR_LANE_ROW_OFFSET, target_lane_row, 20);
    if (need_symbols) {
        put_le32(buf + XAR_FULL_SIZE_SELECTOR_OFFSET, 12u);
        memcpy(buf + XAR_COMPACT_ROWS_OFFSET, full_size_rows, 40);
    }

    return SEMANTIC_XAR_PATCHED;
}
