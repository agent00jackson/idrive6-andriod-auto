// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#ifndef NBT_SEMANTIC_CUE_MAP_H
#define NBT_SEMANTIC_CUE_MAP_H

#include <stddef.h>
#include <stdint.h>

#include "native_model_wire/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SEMANTIC_MAX_EXTRA_ARMS 6u

enum semantic_bargraph_flag {
    SEMANTIC_BG_INVALID = 0u,
    SEMANTIC_BG_ENABLED = 1u,
    SEMANTIC_BG_VISIBLE = 2u,
};

enum semantic_lane_marking {
    SEMANTIC_LANE_MARKING_DOUBLE_SOLID = 2u,
    SEMANTIC_LANE_MARKING_SOLID = 3u,
    SEMANTIC_LANE_MARKING_SHORT_DASHED = 6u,
    SEMANTIC_LANE_MARKING_LONG_DASHED = 12u,
};

struct semantic_basic_cue {
    uint32_t action;
    uint32_t direction;
    uint32_t elem_type;
    float angle_deg;
    uint32_t dist_dm;
    uint32_t norm_m;
    uint32_t bargraph_flag;
    uint32_t bargraph_max_dm;
    uint8_t is_arrival;
    uint8_t extra_arm_count;
    float extra_arm_deg[SEMANTIC_MAX_EXTRA_ARMS];
    uint8_t lane_count;
    struct ng_lane lanes[NG_MAX_LANES];
    char road[NG_MAX_ROAD_BYTES + 1u];
};

/* Pure mapping from NGV1..5 message into verified NavigationBasic CMiLCa fields.
 * Returns 1 on success, 0 if the message cannot be mapped. */
int semantic_map_ng_message(const struct ng_message *msg, struct semantic_basic_cue *out);

/* Returns 1 if prev and next differ ONLY in distance (eligible for in-place
 * DistanceToManeuver refresh without rotating CMiLCa active maneuver IDs). */
int semantic_cue_is_distance_only(const struct semantic_basic_cue *prev,
                                  const struct semantic_basic_cue *next);

/* Lane arrow bit (NG_LANE_*) -> OEM NavigationBasic lane arrow angle in degrees. */
float semantic_lane_degrees(uint16_t bit);

/* Lane boundary index (0..lane_count) -> OEM NavigationBasic divider marking enum. */
uint32_t semantic_lane_boundary_marking(uint32_t boundary_index, uint32_t lane_count);

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_CUE_MAP_H */
