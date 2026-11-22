// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#ifndef NBT_SEMANTIC_CUE_MAP_H
#define NBT_SEMANTIC_CUE_MAP_H

#include <stddef.h>
#include <stdint.h>

#include "native_model_wire/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

struct semantic_basic_cue {
    uint32_t action;
    uint32_t direction;
    uint32_t elem_type;
    float angle_deg;
    uint32_t dist_dm;
    uint32_t norm_m;
    uint8_t is_arrival;
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

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_CUE_MAP_H */
