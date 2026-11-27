// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#include "semantic_cue_map.h"

#include <string.h>

static float normalize_deg(int32_t deg) {
    int32_t m = deg % 360;
    if (m < 0) m += 360;
    return (float)m;
}

static int32_t angle_delta_deg(int32_t a, int32_t b) {
    int32_t d = (a - b) % 360;
    if (d < 0) d += 360;
    if (d > 180) d = 360 - d;
    return d;
}

static void append_roundabout_arm(struct semantic_basic_cue *out, int cw, int32_t travel_deg,
                                  int32_t guided_travel_deg) {
    if (out->extra_arm_count >= SEMANTIC_MAX_EXTRA_ARMS)
        return;
    /* Keep clear of entry (0/360) and the guided exit spoke. */
    if (travel_deg <= 30 || travel_deg >= 330 || angle_delta_deg(travel_deg, guided_travel_deg) < 35)
        return;
    float oem_deg = normalize_deg(cw ? (540 + travel_deg) : (540 - travel_deg));
    for (uint8_t i = 0; i < out->extra_arm_count; ++i) {
        if (angle_delta_deg((int32_t)out->extra_arm_deg[i], (int32_t)oem_deg) < 25)
            return;
    }
    out->extra_arm_deg[out->extra_arm_count++] = oem_deg;
}

static void populate_roundabout_arms(struct semantic_basic_cue *out, int cw,
                                     int32_t guided_travel_deg,
                                     int has_exit_num, int32_t exit_num) {
    out->extra_arm_count = 0u;
    if (has_exit_num && exit_num >= 1 && exit_num <= 6) {
        /* Exact number of passed exits between entry (0 deg) and guided exit. */
        if (exit_num >= 2 && guided_travel_deg > 45) {
            for (int32_t k = 1; k < exit_num; ++k) {
                int32_t arm_travel = (k * guided_travel_deg + exit_num / 2) / exit_num;
                append_roundabout_arm(out, cw, arm_travel, guided_travel_deg);
            }
        }
        /* Also add remaining standard spokes after the guided exit so the ring
         * reads clearly as a multi-arm roundabout. */
        for (int32_t post = guided_travel_deg + 90; post <= 285; post += 90)
            append_roundabout_arm(out, cw, post, guided_travel_deg);
        return;
    }
    /* No exit_number: show standard 4-way cardinal arms around the ring. */
    static const int32_t cardinal[3] = { 90, 180, 270 };
    for (size_t i = 0; i < 3u; ++i)
        append_roundabout_arm(out, cw, cardinal[i], guided_travel_deg);
}

uint32_t semantic_lane_boundary_marking(uint32_t boundary_index, uint32_t lane_count) {
    if (lane_count == 0u || boundary_index == 0u || boundary_index >= lane_count)
        return SEMANTIC_LANE_MARKING_SOLID;
    return SEMANTIC_LANE_MARKING_SHORT_DASHED;
}

float semantic_lane_degrees(uint16_t bit) {
    switch (bit) {
    case NG_LANE_STRAIGHT:     return 0.0f;
    case NG_LANE_LEFT:         return 270.0f;
    case NG_LANE_RIGHT:        return 90.0f;
    case NG_LANE_SLIGHT_LEFT:  return 315.0f;
    case NG_LANE_SLIGHT_RIGHT: return 45.0f;
    case NG_LANE_SHARP_LEFT:   return 225.0f;
    case NG_LANE_SHARP_RIGHT:  return 135.0f;
    case NG_LANE_UTURN_LEFT:   return 181.0f;
    case NG_LANE_UTURN_RIGHT:  return 179.0f;
    default:                   return 0.0f;
    }
}

static int map_aa_type(const struct ng_message *msg, struct semantic_basic_cue *out) {
    const int has_angle = (msg->metadata_flags & NG_META_EXIT_ANGLE) != 0;
    const int has_exit_num = (msg->metadata_flags & NG_META_EXIT_NUMBER) != 0;
    const int32_t exit_angle = has_angle ? msg->exit_angle : 90;
    const int32_t exit_num = has_exit_num ? msg->exit_number : 0;

    switch (msg->aa_type) {
    case 1:  /* DEPART */
    case 2:  /* NAME_CHANGE */
    case 36: /* STRAIGHT */
        out->action = 1u; out->direction = 3u; out->elem_type = 1u; out->angle_deg = 0.0f;
        return 1;
    case 3:  /* SLIGHT_LEFT */
    case 5:  /* KEEP_LEFT */
        out->action = 2u; out->direction = 2u; out->elem_type = 7u; out->angle_deg = 315.0f;
        return 1;
    case 4:  /* SLIGHT_RIGHT */
    case 6:  /* KEEP_RIGHT */
        out->action = 2u; out->direction = 1u; out->elem_type = 7u; out->angle_deg = 45.0f;
        return 1;
    case 7:  /* TURN_LEFT */
        out->action = 2u; out->direction = 2u; out->elem_type = 7u; out->angle_deg = 270.0f;
        return 1;
    case 8:  /* TURN_RIGHT */
        out->action = 2u; out->direction = 1u; out->elem_type = 7u; out->angle_deg = 90.0f;
        return 1;
    case 9:  /* SHARP_LEFT */
        out->action = 2u; out->direction = 2u; out->elem_type = 7u; out->angle_deg = 225.0f;
        return 1;
    case 10: /* SHARP_RIGHT */
        out->action = 2u; out->direction = 1u; out->elem_type = 7u; out->angle_deg = 135.0f;
        return 1;
    case 11: /* UTURN_LEFT */
        out->action = 3u; out->direction = 2u; out->elem_type = 19u; out->angle_deg = 270.0f;
        return 1;
    case 12: /* UTURN_RIGHT */
        out->action = 3u; out->direction = 1u; out->elem_type = 19u; out->angle_deg = 90.0f;
        return 1;
    case 21: /* OFF_RAMP_SLIGHT_LEFT */
    case 23: /* OFF_RAMP_NORMAL_LEFT */
        out->action = 16u; out->direction = 2u; out->elem_type = 10u; out->angle_deg = 315.0f;
        return 1;
    case 22: /* OFF_RAMP_SLIGHT_RIGHT */
    case 24: /* OFF_RAMP_NORMAL_RIGHT */
        out->action = 16u; out->direction = 1u; out->elem_type = 10u; out->angle_deg = 45.0f;
        return 1;
    case 25: /* FORK_LEFT */
        out->action = 14u; out->direction = 2u; out->elem_type = 12u; out->angle_deg = 315.0f;
        return 1;
    case 26: /* FORK_RIGHT */
        out->action = 14u; out->direction = 1u; out->elem_type = 12u; out->angle_deg = 45.0f;
        return 1;
    case 32: /* ROUNDABOUT_ENTER_CW */
    case 33: /* ROUNDABOUT_EXIT_CW */
    case 43: /* ROUNDABOUT_ENTER_AND_EXIT_CW */
    case 44: /* ROUNDABOUT_ENTER_AND_EXIT_CW_WITH_ANGLE */ {
        /* AA measures travel from entry (180 deg in OEM 0x87617c) in circulation direction.
         * CW: (540 + exit_angle) % 360 -> 90->270, 180->0, 270->90 (verified BT81/BT86). */
        int32_t travel_deg = 270;
        if (has_angle && exit_angle > 0 && exit_angle <= 360)
            travel_deg = exit_angle;
        else if (has_exit_num && exit_num >= 1 && exit_num <= 4)
            travel_deg = exit_num * 90;
        out->action = 2u; out->direction = 1u; out->elem_type = 15u;
        out->angle_deg = normalize_deg(540 + travel_deg);
        populate_roundabout_arms(out, 1, travel_deg, has_exit_num, exit_num);
        return 1;
    }
    case 34: /* ROUNDABOUT_ENTER_CCW */
    case 35: /* ROUNDABOUT_EXIT_CCW */
    case 45: /* ROUNDABOUT_ENTER_AND_EXIT_CCW */
    case 46: /* ROUNDABOUT_ENTER_AND_EXIT_CCW_WITH_ANGLE */ {
        /* CCW: (540 - exit_angle) % 360 -> 90->90, 180->0, 270->270 (verified BT86). */
        int32_t travel_deg = 270;
        if (has_angle && exit_angle > 0 && exit_angle <= 360)
            travel_deg = exit_angle;
        else if (has_exit_num && exit_num >= 1 && exit_num <= 4)
            travel_deg = exit_num * 90;
        out->action = 2u; out->direction = 2u; out->elem_type = 16u;
        out->angle_deg = normalize_deg(540 - travel_deg);
        populate_roundabout_arms(out, 0, travel_deg, has_exit_num, exit_num);
        return 1;
    }
    case 39: /* DESTINATION */
    case 40: /* DESTINATION_STRAIGHT */
        out->action = 18u; out->direction = 3u; out->elem_type = 1u; out->angle_deg = 0.0f;
        out->is_arrival = 1u;
        return 1;
    case 41: /* DESTINATION_LEFT */
        out->action = 18u; out->direction = 2u; out->elem_type = 1u; out->angle_deg = 270.0f;
        out->is_arrival = 1u;
        return 1;
    case 42: /* DESTINATION_RIGHT */
        out->action = 18u; out->direction = 1u; out->elem_type = 1u; out->angle_deg = 90.0f;
        out->is_arrival = 1u;
        return 1;
    default:
        return 0;
    }
}

static int map_legacy_turn(const struct ng_message *msg, struct semantic_basic_cue *out) {
    /* Centunits: 6400 = 90 deg left (270 OEM), 19200 = 90 deg right (90 OEM),
     * 3200 = 45 deg left (315 OEM), 22400 = 45 deg right (45 OEM),
     * 9600 = 135 deg left (225 OEM), 16000 = 135 deg right (135 OEM). */
    const uint16_t d = msg->direction_centunits;
    switch (msg->turn) {
    case NG_STRAIGHT:
        out->action = 1u; out->direction = 3u; out->elem_type = 1u; out->angle_deg = 0.0f;
        return 1;
    case NG_LEFT:
        out->action = 2u; out->direction = 2u; out->elem_type = 7u;
        out->angle_deg = (d <= 4800u) ? 315.0f : (d >= 8000u ? 225.0f : 270.0f);
        return 1;
    case NG_RIGHT:
        out->action = 2u; out->direction = 1u; out->elem_type = 7u;
        out->angle_deg = (d >= 20800u) ? 45.0f : (d <= 17600u ? 135.0f : 90.0f);
        return 1;
    case NG_UTURN:
        out->action = 3u; out->elem_type = 19u;
        if (d >= 12800u) {
            out->direction = 1u; out->angle_deg = 90.0f;
        } else {
            out->direction = 2u; out->angle_deg = 270.0f;
        }
        return 1;
    default:
        return 0;
    }
}

int semantic_map_ng_message(const struct ng_message *msg, struct semantic_basic_cue *out) {
    if (!msg || !out || msg->distance_mm > NG_MAX_DISTANCE_MM || msg->lane_count > NG_MAX_LANES)
        return 0;
    memset(out, 0, sizeof(*out));

    int mapped = 0;
    if ((msg->metadata_flags & NG_META_AA_TYPE) && msg->aa_type != 0u)
        mapped = map_aa_type(msg, out);
    else
        mapped = map_legacy_turn(msg, out);
    if (!mapped)
        return 0;

    /* Round mm -> decimetres and metres cleanly (0 mm stays 0 dm / 0 m). */
    out->dist_dm = (msg->distance_mm + 50u) / 100u;
    out->norm_m = (msg->distance_mm + 500u) / 1000u;

    /* OEM approach progress bar (NavigationBasic 0x412d20 / NavigationFully 0x7e6f04):
     *   - straight/depart/name-change: no bargraph (BG_INVALID)
     *   - highway off-ramps / forks: 500 m (5000 dm)
     *   - urban turns / U-turns / roundabouts / arrivals: 250 m (2500 dm)
     * Visible (BG_VISIBLE = 2) once within bargraph_max_dm, hidden (BG_ENABLED = 1) farther out. */
    if (out->action == 1u) {
        out->bargraph_max_dm = 0u;
        out->bargraph_flag = SEMANTIC_BG_INVALID;
    } else {
        out->bargraph_max_dm = (out->action == 14u || out->action == 16u) ? 5000u : 2500u;
        out->bargraph_flag = (out->dist_dm <= out->bargraph_max_dm)
                                 ? SEMANTIC_BG_VISIBLE
                                 : SEMANTIC_BG_ENABLED;
    }

    if ((msg->metadata_flags & NG_META_ROAD) && msg->road_length > 0u) {
        size_t n = msg->road_length;
        if (n > NG_MAX_ROAD_BYTES) n = NG_MAX_ROAD_BYTES;
        memcpy(out->road, msg->road, n);
        out->road[n] = '\0';
    }

    out->lane_count = (uint8_t)msg->lane_count;
    for (uint8_t i = 0; i < out->lane_count; ++i)
        out->lanes[i] = msg->lanes[i];

    return 1;
}

int semantic_cue_is_distance_only(const struct semantic_basic_cue *prev,
                                  const struct semantic_basic_cue *next) {
    if (!prev || !next)
        return 0;
    if (prev->action != next->action ||
        prev->direction != next->direction ||
        prev->elem_type != next->elem_type ||
        prev->angle_deg != next->angle_deg ||
        prev->bargraph_flag != next->bargraph_flag ||
        prev->bargraph_max_dm != next->bargraph_max_dm ||
        prev->is_arrival != next->is_arrival ||
        prev->extra_arm_count != next->extra_arm_count ||
        prev->lane_count != next->lane_count ||
        strcmp(prev->road, next->road) != 0)
        return 0;
    for (uint8_t i = 0; i < prev->extra_arm_count; ++i) {
        if (prev->extra_arm_deg[i] != next->extra_arm_deg[i])
            return 0;
    }
    for (uint8_t i = 0; i < prev->lane_count; ++i) {
        if (prev->lanes[i].arrows != next->lanes[i].arrows ||
            prev->lanes[i].recommended != next->lanes[i].recommended)
            return 0;
    }
    return 1;
}
