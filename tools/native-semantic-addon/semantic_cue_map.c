// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#include "semantic_cue_map.h"

#include <string.h>

static float normalize_deg(int32_t deg) {
    int32_t m = deg % 360;
    if (m < 0) m += 360;
    return (float)m;
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
    const int32_t exit_angle = has_angle ? msg->exit_angle : 90;

    switch (msg->aa_type) {
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
    case 44: /* ROUNDABOUT_ENTER_AND_EXIT_CW_WITH_ANGLE */
        /* AA measures travel from entry (180 deg in OEM 0x87617c) in circulation direction.
         * CW: (540 + exit_angle) % 360 -> 90->270, 180->0, 270->90 (verified BT81/BT86). */
        out->action = 2u; out->direction = 1u; out->elem_type = 15u;
        out->angle_deg = (has_angle && exit_angle > 0 && exit_angle <= 360)
                             ? normalize_deg(540 + exit_angle) : 90.0f;
        return 1;
    case 34: /* ROUNDABOUT_ENTER_CCW */
    case 35: /* ROUNDABOUT_EXIT_CCW */
    case 45: /* ROUNDABOUT_ENTER_AND_EXIT_CCW */
    case 46: /* ROUNDABOUT_ENTER_AND_EXIT_CCW_WITH_ANGLE */
        /* CCW: (540 - exit_angle) % 360 -> 90->90, 180->0, 270->270 (verified BT86). */
        out->action = 2u; out->direction = 2u; out->elem_type = 16u;
        out->angle_deg = (has_angle && exit_angle > 0 && exit_angle <= 360)
                             ? normalize_deg(540 - exit_angle) : 270.0f;
        return 1;
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
        prev->is_arrival != next->is_arrival ||
        prev->lane_count != next->lane_count ||
        strcmp(prev->road, next->road) != 0)
        return 0;
    for (uint8_t i = 0; i < prev->lane_count; ++i) {
        if (prev->lanes[i].arrows != next->lanes[i].arrows ||
            prev->lanes[i].recommended != next->lanes[i].recommended)
            return 0;
    }
    return 1;
}
