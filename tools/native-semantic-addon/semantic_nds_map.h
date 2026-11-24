/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
/* Local OEM NDS road-graph representation and AA-to-NDS maneuver matcher.
 * Pure C, zero external dependencies, host-unit-testable and ARM QNX safe. */
#ifndef NBT_SEMANTIC_NDS_MAP_H
#define NBT_SEMANTIC_NDS_MAP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SNDS_MAX_LINK_PTS     24u
#define SNDS_MAX_GRAPH_LINKS  128u
#define SNDS_MAX_ROAD_NAME    64u
#define SNDS_MAX_JUNCTION_PTS 48u
#define SNDS_MAX_ARMS         6u
#define SNDS_MAX_ARM_PTS      20u

/* 2D point in local East/North metres relative to the car (or anchor). */
struct snds_pt {
    float x; /* East  (+m) */
    float y; /* North (+m) */
};

/* One undirected/directed NDS road link extracted from the local area scan. */
struct snds_link {
    uint64_t link_id;
    uint64_t start_node;
    uint64_t end_node;
    uint8_t pt_count;
    struct snds_pt pts[SNDS_MAX_LINK_PTS];
    float length_m;
    uint8_t is_roundabout; /* 1 if link is part of a roundabout ring */
    uint8_t one_way;       /* 0=bidirectional, 1=start->end only, 2=end->start only */
    uint8_t frc;           /* Functional road class: 0=motorway .. 7=alley/driveway */
    uint8_t lanes;         /* Total lanes on this link (0 if unknown) */
    char road_name[SNDS_MAX_ROAD_NAME];
};

struct snds_graph {
    uint16_t link_count;
    struct snds_link links[SNDS_MAX_GRAPH_LINKS];
};

/* Android Auto maneuver query used to match the upcoming NDS junction. */
struct snds_query {
    float car_x;           /* Local East (m), typically 0 */
    float car_y;           /* Local North (m), typically 0 */
    float car_heading_deg; /* Compass heading: 0=North, 90=East, 180=South, 270=West */
    uint16_t aa_type;      /* Android Auto Maneuver TYPE_* */
    float dist_m;          /* Distance to maneuver in metres */
    int32_t exit_number;   /* Roundabout exit number (1-based), 0 if absent */
    int32_t exit_angle;    /* Roundabout exit angle (deg in circulation dir), 0 if absent */
    char road_name[SNDS_MAX_ROAD_NAME];
};

struct snds_polyline {
    uint8_t pt_count;
    struct snds_pt pts[SNDS_MAX_JUNCTION_PTS];
};

struct snds_arm {
    uint8_t pt_count;
    struct snds_pt pts[SNDS_MAX_ARM_PTS];
    float exit_angle_deg; /* Relative to approach direction */
};

/* Clean, de-cluttered junction subgraph ready for CID-widget-style rendering.
 * Contains ONLY the approach, guided path (through roundabout/turn), roundabout
 * ring (if applicable), and directly connected crossing/exit arms. */
struct snds_junction {
    uint8_t valid;
    uint8_t confidence;       /* 0..100 (>= 65 recommended for HUD minimap display) */
    uint8_t is_roundabout;
    uint8_t roundabout_cw;    /* 1=CW (LHT), 0=CCW (RHT) */
    struct snds_pt center;    /* Junction node or roundabout centroid */
    float approach_heading_deg; /* Compass heading entering the junction (rotated to UP) */
    float roundabout_radius_m;  /* Approximate radius if is_roundabout */

    struct snds_polyline approach; /* Road leading into junction/roundabout */
    struct snds_polyline guided;   /* Full guided spine: approach -> arc/turn -> exit */
    struct snds_polyline ring;     /* Closed roundabout ring (if is_roundabout) */

    uint8_t arm_count;
    struct snds_arm arms[SNDS_MAX_ARMS]; /* Directly connected non-guided arms only */

    uint8_t current_link_lanes;    /* NDS lane count on car's current link (0 if unknown) */
};

/* Convert NDS WGS84 integer coordinates (1e-5 deg or 2^32 units -> local East/North metres). */
struct snds_pt snds_wgs84_to_local_m(double lat_deg, double lon_deg,
                                     double origin_lat_deg, double origin_lon_deg);

/* Normalize a street name for fuzzy matching ("Spring Rd" <-> "Spring Road"). */
void snds_normalize_road_name(const char *src, char *dst, size_t dst_size);

/* Compute street name match score in [0..100]. */
int snds_road_name_score(const char *a, const char *b);

/* Match an Android Auto maneuver against the local NDS road graph.
 * Returns 1 if a valid match (out->valid == 1) was found, 0 otherwise. */
int snds_match_maneuver(const struct snds_graph *graph,
                        const struct snds_query *query,
                        struct snds_junction *out);

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_NDS_MAP_H */
