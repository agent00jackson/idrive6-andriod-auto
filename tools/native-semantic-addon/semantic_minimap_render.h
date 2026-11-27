/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
/* Clean CID-widget-style vector minimap renderer targeting BMW 6WB HUD
 * HUDTerminal.ImageAddPolygon (0x21d, 176x116 viewport, exact OEM color IDs).
 *
 * Uses a 4-layer painter's algorithm (non-guided borders -> non-guided brown
 * fills -> guided orange border -> guided white arrow + arrowhead) so complex
 * roundabouts and junctions merge seamlessly without overlapping internal seams. */
#ifndef NBT_SEMANTIC_MINIMAP_RENDER_H
#define NBT_SEMANTIC_MINIMAP_RENDER_H

#include <stdint.h>

#include "semantic_nds_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SEMANTIC_HUD_MINIMAP_W        176u
#define SEMANTIC_HUD_MINIMAP_H        116u
#define SEMANTIC_HUD_MAX_POLYS        36u
#define SEMANTIC_HUD_MAX_POLY_PTS     28u

/* Verified OEM BMW 6WB HUD color palette IDs on handles 0x1f / 0x20 (bt97-off): */
enum semantic_hud_color {
    SEMANTIC_HUD_COLOR_ORANGE = 2u, /* Road & arrow outer border */
    SEMANTIC_HUD_COLOR_BROWN  = 3u, /* Non-guided road interior fill */
    SEMANTIC_HUD_COLOR_WHITE  = 4u, /* Guided route ribbon & arrowhead */
};

struct semantic_hud_poly {
    uint8_t color;
    uint8_t pt_count;
    uint16_t x[SEMANTIC_HUD_MAX_POLY_PTS];
    uint16_t y[SEMANTIC_HUD_MAX_POLY_PTS];
};

struct semantic_hud_minimap {
    uint32_t valid;
    uint32_t man_id;
    uint8_t poly_count;
    uint8_t is_roundabout;
    uint8_t confidence;
    uint8_t reserved;
    struct semantic_hud_poly polys[SEMANTIC_HUD_MAX_POLYS];
};

/* Render a matched NDS junction into clean 6WB HUD vector polygons.
 * Returns 1 on success, 0 if the junction is invalid or confidence < min_conf. */
int semantic_render_hud_minimap(const struct snds_junction *junc,
                                uint32_t man_id,
                                uint8_t min_conf,
                                struct semantic_hud_minimap *out);

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_MINIMAP_RENDER_H */
