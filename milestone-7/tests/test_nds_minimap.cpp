// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Host unit tests + visual preview rasterizer for:
//   - semantic_nds_map (local NDS graph matching for roundabouts & junctions)
//   - semantic_minimap_render (clean CID-widget-style 4-layer HUD polygon renderer)
//   - semantic_minimap_shm (atomic shared-memory minimap handoff)

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../../tools/native-semantic-addon/semantic_nds_map.h"
#include "../../tools/native-semantic-addon/semantic_minimap_render.h"
#include "../../tools/native-semantic-addon/semantic_minimap_shm.h"
extern "C" {
#include "../../tools/native-semantic-addon/semantic_nds_map.c"
#include "../../tools/native-semantic-addon/semantic_minimap_render.c"
#include "../../tools/native-semantic-addon/semantic_minimap_shm.c"
}

static void addLink(snds_graph& g, uint64_t id, uint64_t n0, uint64_t n1,
                    const snds_pt* pts, uint8_t npts, uint8_t rb, uint8_t ow,
                    uint8_t frc, uint8_t lanes, const char* name) {
    assert(g.link_count < SNDS_MAX_GRAPH_LINKS);
    snds_link& l = g.links[g.link_count++];
    memset(&l, 0, sizeof(l));
    l.link_id = id;
    l.start_node = n0;
    l.end_node = n1;
    l.pt_count = npts;
    for (uint8_t i = 0; i < npts; ++i) l.pts[i] = pts[i];
    l.is_roundabout = rb;
    l.one_way = ow;
    l.frc = frc;
    l.lanes = lanes;
    if (name) snprintf(l.road_name, sizeof(l.road_name), "%s", name);
}

// Simple scanline polygon rasterizer for visual preview verification on host (exact OEM BMW HUD palette).
static void rasterizePreviewPPM(const semantic_hud_minimap& m, const char* path) {
    if (!path || !*path) return;
    const int W = (int)SEMANTIC_HUD_MINIMAP_W;
    const int H = (int)SEMANTIC_HUD_MINIMAP_H;
    unsigned char img[116][176][3];
    memset(img, 0, sizeof(img));

    for (uint8_t pi = 0; pi < m.poly_count; ++pi) {
        const semantic_hud_poly& p = m.polys[pi];
        unsigned char r = 255, g = 255, b = 255;
        if (p.color == SEMANTIC_HUD_COLOR_ORANGE)      { r = 255; g = 96;  b = 0; }
        else if (p.color == SEMANTIC_HUD_COLOR_BROWN)  { r = 74;  g = 26;  b = 0; }
        else if (p.color == SEMANTIC_HUD_COLOR_WHITE)  { r = 255; g = 255; b = 255; }

        int n = p.pt_count;
        for (int y = 0; y < H; ++y) {
            float fy = (float)y + 0.5f;
            float xs[64];
            int nx = 0;
            for (int i = 0; i < n; ++i) {
                int j = (i + 1) % n;
                float y1 = (float)p.y[i], y2 = (float)p.y[j];
                float x1 = (float)p.x[i], x2 = (float)p.x[j];
                if ((y1 <= fy && y2 > fy) || (y2 <= fy && y1 > fy)) {
                    float ix = x1 + (fy - y1) * (x2 - x1) / (y2 - y1);
                    if (nx < 64) xs[nx++] = ix;
                }
            }
            for (int a = 0; a < nx; ++a)
                for (int b2 = a + 1; b2 < nx; ++b2)
                    if (xs[b2] < xs[a]) { float t = xs[a]; xs[a] = xs[b2]; xs[b2] = t; }
            for (int k = 0; k + 1 < nx; k += 2) {
                int x0 = (int)ceilf(xs[k]);
                int x1 = (int)floorf(xs[k + 1]);
                if (x0 < 0) x0 = 0;
                if (x1 >= W) x1 = W - 1;
                for (int x = x0; x <= x1; ++x) {
                    img[y][x][0] = r; img[y][x][1] = g; img[y][x][2] = b;
                }
            }
        }
    }

    FILE* fp = fopen(path, "wb");
    assert(fp != NULL);
    fprintf(fp, "P6\n%d %d\n255\n", W, H);
    fwrite(img, 1, sizeof(img), fp);
    fclose(fp);
}

// Build a synthetic CCW roundabout centered at (0, 120) with radius (rx, ry) and arbitrary exit angles (compass deg).
static void buildRoundaboutGraph(snds_graph& g, float cx, float cy, float rx, float ry,
                                 const float* arm_bearings, const char** arm_names, int n_arms,
                                 int add_clutter) {
    memset(&g, 0, sizeof(g));
    // Nodes 100..100+n_arms-1 around the ring in CCW order starting at South (180 deg = entry)
    // Compass bearing b -> point (cx + rx*sin(b), cy + ry*cos(b))
    auto ringPt = [&](float b_deg) -> snds_pt {
        float r = b_deg * (float)(M_PI / 180.0);
        return { cx + rx * sinf(r), cy + ry * cosf(r) };
    };
    // Approach road: (0, 0) -> South entry node (100)
    snds_pt entry = ringPt(arm_bearings[0]);
    snds_pt app[2] = { {entry.x, 0.0f}, entry };
    addLink(g, 1, 1, 100, app, 2, 0, 0, 3, 2, "Main Street");

    // Ring arcs connecting node 100+k -> 100+(k+1)%n_arms (CCW: decreasing compass angle)
    for (int k = 0; k < n_arms; ++k) {
        float b0 = arm_bearings[k];
        float b1 = arm_bearings[(k + 1) % n_arms];
        float span = fmodf(b0 - b1 + 360.0f, 360.0f);
        if (span < 1.0f) span = 360.0f;
        snds_pt arc[6];
        for (int s = 0; s < 6; ++s) {
            float b = b0 - span * ((float)s / 5.0f);
            arc[s] = ringPt(b);
        }
        addLink(g, 10 + (uint64_t)k, 100 + (uint64_t)k, 100 + (uint64_t)((k + 1) % n_arms),
                arc, 6, 1, 1, 3, 2, "Rotary");
    }

    // Outgoing/crossing arms for k = 1..n_arms-1
    for (int k = 1; k < n_arms; ++k) {
        float b = arm_bearings[k];
        float r = b * (float)(M_PI / 180.0);
        snds_pt p0 = ringPt(b);
        snds_pt p1 = { p0.x + 80.0f * sinf(r), p0.y + 80.0f * cosf(r) };
        snds_pt seg[2] = { p0, p1 };
        addLink(g, 50 + (uint64_t)k, 100 + (uint64_t)k, 200 + (uint64_t)k,
                seg, 2, 0, 0, 3, 2, arm_names[k]);
    }

    if (add_clutter) {
        // Unrelated nearby parallel service road & parking driveway that OEM FULL_ROADS_MAP clutters!
        snds_pt cl1[2] = { {cx + 45.0f, cy - 60.0f}, {cx + 45.0f, cy + 60.0f} };
        addLink(g, 900, 900, 901, cl1, 2, 0, 0, 5, 1, "Frontage Rd");
        snds_pt drv[2] = { ringPt(arm_bearings[1]), {cx + 60.0f, cy + 5.0f} };
        addLink(g, 901, 101, 902, drv, 2, 0, 0, 7, 1, "Parking Driveway"); // frc=7 -> filtered!
    }
}

static void testAll(const char* outDir) {
    // 1. Road name normalization & fuzzy score
    char norm[64];
    snds_normalize_road_name("Spring Road", norm, sizeof(norm));
    assert(strcmp(norm, "spring rd") == 0);
    assert(snds_road_name_score("Spring Rd", "SPRING ROAD") == 100);
    assert(snds_road_name_score("Oak Ave", "Oak Avenue North") == 80);

    // 2. 4-way CCW roundabout: 3rd exit (left, 270 deg)
    {
        snds_graph g;
        const float bearings[4] = { 180.0f, 90.0f, 0.0f, 270.0f };
        const char* names[4] = { "Main St", "East Rd", "North Blvd", "Rotary Way" };
        buildRoundaboutGraph(g, 0.0f, 120.0f, 22.0f, 22.0f, bearings, names, 4, 0);

        snds_query q;
        memset(&q, 0, sizeof(q));
        q.car_x = 0.0f; q.car_y = 0.0f; q.car_heading_deg = 0.0f;
        q.aa_type = 46; // ROUNDABOUT_ENTER_AND_EXIT_CCW_WITH_ANGLE
        q.dist_m = 100.0f;
        q.exit_number = 3;
        q.exit_angle = 270;
        snprintf(q.road_name, sizeof(q.road_name), "Rotary Way");

        snds_junction j;
        assert(snds_match_maneuver(&g, &q, &j) == 1);
        assert(j.valid && j.is_roundabout && j.confidence >= 80);
        assert(j.arm_count == 2); // 1st exit (E) + 2nd exit (N); 3rd exit (W) is guided!
        assert(j.current_link_lanes == 2);

        semantic_hud_minimap mm;
        assert(semantic_render_hud_minimap(&j, 1, 65, &mm) == 1);
        assert(mm.valid && mm.poly_count >= 8 && mm.poly_count <= SEMANTIC_HUD_MAX_POLYS);
        if (outDir) {
            char p[256];
            snprintf(p, sizeof(p), "%s/preview-1-rb4-3rd.ppm", outDir);
            rasterizePreviewPPM(mm, p);
        }

        // Verify SHM round-trip!
        char shmPath[] = "/tmp/test-aa-minimap-shm";
        assert(semantic_minimap_shm_write(shmPath, 0x1234ULL, 7u, &mm) == 1);
        semantic_minimap_record rec;
        assert(semantic_minimap_shm_read(shmPath, &rec) == 1);
        assert(rec.session == 0x1234ULL && rec.sequence == 7u && rec.map.poly_count == mm.poly_count);
        semantic_minimap_shm_clear(shmPath);
    }

    // 3. 5-arm asymmetric CCW roundabout WITH cluttering frontage road & driveway!
    {
        snds_graph g;
        const float bearings[5] = { 180.0f, 115.0f, 45.0f, 340.0f, 255.0f };
        const char* names[5] = { "Main St", "SE Spur", "NE Ave", "North Pkwy", "West Dr" };
        buildRoundaboutGraph(g, 0.0f, 130.0f, 25.0f, 25.0f, bearings, names, 5, 1);

        snds_query q;
        memset(&q, 0, sizeof(q));
        q.car_heading_deg = 0.0f;
        q.aa_type = 46; q.dist_m = 110.0f; q.exit_number = 2; q.exit_angle = 135;
        snprintf(q.road_name, sizeof(q.road_name), "NE Avenue");

        snds_junction j;
        assert(snds_match_maneuver(&g, &q, &j) == 1);
        assert(j.valid && j.is_roundabout && j.confidence >= 80);
        // Must have EXACTLY 3 non-guided connected arms (SE, N, W); clutter & driveway MUST be dropped!
        assert(j.arm_count == 3);

        semantic_hud_minimap mm;
        assert(semantic_render_hud_minimap(&j, 3, 65, &mm) == 1);
        if (outDir) {
            char p[256];
            snprintf(p, sizeof(p), "%s/preview-2-rb5-asym-2nd.ppm", outDir);
            rasterizePreviewPPM(mm, p);
        }
    }

    // 4. Oval / elongated roundabout (real-world non-circular shape, 1st exit)
    {
        snds_graph g;
        const float bearings[4] = { 180.0f, 75.0f, 355.0f, 260.0f };
        const char* names[4] = { "Main St", "Parkway East", "North Rd", "West Ln" };
        buildRoundaboutGraph(g, 0.0f, 120.0f, 34.0f, 18.0f, bearings, names, 4, 0);

        snds_query q;
        memset(&q, 0, sizeof(q));
        q.car_heading_deg = 0.0f;
        q.aa_type = 46; q.dist_m = 100.0f; q.exit_number = 1; q.exit_angle = 105;
        snprintf(q.road_name, sizeof(q.road_name), "Parkway E");

        snds_junction j;
        assert(snds_match_maneuver(&g, &q, &j) == 1);
        assert(j.valid && j.is_roundabout && j.arm_count == 2);

        semantic_hud_minimap mm;
        assert(semantic_render_hud_minimap(&j, 5, 65, &mm) == 1);
        if (outDir) {
            char p[256];
            snprintf(p, sizeof(p), "%s/preview-3-rb-oval-1st.ppm", outDir);
            rasterizePreviewPPM(mm, p);
        }
    }

    // 5. Oblique 4-way intersection (turn left onto "Spring Rd" when NDS says "Spring Road")
    {
        snds_graph g;
        memset(&g, 0, sizeof(g));
        snds_pt app[2] = { {0, 0}, {0, 120} };
        snds_pt left_arm[3] = { {0, 120}, {-35, 132}, {-80, 145} };
        snds_pt straight_arm[2] = { {0, 120}, {10, 200} };
        snds_pt right_arm[2] = { {0, 120}, {75, 100} };
        addLink(g, 1, 1, 2, app, 2, 0, 0, 3, 3, "Oak Ave");
        addLink(g, 2, 2, 3, left_arm, 3, 0, 0, 3, 2, "Spring Road");
        addLink(g, 3, 2, 4, straight_arm, 2, 0, 0, 3, 2, "Oak Ave");
        addLink(g, 4, 2, 5, right_arm, 2, 0, 0, 4, 2, "Elm St");

        snds_query q;
        memset(&q, 0, sizeof(q));
        q.car_heading_deg = 0.0f;
        q.aa_type = 7; // TURN_LEFT
        q.dist_m = 120.0f;
        snprintf(q.road_name, sizeof(q.road_name), "Spring Rd");

        snds_junction j;
        assert(snds_match_maneuver(&g, &q, &j) == 1);
        assert(j.valid && !j.is_roundabout && j.confidence >= 80 && j.arm_count == 2);

        semantic_hud_minimap mm;
        assert(semantic_render_hud_minimap(&j, 7, 65, &mm) == 1);
        if (outDir) {
            char p[256];
            snprintf(p, sizeof(p), "%s/preview-4-oblique-left.ppm", outDir);
            rasterizePreviewPPM(mm, p);
        }
    }

    // 6. Non-maneuver (DEPART / STRAIGHT) must safely return 0 (fallback to CMiLCa)
    {
        snds_graph g;
        memset(&g, 0, sizeof(g));
        snds_pt app[2] = { {0, 0}, {0, 120} };
        addLink(g, 1, 1, 2, app, 2, 0, 0, 3, 2, "Main St");
        snds_query q;
        memset(&q, 0, sizeof(q));
        q.aa_type = 1; q.dist_m = 50.0f;
        snds_junction j;
        assert(snds_match_maneuver(&g, &q, &j) == 0);
    }

    // 7. SHM NULL default-path write / read / clear roundtrip
    {
        semantic_hud_minimap mm;
        memset(&mm, 0, sizeof(mm));
        mm.valid = 1;
        mm.confidence = 95;
        mm.poly_count = 1;
        mm.polys[0].color = 4;
        mm.polys[0].pt_count = 3;
        mm.polys[0].x[0] = 80; mm.polys[0].y[0] = 20;
        mm.polys[0].x[1] = 96; mm.polys[0].y[1] = 20;
        mm.polys[0].x[2] = 88; mm.polys[0].y[2] = 6;

        semantic_minimap_shm_clear(NULL);
        semantic_minimap_record rec;
        assert(semantic_minimap_shm_read(NULL, &rec) == 0);
        assert(semantic_minimap_shm_write(NULL, 0x1234ull, 7u, &mm) == 1);
        assert(semantic_minimap_shm_read(NULL, &rec) == 1);
        assert(rec.session == 0x1234ull && rec.sequence == 7u && rec.map.poly_count == 1u);
        semantic_minimap_shm_clear(NULL);
        assert(semantic_minimap_shm_read(NULL, &rec) == 0);
    }

    // 8. 1-hop approach stub (Michaels/Walgreens entrance before Walton Grove) vs
    //    2nd-hop parking-lot aisles & floating oval rejection
    {
        snds_graph g;
        memset(&g, 0, sizeof(g));
        // Approach on Cumberland Blvd (N): node 1 (0,0) -> node 2 (0,65) -> node 3 (0,100)
        snds_pt a1[2] = { {0, 0}, {0, 65} };
        snds_pt a2[2] = { {0, 65}, {0, 100} };
        snds_pt straight[2] = { {0, 100}, {0, 180} };
        snds_pt walton[2] = { {0, 100}, {70, 100} };     // Right turn into Walton Grove (FRC 6)
        // Hop-1 Michaels/Walgreens plaza entrance at node 2 (35m before turn, FRC 6)
        snds_pt michaels[2] = { {0, 65}, {65, 65} };
        snds_pt walgreens[2] = { {0, 65}, {-65, 65} };
        // Hop-2 parking-lot aisle branching off Michaels at (65,65) -> MUST NOT be drawn!
        snds_pt lot_aisle[2] = { {65, 65}, {65, 130} };
        // Disconnected floating drop-off oval on left (-45, 70) -> MUST NOT be drawn!
        snds_pt oval1[2] = { {-55, 70}, {-35, 70} };
        snds_pt oval2[2] = { {-35, 70}, {-55, 70} };

        addLink(g, 1, 1, 2, a1, 2, 0, 0, 3, 2, "Cumberland Blvd");
        addLink(g, 2, 2, 3, a2, 2, 0, 0, 3, 2, "Cumberland Blvd");
        addLink(g, 3, 3, 4, straight, 2, 0, 0, 3, 2, "Cumberland Blvd");
        addLink(g, 4, 3, 5, walton, 2, 0, 0, 6, 1, "");
        addLink(g, 5, 2, 6, michaels, 2, 0, 0, 6, 1, "");
        addLink(g, 6, 2, 7, walgreens, 2, 0, 0, 6, 1, "");
        addLink(g, 7, 6, 8, lot_aisle, 2, 0, 0, 6, 1, "");
        addLink(g, 8, 9, 10, oval1, 2, 1, 1, 6, 1, "");
        addLink(g, 9, 10, 9, oval2, 2, 1, 1, 6, 1, "");

        snds_query q;
        memset(&q, 0, sizeof(q));
        q.car_heading_deg = 0.0f;
        q.aa_type = 8; // TURN_RIGHT
        q.dist_m = 100.0f;

        snds_junction j;
        assert(snds_match_maneuver(&g, &q, &j) == 1);
        assert(j.valid && !j.is_roundabout && j.arm_count == 1);
        // context[0] = full walton continuation, context[1..2] = 1-hop Michaels & Walgreens stubs
        // (lot_aisle at hop-2 and floating oval MUST be excluded!)
        assert(j.context_count == 3);
        for (uint8_t c = 1; c < j.context_count; ++c) {
            // 1-hop stubs must be trimmed to ~35m (not extending to 65m parking lot!)
            float dx = j.context[c].pts[j.context[c].pt_count - 1].x - j.context[c].pts[0].x;
            float dy = j.context[c].pts[j.context[c].pt_count - 1].y - j.context[c].pts[0].y;
            float len = sqrtf(dx * dx + dy * dy);
            assert(len >= 30.0f && len <= 37.0f);
        }
        semantic_hud_minimap mm;
        assert(semantic_render_hud_minimap(&j, 8, 55, &mm) == 1);
    }

    // 9. Residential FRC 7 stop-sign 4-way intersection without road names
    {
        snds_graph g;
        memset(&g, 0, sizeof(g));
        snds_pt app[2] = { {0, 0}, {0, 80} };
        snds_pt str[2] = { {0, 80}, {0, 160} };
        snds_pt lft[2] = { {0, 80}, {-80, 80} };
        snds_pt rgt[2] = { {0, 80}, {80, 80} };
        addLink(g, 1, 1, 2, app, 2, 0, 0, 7, 1, "");
        addLink(g, 2, 2, 3, str, 2, 0, 0, 7, 1, "");
        addLink(g, 3, 2, 4, lft, 2, 0, 0, 7, 1, "");
        addLink(g, 4, 2, 5, rgt, 2, 0, 0, 7, 1, "");

        snds_query q;
        memset(&q, 0, sizeof(q));
        q.car_heading_deg = 0.0f;
        q.aa_type = 7; // TURN_LEFT
        q.dist_m = 80.0f;

        snds_junction j;
        assert(snds_match_maneuver(&g, &q, &j) == 1);
        assert(j.valid && !j.is_roundabout && j.arm_count == 2);
        semantic_hud_minimap mm;
        assert(semantic_render_hud_minimap(&j, 9, 55, &mm) == 1);
    }

    // 10. Roundabout with reversed one-way ring links (fallback undirected ring trace)
    {
        snds_graph g;
        memset(&g, 0, sizeof(g));
        const float cx = 0.0f, cy = 80.0f, R = 16.0f;
        snds_pt app[2] = { {0, 0}, {cx, cy - R} };
        addLink(g, 1, 10, 1, app, 2, 0, 0, 4, 1, "");
        // 4-link CCW ring marked one_way=2 (reversed!) so strict pass fails and fallback succeeds
        snds_pt r1[2] = { {cx, cy - R}, {cx + R, cy} };
        snds_pt r2[2] = { {cx + R, cy}, {cx, cy + R} };
        snds_pt r3[2] = { {cx, cy + R}, {cx - R, cy} };
        snds_pt r4[2] = { {cx - R, cy}, {cx, cy - R} };
        addLink(g, 2, 1, 2, r1, 2, 1, 2, 4, 1, "");
        addLink(g, 3, 2, 3, r2, 2, 1, 2, 4, 1, "");
        addLink(g, 4, 3, 4, r3, 2, 1, 2, 4, 1, "");
        addLink(g, 5, 4, 1, r4, 2, 1, 2, 4, 1, "");
        snds_pt e1[2] = { {cx + R, cy}, {cx + R + 50, cy} };
        snds_pt e2[2] = { {cx, cy + R}, {cx, cy + R + 50} };
        snds_pt e3[2] = { {cx - R, cy}, {cx - R - 50, cy} };
        addLink(g, 6, 2, 20, e1, 2, 0, 0, 4, 1, "");
        addLink(g, 7, 3, 21, e2, 2, 0, 0, 4, 1, "");
        addLink(g, 8, 4, 22, e3, 2, 0, 0, 4, 1, "");

        snds_query q;
        memset(&q, 0, sizeof(q));
        q.car_heading_deg = 0.0f;
        q.aa_type = 34; // ROUNDABOUT_ENTER_AND_EXIT_CCW
        q.dist_m = 64.0f;
        q.exit_number = 2;

        snds_junction j;
        assert(snds_match_maneuver(&g, &q, &j) == 1);
        assert(j.valid && j.is_roundabout && j.arm_count == 2);
        semantic_hud_minimap mm;
        assert(semantic_render_hud_minimap(&j, 10, 55, &mm) == 1);
    }
}

int main(int argc, char** argv) {
    const char* outDir = (argc > 1) ? argv[1] : NULL;
    testAll(outDir);
    printf("test_nds_minimap: all checks passed\n");
    return 0;
}
