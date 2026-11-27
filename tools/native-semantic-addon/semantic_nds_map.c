/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
#include "semantic_nds_map.h"

#include <ctype.h>
#include "semantic_math.h"
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float rad_to_deg(float r) { return r * (float)(180.0 / M_PI); }

static float wrap_deg_180(float d) {
    while (d > 180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

static float wrap_deg_360(float d) {
    while (d >= 360.0f) d -= 360.0f;
    while (d < 0.0f) d += 360.0f;
    return d;
}

/* Compass heading in degrees: 0=North (+Y), 90=East (+X), 180=South, 270=West. */
static float vec_heading_deg(float dx, float dy) {
    return wrap_deg_360(rad_to_deg(m_atan2f(dx, dy)));
}

static float pt_dist(struct snds_pt a, struct snds_pt b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return m_sqrtf(dx * dx + dy * dy);
}

static int is_roundabout_aa(uint16_t aa_type) {
    return (aa_type >= 32u && aa_type <= 35u) ||
           aa_type == 37u || aa_type == 38u ||
           (aa_type >= 43u && aa_type <= 46u);
}

static int is_roundabout_cw_aa(uint16_t aa_type) {
    return aa_type == 32u || aa_type == 33u || aa_type == 37u ||
           aa_type == 43u || aa_type == 44u;
}

static int is_non_maneuver_aa(uint16_t aa_type) {
    return aa_type == 1u || aa_type == 2u || aa_type == 36u ||
           (aa_type >= 39u && aa_type <= 42u);
}

/* Expected signed turn angle (deg, right > 0, left < 0) for non-roundabout AA maneuvers. */
static float expected_turn_deg(uint16_t aa_type) {
    switch (aa_type) {
    case 3:  case 5:  case 21: case 23: case 25: return -45.0f;  /* slight/keep/ramp/fork left */
    case 4:  case 6:  case 22: case 24: case 26: return  45.0f;  /* slight/keep/ramp/fork right */
    case 7:                                      return -90.0f;  /* turn left */
    case 8:                                      return  90.0f;  /* turn right */
    case 9:                                      return -135.0f; /* sharp left */
    case 10:                                     return  135.0f; /* sharp right */
    case 11:                                     return -175.0f; /* uturn left */
    case 12:                                     return  175.0f; /* uturn right */
    default:                                     return   0.0f;
    }
}

struct snds_pt snds_wgs84_to_local_m(double lat_deg, double lon_deg,
                                     double origin_lat_deg, double origin_lon_deg) {
    struct snds_pt p;
    double dlat = (lat_deg - origin_lat_deg) * (M_PI / 180.0);
    double dlon = (lon_deg - origin_lon_deg) * (M_PI / 180.0);
    double cos_lat = (double)m_cosf((float)(origin_lat_deg * (3.14159265358979323846 / 180.0)));
    const double earth_r = 6371000.0;
    p.x = (float)(dlon * cos_lat * earth_r);
    p.y = (float)(dlat * earth_r);
    return p;
}

/* Expand/canonicalize common US/EU street suffixes so "Spring Rd" matches "Spring Road". */
static const char *canonical_token(const char *tok) {
    static const struct { const char *a; const char *b; } map[] = {
        {"road", "rd"}, {"street", "st"}, {"avenue", "ave"}, {"boulevard", "blvd"},
        {"drive", "dr"}, {"lane", "ln"}, {"court", "ct"}, {"circle", "cir"},
        {"place", "pl"}, {"parkway", "pkwy"}, {"highway", "hwy"}, {"freeway", "fwy"},
        {"terrace", "ter"}, {"trail", "trl"}, {"way", "wy"}, {"north", "n"},
        {"south", "s"}, {"east", "e"}, {"west", "w"}, {"northeast", "ne"},
        {"northwest", "nw"}, {"southeast", "se"}, {"southwest", "sw"},
    };
    for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); ++i) {
        if (strcmp(tok, map[i].a) == 0 || strcmp(tok, map[i].b) == 0)
            return map[i].b;
    }
    return tok;
}

void snds_normalize_road_name(const char *src, char *dst, size_t dst_size) {
    if (!dst || dst_size == 0u) return;
    dst[0] = '\0';
    if (!src || !src[0]) return;

    char out[SNDS_MAX_ROAD_NAME];
    size_t out_len = 0u;
    const char *p = src;
    while (*p) {
        while (*p && !isalnum((unsigned char)*p)) ++p;
        if (!*p) break;
        char tok[32];
        size_t tlen = 0u;
        while (*p && isalnum((unsigned char)*p)) {
            if (tlen + 1u < sizeof(tok))
                tok[tlen++] = (char)tolower((unsigned char)*p);
            ++p;
        }
        tok[tlen] = '\0';
        const char *canon = canonical_token(tok);
        size_t clen = strlen(canon);
        if (out_len > 0u && out_len + 1u < sizeof(out))
            out[out_len++] = ' ';
        for (size_t i = 0; i < clen && out_len + 1u < sizeof(out); ++i)
            out[out_len++] = canon[i];
    }
    out[out_len] = '\0';
    size_t copy = out_len < dst_size - 1u ? out_len : dst_size - 1u;
    memcpy(dst, out, copy);
    dst[copy] = '\0';
}

int snds_road_name_score(const char *a, const char *b) {
    char na[SNDS_MAX_ROAD_NAME], nb[SNDS_MAX_ROAD_NAME];
    snds_normalize_road_name(a, na, sizeof(na));
    snds_normalize_road_name(b, nb, sizeof(nb));
    if (!na[0] || !nb[0]) return 0;
    if (strcmp(na, nb) == 0) return 100;
    if (strstr(na, nb) != NULL || strstr(nb, na) != NULL) return 80;
    /* Compare first word (e.g., "spring" in "spring rd" vs "spring way"). */
    const char *sa = strchr(na, ' ');
    const char *sb = strchr(nb, ' ');
    size_t la = sa ? (size_t)(sa - na) : strlen(na);
    size_t lb = sb ? (size_t)(sb - nb) : strlen(nb);
    if (la >= 3u && la == lb && memcmp(na, nb, la) == 0)
        return 60;
    return 0;
}

static float link_compute_length(const struct snds_link *l) {
    if (l->length_m > 0.1f) return l->length_m;
    float sum = 0.0f;
    for (uint8_t i = 1u; i < l->pt_count; ++i)
        sum += pt_dist(l->pts[i - 1u], l->pts[i]);
    return sum;
}

/* Heading of link when traversed forward (dir=0: start->end) or reversed (dir=1: end->start)
 * at either entry (at_end=0) or exit (at_end=1). */
static float link_heading(const struct snds_link *l, int rev, int at_end) {
    if (l->pt_count < 2u) return 0.0f;
    struct snds_pt a, b;
    if (!rev) {
        if (!at_end) { a = l->pts[0]; b = l->pts[1]; }
        else         { a = l->pts[l->pt_count - 2u]; b = l->pts[l->pt_count - 1u]; }
    } else {
        if (!at_end) { a = l->pts[l->pt_count - 1u]; b = l->pts[l->pt_count - 2u]; }
        else         { a = l->pts[1]; b = l->pts[0]; }
    }
    return vec_heading_deg(b.x - a.x, b.y - a.y);
}

static int link_allows_dir(const struct snds_link *l, int rev) {
    if (l->one_way == 1u && rev) return 0;
    if (l->one_way == 2u && !rev) return 0;
    return 1;
}

static uint64_t link_far_node(const struct snds_link *l, int rev) {
    return rev ? l->start_node : l->end_node;
}

static struct snds_pt link_end_pt(const struct snds_link *l, int rev) {
    return rev ? l->pts[0] : l->pts[l->pt_count - 1u];
}

static struct snds_pt link_start_pt(const struct snds_link *l, int rev) {
    return rev ? l->pts[l->pt_count - 1u] : l->pts[0];
}

/* Check whether link l starts at (node, pt) in forward (rev=0) or reverse (rev=1) order.
 * Uses exact quantized node ID first, falling back to <= 2.5m endpoint proximity for
 * NDS tile borders or rounding boundaries. */
static int link_touches_node(const struct snds_link *l, uint64_t node, struct snds_pt node_pt) {
    if (l->pt_count < 2u) return -1;
    if (l->start_node == node) return 0;
    if (l->end_node == node) return 1;
    float d0 = pt_dist(l->pts[0], node_pt);
    float d1 = pt_dist(l->pts[l->pt_count - 1u], node_pt);
    if (d0 <= 2.5f && d0 <= d1) return 0;
    if (d1 <= 2.5f) return 1;
    return -1;
}

static void poly_append_pt(struct snds_polyline *poly, struct snds_pt p) {
    if (poly->pt_count >= SNDS_MAX_JUNCTION_PTS) return;
    if (poly->pt_count > 0u && pt_dist(poly->pts[poly->pt_count - 1u], p) < 0.5f)
        return;
    poly->pts[poly->pt_count++] = p;
}

static void poly_append_link(struct snds_polyline *poly, const struct snds_link *l, int rev) {
    if (!rev) {
        for (uint8_t i = 0u; i < l->pt_count; ++i)
            poly_append_pt(poly, l->pts[i]);
    } else {
        for (int i = (int)l->pt_count - 1; i >= 0; --i)
            poly_append_pt(poly, l->pts[i]);
    }
}

static void arm_from_link(const struct snds_graph *g, struct snds_arm *arm,
                          int start_idx, int start_rev,
                          float max_len_m, float approach_hdg) {
    const struct snds_link *first = &g->links[start_idx];
    arm->pt_count = 0u;
    arm->exit_angle_deg = wrap_deg_180(link_heading(first, start_rev, 0) - approach_hdg);

    float accum = 0.0f;
    struct snds_pt prev = link_start_pt(first, start_rev);
    arm->pts[arm->pt_count++] = prev;

    int cur_idx = start_idx;
    int cur_rev = start_rev;
    int seen[4] = { start_idx, -1, -1, -1 };
    int seen_n = 1;

    for (int hop = 0; hop < 4 && accum < max_len_m && arm->pt_count < SNDS_MAX_ARM_PTS; ++hop) {
        const struct snds_link *l = &g->links[cur_idx];
        int step = cur_rev ? -1 : 1;
        int idx = cur_rev ? (int)l->pt_count - 2 : 1;
        while (idx >= 0 && idx < (int)l->pt_count && arm->pt_count < SNDS_MAX_ARM_PTS) {
            struct snds_pt cur = l->pts[idx];
            float seg = pt_dist(prev, cur);
            if (seg > 0.2f) {
                if (accum + seg > max_len_m) {
                    float t = (max_len_m - accum) / seg;
                    struct snds_pt cut = { prev.x + (cur.x - prev.x) * t,
                                           prev.y + (cur.y - prev.y) * t };
                    arm->pts[arm->pt_count++] = cut;
                    accum = max_len_m;
                    break;
                }
                arm->pts[arm->pt_count++] = cur;
                accum += seg;
                prev = cur;
            }
            idx += step;
        }
        if (accum >= max_len_m || arm->pt_count >= SNDS_MAX_ARM_PTS)
            break;

        /* Follow straightest continuation link (e.g. past a 15m divided-median crossover). */
        uint64_t far_node = link_far_node(l, cur_rev);
        struct snds_pt far_pt = link_end_pt(l, cur_rev);
        float exit_hdg = link_heading(l, cur_rev, 1);
        int next_idx = -1, next_rev = 0;
        float best_dh = 40.0f;
        for (uint16_t i = 0u; i < g->link_count; ++i) {
            int dup = 0;
            for (int s = 0; s < seen_n; ++s) {
                if (seen[s] == (int)i) { dup = 1; break; }
            }
            if (dup) continue;
            const struct snds_link *cand = &g->links[i];
            if (cand->pt_count < 2u || cand->is_roundabout || cand->frc >= 7u) continue;
            int crev = link_touches_node(cand, far_node, far_pt);
            if (crev < 0) continue;
            float dh = m_fabsf(wrap_deg_180(link_heading(cand, crev, 0) - exit_hdg));
            if (dh < best_dh) {
                best_dh = dh;
                next_idx = (int)i;
                next_rev = crev;
            }
        }
        if (next_idx < 0) break;
        cur_idx = next_idx;
        cur_rev = next_rev;
        if (seen_n < 4) seen[seen_n++] = next_idx;
    }
}

/* Distance from point P to segment AB, plus fractional projection t in [0..1]. */
static float pt_seg_dist(struct snds_pt p, struct snds_pt a, struct snds_pt b, float *out_t) {
    float dx = b.x - a.x, dy = b.y - a.y;
    float len2 = dx * dx + dy * dy;
    float t = 0.0f;
    if (len2 > 1e-6f) {
        t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    if (out_t) *out_t = t;
    struct snds_pt proj = { a.x + t * dx, a.y + t * dy };
    return pt_dist(p, proj);
}

/* Snap car position/heading to the closest direction-aligned link in the graph. */
static int snap_car_to_link(const struct snds_graph *g, const struct snds_query *q,
                            int *out_idx, int *out_rev, float *out_rem_m) {
    struct snds_pt car = { q->car_x, q->car_y };
    float best_cost = 1e9f;
    int best_idx = -1, best_rev = 0;
    float best_rem = 0.0f;

    for (uint16_t i = 0u; i < g->link_count; ++i) {
        const struct snds_link *l = &g->links[i];
        if (l->pt_count < 2u || l->is_roundabout) continue;
        float total_len = link_compute_length(l);

        for (int rev = 0; rev < 2; ++rev) {
            if (!link_allows_dir(l, rev)) continue;
            float accum = 0.0f;
            for (uint8_t k = 1u; k < l->pt_count; ++k) {
                struct snds_pt a = l->pts[k - 1u];
                struct snds_pt b = l->pts[k];
                float seg = pt_dist(a, b);
                float t = 0.0f;
                float d = pt_seg_dist(car, a, b, &t);
                float seg_hdg = rev ? vec_heading_deg(a.x - b.x, a.y - b.y)
                                    : vec_heading_deg(b.x - a.x, b.y - a.y);
                float dh = m_fabsf(wrap_deg_180(seg_hdg - q->car_heading_deg));
                if (d < 40.0f && dh < 55.0f) {
                    float frc_pen = (l->frc >= 7u) ? 22.0f : (l->frc >= 6u ? 12.0f : 0.0f);
                    float cost = d + dh * 0.45f + frc_pen;
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_idx = (int)i;
                        best_rev = rev;
                        float along = accum + t * seg;
                        best_rem = rev ? along : (total_len - along);
                    }
                }
                accum += seg;
            }
        }
    }
    if (best_idx < 0) return 0;
    *out_idx = best_idx;
    *out_rev = best_rev;
    *out_rem_m = best_rem;
    return 1;
}

/* Trace a roundabout starting at entry_node, collecting the full closed ring
 * and every non-roundabout exit arm radiating off the ring in travel order! */
static int match_roundabout_at_node(const struct snds_graph *g,
                                    const struct snds_query *q,
                                    uint64_t entry_node,
                                    int approach_link_idx,
                                    int approach_rev,
                                    float approach_hdg,
                                    const struct snds_polyline *approach_poly,
                                    struct snds_junction *out) {
    /* Walk around the closed roundabout ring (max 16 links).
     * First pass respects one_way flags; fallback pass traces undirected in the AA
     * circulation direction in case a tile encodes roundabout one-way reversed. */
    struct snds_pt entry_pt = link_end_pt(&g->links[approach_link_idx], approach_rev);
    int cw = is_roundabout_cw_aa(q->aa_type);
    int ring_links[16];
    int ring_revs[16];
    uint64_t ring_nodes[17];
    struct snds_pt ring_pts[17];
    int ring_count = 0;

    for (int pass = 0; pass < 2; ++pass) {
        int strict_dir = (pass == 0);
        ring_count = 0;
        uint64_t cur_node = entry_node;
        struct snds_pt cur_pt = entry_pt;
        int visited[SNDS_MAX_GRAPH_LINKS];
        memset(visited, 0, sizeof(visited));

        while (ring_count < 16) {
            int next_idx = -1, next_rev = 0;
            float best_turn = 1e9f;
            for (uint16_t i = 0u; i < g->link_count; ++i) {
                if (visited[i]) continue;
                const struct snds_link *l = &g->links[i];
                if (!l->is_roundabout || l->pt_count < 2u) continue;
                int rev = link_touches_node(l, cur_node, cur_pt);
                if (rev < 0) continue;
                if (strict_dir && !link_allows_dir(l, rev)) continue;
                if (ring_count == 0) {
                    /* First ring link should deflect right for CCW (US/EU RHT) or left for CW (UK LHT). */
                    float turn = wrap_deg_180(link_heading(l, rev, 0) - approach_hdg);
                    float pref = cw ? -55.0f : 55.0f;
                    float diff = m_fabsf(wrap_deg_180(turn - pref));
                    if (diff < best_turn) { best_turn = diff; next_idx = (int)i; next_rev = rev; }
                } else {
                    next_idx = (int)i; next_rev = rev; break;
                }
            }
            if (next_idx < 0) break;
            visited[next_idx] = 1;
            ring_links[ring_count] = next_idx;
            ring_revs[ring_count] = next_rev;
            ring_nodes[ring_count] = cur_node;
            ring_pts[ring_count] = cur_pt;
            ring_count++;
            cur_node = link_far_node(&g->links[next_idx], next_rev);
            cur_pt = link_end_pt(&g->links[next_idx], next_rev);
            if (cur_node == entry_node || pt_dist(cur_pt, entry_pt) <= 2.5f) {
                cur_node = entry_node;
                break;
            }
        }
        if (ring_count >= 2 && cur_node == entry_node)
            break;
        ring_count = 0;
    }
    if (ring_count < 2) return 0;
    ring_nodes[ring_count] = entry_node;
    ring_pts[ring_count] = entry_pt;

    /* Build closed ring polyline & compute centroid + average radius. */
    struct snds_polyline ring_poly;
    memset(&ring_poly, 0, sizeof(ring_poly));
    float cx = 0.0f, cy = 0.0f;
    for (int i = 0; i < ring_count; ++i)
        poly_append_link(&ring_poly, &g->links[ring_links[i]], ring_revs[i]);
    if (ring_poly.pt_count < 3u) return 0;
    for (uint8_t i = 0u; i < ring_poly.pt_count; ++i) {
        cx += ring_poly.pts[i].x;
        cy += ring_poly.pts[i].y;
    }
    cx /= (float)ring_poly.pt_count;
    cy /= (float)ring_poly.pt_count;
    float r_sum = 0.0f;
    for (uint8_t i = 0u; i < ring_poly.pt_count; ++i)
        r_sum += pt_dist((struct snds_pt){cx, cy}, ring_poly.pts[i]);
    float avg_r = r_sum / (float)ring_poly.pt_count;

    /* Collect all connected non-roundabout arms around the ring in circulation order! */
    struct {
        int link_idx;
        int rev;
        int ring_step; /* ring_nodes index where this arm attaches (1..ring_count) */
        int is_exit;   /* 1 if traversable outward from the ring */
        float travel_deg; /* angular sweep from entry around centroid in circulation dir */
    } cand_arms[16];
    int cand_count = 0;

    float entry_bearing = vec_heading_deg(entry_pt.x - cx, entry_pt.y - cy);

    for (int step = 1; step <= ring_count; ++step) {
        uint64_t node = ring_nodes[step % ring_count];
        struct snds_pt rpt = ring_pts[step % ring_count];
        for (uint16_t i = 0u; i < g->link_count && cand_count < 16; ++i) {
            const struct snds_link *l = &g->links[i];
            if (l->is_roundabout || l->pt_count < 2u) continue;
            if ((int)i == approach_link_idx) continue;
            int rev = link_touches_node(l, node, rpt);
            if (rev < 0) continue;
            if (l->frc >= 7u && g->links[approach_link_idx].frc < 7u) continue; /* drop driveways on non-residential roundabouts */

            struct snds_pt arm_pt = link_start_pt(l, rev);
            float b = vec_heading_deg(arm_pt.x - cx, arm_pt.y - cy);
            /* Circulation angle from entry: CCW decreases compass bearing, CW increases. */
            float sweep = cw ? wrap_deg_360(b - entry_bearing)
                             : wrap_deg_360(entry_bearing - b);
            if (sweep < 15.0f && step == ring_count) sweep = 360.0f;

            cand_arms[cand_count].link_idx = (int)i;
            cand_arms[cand_count].rev = rev;
            cand_arms[cand_count].ring_step = step;
            cand_arms[cand_count].is_exit = link_allows_dir(l, rev);
            cand_arms[cand_count].travel_deg = sweep;
            cand_count++;
        }
    }
    if (cand_count == 0) return 0;

    /* Pick the guided exit arm by scoring exit_number, exit_angle, and road_name. */
    int best_arm = -1;
    int best_score = -1000;
    int exit_ordinal = 0;
    for (int k = 0; k < cand_count; ++k) {
        if (!cand_arms[k].is_exit) continue;
        exit_ordinal++;
        int score = 40;
        if (q->exit_number > 0) {
            int diff = abs(exit_ordinal - q->exit_number);
            if (diff == 0) score += 45;
            else if (diff == 1) score += 10;
            else score -= 25 * diff;
        }
        if (q->exit_angle > 0 && q->exit_angle <= 360) {
            float adiff = m_fabsf(wrap_deg_180(cand_arms[k].travel_deg - (float)q->exit_angle));
            if (adiff < 30.0f) score += 35;
            else if (adiff < 55.0f) score += 15;
            else score -= 20;
        }
        if (q->road_name[0]) {
            int ns = snds_road_name_score(q->road_name, g->links[cand_arms[k].link_idx].road_name);
            score += ns / 3;
        }
        if (score > best_score) {
            best_score = score;
            best_arm = k;
        }
    }
    if (best_arm < 0) return 0;

    /* Populate clean snds_junction! */
    memset(out, 0, sizeof(*out));
    out->valid = 1u;
    out->is_roundabout = 1u;
    out->roundabout_cw = (uint8_t)cw;
    out->center.x = cx;
    out->center.y = cy;
    out->roundabout_radius_m = avg_r;
    out->approach_heading_deg = approach_hdg;
    out->approach = *approach_poly;
    out->ring = ring_poly;

    /* Guided spine: approach -> roundabout arc up to chosen exit -> exit arm! */
    out->guided = *approach_poly;
    int exit_step = cand_arms[best_arm].ring_step;
    for (int s = 0; s < exit_step && s < ring_count; ++s)
        poly_append_link(&out->guided, &g->links[ring_links[s]], ring_revs[s]);

    struct snds_arm guided_exit;
    arm_from_link(g, &guided_exit, cand_arms[best_arm].link_idx,
                  cand_arms[best_arm].rev, 70.0f, approach_hdg);
    for (uint8_t j = 0u; j < guided_exit.pt_count; ++j)
        poly_append_pt(&out->guided, guided_exit.pts[j]);

    /* Non-guided arms: all other directly connected arms around the ring. */
    for (int k = 0; k < cand_count && out->arm_count < SNDS_MAX_ARMS; ++k) {
        if (k == best_arm) continue;
        arm_from_link(g, &out->arms[out->arm_count], cand_arms[k].link_idx,
                      cand_arms[k].rev, 115.0f, approach_hdg);
        out->arm_count++;
    }

    int conf = 60 + (best_score > 40 ? (best_score - 40) / 2 : 0);
    if (conf > 98) conf = 98;
    if (conf < 50) conf = 50;
    out->confidence = (uint8_t)conf;
    return 1;
}

/* Match a standard turn / fork / off-ramp at a multi-way intersection node. */
static int match_turn_at_node(const struct snds_graph *g,
                              const struct snds_query *q,
                              uint64_t node,
                              struct snds_pt node_pt,
                              int approach_link_idx,
                              float approach_hdg,
                              float cum_dist_m,
                              const struct snds_polyline *approach_poly,
                              struct snds_junction *out) {
    float exp_turn = expected_turn_deg(q->aa_type);
    int best_idx = -1, best_rev = 0;
    int best_score = -1000;
    float best_adiff = 180.0f;

    int connected_links[12];
    int connected_revs[12];
    int conn_count = 0;

    uint8_t app_frc = g->links[approach_link_idx].frc;
    for (uint16_t i = 0u; i < g->link_count && conn_count < 12; ++i) {
        if ((int)i == approach_link_idx) continue;
        const struct snds_link *l = &g->links[i];
        if (l->pt_count < 2u || l->frc > 7u) continue;
        /* Ignore FRC 7 driveways/alleys on major arterials unless they match the turn direction. */
        int rev = link_touches_node(l, node, node_pt);
        if (rev < 0) continue;

        float out_hdg = link_heading(l, rev, 0);
        float turn = wrap_deg_180(out_hdg - approach_hdg);
        float adiff = m_fabsf(wrap_deg_180(turn - exp_turn));
        int allow_as_arm = (l->frc < 7u || app_frc >= 6u || adiff <= 50.0f);
        if (!allow_as_arm) continue;

        connected_links[conn_count] = (int)i;
        connected_revs[conn_count] = rev;
        conn_count++;

        if (!link_allows_dir(l, rev)) continue;

        /* Require direction consistency (left turn must turn left, right must turn right). */
        if (adiff > 60.0f) continue;

        /* Un-saturated scoring (max ~98 before road-name bonus) so an exact distance match
         * always decisively beats an earlier/later consecutive turn with the same angle
         * (e.g. Michaels entrance 35m before Walton Grove). */
        int score = 42;
        score += (int)((60.0f - adiff) * 0.45f); /* up to +27 for tight angle match */
        float ddiff = m_fabsf(cum_dist_m - q->dist_m);
        float tol = q->dist_m * 0.45f + 40.0f;
        if (ddiff < tol)
            score += (int)((1.0f - ddiff / tol) * 26.0f); /* up to +26 for exact distance */
        if (q->road_name[0])
            score += snds_road_name_score(q->road_name, l->road_name) / 3;
        if (l->frc <= 5u) score += 3;

        if (score > best_score) {
            best_score = score;
            best_idx = (int)i;
            best_rev = rev;
            best_adiff = adiff;
        }
    }

    if (best_idx < 0) return 0;
    if (conn_count < 2 && !(best_adiff < 45.0f && best_score >= 60)) return 0;

    memset(out, 0, sizeof(*out));
    out->valid = 1u;
    out->is_roundabout = 0u;
    out->center = link_start_pt(&g->links[best_idx], best_rev);
    out->approach_heading_deg = approach_hdg;
    out->approach = *approach_poly;
    out->guided = *approach_poly;

    struct snds_arm exit_arm;
    arm_from_link(g, &exit_arm, best_idx, best_rev, 55.0f, approach_hdg);
    for (uint8_t j = 0u; j < exit_arm.pt_count; ++j)
        poly_append_pt(&out->guided, exit_arm.pts[j]);

    for (int k = 0; k < conn_count && out->arm_count < SNDS_MAX_ARMS; ++k) {
        if (connected_links[k] == best_idx) continue;
        arm_from_link(g, &out->arms[out->arm_count], connected_links[k],
                      connected_revs[k], 95.0f, approach_hdg);
        out->arm_count++;
    }

    /* Also emit the full-length underlying exit road into context[0] so the target
     * cross street visibly continues beyond the tip of the guided arrow. */
    if (out->context_count < SNDS_MAX_CONTEXT) {
        arm_from_link(g, &out->context[out->context_count], best_idx, best_rev,
                      95.0f, approach_hdg);
        if (out->context[out->context_count].pt_count >= 2u)
            out->context_count++;
    }

    int conf = best_score;
    if (conf > 98) conf = 98;
    if (conf < 40) conf = 40;
    out->confidence = (uint8_t)conf;
    return 1;
}

/* Minimum distance from point P to a polyline of pts[0..n-1]. */
static float pt_poly_dist(struct snds_pt p, const struct snds_pt *pts, uint8_t n) {
    float best = 1e9f;
    for (uint8_t k = 1u; k < n; ++k) {
        float d = pt_seg_dist(p, pts[k - 1u], pts[k], NULL);
        if (d < best) best = d;
    }
    return best;
}

static int link_already_covered(const struct snds_link *l, const struct snds_junction *j) {
    if (l->pt_count < 2u) return 1;
    struct snds_pt mid = {
        0.5f * (l->pts[0].x + l->pts[l->pt_count - 1u].x),
        0.5f * (l->pts[0].y + l->pts[l->pt_count - 1u].y),
    };
    if (l->pt_count >= 3u)
        mid = l->pts[l->pt_count / 2u];
    const float tol = 3.2f;
    if (j->guided.pt_count >= 2u && pt_poly_dist(mid, j->guided.pts, j->guided.pt_count) < tol)
        return 1;
    if (j->ring.pt_count >= 2u && pt_poly_dist(mid, j->ring.pts, j->ring.pt_count) < tol)
        return 1;
    for (uint8_t a = 0u; a < j->arm_count; ++a) {
        if (j->arms[a].pt_count >= 2u &&
            pt_poly_dist(mid, j->arms[a].pts, j->arms[a].pt_count) < tol)
            return 1;
    }
    for (uint8_t c = 0u; c < j->context_count; ++c) {
        if (j->context[c].pt_count >= 2u &&
            pt_poly_dist(mid, j->context[c].pts, j->context[c].pt_count) < tol)
            return 1;
    }
    return 0;
}

static void arm_append_pts(struct snds_arm *arm, const struct snds_link *l, int rev) {
    if (!rev) {
        for (uint8_t k = 0u; k < l->pt_count && arm->pt_count < SNDS_MAX_ARM_PTS; ++k) {
            if (arm->pt_count == 0u || pt_dist(arm->pts[arm->pt_count - 1u], l->pts[k]) >= 1.0f)
                arm->pts[arm->pt_count++] = l->pts[k];
        }
    } else {
        for (int k = (int)l->pt_count - 1; k >= 0 && arm->pt_count < SNDS_MAX_ARM_PTS; --k) {
            if (arm->pt_count == 0u || pt_dist(arm->pts[arm->pt_count - 1u], l->pts[k]) >= 1.0f)
                arm->pts[arm->pt_count++] = l->pts[k];
        }
    }
}

/* Emit a single 1-hop road link trimmed to max_len_m (NEVER chains into 2nd-hop
 * parking-lot aisles or side-lot ovals). */
static void single_link_stub(struct snds_arm *arm, const struct snds_link *l, int rev,
                             float max_len_m, float approach_hdg) {
    arm->pt_count = 0u;
    arm->exit_angle_deg = wrap_deg_180(link_heading(l, rev, 0) - approach_hdg);
    float accum = 0.0f;
    struct snds_pt prev = link_start_pt(l, rev);
    arm->pts[arm->pt_count++] = prev;
    int step = rev ? -1 : 1;
    int idx = rev ? (int)l->pt_count - 2 : 1;
    while (idx >= 0 && idx < (int)l->pt_count && arm->pt_count < SNDS_MAX_ARM_PTS) {
        struct snds_pt cur = l->pts[idx];
        float seg = pt_dist(prev, cur);
        if (seg > 0.2f) {
            if (accum + seg > max_len_m) {
                float t = (max_len_m - accum) / seg;
                struct snds_pt cut = { prev.x + (cur.x - prev.x) * t,
                                       prev.y + (cur.y - prev.y) * t };
                arm->pts[arm->pt_count++] = cut;
                break;
            }
            arm->pts[arm->pt_count++] = cur;
            accum += seg;
            prev = cur;
        }
        idx += step;
    }
}

/* Collect ONLY high-signal, non-cluttered supporting context roads:
 *   1. 1-hop branches directly touching intermediate nodes along the approach path
 *      within 70m of the junction (e.g. Michaels/Walgreens plaza entrance right before
 *      Walton Grove), trimmed to crisp 35m stubs (NEVER chaining 2+ hops into parking
 *      lots or drawing non-guided drop-off ovals!).
 *   2. Immediate parallel opposing carriageway (FRC <= 4 arterial/highway only) running
 *      within 6..26m laterally of the approach (e.g. dual-carriageway Spring Rd SE).
 *   Zero disconnected / floating roads or distant side-lot loops! */
static void collect_context_roads(const struct snds_graph *g,
                                  uint8_t approach_frc,
                                  struct snds_junction *out) {
    enum { MAX_CONTEXT_EMIT = 6 };
    uint8_t used[SNDS_MAX_GRAPH_LINKS];
    memset(used, 0, sizeof(used));

    for (uint16_t i = 0u; i < g->link_count; ++i) {
        const struct snds_link *l = &g->links[i];
        if (l->pt_count < 2u || l->is_roundabout || l->frc > 7u) { used[i] = 1u; continue; }
        if (link_already_covered(l, out)) { used[i] = 1u; continue; }
    }

    /* 1. 1-hop branches directly touching intermediate approach vertices (within 70m of center). */
    uint8_t max_branch_frc = (approach_frc >= 6u) ? 7u : 6u;
    for (uint8_t k = 0u; k < out->approach.pt_count && out->context_count < MAX_CONTEXT_EMIT; ++k) {
        struct snds_pt apt = out->approach.pts[k];
        float d_ctr = pt_dist(apt, out->center);
        if (d_ctr < 6.0f || d_ctr > 70.0f) continue; /* skip turn node itself & far behind */

        for (uint16_t i = 0u; i < g->link_count && out->context_count < MAX_CONTEXT_EMIT; ++i) {
            if (used[i]) continue;
            const struct snds_link *l = &g->links[i];
            if (l->frc > max_branch_frc) continue;
            int rev = link_touches_node(l, 0ULL, apt);
            if (rev < 0) continue;

            /* Skip nearly-collinear segments (already part of the approach corridor). */
            float bhdg = link_heading(l, rev, 0);
            float dh_fwd = m_fabsf(wrap_deg_180(bhdg - out->approach_heading_deg));
            float dh_bwd = m_fabsf(wrap_deg_180(bhdg - (out->approach_heading_deg + 180.0f)));
            if (dh_fwd < 22.0f || dh_bwd < 22.0f) continue;

            /* If this 1-hop link is a major slip ramp whose far end connects directly to one
             * of the junction crossing arms, keep up to 70m; otherwise trim to crisp 35m stub. */
            float stub_len = 35.0f;
            struct snds_pt far_pt = link_end_pt(l, rev);
            if (l->frc <= 5u) {
                for (uint8_t a = 0u; a < out->arm_count; ++a) {
                    if (out->arms[a].pt_count >= 2u &&
                        pt_poly_dist(far_pt, out->arms[a].pts, out->arms[a].pt_count) <= 10.0f) {
                        stub_len = 70.0f;
                        break;
                    }
                }
            }

            struct snds_arm *ctx = &out->context[out->context_count];
            memset(ctx, 0, sizeof(*ctx));
            single_link_stub(ctx, l, rev, stub_len, out->approach_heading_deg);
            used[i] = 1u;
            if (ctx->pt_count >= 2u) {
                float slen = 0.0f;
                for (uint8_t p = 1u; p < ctx->pt_count; ++p)
                    slen += pt_dist(ctx->pts[p - 1u], ctx->pts[p]);
                if (slen >= 7.0f)
                    out->context_count++;
            }
        }
    }

    /* 2. Immediate parallel opposing carriageway (FRC <= 4 arterial/highway only, 6..26m lateral). */
    if (out->context_count < MAX_CONTEXT_EMIT && !out->is_roundabout) {
        float h_rad = out->approach_heading_deg * (float)(M_PI / 180.0);
        float cos_h = m_cosf(h_rad);
        float sin_h = m_sinf(h_rad);
        int par_seed = -1;
        float best_lat = 26.0f;

        for (uint16_t i = 0u; i < g->link_count; ++i) {
            if (used[i]) continue;
            const struct snds_link *l = &g->links[i];
            if (l->frc > 4u || l->is_roundabout) continue;
            float lhdg = link_heading(l, 0, 0);
            float d1 = m_fabsf(wrap_deg_180(lhdg - out->approach_heading_deg));
            float d2 = m_fabsf(wrap_deg_180(lhdg - (out->approach_heading_deg + 180.0f)));
            if (d1 > 22.0f && d2 > 22.0f) continue;

            struct snds_pt mid = l->pts[l->pt_count / 2u];
            float dx = mid.x - out->center.x;
            float dy = mid.y - out->center.y;
            float lat = m_fabsf(dx * cos_h - dy * sin_h);
            float lon = dx * sin_h + dy * cos_h;
            if (lat >= 6.0f && lat <= best_lat && lon >= -80.0f && lon <= 80.0f) {
                best_lat = lat;
                par_seed = (int)i;
            }
        }

        if (par_seed >= 0) {
            int chain_idx[10], chain_rev[10];
            int head = 5, tail = 5;
            chain_idx[head] = par_seed;
            chain_rev[head] = 0;
            used[par_seed] = 1u;

            for (int hop = 0; hop < 4 && head > 0; ++hop) {
                const struct snds_link *cl = &g->links[chain_idx[head]];
                int crev = chain_rev[head];
                uint64_t snode = crev ? cl->end_node : cl->start_node;
                struct snds_pt spt = link_start_pt(cl, crev);
                if (pt_dist(spt, out->center) > 105.0f) break;
                float shdg = link_heading(cl, crev, 0);
                int prev_i = -1, prev_rev = 0;
                float best_dh = 25.0f;
                for (uint16_t i = 0u; i < g->link_count; ++i) {
                    if (used[i]) continue;
                    const struct snds_link *cand = &g->links[i];
                    if (cand->frc > 4u || cand->is_roundabout) continue;
                    int touch = link_touches_node(cand, snode, spt);
                    if (touch < 0) continue;
                    int pdir = 1 - touch;
                    float dh = m_fabsf(wrap_deg_180(link_heading(cand, pdir, 1) - shdg));
                    if (dh < best_dh) { best_dh = dh; prev_i = (int)i; prev_rev = pdir; }
                }
                if (prev_i < 0) break;
                used[prev_i] = 1u;
                head--;
                chain_idx[head] = prev_i;
                chain_rev[head] = prev_rev;
            }

            for (int hop = 0; hop < 4 && tail < 9; ++hop) {
                const struct snds_link *cl = &g->links[chain_idx[tail]];
                int crev = chain_rev[tail];
                uint64_t enode = link_far_node(cl, crev);
                struct snds_pt ept = link_end_pt(cl, crev);
                if (pt_dist(ept, out->center) > 105.0f) break;
                float ehdg = link_heading(cl, crev, 1);
                int next_i = -1, next_rev = 0;
                float best_dh = 25.0f;
                for (uint16_t i = 0u; i < g->link_count; ++i) {
                    if (used[i]) continue;
                    const struct snds_link *cand = &g->links[i];
                    if (cand->frc > 4u || cand->is_roundabout) continue;
                    int crev2 = link_touches_node(cand, enode, ept);
                    if (crev2 < 0) continue;
                    float dh = m_fabsf(wrap_deg_180(link_heading(cand, crev2, 0) - ehdg));
                    if (dh < best_dh) { best_dh = dh; next_i = (int)i; next_rev = crev2; }
                }
                if (next_i < 0) break;
                used[next_i] = 1u;
                tail++;
                chain_idx[tail] = next_i;
                chain_rev[tail] = next_rev;
            }

            struct snds_arm *ctx = &out->context[out->context_count];
            memset(ctx, 0, sizeof(*ctx));
            for (int k = head; k <= tail; ++k)
                arm_append_pts(ctx, &g->links[chain_idx[k]], chain_rev[k]);
            if (ctx->pt_count >= 2u)
                out->context_count++;
        }
    }
}

/* Prepend backward approach links if the approach starts closer than min_back_m to center. */
static void extend_approach_backward(const struct snds_graph *g,
                                     int snap_idx, int snap_rev,
                                     float min_back_m,
                                     struct snds_junction *out) {
    if (out->guided.pt_count < 2u) return;
    if (pt_dist(out->guided.pts[0], out->center) >= min_back_m) return;

    int back_idx[3], back_rev[3];
    int back_n = 0;
    int cur_i = snap_idx, cur_r = snap_rev;
    for (int hop = 0; hop < 3; ++hop) {
        const struct snds_link *cl = &g->links[cur_i];
        uint64_t snode = cur_r ? cl->end_node : cl->start_node;
        struct snds_pt spt = link_start_pt(cl, cur_r);
        if (pt_dist(spt, out->center) >= min_back_m) break;
        float shdg = link_heading(cl, cur_r, 0);
        int prev_i = -1, prev_r = 0;
        float best_dh = 40.0f;
        for (uint16_t i = 0u; i < g->link_count; ++i) {
            if ((int)i == cur_i || (int)i == snap_idx) continue;
            const struct snds_link *cand = &g->links[i];
            if (cand->pt_count < 2u || cand->is_roundabout || cand->frc > 7u) continue;
            int touch = link_touches_node(cand, snode, spt);
            if (touch < 0) continue;
            int pdir = 1 - touch;
            if (!link_allows_dir(cand, pdir)) continue;
            float dh = m_fabsf(wrap_deg_180(link_heading(cand, pdir, 1) - shdg));
            if (dh < best_dh) { best_dh = dh; prev_i = (int)i; prev_r = pdir; }
        }
        if (prev_i < 0) break;
        back_idx[back_n] = prev_i;
        back_rev[back_n] = prev_r;
        back_n++;
        cur_i = prev_i;
        cur_r = prev_r;
    }
    if (back_n == 0) return;

    struct snds_polyline rebuilt;
    memset(&rebuilt, 0, sizeof(rebuilt));
    for (int k = back_n - 1; k >= 0; --k)
        poly_append_link(&rebuilt, &g->links[back_idx[k]], back_rev[k]);
    for (uint8_t k = 0u; k < out->guided.pt_count; ++k)
        poly_append_pt(&rebuilt, out->guided.pts[k]);
    if (rebuilt.pt_count >= 2u)
        out->guided = rebuilt;

    struct snds_polyline rebuilt_app;
    memset(&rebuilt_app, 0, sizeof(rebuilt_app));
    for (int k = back_n - 1; k >= 0; --k)
        poly_append_link(&rebuilt_app, &g->links[back_idx[k]], back_rev[k]);
    for (uint8_t k = 0u; k < out->approach.pt_count; ++k)
        poly_append_pt(&rebuilt_app, out->approach.pts[k]);
    if (rebuilt_app.pt_count >= 2u)
        out->approach = rebuilt_app;
}

int snds_match_maneuver(const struct snds_graph *graph,
                        const struct snds_query *query,
                        struct snds_junction *out) {
    if (!graph || !query || !out || graph->link_count == 0u)
        return 0;
    memset(out, 0, sizeof(*out));
    if (is_non_maneuver_aa(query->aa_type))
        return 0;

    int cur_idx = -1, cur_rev = 0;
    float rem_m = 0.0f;
    if (!snap_car_to_link(graph, query, &cur_idx, &cur_rev, &rem_m))
        return 0;

    int snap_idx = cur_idx, snap_rev = cur_rev;
    uint8_t car_lanes = graph->links[cur_idx].lanes;
    const int want_rb = is_roundabout_aa(query->aa_type);
    const float max_walk_m = query->dist_m * 1.5f + 80.0f;
    const float min_match_m = query->dist_m > 50.0f ? query->dist_m * 0.45f : 0.0f;

    struct snds_polyline approach;
    memset(&approach, 0, sizeof(approach));
    poly_append_link(&approach, &graph->links[cur_idx], cur_rev);

    float cum_m = rem_m;
    int visited[SNDS_MAX_GRAPH_LINKS];
    memset(visited, 0, sizeof(visited));
    visited[cur_idx] = 1;

    struct snds_junction best;
    memset(&best, 0, sizeof(best));

    for (int hops = 0; hops < 24 && cum_m <= max_walk_m; ++hops) {
        const struct snds_link *cur_link = &graph->links[cur_idx];
        uint64_t node = link_far_node(cur_link, cur_rev);
        struct snds_pt node_pt = link_end_pt(cur_link, cur_rev);
        float app_hdg = link_heading(cur_link, cur_rev, 1);

        if (cum_m >= min_match_m) {
            struct snds_junction cand;
            if (want_rb) {
                if (match_roundabout_at_node(graph, query, node, cur_idx, cur_rev,
                                             app_hdg, &approach, &cand)) {
                    if (!best.valid || cand.confidence > best.confidence)
                        best = cand;
                    break; /* First roundabout along approach within window */
                }
            } else {
                if (match_turn_at_node(graph, query, node, node_pt, cur_idx, app_hdg,
                                       cum_m, &approach, &cand)) {
                    if (!best.valid || cand.confidence > best.confidence)
                        best = cand;
                }
            }
        }

        /* Continue straightest along the current road to the next node. */
        int next_idx = -1, next_rev = 0;
        float best_straight_cost = 1e9f;
        for (uint16_t i = 0u; i < graph->link_count; ++i) {
            if (visited[i]) continue;
            const struct snds_link *l = &graph->links[i];
            if (l->pt_count < 2u || l->is_roundabout || l->frc > 7u) continue;
            int rev = link_touches_node(l, node, node_pt);
            if (rev < 0 || !link_allows_dir(l, rev)) continue;

            float dh = m_fabsf(wrap_deg_180(link_heading(l, rev, 0) - app_hdg));
            if (dh > 55.0f) continue;
            float cost = dh + (float)abs((int)l->frc - (int)cur_link->frc) * 10.0f;
            if (snds_road_name_score(cur_link->road_name, l->road_name) >= 80)
                cost -= 15.0f;
            if (cost < best_straight_cost) {
                best_straight_cost = cost;
                next_idx = (int)i;
                next_rev = rev;
            }
        }
        if (next_idx < 0) break;
        visited[next_idx] = 1;
        cur_idx = next_idx;
        cur_rev = next_rev;
        cum_m += link_compute_length(&graph->links[cur_idx]);
        poly_append_link(&approach, &graph->links[cur_idx], cur_rev);
    }

    if (!best.valid) return 0;
    extend_approach_backward(graph, snap_idx, snap_rev, 95.0f, &best);
    collect_context_roads(graph, graph->links[snap_idx].frc, &best);
    best.current_link_lanes = car_lanes;
    *out = best;
    return 1;
}
