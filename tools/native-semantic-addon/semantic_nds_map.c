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
    return (aa_type >= 32u && aa_type <= 35u) || (aa_type >= 43u && aa_type <= 46u);
}

static int is_roundabout_cw_aa(uint16_t aa_type) {
    return aa_type == 32u || aa_type == 33u || aa_type == 43u || aa_type == 44u;
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

static void arm_from_link(struct snds_arm *arm, const struct snds_link *l, int rev,
                          float max_len_m, float approach_hdg) {
    arm->pt_count = 0u;
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
    arm->exit_angle_deg = wrap_deg_180(link_heading(l, rev, 0) - approach_hdg);
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
                    float cost = d + dh * 0.45f + (l->frc >= 6u ? 15.0f : 0.0f);
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
    /* Verify at least one roundabout link leaves entry_node. */
    int rb_idx = -1;
    for (uint16_t i = 0u; i < g->link_count; ++i) {
        const struct snds_link *l = &g->links[i];
        if (!l->is_roundabout || l->pt_count < 2u) continue;
        if ((l->start_node == entry_node && link_allows_dir(l, 0)) ||
            (l->end_node == entry_node && link_allows_dir(l, 1))) {
            rb_idx = (int)i; break;
        }
    }
    if (rb_idx < 0) return 0;

    /* Walk around the closed roundabout ring (max 16 links). */
    int ring_links[16];
    int ring_revs[16];
    uint64_t ring_nodes[17];
    int ring_count = 0;
    uint64_t cur_node = entry_node;
    int visited[SNDS_MAX_GRAPH_LINKS];
    memset(visited, 0, sizeof(visited));

    while (ring_count < 16) {
        int next_idx = -1, next_rev = 0;
        for (uint16_t i = 0u; i < g->link_count; ++i) {
            if (visited[i]) continue;
            const struct snds_link *l = &g->links[i];
            if (!l->is_roundabout || l->pt_count < 2u) continue;
            if (l->start_node == cur_node && link_allows_dir(l, 0)) {
                next_idx = (int)i; next_rev = 0; break;
            }
            if (l->end_node == cur_node && link_allows_dir(l, 1)) {
                next_idx = (int)i; next_rev = 1; break;
            }
        }
        if (next_idx < 0) break;
        visited[next_idx] = 1;
        ring_links[ring_count] = next_idx;
        ring_revs[ring_count] = next_rev;
        ring_nodes[ring_count] = cur_node;
        ring_count++;
        cur_node = link_far_node(&g->links[next_idx], next_rev);
        if (cur_node == entry_node) break;
    }
    if (ring_count < 2 || cur_node != entry_node) return 0;
    ring_nodes[ring_count] = entry_node;

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

    int cw = is_roundabout_cw_aa(q->aa_type);
    struct snds_pt entry_pt = link_end_pt(&g->links[approach_link_idx], approach_rev);
    float entry_bearing = vec_heading_deg(entry_pt.x - cx, entry_pt.y - cy);

    for (int step = 1; step <= ring_count; ++step) {
        uint64_t node = ring_nodes[step % ring_count];
        for (uint16_t i = 0u; i < g->link_count && cand_count < 16; ++i) {
            const struct snds_link *l = &g->links[i];
            if (l->is_roundabout || l->pt_count < 2u) continue;
            if ((int)i == approach_link_idx) continue;
            int rev = -1;
            if (l->start_node == node) rev = 0;
            else if (l->end_node == node) rev = 1;
            else continue;
            if (l->frc >= 7u) continue; /* drop driveways/parking alleys */

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
    arm_from_link(&guided_exit, &g->links[cand_arms[best_arm].link_idx],
                  cand_arms[best_arm].rev, 65.0f, approach_hdg);
    for (uint8_t j = 0u; j < guided_exit.pt_count; ++j)
        poly_append_pt(&out->guided, guided_exit.pts[j]);

    /* Non-guided arms: all other directly connected arms around the ring. */
    for (int k = 0; k < cand_count && out->arm_count < SNDS_MAX_ARMS; ++k) {
        if (k == best_arm) continue;
        arm_from_link(&out->arms[out->arm_count], &g->links[cand_arms[k].link_idx],
                      cand_arms[k].rev, 55.0f, approach_hdg);
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
                              int approach_link_idx,
                              float approach_hdg,
                              float cum_dist_m,
                              const struct snds_polyline *approach_poly,
                              struct snds_junction *out) {
    float exp_turn = expected_turn_deg(q->aa_type);
    int best_idx = -1, best_rev = 0;
    int best_score = -1000;

    int connected_links[12];
    int connected_revs[12];
    int conn_count = 0;

    for (uint16_t i = 0u; i < g->link_count && conn_count < 12; ++i) {
        if ((int)i == approach_link_idx) continue;
        const struct snds_link *l = &g->links[i];
        if (l->pt_count < 2u || l->frc >= 7u) continue;
        int rev = -1;
        if (l->start_node == node) rev = 0;
        else if (l->end_node == node) rev = 1;
        else continue;

        connected_links[conn_count] = (int)i;
        connected_revs[conn_count] = rev;
        conn_count++;

        if (!link_allows_dir(l, rev)) continue;
        float out_hdg = link_heading(l, rev, 0);
        float turn = wrap_deg_180(out_hdg - approach_hdg);
        float adiff = m_fabsf(wrap_deg_180(turn - exp_turn));

        /* Require direction consistency (left turn must turn left, right must turn right). */
        if (adiff > 60.0f) continue;

        int score = 50;
        score += (int)((60.0f - adiff) * 0.6f); /* up to +36 for tight angle match */
        float ddiff = m_fabsf(cum_dist_m - q->dist_m);
        float tol = q->dist_m * 0.35f + 35.0f;
        if (ddiff < tol)
            score += (int)((1.0f - ddiff / tol) * 20.0f);
        if (q->road_name[0])
            score += snds_road_name_score(q->road_name, l->road_name) / 3;
        if (l->frc <= 4u) score += 5;

        if (score > best_score) {
            best_score = score;
            best_idx = (int)i;
            best_rev = rev;
        }
    }

    if (best_idx < 0 || conn_count < 2) return 0;

    memset(out, 0, sizeof(*out));
    out->valid = 1u;
    out->is_roundabout = 0u;
    out->center = link_start_pt(&g->links[best_idx], best_rev);
    out->approach_heading_deg = approach_hdg;
    out->approach = *approach_poly;
    out->guided = *approach_poly;

    struct snds_arm exit_arm;
    arm_from_link(&exit_arm, &g->links[best_idx], best_rev, 70.0f, approach_hdg);
    for (uint8_t j = 0u; j < exit_arm.pt_count; ++j)
        poly_append_pt(&out->guided, exit_arm.pts[j]);

    for (int k = 0; k < conn_count && out->arm_count < SNDS_MAX_ARMS; ++k) {
        if (connected_links[k] == best_idx) continue;
        arm_from_link(&out->arms[out->arm_count], &g->links[connected_links[k]],
                      connected_revs[k], 55.0f, approach_hdg);
        out->arm_count++;
    }

    int conf = best_score;
    if (conf > 98) conf = 98;
    if (conf < 40) conf = 40;
    out->confidence = (uint8_t)conf;
    return 1;
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
                if (match_turn_at_node(graph, query, node, cur_idx, app_hdg,
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
            if (l->pt_count < 2u || l->is_roundabout || l->frc >= 7u) continue;
            int rev = -1;
            if (l->start_node == node && link_allows_dir(l, 0)) rev = 0;
            else if (l->end_node == node && link_allows_dir(l, 1)) rev = 1;
            else continue;

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
    best.current_link_lanes = car_lanes;
    *out = best;
    return 1;
}
