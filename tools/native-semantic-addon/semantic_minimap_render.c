/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
#include "semantic_minimap_render.h"

#include "semantic_math.h"
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct fpt { float x, y; };

static float fdist(struct fpt a, struct fpt b) {
    float dx = b.x - a.x, dy = b.y - a.y;
    return m_sqrtf(dx * dx + dy * dy);
}

/* Rotate world point relative to junction center so approach_heading points UP (-Y screen). */
static struct fpt world_to_local(struct snds_pt p, struct snds_pt center,
                                 float cos_h, float sin_h) {
    float dx = p.x - center.x;
    float dy = p.y - center.y;
    /* Approach heading H (0=N, 90=E) has unit vector (sin H, cos H).
     * Right vector is (cos H, -sin H).
     * Screen X = dot(d, right), Screen Y_up = dot(d, forward). */
    struct fpt r;
    r.x = dx * cos_h - dy * sin_h;
    r.y = dx * sin_h + dy * cos_h;
    return r;
}

/* Clip a polyline to circle of radius R (clean CID widget cut), including long
 * chords whose endpoints both lie outside R while crossing through the circle. */
static int clip_polyline_radius(const struct fpt *in, int n, float radius,
                                struct fpt *out, int max_out) {
    if (n < 2 || max_out < 2) return 0;
    const float r2_lim = radius * radius;
    int m = 0;
    /* Find first point inside radius */
    int start = 0;
    while (start < n && (in[start].x * in[start].x + in[start].y * in[start].y) > r2_lim)
        start++;
    if (start >= n) {
        /* All vertices lie outside R: check if any segment pierces the circle as a chord. */
        for (int i = 1; i < n; ++i) {
            struct fpt a = in[i - 1], b = in[i];
            float dx = b.x - a.x, dy = b.y - a.y;
            float A = dx * dx + dy * dy;
            if (A <= 1e-6f) continue;
            float B = 2.0f * (a.x * dx + a.y * dy);
            float C = a.x * a.x + a.y * a.y - r2_lim;
            float disc = B * B - 4.0f * A * C;
            if (disc > 0.0f) {
                float sq = m_sqrtf(disc);
                float t1 = (-B - sq) / (2.0f * A);
                float t2 = (-B + sq) / (2.0f * A);
                if (t1 < 0.0f) t1 = 0.0f;
                if (t2 > 1.0f) t2 = 1.0f;
                if (t2 > t1 + 1e-3f) {
                    out[0] = (struct fpt){ a.x + t1 * dx, a.y + t1 * dy };
                    out[1] = (struct fpt){ a.x + t2 * dx, a.y + t2 * dy };
                    return 2;
                }
            }
        }
        return 0;
    }
    if (start > 0) {
        /* Intersect entering segment with circle */
        struct fpt a = in[start - 1], b = in[start];
        float dx = b.x - a.x, dy = b.y - a.y;
        float A = dx * dx + dy * dy;
        float B = 2.0f * (a.x * dx + a.y * dy);
        float C = a.x * a.x + a.y * a.y - r2_lim;
        float disc = B * B - 4.0f * A * C;
        if (disc >= 0.0f && A > 1e-6f) {
            float t = (-B - m_sqrtf(disc)) / (2.0f * A);
            if (t < 0.0f || t > 1.0f) t = (-B + m_sqrtf(disc)) / (2.0f * A);
            if (t >= 0.0f && t <= 1.0f)
                out[m++] = (struct fpt){ a.x + t * dx, a.y + t * dy };
        }
    }
    for (int i = start; i < n && m < max_out; ++i) {
        struct fpt p = in[i];
        float r2 = p.x * p.x + p.y * p.y;
        if (r2 <= r2_lim) {
            out[m++] = p;
        } else if (m > 0) {
            struct fpt a = out[m - 1];
            float dx = p.x - a.x, dy = p.y - a.y;
            float A = dx * dx + dy * dy;
            float B = 2.0f * (a.x * dx + a.y * dy);
            float C = a.x * a.x + a.y * a.y - r2_lim;
            float disc = B * B - 4.0f * A * C;
            if (disc >= 0.0f && A > 1e-6f) {
                float t = (-B + m_sqrtf(disc)) / (2.0f * A);
                if (t < 0.0f || t > 1.0f) t = (-B - m_sqrtf(disc)) / (2.0f * A);
                if (t >= 0.0f && t <= 1.0f)
                    out[m++] = (struct fpt){ a.x + t * dx, a.y + t * dy };
            }
            break;
        }
    }
    return m;
}

/* Simplify polyline in screen pixel space (iterative Douglas-Peucker / collinear prune). */
static int simplify_screen_poly(const struct fpt *in, int n, float eps,
                                struct fpt *out, int max_out) {
    if (n <= 2) {
        int c = n < max_out ? n : max_out;
        for (int i = 0; i < c; ++i) out[i] = in[i];
        return c;
    }
    uint8_t keep[64];
    int lim = n < 64 ? n : 64;
    memset(keep, 0, (size_t)lim);
    keep[0] = 1;
    keep[lim - 1] = 1;

    int stack_l[64], stack_r[64], sp = 0;
    stack_l[sp] = 0; stack_r[sp] = lim - 1; sp++;
    while (sp > 0) {
        --sp;
        int l = stack_l[sp], r = stack_r[sp];
        if (r <= l + 1) continue;
        struct fpt a = in[l], b = in[r];
        float dx = b.x - a.x, dy = b.y - a.y;
        float len = m_sqrtf(dx * dx + dy * dy);
        float max_d = -1.0f;
        int max_i = -1;
        for (int i = l + 1; i < r; ++i) {
            float d;
            if (len < 1e-4f) {
                d = fdist(in[i], a);
            } else {
                d = m_fabsf((in[i].x - a.x) * dy - (in[i].y - a.y) * dx) / len;
            }
            if (d > max_d) { max_d = d; max_i = i; }
        }
        if (max_d > eps && max_i > l && max_i < r) {
            keep[max_i] = 1;
            if (sp + 2 <= 64) {
                stack_l[sp] = l; stack_r[sp] = max_i; sp++;
                stack_l[sp] = max_i; stack_r[sp] = r; sp++;
            }
        }
    }

    int m = 0;
    for (int i = 0; i < lim && m < max_out; ++i) {
        if (keep[i]) {
            if (m == 0 || fdist(out[m - 1], in[i]) >= 1.2f)
                out[m++] = in[i];
        }
    }
    return m;
}

static uint16_t clamp_u16(float v, float lo, float hi) {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (uint16_t)(v + 0.5f);
}

static int emit_poly(struct semantic_hud_minimap *out, uint8_t color,
                     const struct fpt *pts, int n) {
    if (!out || out->poly_count >= SEMANTIC_HUD_MAX_POLYS || n < 3)
        return 0;
    if (n > (int)SEMANTIC_HUD_MAX_POLY_PTS)
        n = (int)SEMANTIC_HUD_MAX_POLY_PTS;
    struct semantic_hud_poly *p = &out->polys[out->poly_count++];
    p->color = color;
    p->pt_count = (uint8_t)n;
    for (int i = 0; i < n; ++i) {
        p->x[i] = clamp_u16(pts[i].x, 2.0f, (float)(SEMANTIC_HUD_MINIMAP_W - 2u));
        p->y[i] = clamp_u16(pts[i].y, 2.0f, (float)(SEMANTIC_HUD_MINIMAP_H - 2u));
    }
    return 1;
}

/* Convert an open polyline into a clean constant-width miter-clamped ribbon polygon.
 * If extend_end > 0, extends the start/end cap along tangent (used for outer border!). */
static int stroke_ribbon(const struct fpt *line, int n, float half_w,
                         float extend_start, float extend_end,
                         struct fpt *poly_out, int max_out) {
    if (n < 2 || n > 12 || max_out < 2 * n) return 0;
    struct fpt left[14], right[14];
    for (int i = 0; i < n; ++i) {
        float tx = 0.0f, ty = 0.0f;
        if (i == 0) {
            tx = line[1].x - line[0].x;
            ty = line[1].y - line[0].y;
        } else if (i == n - 1) {
            tx = line[n - 1].x - line[n - 2].x;
            ty = line[n - 1].y - line[n - 2].y;
        } else {
            float tx1 = line[i].x - line[i - 1].x, ty1 = line[i].y - line[i - 1].y;
            float l1 = m_sqrtf(tx1 * tx1 + ty1 * ty1);
            if (l1 > 1e-4f) { tx1 /= l1; ty1 /= l1; }
            float tx2 = line[i + 1].x - line[i].x, ty2 = line[i + 1].y - line[i].y;
            float l2 = m_sqrtf(tx2 * tx2 + ty2 * ty2);
            if (l2 > 1e-4f) { tx2 /= l2; ty2 /= l2; }
            tx = tx1 + tx2; ty = ty1 + ty2;
            if (tx * tx + ty * ty < 1e-4f) { tx = tx1; ty = ty1; }
        }
        float tl = m_sqrtf(tx * tx + ty * ty);
        if (tl < 1e-4f) { tx = 0.0f; ty = -1.0f; tl = 1.0f; }
        tx /= tl; ty /= tl;

        /* Normal vector perpendicular to tangent */
        float nx = -ty, ny = tx;
        float scale = 1.0f;
        if (i > 0 && i < n - 1) {
            float sx = line[i].x - line[i - 1].x, sy = line[i].y - line[i - 1].y;
            float sl = m_sqrtf(sx * sx + sy * sy);
            if (sl > 1e-4f) {
                float snx = -sy / sl, sny = sx / sl;
                float dot = nx * snx + ny * sny;
                if (dot > 0.45f) scale = 1.0f / dot;
                else scale = 1.8f;
                if (scale > 1.8f) scale = 1.8f;
            }
        }
        struct fpt base = line[i];
        if (i == 0 && extend_start != 0.0f) {
            base.x -= tx * extend_start;
            base.y -= ty * extend_start;
        } else if (i == n - 1 && extend_end != 0.0f) {
            base.x += tx * extend_end;
            base.y += ty * extend_end;
        }
        left[i].x = base.x + nx * half_w * scale;
        left[i].y = base.y + ny * half_w * scale;
        right[i].x = base.x - nx * half_w * scale;
        right[i].y = base.y - ny * half_w * scale;
    }

    int m = 0;
    for (int i = 0; i < n && m < max_out; ++i) poly_out[m++] = left[i];
    for (int i = n - 1; i >= 0 && m < max_out; --i) poly_out[m++] = right[i];
    return m;
}

/* Emit a smooth filled regular/NDS-shaped closed circle or disk (for roundabout island/ring). */
static void emit_disk(struct semantic_hud_minimap *out, uint8_t color,
                      struct fpt center, float radius, int segs) {
    if (segs > 20) segs = 20;
    struct fpt pts[20];
    for (int i = 0; i < segs; ++i) {
        float a = (float)(2.0 * M_PI * (double)i / (double)segs);
        pts[i].x = center.x + m_cosf(a) * radius;
        pts[i].y = center.y + m_sinf(a) * radius;
    }
    (void)emit_poly(out, color, pts, segs);
}

/* Emit a clean thick roundabout ring as two overlapping C-arc ribbons (avoids self-intersecting
 * donut polygons on flat 2D HUDTerminal polygon rasterizers, and keeps each polygon <= 12 verts). */
static void emit_ring_arcs(struct semantic_hud_minimap *out, uint8_t color,
                           const struct fpt *ring, int n, float half_w) {
    if (n < 4) return;
    /* Split closed ring into 2 halves meeting at ring[0] and ring[mid], with slight tangent
     * cap overlap (1.2 px) so seams fuse cleanly without needing an extra chord segment. */
    int mid = n / 2;
    struct fpt half1[10], half2[10];
    int n1 = 0, n2 = 0;
    for (int i = 0; i <= mid && n1 < 10; ++i)
        half1[n1++] = ring[i % n];
    for (int i = mid; i <= n && n2 < 10; ++i)
        half2[n2++] = ring[i % n];

    struct fpt poly[SEMANTIC_HUD_MAX_POLY_PTS];
    int m1 = stroke_ribbon(half1, n1, half_w, 1.2f, 1.2f, poly, (int)SEMANTIC_HUD_MAX_POLY_PTS);
    if (m1 >= 3) (void)emit_poly(out, color, poly, m1);
    int m2 = stroke_ribbon(half2, n2, half_w, 1.2f, 1.2f, poly, (int)SEMANTIC_HUD_MAX_POLY_PTS);
    if (m2 >= 3) (void)emit_poly(out, color, poly, m2);
}

/* Build a crisp OEM BMW triangular arrowhead at the tip of `line`. */
static void emit_arrowhead(struct semantic_hud_minimap *out, uint8_t color,
                           struct fpt tip, float tx, float ty,
                           float head_len, float head_half_w, float border_pad) {
    float nx = -ty, ny = tx;
    struct fpt tri[3];
    tri[0].x = tip.x + tx * (head_len * 0.55f + border_pad * 1.4f);
    tri[0].y = tip.y + ty * (head_len * 0.55f + border_pad * 1.4f);
    tri[1].x = tip.x - tx * (head_len * 0.45f + border_pad) + nx * (head_half_w + border_pad * 1.3f);
    tri[1].y = tip.y - ty * (head_len * 0.45f + border_pad) + ny * (head_half_w + border_pad * 1.3f);
    tri[2].x = tip.x - tx * (head_len * 0.45f + border_pad) - nx * (head_half_w + border_pad * 1.3f);
    tri[2].y = tip.y - ty * (head_len * 0.45f + border_pad) - ny * (head_half_w + border_pad * 1.3f);
    (void)emit_poly(out, color, tri, 3);
}

/* Trim the last `trim_px` from a screen polyline so the shaft ends cleanly inside the arrowhead. */
static int trim_end_px(struct fpt *pts, int n, float trim_px,
                       struct fpt *out_tip, float *out_tx, float *out_ty) {
    if (n < 2) return n;
    *out_tip = pts[n - 1];
    float dx = pts[n - 1].x - pts[n - 2].x;
    float dy = pts[n - 1].y - pts[n - 2].y;
    float dl = m_sqrtf(dx * dx + dy * dy);
    if (dl < 1e-4f) { *out_tx = 0.0f; *out_ty = -1.0f; }
    else { *out_tx = dx / dl; *out_ty = dy / dl; }

    float rem = trim_px;
    while (n >= 2 && rem > 0.0f) {
        float seg = fdist(pts[n - 2], pts[n - 1]);
        if (seg > rem + 1.0f) {
            float t = (seg - rem) / seg;
            pts[n - 1].x = pts[n - 2].x + (pts[n - 1].x - pts[n - 2].x) * t;
            pts[n - 1].y = pts[n - 2].y + (pts[n - 1].y - pts[n - 2].y) * t;
            rem = 0.0f;
        } else {
            rem -= seg;
            n--;
        }
    }
    return n;
}

/* Resample a closed ring polyline to `target_n` evenly spaced smooth points. */
static int resample_ring(const struct fpt *in, int n, struct fpt *out, int target_n) {
    if (n < 3 || target_n < 6 || target_n > 16) return 0;
    float total = 0.0f;
    for (int i = 0; i < n; ++i)
        total += fdist(in[i], in[(i + 1) % n]);
    if (total < 1.0f) return 0;
    float step = total / (float)target_n;
    for (int k = 0; k < target_n; ++k) {
        float target = (float)k * step;
        float acc = 0.0f;
        for (int i = 0; i < n; ++i) {
            struct fpt a = in[i], b = in[(i + 1) % n];
            float seg = fdist(a, b);
            if (acc + seg >= target || i == n - 1) {
                float t = seg > 1e-4f ? (target - acc) / seg : 0.0f;
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;
                out[k].x = a.x + (b.x - a.x) * t;
                out[k].y = a.y + (b.y - a.y) * t;
                break;
            }
            acc += seg;
        }
    }
    return target_n;
}

int semantic_render_hud_minimap(const struct snds_junction *junc,
                                uint32_t man_id,
                                uint8_t min_conf,
                                struct semantic_hud_minimap *out) {
    if (!junc || !out || !junc->valid || junc->confidence < min_conf)
        return 0;
    if (junc->guided.pt_count < 2u)
        return 0;

    memset(out, 0, sizeof(*out));
    out->man_id = man_id;
    out->is_roundabout = junc->is_roundabout;
    out->confidence = junc->confidence;

    const float h_rad = junc->approach_heading_deg * (float)(M_PI / 180.0);
    const float cos_h = m_cosf(h_rad);
    const float sin_h = m_sinf(h_rad);

    /* Choose zoomed-out world clipping radius showing surrounding intersection context:
     *   - Roundabout: ~2.7x radius (clamped 55..110 m) for clear center island + surrounding feeders
     *   - Intersection/turn: 105 m so dual carriageways, slip ramps, and adjacent streets are visible */
    float clip_r = 105.0f;
    if (junc->is_roundabout) {
        clip_r = junc->roundabout_radius_m * 2.7f;
        if (clip_r < 55.0f) clip_r = 55.0f;
        if (clip_r > 110.0f) clip_r = 110.0f;
    }

    /* Map world [-clip_r..+clip_r] into the exact 6WB/HUD 120x72 vector surface
     * (clip box x=[0..118], y=[0..71], non-faded zone x=[0..103]). */
    const float cx_px = junc->is_roundabout ? 62.0f : 64.0f;
    const float cy_px = 38.0f;
    const float px_radius = junc->is_roundabout ? 31.0f : 33.0f;
    const float scale = px_radius / clip_r;

    /* Helper coordinate transform buffers: local world (x_right, y_up) -> screen (sx, sy). */
    struct fpt tmp_w[SNDS_MAX_JUNCTION_PTS];
    struct fpt tmp_c[SNDS_MAX_JUNCTION_PTS];

    /* 1. Prepare all non-guided roads (directly connected arms + surrounding context roads).
     * Cap total background polylines so Layer 1 + Layer 2 leave >= 4 slots for Layer 3 + 4! */
    enum { MAX_BG_ROADS = 14 };
    struct fpt arms_px[MAX_BG_ROADS][8];
    int arm_lens[MAX_BG_ROADS];
    uint8_t arm_is_ctx[MAX_BG_ROADS];
    int valid_arms = 0;

    for (uint8_t a = 0u; a < junc->arm_count && valid_arms < MAX_BG_ROADS; ++a) {
        int n = (int)junc->arms[a].pt_count;
        if (n < 2) continue;
        for (int i = 0; i < n; ++i)
            tmp_w[i] = world_to_local(junc->arms[a].pts[i], junc->center, cos_h, sin_h);
        int nc = clip_polyline_radius(tmp_w, n, clip_r, tmp_c, (int)SNDS_MAX_JUNCTION_PTS);
        if (nc < 2) continue;
        for (int i = 0; i < nc; ++i) {
            tmp_c[i].x = cx_px + tmp_c[i].x * scale;
            tmp_c[i].y = cy_px - tmp_c[i].y * scale;
        }
        int ns = simplify_screen_poly(tmp_c, nc, 1.4f, arms_px[valid_arms], 6);
        if (ns >= 2) {
            arm_lens[valid_arms] = ns;
            arm_is_ctx[valid_arms] = 0u;
            valid_arms++;
        }
    }
    for (uint8_t c = 0u; c < junc->context_count && valid_arms < MAX_BG_ROADS; ++c) {
        int n = (int)junc->context[c].pt_count;
        if (n < 2) continue;
        for (int i = 0; i < n; ++i)
            tmp_w[i] = world_to_local(junc->context[c].pts[i], junc->center, cos_h, sin_h);
        int nc = clip_polyline_radius(tmp_w, n, clip_r, tmp_c, (int)SNDS_MAX_JUNCTION_PTS);
        if (nc < 2) continue;
        for (int i = 0; i < nc; ++i) {
            tmp_c[i].x = cx_px + tmp_c[i].x * scale;
            tmp_c[i].y = cy_px - tmp_c[i].y * scale;
        }
        int ns = simplify_screen_poly(tmp_c, nc, 1.3f, arms_px[valid_arms], 6);
        if (ns >= 2) {
            arm_lens[valid_arms] = ns;
            arm_is_ctx[valid_arms] = 1u;
            valid_arms++;
        }
    }

    /* 2. Prepare roundabout ring in screen space (if roundabout).
     * 10-sided smooth ring -> two 12-vertex half-rings (matches OEM's max n=12 polygons
     * and stays sub-pixel smooth at r ~ 12 px while keeping TGL DLBs lean). */
    struct fpt ring_px[16];
    int ring_n = 0;
    if (junc->is_roundabout && junc->ring.pt_count >= 4u) {
        int rn = (int)junc->ring.pt_count;
        for (int i = 0; i < rn; ++i) {
            struct fpt lp = world_to_local(junc->ring.pts[i], junc->center, cos_h, sin_h);
            tmp_c[i].x = cx_px + lp.x * scale;
            tmp_c[i].y = cy_px - lp.y * scale;
        }
        ring_n = resample_ring(tmp_c, rn, ring_px, 10);
    }

    /* 3. Prepare guided route polyline in screen space (clipped slightly tighter for arrowhead room). */
    int gn = (int)junc->guided.pt_count;
    for (int i = 0; i < gn; ++i)
        tmp_w[i] = world_to_local(junc->guided.pts[i], junc->center, cos_h, sin_h);
    int gc = clip_polyline_radius(tmp_w, gn, clip_r * 0.94f, tmp_c, (int)SNDS_MAX_JUNCTION_PTS);
    if (gc < 2) return 0;
    for (int i = 0; i < gc; ++i) {
        tmp_c[i].x = cx_px + tmp_c[i].x * scale;
        tmp_c[i].y = cy_px - tmp_c[i].y * scale;
    }
    struct fpt guided_px[14];
    int gs = simplify_screen_poly(tmp_c, gc, 1.5f, guided_px, 6);
    if (gs < 2) return 0;

    /* OEM BMW visual widths calibrated for the zoomed-out 120x72 6WB/HUD vector surface:
     *   - Roundabout: medium strokes (2.8+1.4 px) keep a bold pitch-black center island
     *   - Intersection: sleeker context/road strokes (1.8+1.0 px; secondary ctx 1.4+0.9 px)
     *     so parallel dual carriageways and slip ramps stay sharply separated, while the
     *     guided white route arrow (2.8+1.3 px) pops clearly on top! */
    const float road_fill_hw  = junc->is_roundabout ? 2.8f : 1.8f;
    const float border_w      = junc->is_roundabout ? 1.4f : 1.0f;
    const float road_bord_hw  = road_fill_hw + border_w;
    const float ctx_fill_hw   = junc->is_roundabout ? 2.0f : 1.4f;
    const float ctx_bord_hw   = ctx_fill_hw + 0.9f;
    const float arrow_fill_hw = junc->is_roundabout ? 2.8f : 2.6f;
    const float arrow_bord_hw = arrow_fill_hw + (junc->is_roundabout ? 1.4f : 1.3f);
    const float head_len      = junc->is_roundabout ? 11.5f : 11.0f;
    const float head_hw       = junc->is_roundabout ? 7.8f : 7.2f;

    /* Reserve 4 slots for Layer 3 + 4 (guided route border/fill + arrowhead border/fill).
     * If ring_n >= 6, reserve 4 more slots for ring borders/fills. */
    int max_bg_slots = (int)SEMANTIC_HUD_MAX_POLYS - 4 - (ring_n >= 6 ? 4 : 2);
    int emit_arms = valid_arms;
    if (emit_arms * 2 > max_bg_slots)
        emit_arms = max_bg_slots / 2;

    struct fpt poly[SEMANTIC_HUD_MAX_POLY_PTS];

    /* ================= LAYER 1: Non-guided road ORANGE borders ================= */
    for (int a = 0; a < emit_arms; ++a) {
        float hw = arm_is_ctx[a] ? ctx_bord_hw : road_bord_hw;
        int m = stroke_ribbon(arms_px[a], arm_lens[a], hw, 0.0f, 0.0f,
                              poly, (int)SEMANTIC_HUD_MAX_POLY_PTS);
        if (m >= 3) (void)emit_poly(out, SEMANTIC_HUD_COLOR_ORANGE, poly, m);
    }
    if (ring_n >= 6) {
        emit_ring_arcs(out, SEMANTIC_HUD_COLOR_ORANGE, ring_px, ring_n, road_bord_hw);
    } else if (!junc->is_roundabout) {
        /* Smooth junction hub disk so multi-arm borders blend cleanly without spikes */
        emit_disk(out, SEMANTIC_HUD_COLOR_ORANGE, (struct fpt){cx_px, cy_px}, road_bord_hw, 8);
    }

    /* ================= LAYER 2: Non-guided road DARK BROWN fills =================
     * Drawn ON TOP of ALL Layer-1 borders -> merges every arm & roundabout ring
     * seamlessly with ZERO messy internal orange seam lines! */
    for (int a = 0; a < emit_arms; ++a) {
        float hw = arm_is_ctx[a] ? ctx_fill_hw : road_fill_hw;
        int m = stroke_ribbon(arms_px[a], arm_lens[a], hw, 0.0f, 0.5f,
                              poly, (int)SEMANTIC_HUD_MAX_POLY_PTS);
        if (m >= 3) (void)emit_poly(out, SEMANTIC_HUD_COLOR_BROWN, poly, m);
    }
    if (ring_n >= 6) {
        emit_ring_arcs(out, SEMANTIC_HUD_COLOR_BROWN, ring_px, ring_n, road_fill_hw);
    } else if (!junc->is_roundabout) {
        emit_disk(out, SEMANTIC_HUD_COLOR_BROWN, (struct fpt){cx_px, cy_px}, road_fill_hw, 8);
    }

    /* ================= LAYER 3: Guided route + arrowhead ORANGE border ================= */
    struct fpt tip;
    float tx = 0.0f, ty = -1.0f;
    struct fpt shaft_px[14];
    memcpy(shaft_px, guided_px, sizeof(struct fpt) * (size_t)gs);
    int shaft_n = trim_end_px(shaft_px, gs, head_len * 0.40f, &tip, &tx, &ty);

    if (shaft_n >= 2) {
        int m = stroke_ribbon(shaft_px, shaft_n, arrow_bord_hw, 0.0f, 0.0f,
                              poly, (int)SEMANTIC_HUD_MAX_POLY_PTS);
        if (m >= 3) (void)emit_poly(out, SEMANTIC_HUD_COLOR_ORANGE, poly, m);
    }
    emit_arrowhead(out, SEMANTIC_HUD_COLOR_ORANGE, tip, tx, ty, head_len, head_hw, border_w);

    /* ================= LAYER 4: Guided route + arrowhead BRIGHT WHITE fill ================= */
    if (shaft_n >= 2) {
        int m = stroke_ribbon(shaft_px, shaft_n, arrow_fill_hw, 0.5f, 1.5f,
                              poly, (int)SEMANTIC_HUD_MAX_POLY_PTS);
        if (m >= 3) (void)emit_poly(out, SEMANTIC_HUD_COLOR_WHITE, poly, m);
    }
    emit_arrowhead(out, SEMANTIC_HUD_COLOR_WHITE, tip, tx, ty, head_len, head_hw, 0.0f);

    out->valid = (out->poly_count >= 2u) ? 1u : 0u;
    return out->valid ? 1 : 0;
}
