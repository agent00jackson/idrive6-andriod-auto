// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
/* Live OEM NDS map accessor bridge for NavigationBasic (exact X20511I ARM QNX). */
#include "semantic_nds_probe.h"

#include "semantic_math.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#if defined(__QNXNTO__) && defined(__arm__)

#define FN(address, result, ...) ((result (*)(__VA_ARGS__))(uintptr_t)(address))

enum {
    /* Exact X20511I NavigationBasic pins */
    TCARPOS_EXTRACT_LONLAT = 0x002ddb98u, /* void(int32_t out[2], const void *pos_plus_16) */
    PSI_ADAS_PRIMARY_VPTR  = 0x0105d0c8u, /* CPsiAdasAccessor primary vtable (+0) */
    PSI_ADAS_IFACE_VPTR    = 0x0105d120u, /* IAdasAccessor secondary vtable (+216) */
    PSI_ADAS_IFACE_OFFSET  = 216u,
    PSI_ADAS_COORDSYS_OFF  = 220u,        /* uint32_t mCoordSystem (0 == raw NDS 32-bit) */
    PSI_ADAS_INIT_FLAG_OFF = 280u,        /* uint8_t mInitialized on CPsiAdasAccessor */
    ROAD_TREE_PROV_VPTR    = 0x00f283e8u, /* CRoadTreeProvider primary vtable */
    ROAD_TREE_ADAS_OFF     = 684u,        /* IAdasAccessor* mAdasAccessor */
    ADAS_LINK_INFO_CTOR    = 0x0065d068u, /* CAdasLinkInfo::CAdasLinkInfo() */
    ADAS_LINK_INFO_DTOR    = 0x0065d028u, /* CAdasLinkInfo::~CAdasLinkInfo() */
    ADAS_ATTR_STRIDE       = 48u,         /* sizeof(CAdasAttribute) */

    /* Verified CPsiAdasAccessor::getLinkInfo attribute IDs (0x0093e4cc jump table) */
    ADAS_ATTR_TRAVEL_DIR   = 0u,  /* 0=both, 1=fwd one-way, 2=bwd one-way */
    ADAS_ATTR_FRC          = 9u,  /* FunctionalRoadClass 0..7 */
    ADAS_ATTR_ROUNDABOUT   = 37u, /* NDS FormOfWay == 4 (Roundabout) */

    NDS_HEAP_SCAN_BEGIN    = 0x01000000u,
    NDS_HEAP_SCAN_END      = 0x28000000u,
};

static const double NDS_UNITS_PER_DEG = 4294967296.0 / 360.0;
static const double DEG_PER_NDS_UNIT  = 360.0 / 4294967296.0;

static uint32_t g_cached_adas_iface = 0u;
static uint32_t g_cached_rtp = 0u;

static int safe_read_mem(int fd, uint32_t addr, void *out, size_t len) {
    if (fd < 0 || addr < 0x10000u || addr >= 0x80000000u || len > 4096u)
        return 0;
    if (lseek(fd, (off_t)addr, SEEK_SET) != (off_t)addr)
        return 0;
    return read(fd, out, len) == (ssize_t)len;
}

static int safe_read_word(int fd, uint32_t addr, uint32_t *out) {
    return safe_read_mem(fd, addr, out, sizeof(*out));
}

static int validate_adas_iface(int fd, uint32_t iface_addr) {
    uint32_t iface_vptr = 0u, prim_vptr = 0u, coord_sys = 99u;
    uint8_t init_flag = 0u;
    if (iface_addr < 0x10000u + PSI_ADAS_IFACE_OFFSET || iface_addr >= 0x80000000u)
        return 0;
    uint32_t prim_addr = iface_addr - PSI_ADAS_IFACE_OFFSET;
    if (!safe_read_word(fd, iface_addr, &iface_vptr) || iface_vptr != PSI_ADAS_IFACE_VPTR)
        return 0;
    if (!safe_read_word(fd, prim_addr, &prim_vptr) || prim_vptr != PSI_ADAS_PRIMARY_VPTR)
        return 0;
    if (!safe_read_word(fd, prim_addr + PSI_ADAS_COORDSYS_OFF, &coord_sys) || coord_sys != 0u)
        return 0;
    if (!safe_read_mem(fd, prim_addr + PSI_ADAS_INIT_FLAG_OFF, &init_flag, 1u) || !init_flag)
        return 0;
    return 1;
}

static uint32_t find_live_adas_iface(int fd) {
    if (g_cached_adas_iface && validate_adas_iface(fd, g_cached_adas_iface))
        return g_cached_adas_iface;
    g_cached_adas_iface = 0u;

    /* Fast path: check cached CRoadTreeProvider->mAdasAccessor first. */
    if (g_cached_rtp) {
        uint32_t rtp_vptr = 0u, adas_ptr = 0u;
        if (safe_read_word(fd, g_cached_rtp, &rtp_vptr) && rtp_vptr == ROAD_TREE_PROV_VPTR &&
            safe_read_word(fd, g_cached_rtp + ROAD_TREE_ADAS_OFF, &adas_ptr) &&
            validate_adas_iface(fd, adas_ptr)) {
            g_cached_adas_iface = adas_ptr;
            return adas_ptr;
        }
    }

    uint32_t page[1024];
    uint32_t addr = NDS_HEAP_SCAN_BEGIN;
    while (addr < NDS_HEAP_SCAN_END) {
        if (lseek(fd, (off_t)addr, SEEK_SET) != (off_t)addr ||
            read(fd, page, sizeof(page)) != (ssize_t)sizeof(page)) {
            addr = (addr + 65536u) & ~65535u;
            continue;
        }
        for (unsigned i = 0u; i < 1024u; ++i) {
            uint32_t w = page[i];
            if (w == PSI_ADAS_IFACE_VPTR) {
                uint32_t cand = addr + i * 4u;
                if (validate_adas_iface(fd, cand)) {
                    g_cached_adas_iface = cand;
                    return cand;
                }
            } else if (w == ROAD_TREE_PROV_VPTR) {
                uint32_t rtp = addr + i * 4u;
                uint32_t adas_ptr = 0u;
                if (safe_read_word(fd, rtp + ROAD_TREE_ADAS_OFF, &adas_ptr) &&
                    validate_adas_iface(fd, adas_ptr)) {
                    g_cached_rtp = rtp;
                    g_cached_adas_iface = adas_ptr;
                    return adas_ptr;
                }
            }
        }
        addr += 4096u;
    }
    return 0u;
}

void semantic_nds_capture_car_pos(const void *tcarpos_ptr,
                                  uint64_t now_ms,
                                  struct semantic_car_gps *out) {
    if (!tcarpos_ptr || !out)
        return;
    int32_t nds_lonlat[2] = {0, 0};
    FN(TCARPOS_EXTRACT_LONLAT, void, int32_t *, const void *)(
        nds_lonlat, (const uint8_t *)tcarpos_ptr + 16u);
    if (nds_lonlat[0] == 0 && nds_lonlat[1] == 0)
        return;

    double lon = (double)nds_lonlat[0] * DEG_PER_NDS_UNIT;
    double lat = (double)nds_lonlat[1] * DEG_PER_NDS_UNIT;
    if (lon < -180.0 || lon > 180.0 || lat < -85.0 || lat > 85.0)
        return;

    int32_t raw_heading = *(const int32_t *)tcarpos_ptr;
    float hdg = (float)raw_heading;
    while (hdg < 0.0f) hdg += 360.0f;
    while (hdg >= 360.0f) hdg -= 360.0f;

    out->lon_deg = lon;
    out->lat_deg = lat;
    out->heading_deg = hdg;
    out->timestamp_ms = now_ms;
    out->valid = 1;
}

static int32_t clamp_deg_to_nds(double deg) {
    if (deg > 179.9999) deg = 179.9999;
    if (deg < -179.9999) deg = -179.9999;
    return (int32_t)(deg * NDS_UNITS_PER_DEG);
}

static uint64_t node_id_from_nds(int32_t lon_nds, int32_t lat_nds) {
    /* Quantize to ~0.35 m grid (>> 5 of 2^32 units = ~0.3m at equator) so
     * co-located NDS link endpoints share an identical topological node ID. */
    uint32_t qx = (uint32_t)((lon_nds + 16) >> 5);
    uint32_t qy = (uint32_t)((lat_nds + 16) >> 5);
    return ((uint64_t)qx << 32) | (uint64_t)qy;
}

int semantic_nds_query_local_graph(int mem_fd,
                                   const struct semantic_car_gps *gps,
                                   float radius_m,
                                   struct snds_graph *out_graph) {
    if (!gps || !gps->valid || !out_graph)
        return 0;
    memset(out_graph, 0, sizeof(*out_graph));

    uint32_t adas_addr = find_live_adas_iface(mem_fd);
    if (!adas_addr)
        return 0;

    void *adas = (void *)(uintptr_t)adas_addr;
    void **vtbl = *(void ***)adas;
    if (!vtbl || (uint32_t)(uintptr_t)vtbl != PSI_ADAS_IFACE_VPTR)
        return 0;

    if (radius_m < 140.0f) radius_m = 140.0f;
    if (radius_m > 520.0f) radius_m = 520.0f;

    /* Tight corridor bounding box covering both the car position and the predicted
     * maneuver point (~dist_m ahead along heading_deg) with +-110m margin, avoiding
     * dense urban link-cap truncation from unrelated roads behind/beside the car. */
    float dist_ahead = radius_m - 120.0f;
    if (dist_ahead < 40.0f) dist_ahead = 40.0f;
    float h_rad = gps->heading_deg * (float)(3.14159265358979323846 / 180.0);
    double cos_lat = (double)m_cosf((float)(gps->lat_deg * (3.14159265358979323846 / 180.0)));
    if (cos_lat < 0.1) cos_lat = 0.1;

    double ahead_dlat = ((double)dist_ahead * (double)m_cosf(h_rad)) / 111320.0;
    double ahead_dlon = ((double)dist_ahead * (double)m_sinf(h_rad)) / (111320.0 * cos_lat);
    double pad_lat = 110.0 / 111320.0;
    double pad_lon = 110.0 / (111320.0 * cos_lat);

    double lat_min = gps->lat_deg < (gps->lat_deg + ahead_dlat) ? gps->lat_deg : (gps->lat_deg + ahead_dlat);
    double lat_max = gps->lat_deg > (gps->lat_deg + ahead_dlat) ? gps->lat_deg : (gps->lat_deg + ahead_dlat);
    double lon_min = gps->lon_deg < (gps->lon_deg + ahead_dlon) ? gps->lon_deg : (gps->lon_deg + ahead_dlon);
    double lon_max = gps->lon_deg > (gps->lon_deg + ahead_dlon) ? gps->lon_deg : (gps->lon_deg + ahead_dlon);

    int32_t bbox[4];
    bbox[0] = clamp_deg_to_nds(lon_min - pad_lon);
    bbox[1] = clamp_deg_to_nds(lat_min - pad_lat);
    bbox[2] = clamp_deg_to_nds(lon_max + pad_lon);
    bbox[3] = clamp_deg_to_nds(lat_max + pad_lat);

    typedef void *(*create_scan_fn)(void *, const int32_t *);
    typedef uint64_t (*scan_id_fn)(void *);
    typedef void (*scan_dtor_fn)(void *);
    typedef int (*get_link_info_fn)(void *, uint64_t, uint32_t, const void *, uint32_t, void *);

    void *scan = ((create_scan_fn)vtbl[19])(adas, bbox);
    if (!scan)
        return 0;

    uint64_t ids[SNDS_MAX_GRAPH_LINKS];
    uint16_t id_count = 0u;
    void **svtbl = *(void ***)scan;
    if (svtbl) {
        uint64_t lid = ((scan_id_fn)svtbl[2])(scan);
        while (lid != 0u && id_count < SNDS_MAX_GRAPH_LINKS) {
            int dup = 0;
            for (uint16_t k = 0u; k < id_count; ++k) {
                if (ids[k] == lid) { dup = 1; break; }
            }
            if (!dup)
                ids[id_count++] = lid;
            lid = ((scan_id_fn)svtbl[3])(scan);
        }
        ((scan_dtor_fn)svtbl[1])(scan);
    }

    uint32_t req_attrs[3] = {
        ADAS_ATTR_TRAVEL_DIR,
        ADAS_ATTR_FRC,
        ADAS_ATTR_ROUNDABOUT,
    };
    /* Stack-backed read-only Dinkumware std::vector<uint32_t> layout: {al, first, last, end} */
    uint32_t attr_vec[4] = {
        0u,
        (uint32_t)(uintptr_t)&req_attrs[0],
        (uint32_t)(uintptr_t)&req_attrs[3],
        (uint32_t)(uintptr_t)&req_attrs[3],
    };

    get_link_info_fn get_link_info = (get_link_info_fn)vtbl[14];
    for (uint16_t i = 0u; i < id_count && out_graph->link_count < SNDS_MAX_GRAPH_LINKS; ++i) {
        _Alignas(8) uint8_t info[64];
        memset(info, 0, sizeof(info));
        FN(ADAS_LINK_INFO_CTOR, void, void *)(info);

        int ok = get_link_info(adas, ids[i], 0u, attr_vec, 0u, info);
        if (ok) {
            uint32_t pts_beg = *(const uint32_t *)(info + 4u);
            uint32_t pts_end = *(const uint32_t *)(info + 8u);
            if (pts_beg >= 0x10000u && pts_end > pts_beg && ((pts_end - pts_beg) % 8u) == 0u) {
                uint32_t raw_n = (pts_end - pts_beg) / 8u;
                if (raw_n >= 2u && raw_n <= 4096u) {
                    const int32_t *raw_pts = (const int32_t *)(uintptr_t)pts_beg;
                    struct snds_link *lnk = &out_graph->links[out_graph->link_count];
                    memset(lnk, 0, sizeof(*lnk));
                    lnk->link_id = ids[i];
                    lnk->frc = 4u; /* default urban collector if attribute missing */
                    lnk->start_node = node_id_from_nds(raw_pts[0], raw_pts[1]);
                    lnk->end_node = node_id_from_nds(raw_pts[(raw_n - 1u) * 2u + 0u],
                                                     raw_pts[(raw_n - 1u) * 2u + 1u]);

                    /* Copy (or stride-downsample) shape points into local metres. */
                    if (raw_n <= SNDS_MAX_LINK_PTS) {
                        for (uint32_t k = 0u; k < raw_n; ++k) {
                            double plon = (double)raw_pts[k * 2u + 0u] * DEG_PER_NDS_UNIT;
                            double plat = (double)raw_pts[k * 2u + 1u] * DEG_PER_NDS_UNIT;
                            lnk->pts[k] = snds_wgs84_to_local_m(
                                plat, plon, gps->lat_deg, gps->lon_deg);
                        }
                        lnk->pt_count = (uint8_t)raw_n;
                    } else {
                        for (uint32_t k = 0u; k < SNDS_MAX_LINK_PTS; ++k) {
                            uint32_t idx = (k == SNDS_MAX_LINK_PTS - 1u)
                                ? (raw_n - 1u)
                                : (k * (raw_n - 1u)) / (SNDS_MAX_LINK_PTS - 1u);
                            double plon = (double)raw_pts[idx * 2u + 0u] * DEG_PER_NDS_UNIT;
                            double plat = (double)raw_pts[idx * 2u + 1u] * DEG_PER_NDS_UNIT;
                            lnk->pts[k] = snds_wgs84_to_local_m(
                                plat, plon, gps->lat_deg, gps->lon_deg);
                        }
                        lnk->pt_count = (uint8_t)SNDS_MAX_LINK_PTS;
                    }

                    /* Compute polyline length in metres. */
                    float len_m = 0.0f;
                    for (uint8_t k = 1u; k < lnk->pt_count; ++k) {
                        float dx = lnk->pts[k].x - lnk->pts[k - 1u].x;
                        float dy = lnk->pts[k].y - lnk->pts[k - 1u].y;
                        len_m += m_sqrtf(dx * dx + dy * dy);
                    }
                    lnk->length_m = len_m;

                    /* Parse CAdasAttribute vector (48-byte stride) at info+20..info+24. */
                    uint32_t abeg = *(const uint32_t *)(info + 20u);
                    uint32_t aend = *(const uint32_t *)(info + 24u);
                    if (abeg >= 0x10000u && aend > abeg &&
                        ((aend - abeg) % ADAS_ATTR_STRIDE) == 0u) {
                        uint32_t an = (aend - abeg) / ADAS_ATTR_STRIDE;
                        if (an > 32u) an = 32u;
                        for (uint32_t a = 0u; a < an; ++a) {
                            const uint8_t *at = (const uint8_t *)(uintptr_t)(abeg + a * ADAS_ATTR_STRIDE);
                            uint16_t aid = *(const uint16_t *)(at + 0u);
                            if (aid == ADAS_ATTR_TRAVEL_DIR) {
                                uint32_t td = *(const uint32_t *)(at + 16u);
                                lnk->one_way = (td == 1u) ? 1u : (td == 2u ? 2u : 0u);
                            } else if (aid == ADAS_ATTR_FRC) {
                                uint32_t frc = *(const uint32_t *)(at + 16u);
                                if (frc <= 8u) lnk->frc = (uint8_t)frc;
                            } else if (aid == ADAS_ATTR_ROUNDABOUT) {
                                if (at[16u] != 0u) lnk->is_roundabout = 1u;
                            }
                        }
                    }

                    if (lnk->frc < 7u || lnk->is_roundabout)
                        out_graph->link_count++;
                }
            }
        }
        FN(ADAS_LINK_INFO_DTOR, void, void *)(info);
    }

    return (int)out_graph->link_count;
}

#else

void semantic_nds_capture_car_pos(const void *tcarpos_ptr,
                                  uint64_t now_ms,
                                  struct semantic_car_gps *out) {
    (void)tcarpos_ptr;
    (void)now_ms;
    if (out) memset(out, 0, sizeof(*out));
}

int semantic_nds_query_local_graph(int mem_fd,
                                   const struct semantic_car_gps *gps,
                                   float radius_m,
                                   struct snds_graph *out_graph) {
    (void)mem_fd;
    (void)gps;
    (void)radius_m;
    if (out_graph) memset(out_graph, 0, sizeof(*out_graph));
    return 0;
}

#endif
