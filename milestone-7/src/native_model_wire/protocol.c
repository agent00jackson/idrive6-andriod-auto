#include "protocol.h"
#include <string.h>

static uint16_t rd16(const unsigned char *p) {
    return (uint16_t)((uint16_t)p[0] | (uint16_t)p[1] << 8);
}
static uint32_t rd32(const unsigned char *p) {
    return (uint32_t)rd16(p) | (uint32_t)rd16(p + 2) << 16;
}
static void wr16(unsigned char *p, uint16_t n) {
    p[0] = (unsigned char)n; p[1] = (unsigned char)(n >> 8);
}
static void wr32(unsigned char *p, uint32_t n) {
    wr16(p, (uint16_t)n); wr16(p + 2, (uint16_t)(n >> 16));
}
enum ng_result ng_decode(const unsigned char *p, size_t n, struct ng_message *m) {
    unsigned i, version;
    if (!p || !m || n != NG_FRAME_BYTES || rd16(p + 6) != NG_FRAME_BYTES) return NG_BAD_FRAME;
    if (!memcmp(p,"NGV1",4) && rd16(p+4)==1 && !rd32(p+72) && !rd32(p+76)) version=1;
    else if (!memcmp(p,"NGV2",4) && rd16(p+4)==2 && !rd32(p+76)) version=2;
    else if (!memcmp(p,"NGV3",4) && rd16(p+4)==3 && rd32(p+76)==1) version=3;
    else if (!memcmp(p,"NGV4",4) && rd16(p+4)==4 && rd32(p+76)==2) version=4;
    else return NG_BAD_FRAME;
    memset(m, 0, sizeof(*m));
    m->operation = rd32(p + 8); m->sequence = rd32(p + 12);
    m->session = (uint64_t)rd32(p + 16) | (uint64_t)rd32(p + 20) << 32;
    m->maneuver = rd32(p + 24); m->distance_mm = rd32(p + 28);
    m->turn = rd32(p + 32); m->direction_centunits = rd16(p + 36);
    m->lane_count = rd16(p + 38);
    m->phase = version>=2 ? rd32(p+72) : 0;
    m->lane_mode = version>=3 ? version-2 : 0u;
    if ((!m->session && m->operation != NG_PROBE) || !m->sequence || m->lane_count > NG_MAX_LANES ||
        m->operation < NG_BEGIN || m->operation > NG_PROBE) return NG_BAD_VALUE;
    for (i = 0; i < NG_MAX_LANES; ++i) {
        const unsigned char *q = p + 40 + 4 * i;
        const uint16_t arrows = rd16(q), recommended = rd16(q+2);
        m->lanes[i].arrows = arrows;
        m->lanes[i].recommended = recommended;
        if ((i < m->lane_count) != (arrows != 0) || (!arrows && recommended)) return NG_BAD_VALUE;
        if (version == 4) {
            if ((arrows & ~NG_LANE_ARROW_MASK) || (recommended & ~arrows)) return NG_BAD_VALUE;
        } else {
            if (recommended > 1 || (arrows & ~0x7fu)) return NG_BAD_VALUE;
            if (version == 3 && arrows && arrows != NG_LANE_STRAIGHT &&
                arrows != NG_LANE_LEFT && arrows != NG_LANE_RIGHT) return NG_BAD_VALUE;
        }
    }
    if (m->operation == NG_STOP || m->operation == NG_PROBE) {
        if (version!=1 || m->phase) return NG_BAD_VALUE;
        if ((m->operation == NG_PROBE && m->session) || m->maneuver || m->distance_mm || m->turn ||
            m->direction_centunits || m->lane_count) return NG_BAD_VALUE;
    } else if (!m->maneuver || m->distance_mm > NG_MAX_DISTANCE_MM ||
               m->turn < NG_STRAIGHT || m->turn > NG_UTURN ||
               m->direction_centunits > 25599 ||
               (version>=2 && (m->phase<1 || m->phase>4)) ||
               (version>=3 && !m->lane_count)) {
        return NG_BAD_VALUE;
    }
    return NG_OK;
}
void ng_encode(unsigned char p[NG_FRAME_BYTES], const struct ng_message *m) {
    unsigned i;
    memset(p, 0, NG_FRAME_BYTES); memcpy(p, m->lane_mode==2 ? "NGV4" : m->lane_mode ? "NGV3" : m->phase ? "NGV2" : "NGV1", 4);
    wr16(p + 4, m->lane_mode==2 ? 4 : m->lane_mode ? 3 : m->phase ? 2 : 1); wr16(p + 6, NG_FRAME_BYTES);
    wr32(p + 8, m->operation); wr32(p + 12, m->sequence);
    wr32(p + 16, (uint32_t)m->session); wr32(p + 20, (uint32_t)(m->session >> 32));
    wr32(p + 24, m->maneuver); wr32(p + 28, m->distance_mm);
    wr32(p + 32, m->turn); wr16(p + 36, (uint16_t)m->direction_centunits);
    wr16(p + 38, m->lane_count);
    for (i = 0; i < NG_MAX_LANES; ++i) {
        wr16(p + 40 + 4 * i, m->lanes[i].arrows);
        wr16(p + 42 + 4 * i, m->lanes[i].recommended);
    }
    wr32(p+72,m->phase);
    wr32(p+76,m->lane_mode);
}
enum ng_result ng_apply(struct ng_state *s, const struct ng_message *m,
                        const struct ng_backend *b) {
    struct ng_message checked;
    unsigned char wire[NG_FRAME_BYTES];
    enum ng_result valid;
    if (!s || !m || !b || !b->update || !b->clear) return NG_BACKEND_REJECTED;
    if (m->lane_count > NG_MAX_LANES) return NG_BAD_VALUE;
    ng_encode(wire, m);
    valid = ng_decode(wire, sizeof(wire), &checked);
    if (valid != NG_OK) return valid;
    if (!m->session || !m->sequence) return NG_BAD_VALUE;
    if (m->operation == NG_BEGIN) {
        if (s->active || s->session == m->session) return NG_BAD_SESSION;
    } else {
        if (!s->active || s->session != m->session) return NG_BAD_SESSION;
        if (m->sequence <= s->sequence) return NG_STALE;
    }
    if (m->operation == NG_STOP) {
        if (!b->clear(b->context, m->session)) return NG_BACKEND_REJECTED;
        s->active = 0; s->sequence = m->sequence; s->maneuver = 0;
    } else if (m->operation == NG_BEGIN || m->operation == NG_UPDATE) {
        if (!b->update(b->context, m)) return NG_BACKEND_REJECTED;
        s->active = 1; s->session = m->session;
        s->sequence = m->sequence; s->maneuver = m->maneuver;
    } else return NG_BAD_VALUE;
    return NG_OK;
}
