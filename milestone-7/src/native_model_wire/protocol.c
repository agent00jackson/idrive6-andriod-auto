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
static int utf8_valid(const unsigned char *p, size_t n) {
    size_t i=0;
    while(i<n) {
        unsigned char a=p[i++];
        if(a<0x80) { if(!a)return 0; continue; }
        if(a>=0xc2&&a<=0xdf) {
            if(i>=n||(p[i]&0xc0)!=0x80)return 0;
            ++i; continue;
        }
        if(a>=0xe0&&a<=0xef) {
            if(i+1>=n||(p[i]&0xc0)!=0x80||(p[i+1]&0xc0)!=0x80)return 0;
            if((a==0xe0&&p[i]<0xa0)||(a==0xed&&p[i]>=0xa0))return 0;
            i+=2; continue;
        }
        if(a>=0xf0&&a<=0xf4) {
            if(i+2>=n||(p[i]&0xc0)!=0x80||(p[i+1]&0xc0)!=0x80||(p[i+2]&0xc0)!=0x80)return 0;
            if((a==0xf0&&p[i]<0x90)||(a==0xf4&&p[i]>=0x90))return 0;
            i+=3; continue;
        }
        return 0;
    }
    return 1;
}
static int zero_bytes(const unsigned char *p,size_t n) {
    for(size_t i=0;i<n;++i)if(p[i])return 0;
    return 1;
}
enum ng_result ng_decode(const unsigned char *p, size_t n, struct ng_message *m) {
    unsigned i, version;
    if (!p || !m || (n != NG_FRAME_BYTES && n != NG_EXT_FRAME_BYTES) || rd16(p + 6) != n) return NG_BAD_FRAME;
    if(n==NG_EXT_FRAME_BYTES && !memcmp(p,"NGV5",4) && rd16(p+4)==5) version=5;
    else if(n==NG_FRAME_BYTES && !memcmp(p,"NGV1",4) && rd16(p+4)==1 && !rd32(p+72) && !rd32(p+76)) version=1;
    else if(n==NG_FRAME_BYTES && !memcmp(p,"NGV2",4) && rd16(p+4)==2 && !rd32(p+76)) version=2;
    else if(n==NG_FRAME_BYTES && !memcmp(p,"NGV3",4) && rd16(p+4)==3 && rd32(p+76)==1) version=3;
    else if(n==NG_FRAME_BYTES && !memcmp(p,"NGV4",4) && rd16(p+4)==4 && rd32(p+76)==2) version=4;
    else return NG_BAD_FRAME;
    memset(m, 0, sizeof(*m));
    m->operation = rd32(p + 8); m->sequence = rd32(p + 12);
    m->session = (uint64_t)rd32(p + 16) | (uint64_t)rd32(p + 20) << 32;
    m->maneuver = rd32(p + 24); m->distance_mm = rd32(p + 28);
    m->turn = rd32(p + 32); m->direction_centunits = rd16(p + 36);
    m->lane_count = rd16(p + 38);
    m->phase = version>=2 ? rd32(p+72) : 0;
    m->lane_mode = version==5 ? rd32(p+76) : version>=3 ? version-2 : 0u;
    if(m->lane_mode>2) return NG_BAD_VALUE;
    if ((!m->session && m->operation != NG_PROBE) || !m->sequence || m->lane_count > NG_MAX_LANES ||
        m->operation < NG_BEGIN || m->operation > NG_PROBE) return NG_BAD_VALUE;
    for (i = 0; i < NG_MAX_LANES; ++i) {
        const unsigned char *q = p + 40 + 4 * i;
        const uint16_t arrows = rd16(q), recommended = rd16(q+2);
        m->lanes[i].arrows = arrows;
        m->lanes[i].recommended = recommended;
        if ((i < m->lane_count) != (arrows != 0) || (!arrows && recommended)) return NG_BAD_VALUE;
        if (version == 4 || (version==5 && m->lane_mode==2)) {
            if ((arrows & ~NG_LANE_ARROW_MASK) || (recommended & ~arrows)) return NG_BAD_VALUE;
        } else {
            if (recommended > 1 || (arrows & ~0x7fu)) return NG_BAD_VALUE;
            if ((version == 3 || (version==5 && m->lane_mode==1)) && arrows && arrows != NG_LANE_STRAIGHT &&
                arrows != NG_LANE_LEFT && arrows != NG_LANE_RIGHT) return NG_BAD_VALUE;
        }
    }
    if(version==5) {
        const uint16_t flags=rd16(p+80), road_length=rd16(p+84);
        const unsigned char *road=p+96;
        m->metadata_flags=flags; m->aa_type=rd16(p+82); m->road_length=road_length;
        m->exit_number=(int32_t)rd32(p+88); m->exit_angle=(int32_t)rd32(p+92);
        if(!flags || (flags&~NG_META_MASK) || !(flags&NG_META_AA_TYPE) || rd16(p+86) ||
           m->aa_type>50 || (m->aa_type>=30&&m->aa_type<=31)) return NG_BAD_VALUE;
        if(flags&NG_META_ROAD) {
            if(!road_length || road_length>NG_MAX_ROAD_BYTES || road[road_length] ||
               !zero_bytes(road+road_length+1,NG_MAX_ROAD_BYTES-road_length) ||
               !utf8_valid(road,road_length)) return NG_BAD_VALUE;
            memcpy(m->road,road,road_length);
        } else if(road_length || !zero_bytes(road,NG_MAX_ROAD_BYTES+1u)) return NG_BAD_VALUE;
        if((flags&NG_META_EXIT_NUMBER) ?
              (m->exit_number<1 || m->exit_number>255) : m->exit_number!=0) return NG_BAD_VALUE;
        if((flags&NG_META_EXIT_ANGLE) ?
              (m->exit_angle<0 || m->exit_angle>360) : m->exit_angle!=0) return NG_BAD_VALUE;
        if((flags&(NG_META_EXIT_NUMBER|NG_META_EXIT_ANGLE)) &&
           !((m->aa_type>=32&&m->aa_type<=35)||(m->aa_type>=43&&m->aa_type<=46))) return NG_BAD_VALUE;
    }
    if (m->operation == NG_STOP || m->operation == NG_PROBE) {
        if (version!=1 || m->phase) return NG_BAD_VALUE;
        if ((m->operation == NG_PROBE && m->session) || m->maneuver || m->distance_mm || m->turn ||
            m->direction_centunits || m->lane_count) return NG_BAD_VALUE;
    } else if (!m->maneuver || m->distance_mm > NG_MAX_DISTANCE_MM ||
               m->turn < NG_STRAIGHT || m->turn > NG_UTURN ||
               m->direction_centunits > 25599 ||
               (version>=2 && version<5 && (m->phase<1 || m->phase>4)) ||
               (version==5 && (m->phase>4 || (m->lane_mode && !m->phase))) ||
               (version>=3 && version<5 && !m->lane_count) ||
               (version==5 && m->lane_mode && !m->lane_count)) {
        return NG_BAD_VALUE;
    }
    return NG_OK;
}
size_t ng_frame_size(const struct ng_message *m) {
    return m && m->metadata_flags ? NG_EXT_FRAME_BYTES : NG_FRAME_BYTES;
}
void ng_encode(unsigned char p[NG_MAX_FRAME_BYTES], const struct ng_message *m) {
    unsigned i;
    const int extended=m->metadata_flags!=0;
    const size_t frame_bytes=extended ? NG_EXT_FRAME_BYTES : NG_FRAME_BYTES;
    memset(p, 0, frame_bytes);
    memcpy(p, extended ? "NGV5" : m->lane_mode==2 ? "NGV4" : m->lane_mode ? "NGV3" : m->phase ? "NGV2" : "NGV1", 4);
    wr16(p + 4, extended ? 5 : m->lane_mode==2 ? 4 : m->lane_mode ? 3 : m->phase ? 2 : 1);
    wr16(p + 6, (uint16_t)frame_bytes);
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
    if(extended) {
        wr16(p+80,m->metadata_flags); wr16(p+82,m->aa_type); wr16(p+84,m->road_length);
        wr32(p+88,(uint32_t)m->exit_number); wr32(p+92,(uint32_t)m->exit_angle);
        if(m->road_length<=NG_MAX_ROAD_BYTES)memcpy(p+96,m->road,m->road_length);
    }
}
enum ng_result ng_apply(struct ng_state *s, const struct ng_message *m,
                        const struct ng_backend *b) {
    struct ng_message checked;
    unsigned char wire[NG_MAX_FRAME_BYTES];
    size_t wire_bytes;
    enum ng_result valid;
    if (!s || !m || !b || !b->update || !b->clear) return NG_BACKEND_REJECTED;
    if (m->lane_count > NG_MAX_LANES) return NG_BAD_VALUE;
    wire_bytes=ng_frame_size(m);
    ng_encode(wire, m);
    valid = ng_decode(wire, wire_bytes, &checked);
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
