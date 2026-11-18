#ifndef NG_CAPABILITIES_H
#define NG_CAPABILITIES_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define NG_CAP_BYTES 16u
#define NG_CAP_TYPED_TURNS 1u
#define NG_CAP_TYPED_LANES 2u
#define NG_CAP_EXTENDED_FIELDS 4u
#define NG_CAP_OFF_RAMPS 8u
#define NG_CAP_TYPED (NG_CAP_TYPED_TURNS | NG_CAP_TYPED_LANES)
#define NG_CAP_SUPPORTED (NG_CAP_TYPED | NG_CAP_EXTENDED_FIELDS | NG_CAP_OFF_RAMPS)
#define NG_CAP_MODEL_ID 0x7f4e4701u

static inline uint32_t ng_cap_word(const unsigned char *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static inline void ng_cap_put(unsigned char *p, uint32_t value) {
    for (unsigned i=0; i<4; ++i) p[i]=(unsigned char)(value >> (8*i));
}
static inline void ng_cap_request(unsigned char *p, uint32_t sequence) {
    memcpy(p,"NGC1",4);
    ng_cap_put(p+4,sequence);
    ng_cap_put(p+8,0);
    ng_cap_put(p+12,NG_CAP_MODEL_ID);
}
static inline uint32_t ng_cap_query(const unsigned char *p, size_t size) {
    if (size!=NG_CAP_BYTES || memcmp(p,"NGC1",4) || ng_cap_word(p+8) || ng_cap_word(p+12)!=NG_CAP_MODEL_ID)
        return 0;
    return ng_cap_word(p+4);
}
static inline void ng_cap_response(unsigned char *p, uint32_t sequence, uint32_t capabilities) {
    memcpy(p,"NGR1",4);
    ng_cap_put(p+4,sequence);
    ng_cap_put(p+8,capabilities);
    ng_cap_put(p+12,NG_CAP_MODEL_ID);
}
static inline uint32_t ng_cap_read(const unsigned char *p, size_t size, uint32_t sequence) {
    if (size!=NG_CAP_BYTES || memcmp(p,"NGR1",4) || !sequence ||
        ng_cap_word(p+4)!=sequence || ng_cap_word(p+12)!=NG_CAP_MODEL_ID)
        return 0;
    return ng_cap_word(p+8)&NG_CAP_SUPPORTED;
}

#endif
