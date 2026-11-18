// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#ifndef NBT_NATIVE_MODEL_PROTOCOL_HPP
#define NBT_NATIVE_MODEL_PROTOCOL_HPP
#include "cluster/nav_msg.hpp"
#include <stdint.h>
#include <string.h>
extern "C" {
#include "native_model_wire/protocol.h"
}

namespace nbt {
namespace native_model {
// NGV1 is a local adapter protocol, not an OEM network/service interface.
const size_t kFrameBytes = 80, kReplyBytes = 16, kMaxLanes = 8;
const uint32_t kModelId = 0x7f4e4701, kMaxDistanceMm = 2000000000;
enum Operation { Begin = 1, Update = 2, Stop = 3, Probe = 4 };
struct Lane {
    uint16_t arrows;
    uint8_t recommended;
    Lane() : arrows(0), recommended(0) {}
};
struct Message {
    uint32_t operation, sequence;
    uint64_t session;
    uint32_t maneuver, distanceMm, turn;
    uint16_t direction, laneCount;
    Lane lanes[kMaxLanes];
    Message()
        : operation(0), sequence(0), session(0), maneuver(0), distanceMm(0), turn(0), direction(0), laneCount(0) {}
};
inline uint16_t rd16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
inline uint32_t rd32(const uint8_t *p) { return uint32_t(rd16(p)) | uint32_t(rd16(p + 2)) << 16; }
inline void wr16(uint8_t *p, uint16_t v) {
    p[0] = uint8_t(v);
    p[1] = uint8_t(v >> 8);
}
inline void wr32(uint8_t *p, uint32_t v) {
    wr16(p, uint16_t(v));
    wr16(p + 2, uint16_t(v >> 16));
}
inline ng_message nativeMessage(const Message &m) {
    ng_message n;
    memset(&n, 0, sizeof(n));
    n.operation = m.operation;
    n.sequence = m.sequence;
    n.session = m.session;
    n.maneuver = m.maneuver;
    n.distance_mm = m.distanceMm;
    n.turn = m.turn;
    n.direction_centunits = m.direction;
    n.lane_count = m.laneCount;
    for (size_t i = 0; i < kMaxLanes; ++i) {
        n.lanes[i].arrows = m.lanes[i].arrows;
        n.lanes[i].recommended = m.lanes[i].recommended;
    }
    return n;
}
inline bool encode(const Message &m, uint8_t *p) {
    if (!p || m.laneCount > kMaxLanes)
        return false;
    ng_message n = nativeMessage(m), checked;
    ng_encode(p, &n);
    return ng_decode(p, kFrameBytes, &checked) == NG_OK;
}
inline bool decode(const uint8_t *p, size_t n, Message &out) {
    ng_message m;
    if (ng_decode(p, n, &m) != NG_OK)
        return false;
    Message v;
    v.operation = m.operation;
    v.sequence = m.sequence;
    v.session = m.session;
    v.maneuver = m.maneuver;
    v.distanceMm = m.distance_mm;
    v.turn = m.turn;
    v.direction = m.direction_centunits;
    v.laneCount = m.lane_count;
    for (size_t i = 0; i < kMaxLanes; ++i) {
        v.lanes[i].arrows = m.lanes[i].arrows;
        v.lanes[i].recommended = m.lanes[i].recommended;
    }
    out = v;
    return true;
}
inline bool replyAccepted(const uint8_t *p, size_t n, uint32_t seq) {
    return p && n == kReplyBytes && !memcmp(p, "NGA1", 4) && rd32(p + 4) == seq && rd32(p + 8) == 0 &&
           rd32(p + 12) == kModelId;
}
// AndroidX Maneuver TYPE_* values and the native wire's 256-circle centunits.
inline bool turnDirection(int aaType, uint32_t &turn, uint16_t &direction) {
    if (aaType == 7) { turn = NG_LEFT; direction = 6400; return true; }
    if (aaType == 8) { turn = NG_RIGHT; direction = 19200; return true; }
    if (aaType == 36) { turn = NG_STRAIGHT; direction = 0; return true; }
    return false;
}
inline bool fromNav(const kombinav::NavView &v, Message &m) {
    double metres = 0;
    if (!kombinav::distanceMetres(v, metres) || !(metres >= 0 && metres <= 2000000.0) ||
        !turnDirection(v.aa_type, m.turn, m.direction))
        return false;
    m.distanceMm = uint32_t(metres * 1000.0 + 0.5);
    // E114 renders the turn and distance. The stock lane style still
    // requires a qualified native current lane point.
    m.laneCount = 0;
    for (size_t i = 0; i < kMaxLanes; ++i)
        m.lanes[i] = Lane();
    return true;
}
} // namespace native_model
} // namespace nbt
#endif
