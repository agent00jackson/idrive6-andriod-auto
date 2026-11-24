// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#ifndef NBT_NATIVE_MODEL_PROTOCOL_HPP
#define NBT_NATIVE_MODEL_PROTOCOL_HPP
#include "cluster/nav_msg.hpp"
#include "native_model_wire/capabilities.h"
#include <stdint.h>
#include <string.h>
extern "C" {
#include "native_model_wire/protocol.h"
}

namespace nbt {
namespace native_model {
// NGV1 is a local adapter protocol, not an OEM network/service interface.
const size_t kFrameBytes = 80, kMaxFrameBytes = 160, kReplyBytes = 16, kMaxLanes = 8;
const uint32_t kModelId = 0x7f4e4701, kMaxDistanceMm = 2000000000;
enum Operation { Begin = 1, Update = 2, Stop = 3, Probe = 4 };
struct Lane {
    uint16_t arrows;
    uint16_t recommended;
    Lane() : arrows(0), recommended(0) {}
};
struct Message {
    uint32_t operation, sequence;
    uint64_t session;
    uint32_t maneuver, distanceMm, turn;
    uint16_t direction, laneCount;
    uint32_t phase, laneMode;
    Lane lanes[kMaxLanes];
    uint16_t metadataFlags, aaType, roadLength;
    int32_t exitNumber, exitAngle;
    char road[NG_MAX_ROAD_BYTES + 1];
    Message()
        : operation(0), sequence(0), session(0), maneuver(0), distanceMm(0), turn(0), direction(0), laneCount(0),
          phase(0), laneMode(0), metadataFlags(0), aaType(0), roadLength(0), exitNumber(0), exitAngle(0) {
        memset(road,0,sizeof(road));
    }
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
    n.phase = m.phase;
    n.lane_mode = m.laneMode;
    n.metadata_flags = m.metadataFlags;
    n.aa_type = m.aaType;
    n.road_length = m.roadLength;
    n.exit_number = m.exitNumber;
    n.exit_angle = m.exitAngle;
    memcpy(n.road,m.road,sizeof(n.road));
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
    return ng_decode(p, ng_frame_size(&n), &checked) == NG_OK;
}
inline size_t frameSize(const Message& m) {
    ng_message n=nativeMessage(m);
    return ng_frame_size(&n);
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
    v.phase = m.phase;
    v.laneMode = m.lane_mode;
    v.metadataFlags = m.metadata_flags;
    v.aaType = m.aa_type;
    v.roadLength = m.road_length;
    v.exitNumber = m.exit_number;
    v.exitAngle = m.exit_angle;
    memcpy(v.road,m.road,sizeof(v.road));
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
// The stock E114 receiver accepts only these three semantic tuples.
inline bool basicTurn(const Message& message) {
    return (message.turn == NG_LEFT && message.direction == 6400) ||
           (message.turn == NG_RIGHT && message.direction == 19200) ||
           (message.turn == NG_STRAIGHT && message.direction == 0);
}

inline bool adaptToCapabilities(Message& message, unsigned capabilities) {
    if (!(capabilities & NG_CAP_TYPED_TURNS) && !basicTurn(message)) return false;
    const unsigned offRampCapabilities = NG_CAP_EXTENDED_FIELDS | NG_CAP_OFF_RAMPS;
    if (message.aaType >= 21 && message.aaType <= 24 &&
        (capabilities & offRampCapabilities) != offRampCapabilities)
        return false;
    const unsigned forkCapabilities = NG_CAP_EXTENDED_FIELDS | NG_CAP_FORKS;
    if ((message.aaType == 25 || message.aaType == 26) &&
        (capabilities & forkCapabilities) != forkCapabilities)
        return false;
    const unsigned arrivalCapabilities = NG_CAP_EXTENDED_FIELDS | NG_CAP_ARRIVAL;
    if (message.aaType >= 39 && message.aaType <= 42 &&
        (capabilities & arrivalCapabilities) != arrivalCapabilities)
        return false;
    if (!(capabilities & NG_CAP_EXTENDED_FIELDS)) {
        if ((message.aaType>=32&&message.aaType<=35)||(message.aaType>=43&&message.aaType<=46))
            return false;
        message.metadataFlags=message.aaType=message.roadLength=0;
        message.exitNumber=message.exitAngle=0;
        memset(message.road,0,sizeof(message.road));
    }
    if (!(capabilities & NG_CAP_TYPED_LANES)) {
        message.laneCount = 0;
        message.laneMode = message.phase = 0;
        for (auto& lane : message.lanes) lane = Lane();
    }
    return true;
}

inline bool samePayload(const Message& left, const Message& right) {
    if (left.turn != right.turn || left.direction != right.direction || left.distanceMm != right.distanceMm ||
        left.phase != right.phase || left.laneMode != right.laneMode || left.laneCount != right.laneCount ||
        left.metadataFlags != right.metadataFlags || left.aaType != right.aaType ||
        left.exitNumber != right.exitNumber || left.exitAngle != right.exitAngle ||
        left.roadLength != right.roadLength || memcmp(left.road,right.road,sizeof(left.road)))
        return false;
    for (unsigned i = 0; i < left.laneCount; ++i) {
        if (left.lanes[i].arrows != right.lanes[i].arrows ||
            left.lanes[i].recommended != right.lanes[i].recommended)
            return false;
    }
    return true;
}

inline bool turnDirection(int type, uint32_t& turn, uint16_t& angle) {
    switch (type) {
        // Match the Android Auto display's shared keep and slight-turn shapes.
        case 3: case 5: turn = NG_LEFT; angle = 3200; return true;
        case 4: case 6: turn = NG_RIGHT; angle = 22400; return true;
        case 7: turn = NG_LEFT; angle = 6400; return true;
        case 8: turn = NG_RIGHT; angle = 19200; return true;
        case 9: turn = NG_LEFT; angle = 9600; return true;
        case 10: turn = NG_RIGHT; angle = 16000; return true;
        case 11: turn = NG_UTURN; angle = 6400; return true;
        case 12: turn = NG_UTURN; angle = 19200; return true;
        case 21: turn = NG_LEFT; angle = 3200; return true;   // slight highway exit
        case 22: turn = NG_RIGHT; angle = 22400; return true;
        case 23: turn = NG_LEFT; angle = 6400; return true;   // normal highway exit
        case 24: turn = NG_RIGHT; angle = 19200; return true;
        case 25: turn = NG_LEFT; angle = 6400; return true;   // selected fork branch
        case 26: turn = NG_RIGHT; angle = 19200; return true;
        case 1: case 2: case 36: turn = NG_STRAIGHT; angle = 0; return true; // DEPART, NAME_CHANGE or STRAIGHT
        case 32: case 33: case 43: case 44: turn = NG_RIGHT; angle = 19200; return true;
        case 34: case 35: case 45: case 46: turn = NG_LEFT; angle = 6400; return true;
        case 39: case 40: turn = NG_STRAIGHT; angle = 0; return true; // destination or straight arrival
        case 41: turn = NG_LEFT; angle = 6400; return true;
        case 42: turn = NG_RIGHT; angle = 19200; return true;
        default: return false;
    }
}

inline bool validUtf8(const char *p, size_t n) {
    size_t i=0;
    while(i<n) {
        const uint8_t a=uint8_t(p[i++]);
        if(a<0x80) { if(!a)return false; continue; }
        if(a>=0xc2&&a<=0xdf) {
            if(i>=n||(uint8_t(p[i])&0xc0)!=0x80)return false;
            ++i; continue;
        }
        if(a>=0xe0&&a<=0xef) {
            if(i+1>=n||(uint8_t(p[i])&0xc0)!=0x80||(uint8_t(p[i+1])&0xc0)!=0x80)return false;
            if((a==0xe0&&uint8_t(p[i])<0xa0)||(a==0xed&&uint8_t(p[i])>=0xa0))return false;
            i+=2; continue;
        }
        if(a>=0xf0&&a<=0xf4) {
            if(i+2>=n||(uint8_t(p[i])&0xc0)!=0x80||(uint8_t(p[i+1])&0xc0)!=0x80||
               (uint8_t(p[i+2])&0xc0)!=0x80)return false;
            if((a==0xf0&&uint8_t(p[i])<0x90)||(a==0xf4&&uint8_t(p[i])>=0x90))return false;
            i+=3; continue;
        }
        return false;
    }
    return true;
}

inline bool lanesFromNav(const kombinav::NavView& view, Message& message) {
    if (view.lanes.empty() || view.lanes.size() > kMaxLanes) return false;
    static const uint16_t arrowFor[] = {
        0, NG_LANE_STRAIGHT, NG_LANE_SLIGHT_LEFT, NG_LANE_SLIGHT_RIGHT, NG_LANE_LEFT,
        NG_LANE_RIGHT, NG_LANE_SHARP_LEFT, NG_LANE_SHARP_RIGHT, NG_LANE_UTURN_LEFT, NG_LANE_UTURN_RIGHT
    };
    Lane candidate[kMaxLanes];
    for (size_t i = 0; i < view.lanes.size(); ++i) {
        if (view.lanes[i].empty()) return false;
        Lane& lane = candidate[i];
        for (const auto& arrow : view.lanes[i]) {
            if (arrow.shape <= 0 || size_t(arrow.shape) >= sizeof(arrowFor) / sizeof(*arrowFor)) return false;
            const uint16_t bit = arrowFor[arrow.shape];
            if ((lane.arrows & bit) && bool(lane.recommended & bit) != arrow.highlighted) return false;
            lane.arrows |= bit;
            if (arrow.highlighted) lane.recommended |= bit;
        }
    }
    for (size_t i = 0; i < view.lanes.size(); ++i) message.lanes[i] = candidate[i];
    message.laneCount = uint16_t(view.lanes.size());
    message.phase = 4;
    message.laneMode = 2;
    return true;
}

inline bool fromNav(const kombinav::NavView& view, Message& out, unsigned capabilities = 0) {
    double metres = 0;
    Message message;
    if (!kombinav::distanceMetres(view, metres) || !(metres >= 0 && metres <= 2000000.0) ||
        !turnDirection(view.aa_type, message.turn, message.direction))
        return false;
    message.aaType=uint16_t(view.aa_type);
    if(!adaptToCapabilities(message,capabilities))return false;
    message.distanceMm = uint32_t(metres * 1000.0 + 0.5);
    if (capabilities & NG_CAP_TYPED_LANES) lanesFromNav(view, message);
    if (capabilities & NG_CAP_EXTENDED_FIELDS) {
        message.metadataFlags=NG_META_AA_TYPE;
        // Core uses a single space for an unnamed road on the manual display.
        if (!view.road.empty() && view.road != " " && validUtf8(view.road.data(), view.road.size())) {
            size_t length = view.road.size();
            if (length > NG_MAX_ROAD_BYTES) {
                length = NG_MAX_ROAD_BYTES;
                while ((uint8_t(view.road[length]) & 0xc0) == 0x80)
                    --length;
            }
            message.metadataFlags |= NG_META_ROAD;
            message.roadLength = uint16_t(length);
            memcpy(message.road, view.road.data(), length);
        }
        const bool roundabout=(view.aa_type>=32&&view.aa_type<=35)||(view.aa_type>=43&&view.aa_type<=46);
        if(roundabout&&view.has_exit_number&&view.exit_number>=1&&view.exit_number<=255) {
            message.metadataFlags|=NG_META_EXIT_NUMBER;
            message.exitNumber=int32_t(view.exit_number);
        }
        if(roundabout&&view.has_exit_angle&&view.exit_angle>=0&&view.exit_angle<=360) {
            message.metadataFlags|=NG_META_EXIT_ANGLE;
            message.exitAngle=int32_t(view.exit_angle);
        }
    }
    out = message;
    return true;
}
} // namespace native_model
} // namespace nbt
#endif
