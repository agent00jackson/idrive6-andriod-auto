// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#ifndef NBT_NATIVE_CONTROLLER_PROTOCOL_HPP
#define NBT_NATIVE_CONTROLLER_PROTOCOL_HPP

#include "native_model_protocol.hpp"

namespace nbt {
namespace native_model {
namespace controller {
constexpr size_t kBytes = 64;
constexpr uint32_t kRouteSession = 1024u;
constexpr uint32_t kCompletedRearm = 512u;
constexpr uint32_t kRunningReadyOwner = 7u;
constexpr uint32_t kPreflightReset = 32u;
constexpr uint32_t kPreflightFresh = 64u;
constexpr uint32_t kPreflightEligible = 128u;

enum Operation { kProbe, kArm, kHeartbeat, kStop, kReset, kRearm };
enum Phase { kIdle, kArmed, kSelected, kEmptyReturned, kYielded, kRefused };

struct Reply {
    uint32_t phase, result, sequence, flags;
    uint64_t nonce, epoch, deadline;
};

inline uint64_t read64(const uint8_t* p) { return uint64_t(rd32(p + 4)) << 32 | rd32(p); }

inline void write64(uint8_t* p, uint64_t value) {
    wr32(p, uint32_t(value));
    wr32(p + 4, uint32_t(value >> 32));
}

inline void encode(uint8_t* bytes, Operation operation, uint32_t sequence, uint64_t nonce,
                   uint64_t epoch, uint32_t fullPid, uint32_t producer, uint64_t session, uint64_t now) {
    memset(bytes, 0, kBytes);
    memcpy(bytes, "NCT1", 4);
    bytes[4] = 1;
    bytes[6] = kBytes;
    wr32(bytes + 8, operation);
    wr32(bytes + 12, sequence);
    write64(bytes + 16, nonce);
    write64(bytes + 24, epoch);
    wr32(bytes + 32, fullPid);
    wr32(bytes + 36, producer);
    write64(bytes + 40, session);
    write64(bytes + 48, now);
}

inline bool decode(const uint8_t* bytes, size_t size, uint32_t sequence, Reply& reply) {
    if (size != kBytes || memcmp(bytes, "NCS1", 4) || rd16(bytes + 4) != 1 ||
        rd16(bytes + 6) != kBytes || rd32(bytes + 32) != sequence ||
        rd32(bytes + 56) != kModelId || rd32(bytes + 60) != 3201)
        return false;
    reply = {rd32(bytes + 8), rd32(bytes + 12), sequence, rd32(bytes + 36),
             read64(bytes + 16), read64(bytes + 24), read64(bytes + 40)};
    return reply.nonce && reply.phase <= kRefused && !(reply.flags & ~2047u);
}
}
}
}
#endif
