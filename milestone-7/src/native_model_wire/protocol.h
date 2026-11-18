#ifndef NATIVE_GUIDANCE_PROTOCOL_H
#define NATIVE_GUIDANCE_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

/* Little-endian, fixed-size local messages. Never cast untrusted wire bytes. */
#define NG_FRAME_BYTES 80u
#define NG_MAX_LANES 8u
#define NG_MAX_DISTANCE_MM 2000000000u
#ifdef __QNXNTO__
#define NG_SOCKET_ROOT "/dev/shmem"
#else
#define NG_SOCKET_ROOT "/tmp"
#endif
enum ng_operation { NG_BEGIN = 1, NG_UPDATE = 2, NG_STOP = 3, NG_PROBE = 4 };
enum ng_turn { NG_STRAIGHT = 1, NG_LEFT = 2, NG_RIGHT = 3, NG_UTURN = 4 };
enum ng_result {
    NG_OK = 0, NG_BAD_FRAME = 1, NG_BAD_VALUE = 2,
    NG_BAD_SESSION = 3, NG_STALE = 4, NG_BACKEND_REJECTED = 5
};
enum ng_lane_arrow {
    NG_LANE_STRAIGHT=1, NG_LANE_LEFT=2, NG_LANE_RIGHT=4,
    NG_LANE_SLIGHT_LEFT=8, NG_LANE_SLIGHT_RIGHT=16,
    NG_LANE_SHARP_LEFT=32, NG_LANE_SHARP_RIGHT=64,
    NG_LANE_UTURN_LEFT=128, NG_LANE_UTURN_RIGHT=256
};
#define NG_LANE_ARROW_MASK 511u
#define NG_MAX_LANE_ARROWS 9u
struct ng_lane { uint16_t arrows; uint16_t recommended; };
struct ng_message {
    uint32_t operation, sequence;
    uint64_t session;
    uint32_t maneuver, distance_mm, turn;
    uint16_t direction_centunits;
    uint16_t lane_count;
    struct ng_lane lanes[NG_MAX_LANES];
    /* 0 encodes unchanged NGV1/fixed-NOW diagnostic behavior. NGV2 carries
     * explicit native MANPHASE1..4 in reserved word72, never lane word68. */
    uint32_t phase;
    /* 0 legacy, 1 NGV3 single direction, 2 NGV4 per-direction highlights. */
    uint32_t lane_mode;
};
struct ng_state {
    uint64_t session;
    uint32_t sequence, maneuver;
    unsigned active;
};
struct ng_backend {
    /* Success means the update was accepted; failure must preserve ownership. */
    int (*update)(void *, const struct ng_message *);
    int (*clear)(void *, uint64_t session);
    void *context;
};
enum ng_result ng_decode(const unsigned char *, size_t, struct ng_message *);
enum ng_result ng_apply(struct ng_state *, const struct ng_message *,
                        const struct ng_backend *);
void ng_encode(unsigned char [NG_FRAME_BYTES], const struct ng_message *);
#endif
