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
struct ng_lane { uint16_t arrows; uint8_t recommended; };
struct ng_message {
    uint32_t operation, sequence;
    uint64_t session;
    uint32_t maneuver, distance_mm, turn;
    uint16_t direction_centunits;
    uint16_t lane_count;
    struct ng_lane lanes[NG_MAX_LANES];
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
