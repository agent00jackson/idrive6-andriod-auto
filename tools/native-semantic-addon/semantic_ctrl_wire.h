// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#ifndef NBT_SEMANTIC_CTRL_WIRE_H
#define NBT_SEMANTIC_CTRL_WIRE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SC_WIRE_BYTES 64u
#define SC_MODEL_ID 0x7f4e4701u
#define SC_VIEW_ID 3201u

/* Flags matching milestone-7/src/native_controller_protocol.hpp */
#define SC_FLAG_RUNNING_READY_OWNER 7u
#define SC_FLAG_PREFLIGHT_RESET 32u
#define SC_FLAG_PREFLIGHT_FRESH 64u
#define SC_FLAG_PREFLIGHT_ELIGIBLE 128u
#define SC_FLAG_COMPLETED_REARM 512u
#define SC_FLAG_ROUTE_SESSION 1024u

enum sc_operation {
    SC_OP_PROBE = 0,
    SC_OP_ARM = 1,
    SC_OP_HEARTBEAT = 2,
    SC_OP_STOP = 3,
    SC_OP_RESET = 4,
    SC_OP_REARM = 5,
};

enum sc_phase {
    SC_PHASE_IDLE = 0,
    SC_PHASE_ARMED = 1,
    SC_PHASE_SELECTED = 2,
    SC_PHASE_EMPTY_RETURNED = 3,
    SC_PHASE_YIELDED = 4,
    SC_PHASE_REFUSED = 5,
};

struct sc_request {
    uint32_t operation;
    uint32_t sequence;
    uint64_t nonce;
    uint64_t epoch;
    uint32_t full_pid;
    uint32_t producer;
    uint64_t session;
    uint64_t now_ms;
};

struct sc_reply {
    uint32_t phase;
    uint32_t result;
    uint32_t sequence;
    uint32_t flags;
    uint64_t nonce;
    uint64_t epoch;
    uint64_t deadline;
};

struct sc_ops {
    /* Return 1 if we can safely start/reuse our trip (no foreign BMW trip). */
    int (*can_select)(void *ctx, uint32_t *out_foreign_active);
    /* Start or reuse trip + attach Basic's route + StartGuidance. Returns 1 on success. */
    int (*start_route)(void *ctx, uint64_t session);
    /* Stop/retire our owned guidance cleanly. Returns 1 on success. */
    int (*stop_route)(void *ctx, uint64_t session);
    void *ctx;
};

struct sc_machine {
    uint32_t phase;
    uint64_t nonce;
    uint64_t epoch;
    uint64_t session;
    uint64_t deadline_ms;
    uint32_t last_seq;
    uint32_t lease_ms;
};

void sc_machine_init(struct sc_machine *m, uint64_t initial_nonce, uint32_t lease_ms);
int sc_decode_request(const uint8_t *bytes, size_t size, struct sc_request *out);
void sc_encode_reply(uint8_t out[SC_WIRE_BYTES], const struct sc_reply *reply);

/* Process one decoded NCT1 request and populate reply. */
void sc_machine_handle(struct sc_machine *m, const struct sc_request *req,
                       uint64_t now_ms, const struct sc_ops *ops,
                       struct sc_reply *reply);

/* Periodic tick: retires an armed/selected session if lease expired or foreign
 * BMW route took over. Returns 1 if a state transition occurred. */
int sc_machine_tick(struct sc_machine *m, uint64_t now_ms, const struct sc_ops *ops);

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_CTRL_WIRE_H */
