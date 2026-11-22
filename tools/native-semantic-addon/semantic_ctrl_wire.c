// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#include "semantic_ctrl_wire.h"

#include <string.h>

static uint16_t rd16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)rd16(p) | ((uint32_t)rd16(p + 2) << 16);
}
static uint64_t rd64(const uint8_t *p) {
    return ((uint64_t)rd32(p + 4) << 32) | (uint64_t)rd32(p);
}
static void wr16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
}
static void wr32(uint8_t *p, uint32_t v) {
    wr16(p, (uint16_t)(v & 0xffffu));
    wr16(p + 2, (uint16_t)((v >> 16) & 0xffffu));
}
static void wr64(uint8_t *p, uint64_t v) {
    wr32(p, (uint32_t)(v & 0xffffffffu));
    wr32(p + 4, (uint32_t)(v >> 32));
}

void sc_machine_init(struct sc_machine *m, uint64_t initial_nonce, uint32_t lease_ms) {
    memset(m, 0, sizeof(*m));
    m->phase = SC_PHASE_IDLE;
    m->nonce = initial_nonce ? initial_nonce : 1u;
    m->lease_ms = lease_ms ? lease_ms : 10000u;
}

int sc_decode_request(const uint8_t *bytes, size_t size, struct sc_request *out) {
    if (!bytes || !out || size != SC_WIRE_BYTES)
        return 0;
    if (memcmp(bytes, "NCT1", 4) != 0 || rd16(bytes + 4) != 1u || rd16(bytes + 6) != SC_WIRE_BYTES)
        return 0;
    out->operation = rd32(bytes + 8);
    out->sequence = rd32(bytes + 12);
    out->nonce = rd64(bytes + 16);
    out->epoch = rd64(bytes + 24);
    out->full_pid = rd32(bytes + 32);
    out->producer = rd32(bytes + 36);
    out->session = rd64(bytes + 40);
    out->now_ms = rd64(bytes + 48);
    return out->operation <= SC_OP_REARM && out->sequence != 0u;
}

void sc_encode_reply(uint8_t out[SC_WIRE_BYTES], const struct sc_reply *reply) {
    memset(out, 0, SC_WIRE_BYTES);
    memcpy(out, "NCS1", 4);
    wr16(out + 4, 1u);
    wr16(out + 6, SC_WIRE_BYTES);
    wr32(out + 8, reply->phase);
    wr32(out + 12, reply->result);
    wr64(out + 16, reply->nonce);
    wr64(out + 24, reply->epoch);
    wr32(out + 32, reply->sequence);
    wr32(out + 36, reply->flags);
    wr64(out + 40, reply->deadline);
    wr32(out + 56, SC_MODEL_ID);
    wr32(out + 60, SC_VIEW_ID);
}

static uint32_t base_flags(const struct sc_machine *m, int eligible) {
    uint32_t f = SC_FLAG_RUNNING_READY_OWNER | SC_FLAG_ROUTE_SESSION |
                 SC_FLAG_PREFLIGHT_RESET | SC_FLAG_PREFLIGHT_FRESH;
    if (eligible)
        f |= SC_FLAG_PREFLIGHT_ELIGIBLE;
    if (m->phase == SC_PHASE_EMPTY_RETURNED)
        f |= SC_FLAG_COMPLETED_REARM;
    return f;
}

static void fill_reply(const struct sc_machine *m, uint32_t seq, uint32_t result,
                       int eligible, struct sc_reply *reply) {
    reply->phase = m->phase;
    reply->result = result;
    reply->sequence = seq;
    reply->flags = base_flags(m, eligible);
    reply->nonce = m->nonce;
    reply->epoch = m->epoch;
    reply->deadline = m->deadline_ms;
}

void sc_machine_handle(struct sc_machine *m, const struct sc_request *req,
                       uint64_t now_ms, const struct sc_ops *ops,
                       struct sc_reply *reply) {
    uint32_t foreign = 0;
    int eligible = (ops && ops->can_select) ? ops->can_select(ops->ctx, &foreign) : 1;

    switch (req->operation) {
    case SC_OP_PROBE:
        fill_reply(m, req->sequence, 0u, eligible, reply);
        return;

    case SC_OP_ARM:
        if (m->phase != SC_PHASE_IDLE || req->nonce != m->nonce || !req->epoch || !req->session) {
            fill_reply(m, req->sequence, 1u, eligible, reply);
            return;
        }
        if (!eligible || foreign) {
            m->phase = SC_PHASE_YIELDED;
            fill_reply(m, req->sequence, 1u, 0, reply);
            return;
        }
        if (ops && ops->start_route && !ops->start_route(ops->ctx, req->session)) {
            fill_reply(m, req->sequence, 2u, eligible, reply);
            return;
        }
        m->epoch = req->epoch;
        m->session = req->session;
        m->last_seq = req->sequence;
        m->deadline_ms = now_ms + m->lease_ms;
        m->phase = SC_PHASE_ARMED;
        fill_reply(m, req->sequence, 0u, 1, reply);
        if (req->now_ms + m->lease_ms > reply->deadline)
            reply->deadline = req->now_ms + m->lease_ms;
        return;

    case SC_OP_HEARTBEAT:
        if (req->nonce != m->nonce || req->epoch != m->epoch) {
            fill_reply(m, req->sequence, 1u, eligible, reply);
            return;
        }
        if (m->phase == SC_PHASE_ARMED || m->phase == SC_PHASE_SELECTED) {
            if (foreign) {
                m->phase = SC_PHASE_YIELDED;
                fill_reply(m, req->sequence, 1u, 0, reply);
                return;
            }
            m->phase = SC_PHASE_SELECTED;
            m->last_seq = req->sequence;
            m->deadline_ms = now_ms + m->lease_ms;
            fill_reply(m, req->sequence, 0u, 1, reply);
            if (req->now_ms + m->lease_ms > reply->deadline)
                reply->deadline = req->now_ms + m->lease_ms;
            return;
        }
        if (m->phase == SC_PHASE_EMPTY_RETURNED) {
            fill_reply(m, req->sequence, 0u, eligible, reply);
            return;
        }
        fill_reply(m, req->sequence, 1u, eligible, reply);
        return;

    case SC_OP_STOP:
        if (req->nonce != m->nonce || req->epoch != m->epoch) {
            fill_reply(m, req->sequence, 1u, eligible, reply);
            return;
        }
        if (m->phase == SC_PHASE_ARMED || m->phase == SC_PHASE_SELECTED) {
            if (ops && ops->stop_route)
                (void)ops->stop_route(ops->ctx, m->session);
            m->phase = SC_PHASE_EMPTY_RETURNED;
            m->deadline_ms = 0;
        }
        fill_reply(m, req->sequence, 0u, eligible, reply);
        return;

    case SC_OP_REARM:
    case SC_OP_RESET:
        if (m->phase == SC_PHASE_EMPTY_RETURNED || m->phase == SC_PHASE_YIELDED) {
            if (req->nonce != m->nonce) {
                fill_reply(m, req->sequence, 1u, eligible, reply);
                return;
            }
            m->phase = SC_PHASE_IDLE;
            m->epoch = 0;
            m->session = 0;
            m->deadline_ms = 0;
            m->last_seq = 0;
            if (++m->nonce == 0u) ++m->nonce;
            fill_reply(m, req->sequence, 0u, eligible, reply);
            return;
        }
        fill_reply(m, req->sequence, 1u, eligible, reply);
        return;

    default:
        fill_reply(m, req->sequence, 1u, eligible, reply);
        return;
    }
}

int sc_machine_tick(struct sc_machine *m, uint64_t now_ms, const struct sc_ops *ops) {
    if (m->phase != SC_PHASE_ARMED && m->phase != SC_PHASE_SELECTED)
        return 0;
    uint32_t foreign = 0;
    if (ops && ops->can_select)
        (void)ops->can_select(ops->ctx, &foreign);
    if (foreign) {
        m->phase = SC_PHASE_YIELDED;
        m->deadline_ms = 0;
        return 1;
    }
    if (m->deadline_ms && now_ms >= m->deadline_ms) {
        if (ops && ops->stop_route)
            (void)ops->stop_route(ops->ctx, m->session);
        m->phase = SC_PHASE_EMPTY_RETURNED;
        m->deadline_ms = 0;
        return 1;
    }
    return 0;
}
