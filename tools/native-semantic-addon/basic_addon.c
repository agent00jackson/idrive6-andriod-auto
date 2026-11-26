// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
/* Production NavigationBasic semantic guidance addon (exact X20511I ARM QNX).
 * Speaks NGV1..5 + NGC1 (NG_CAP_SUPPORTED) + ng_alive on NG_SOCKET_ROOT so
 * nbt-aad's Semantic Navigation Mode drives untouched NavigationFully and
 * untouched NavNBTEvoController (6WB / 6WA / HUD / J29 RIC / CID).
 * Emulator-only mapless NDS stubs are intentionally excluded. */

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "fixture-provider.h"
#include "native_model_wire/alive_wire.h"
#include "native_model_wire/capabilities.h"
#include "native_model_wire/protocol.h"
#include "semantic_cue_map.h"
#include "semantic_handoff.h"
#include "semantic_minimap_render.h"
#include "semantic_minimap_shm.h"
#include "semantic_nds_map.h"
#include "semantic_nds_probe.h"
#include "semantic_xar_hook.h"

#if !defined(__QNXNTO__) || !defined(__arm__)
#error basic_addon.c targets ARM QNX NavigationBasic
#endif

#define FN(a, result, ...) ((result (*)(__VA_ARGS__))(uintptr_t)(a))

struct qnx_sockaddr_un {
    uint8_t sun_len;
    uint8_t sun_family;
    char sun_path[104];
};

static int qnx_chmod(const char *path, mode_t mode) {
    static int (*fn)(const char *, mode_t);
    if (!fn) {
        void *h = dlopen("libc.so.3", RTLD_NOW | RTLD_LOCAL);
        if (h) fn = (int (*)(const char *, mode_t))dlsym(h, "chmod");
    }
    return fn ? fn(path, mode) : -1;
}

static uint64_t mono_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

static void basic_log(const char *fmt, ...) {
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    size_t len = (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1u;
    int fd = open("/dev/shmem/semantic-basic.log", O_WRONLY | O_CREAT | O_APPEND, 0600);
    if (fd >= 0) {
        (void)write(fd, line, len);
        close(fd);
    }
}

enum {
    ADMINISTRATOR = 0x011cd4cc,
    DATA_ACCESS_READY = 0x02520904,
    COMPONENT_VPTR = 0x00f86c30,
    STUB_VPTR = 0x00f87d40,
    CALC_VPTR = 0x00f87420,
    RESOLVER_VPTR = 0x00f872a0,
    ROUTE_DATA_VPTR = 0x00f872f8,
    MUX_VPTR = 0x00ee1690,
    MUX_OWNER_HINT = 0x02795e98,
    MUX9_HINT = 0x02795d88,
    HEAP_SCAN_BEGIN = 0x0255c000,
    HEAP_SCAN_END = 0x20000000,
    RG_VPTR = 0x00f44450,
    RG_CARPOS_OFF = 0x24c,
    RG_CARPOS_VPTR = 0x00f446b8,
    RG_CARPOS_ORIG = 0x00402674,
    RG_DTMCALC_OFF = 0x308,
    RG_DTMCALC_VPTR = 0x00f447e0,
    RG_DTMCALC_ORIG = 0x00402268,
    MILCA_VPTR = 0x00f3f3c8,
};

enum {
    PHASE_INSTALL_RESOLVER = 1,
    PHASE_CALC_ROUTE = 2,
    PHASE_ACTIVATE_GUIDANCE = 5,
    PHASE_PUBLISH_CUE = 6,
    PHASE_CLEAR_GUIDANCE = 8,
};

struct owner_event {
    void **vtable;
    uint32_t native_prefix[5];
    uint32_t component;
    uint32_t stub;
    uint16_t thread;
    uint16_t phase;
    uint32_t arg0;
};
_Static_assert(offsetof(struct owner_event, component) == 24, "CHBEvent prefix");

static volatile uint32_t owner_dispatch_state;
static volatile uint32_t hud_dispatch_state;
static volatile uint32_t resolver_wrapper_state;
static volatile uint32_t resolver_intercept_kind;
static volatile uint32_t resolver_armed_flag;
static volatile uint32_t active_route_id;
static volatile uint32_t created_route_id;
static volatile uint32_t tour_high_cached;
static volatile uint32_t tour_low_cached;
static uint32_t resolver_vtable_copy[10];
static uint32_t wrapped_resolver_addr;
static uint32_t wrapped_stub_addr;
static uint32_t rg_carpos_vtable_copy[8];   /* [-2..+5]: offset-to-top, RTTI, 6 slots */
static uint32_t rg_dtmcalc_vtable_copy[5];  /* [-2..+2]: offset-to-top, RTTI, 3 slots */
static volatile uint32_t g_rg_gps_suppressed;
static uint32_t fixture_source_handle;
static uint32_t fixture_factory_calls;

/* Pending cue passed from worker thread to ctx 9 (CRouteGuidance/CMiLCa). */
static struct semantic_basic_cue g_pending_cue;
static uint32_t g_pending_dtm_only;
static uint32_t g_next_man_id = 1u;
static uint32_t g_active_man_id = 0u;
static uint32_t g_sort_counter = 1u;

struct fixture_uuid { _Alignas(8) uint32_t word[4]; };
static const struct fixture_uuid routing_implementation = {
    {0x36424449, 0x454c5846, 0x41504341, 0x01000001}
};
static const uint32_t routing_interface_expected[4] = {
    0x931e4541, 0x526d541e, 0x6d785403, 0x8578f2ad
};

struct routing_factory;
struct routing_factory_vtable {
    void (*reserved[3])(void);
    const struct basic_fixture_routing *(*create)(struct routing_factory *, const uint32_t *);
};
struct routing_factory { const struct routing_factory_vtable *vtable; };

static const struct basic_fixture_routing *create_routing(
    struct routing_factory *factory, const uint32_t *parameters) {
    (void)factory;
    if (!fixture_source_handle || (parameters[2] & 0xffffu) != 1u)
        return NULL;
    ++fixture_factory_calls;
    return &basic_fixture_routing;
}

static const struct routing_factory_vtable routing_factory_methods = {
    .create = create_routing,
};
static struct routing_factory routing_factory = { .vtable = &routing_factory_methods };

static uint32_t register_fixture_source(void) {
    const uint32_t *iface = (const uint32_t *)(uintptr_t)0x11ceb20;
    if (memcmp(iface, routing_interface_expected, sizeof(routing_interface_expected)) != 0)
        return 0;
    if (fixture_source_handle)
        return fixture_source_handle;
    static _Alignas(8) uint8_t descriptor[0x80];
    static const char name[] = "FixtureRouting";
    uint32_t data_access = *(uint32_t *)(uintptr_t)0x25200b8;
    if (data_access < 0x10000u)
        return 0;
    FN(0x68587c, void, void *, const char *, uint32_t, uint32_t, uint32_t)(
        descriptor, name, 1, 1, 1);
    FN(0x580788, void, void *, const void *, const void *)(
        descriptor, iface, &routing_implementation);
    uint32_t handle = FN(0x687134, uint32_t, void *, void *, uint32_t, uint32_t)(
        (void *)(uintptr_t)data_access, descriptor, 0, 0);
    if (!handle || handle == UINT32_MAX)
        return 0;
    fixture_source_handle = handle;
    FN(0x71fed4, void, const void *, void *)(&routing_implementation, &routing_factory);
    uint8_t request[0x40];
    memset(request, 0, sizeof(request));
    FN(0x681488, void, void *, void *, uint32_t, const void *, uint32_t, uint32_t)(
        request, NULL, 0, iface, handle, 0);
    const void *accessor = FN(0x26648c, const void *, const void *, const void *)(
        &routing_implementation, request + 4);
    if (*(uint32_t *)(request + 0x30) != 1u ||
        accessor != &basic_fixture_routing || fixture_factory_calls != 1u)
        return 0;
    return handle;
}

static int read_at(int fd, uint32_t address, void *value, size_t size) {
    return address >= 0x10000u && address <= 0x7fffffffu && size <= 4096u &&
           size <= 0x80000000u - address &&
           lseek(fd, (off_t)address, SEEK_SET) == (off_t)address &&
           read(fd, value, size) == (ssize_t)size;
}

static int word(int fd, uint32_t address, uint32_t *value) {
    return read_at(fd, address, value, sizeof(*value));
}

static int verify_identity(int fd, const void *administrator) {
    uint32_t observed = 0;
    const struct { uint32_t address, opcode; } pins[] = {
        {0x56c334, 0xe92d4037}, {0x57bb80, 0xe92d40f7},
        {0x57bf34, 0xe59fc268}, {0x85c82c, 0xe92d40f0},
    };
    if (!administrator || !word(fd, ADMINISTRATOR, &observed) ||
        observed != (uintptr_t)administrator)
        return 0;
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        if (!word(fd, pins[i].address, &observed) || observed != pins[i].opcode)
            return 0;
    }
    return 1;
}

static int component_valid(int fd, uint32_t candidate) {
    uint32_t observed = 0;
    const struct { uint32_t offset, expected; } fields[] = {
        {0, COMPONENT_VPTR}, {0xac, STUB_VPTR},
        {0x2b0, CALC_VPTR}, {0x464, ROUTE_DATA_VPTR},
        {0x1d8, candidate}, {0x31c, candidate}, {0x488, candidate},
    };
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        if (!word(fd, candidate + fields[i].offset, &observed) ||
            observed != fields[i].expected)
            return 0;
    }
    return 1;
}

static int check_owner_mux(int fd, uint32_t mux, uint32_t *component, uint16_t *thread) {
    uint32_t found_component = 0, start = 0, end = 0, limit = 0, raw_thread = 0;
    unsigned components = 0;
    uint32_t vptr = 0;
    if (!word(fd, mux, &vptr) || vptr != MUX_VPTR ||
        !word(fd, mux + 0x1c, &start) || !word(fd, mux + 0x20, &end) ||
        !word(fd, mux + 0x24, &limit) || !word(fd, mux + 0x70, &raw_thread) ||
        start < 0x10000u || start > end || end > limit ||
        (end - start) % 4u != 0 || (end - start) / 4u > 64u ||
        (raw_thread & 0xffffu) == 0u || (raw_thread & 0xffffu) >= 0x7fffu)
        return 0;
    for (uint32_t at = start; at < end; at += 4u) {
        uint32_t entry = 0, ev = 0;
        if (!word(fd, at, &entry) || entry < 0x10000u + 0xacu ||
            !word(fd, entry, &ev) || ev != STUB_VPTR)
            continue;
        uint32_t cand = entry - 0xacu;
        if (component_valid(fd, cand)) {
            found_component = cand;
            ++components;
        }
    }
    if (components != 1u)
        return 0;
    *component = found_component;
    *thread = (uint16_t)raw_thread;
    return 1;
}

static int check_rg_milca_mux(int fd, uint32_t mux, uint32_t *out_rg, uint32_t *out_milca,
                              uint16_t *out_thread) {
    uint32_t vptr = 0, start = 0, end = 0, limit = 0, raw_tid = 0;
    uint32_t rg = 0, milca = 0;
    unsigned rg_count = 0, milca_count = 0;
    if (!word(fd, mux, &vptr) || vptr != MUX_VPTR ||
        !word(fd, mux + 0x1c, &start) || !word(fd, mux + 0x20, &end) ||
        !word(fd, mux + 0x24, &limit) || !word(fd, mux + 0x70, &raw_tid) ||
        start < 0x10000u || end < start || end > limit ||
        (end - start) % 4u != 0 || (end - start) / 4u > 64u ||
        (raw_tid & 0xffffu) == 0u || (raw_tid & 0xffffu) >= 0x7fffu)
        return 0;
    for (uint32_t at = start; at < end; at += 4u) {
        uint32_t ent = 0, ev = 0;
        if (!word(fd, at, &ent) || ent < 0x10000u || !word(fd, ent, &ev))
            continue;
        if (ev == RG_VPTR) { rg = ent; ++rg_count; }
        else if (ev == MILCA_VPTR) { milca = ent; ++milca_count; }
    }
    if (rg_count != 1u || milca_count != 1u)
        return 0;
    *out_rg = rg;
    *out_milca = milca;
    *out_thread = (uint16_t)raw_tid;
    return 1;
}

/* Dynamically locate both CHBMultiplexer heap objects by MUX_VPTR so live cars
 * with NDS maps (different heap addresses than the mapless emulator) work reliably. */
static int find_all_mux_targets(int fd,
                                uint32_t *out_component, uint16_t *out_owner_tid,
                                uint32_t *out_rg, uint32_t *out_milca, uint16_t *out_rg_tid) {
    int have_owner = (*out_component != 0u);
    int have_rg = (*out_rg != 0u && *out_milca != 0u);

    if (!have_owner && check_owner_mux(fd, MUX_OWNER_HINT, out_component, out_owner_tid))
        have_owner = 1;
    if (!have_rg && check_rg_milca_mux(fd, MUX9_HINT, out_rg, out_milca, out_rg_tid))
        have_rg = 1;
    if (have_owner && have_rg)
        return 1;

    uint32_t page[1024];
    uint32_t addr = HEAP_SCAN_BEGIN;
    while (addr < HEAP_SCAN_END && !(have_owner && have_rg)) {
        if (lseek(fd, (off_t)addr, SEEK_SET) != (off_t)addr ||
            read(fd, page, sizeof(page)) != (ssize_t)sizeof(page)) {
            addr = (addr + 65536u) & ~65535u;
            continue;
        }
        for (unsigned i = 0; i < 1024u && !(have_owner && have_rg); ++i) {
            if (page[i] != MUX_VPTR)
                continue;
            uint32_t cand = addr + i * 4u;
            if (!have_owner && check_owner_mux(fd, cand, out_component, out_owner_tid))
                have_owner = 1;
            else if (!have_rg && check_rg_milca_mux(fd, cand, out_rg, out_milca, out_rg_tid))
                have_rg = 1;
        }
        addr += 4096u;
    }
    return have_owner && have_rg;
}

static void destroy_ev(void *raw) { FN(0x1457f8, void, void *)(raw); }
static void delete_ev(void *raw) { destroy_ev(raw); free(raw); }
static void *cast_ev(void *raw, const void *type) { (void)raw; (void)type; return NULL; }
static void send_self(void *raw) {
    struct owner_event *event = raw;
    FN(0x145788, void, void *, unsigned, unsigned)(raw, event->thread, 0);
}

static int deliver_fixture_resolution(void *resolver) {
    uint8_t *resolved = calloc(1, 256);
    uint8_t *location = calloc(1, 256);
    if (!resolved || !location) {
        free(resolved);
        free(location);
        return 0;
    }
    FN(0x793c30, void, void *)(resolved);
    *(uint32_t *)(location + 0x40) = 0;
    *(uint32_t *)(location + 0x44) = 0;
    *(uint32_t *)(location + 0x48) = 4823;
    *(uint32_t *)(location + 0x4c) = 4823;
    FN(0x793250, void, void *, unsigned, unsigned, unsigned, void *, void *, void *)(
        location, 0, 4, 0, location + 0x40, location + 0x44, location + 0x48);
    *(uint32_t *)(resolved + 4) = (uint32_t)(uintptr_t)location;
    *(uint32_t *)(resolved + 8) = (uint32_t)(uintptr_t)(location + 32);
    *(uint32_t *)(resolved + 12) = (uint32_t)(uintptr_t)(location + 32);
    FN(0x56f970, void, void *, void *)(resolver, resolved);
    return 1;
}

/* BMW NavigationBasic keeps cleared tour vector slots as NULL pointers (0x0)
 * instead of shrinking stub+0x14c..0x150 after an OEM route ends. Count only
 * live (non-null) tour pointers and locate the latest live one. */
static unsigned scan_live_tours_direct(uint32_t stub, uint32_t *out_last_tour) {
    uint32_t start = *(uint32_t *)(uintptr_t)(stub + 0x14c);
    uint32_t end = *(uint32_t *)(uintptr_t)(stub + 0x150);
    if (out_last_tour)
        *out_last_tour = 0u;
    if (start == end)
        return 0u;
    if (start < 0x10000u || end < start || (end - start) % 4u != 0u ||
        (end - start) / 4u > 64u)
        return 0xffffu;
    unsigned count = 0u;
    for (uint32_t p = start; p < end; p += 4u) {
        uint32_t tour = *(uint32_t *)(uintptr_t)p;
        if (tour >= 0x10000u && tour < 0x80000000u) {
            if (out_last_tour)
                *out_last_tour = tour;
            ++count;
        }
    }
    return count;
}

static unsigned scan_live_tours_mem(int mem_fd, uint32_t stub, uint32_t *out_last_tour) {
    uint32_t start = 0, end = 0;
    if (out_last_tour)
        *out_last_tour = 0u;
    if (!word(mem_fd, stub + 0x14c, &start) || !word(mem_fd, stub + 0x150, &end))
        return 0u;
    if (start == end || start < 0x10000u || end < start ||
        (end - start) % 4u != 0u || (end - start) / 4u > 64u)
        return 0u;
    unsigned count = 0u;
    for (uint32_t p = start; p < end; p += 4u) {
        uint32_t tour = 0;
        if (word(mem_fd, p, &tour) && tour >= 0x10000u && tour < 0x80000000u) {
            if (out_last_tour)
                *out_last_tour = tour;
            ++count;
        }
    }
    return count;
}

static int route_exists_in_stub(int mem_fd, uint32_t stub, uint32_t route_id) {
    uint32_t rb = 0, re = 0;
    if (!route_id || !word(mem_fd, stub + 0x15c, &rb) || !word(mem_fd, stub + 0x160, &re))
        return 0;
    if (rb < 0x10000u || re < rb || (re - rb) % 4u != 0u || (re - rb) / 4u > 64u)
        return 0;
    for (uint32_t p = rb; p < re; p += 4u) {
        uint32_t robj = 0, id = 0;
        if (word(mem_fd, p, &robj) && robj >= 0x10000u && robj < 0x80000000u &&
            word(mem_fd, robj + 0x18u, &id) && id == route_id)
            return 1;
    }
    return 0;
}

static int try_fixture_resolve(void *resolver, uint32_t kind) {
    if ((uint32_t)(uintptr_t)resolver != wrapped_resolver_addr ||
        !__atomic_load_n(&resolver_armed_flag, __ATOMIC_ACQUIRE) ||
        __atomic_load_n(&resolver_intercept_kind, __ATOMIC_ACQUIRE) != 0u)
        return 0;
    uint32_t work_start = *(uint32_t *)((uint8_t *)resolver + 0x2c);
    uint32_t work_end = *(uint32_t *)((uint8_t *)resolver + 0x30);
    uint32_t token = *(uint32_t *)((uint8_t *)resolver + 0x20);
    if (scan_live_tours_direct(wrapped_stub_addr, NULL) != 0u ||
        work_end < work_start || work_end - work_start != 156u || !token)
        return 0;
    *(uint32_t *)resolver = RESOLVER_VPTR;
    if (!deliver_fixture_resolution(resolver))
        return 0;
    __atomic_store_n(&resolver_intercept_kind, kind, __ATOMIC_RELEASE);
    return 1;
}

static void wrapped_resolve_failed(void *resolver) {
    if (try_fixture_resolve(resolver, 2u))
        return;
    FN(0x56f5e0, void, void *)(resolver);
}

static void wrapped_resolve_ok(void *resolver, void *resolved) {
    const uint32_t *res = (const uint32_t *)resolved;
    int empty = (!res || !res[1] || res[2] < res[1] || ((res[2] - res[1]) >> 5) == 0);
    if (empty && try_fixture_resolve(resolver, 1u))
        return;
    FN(0x56f970, void, void *, void *)(resolver, resolved);
}

static int calculate_fixture_route(struct owner_event *event,
                                   uint32_t tour_high, uint32_t tour_low) {
    uint32_t source_handle = register_fixture_source();
    if (!source_handle) {
        basic_log("SEMANTIC_BASIC calc_fail stage=source_handle\n");
        return 0;
    }
    uint8_t *calculate = calloc(1, 4096);
    if (!calculate)
        return 0;
    *(uint32_t *)(calculate + 0x30) = 0x100;
    *(uint32_t *)(calculate + 0x34) = 17;
    *(uint32_t *)(calculate + 0x38) = 2;
    *(uint32_t *)(calculate + 0x50) = 1;
    *(uint32_t *)(calculate + 0x58) = 1;
    *(uint32_t *)(calculate + 0x5c) = tour_high;
    *(uint32_t *)(calculate + 0x60) = tour_low;
    uint32_t calc_before = *(uint32_t *)(uintptr_t)(event->stub + 0x44);
    FN(0x85c82c, void, void *, void *)((void *)(uintptr_t)event->stub, calculate);
    uint32_t calc_after = *(uint32_t *)(uintptr_t)(event->stub + 0x44);
    uint32_t pending_begin = *(uint32_t *)(uintptr_t)(event->component + 0x2b0 + 0x74);
    uint32_t pending_end = *(uint32_t *)(uintptr_t)(event->component + 0x2b0 + 0x78);
    if (calc_after != calc_before + 1u ||
        pending_end < pending_begin + 0x2e8u ||
        (pending_end - pending_begin) % 0x2e8u != 0u) {
        basic_log("SEMANTIC_BASIC calc_fail stage=pending before=%u after=%u pb=%#x pe=%#x th=%#x tl=%#x\n",
                  calc_before, calc_after, pending_begin, pending_end, tour_high, tour_low);
        return 0;
    }
    uint32_t pending_item = pending_end - 0x2e8u;
    if (*(uint32_t *)(uintptr_t)(pending_item + 4) != calc_after ||
        *(uint32_t *)(uintptr_t)(pending_item + 0x10) != tour_high ||
        *(uint32_t *)(uintptr_t)(pending_item + 0x14) != tour_low ||
        *(uint32_t *)(uintptr_t)(pending_item + 0x2e4) != event->stub + 0x120u) {
        basic_log("SEMANTIC_BASIC calc_fail stage=pending_fields id=%u th=%#x tl=%#x cb=%#x\n",
                  *(uint32_t *)(uintptr_t)(pending_item + 4),
                  *(uint32_t *)(uintptr_t)(pending_item + 0x10),
                  *(uint32_t *)(uintptr_t)(pending_item + 0x14),
                  *(uint32_t *)(uintptr_t)(pending_item + 0x2e4));
        return 0;
    }

    uint8_t *route_result = calloc(1, 256);
    if (!route_result)
        return 0;
    FN(0x56c728, void, void *)(route_result);
    FN(0x7116f0, void, void *, uint32_t, uint32_t, uint32_t,
       uint32_t, uint32_t, uint32_t, uint32_t)(route_result + 8, 3, 1, 0, 0, 0, 2, 1);
    uint32_t container = *(uint32_t *)(route_result + 0x18);
    if (container < 0x10000u) {
        basic_log("SEMANTIC_BASIC calc_fail stage=container\n");
        return 0;
    }
    *(uint32_t *)(uintptr_t)(container + 0x50) = source_handle;
    uint32_t point[2] = {0, 0};
    FN(0x711428, void, void *, const uint32_t *, uint64_t,
       uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t)(
        (void *)(uintptr_t)container, point, 2, 0, 0, 0, 0, 0, 0);
    point[0] = 4823;
    FN(0x711428, void, void *, const uint32_t *, uint64_t,
       uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t)(
        (void *)(uintptr_t)container, point, 4, 0, 0, 0, 0, 0, 0);
    point[1] = 4823;
    FN(0x711318, void, void *, const uint32_t *, uint32_t)(
        (void *)(uintptr_t)container, point, 0);
    if (*(uint32_t *)(uintptr_t)container != 6u) {
        basic_log("SEMANTIC_BASIC calc_fail stage=container_links count=%u\n",
                  *(uint32_t *)(uintptr_t)container);
        return 0;
    }

    uint32_t route_data = event->component + 0x464u;
    FN(0x577fe0, void, void *, void *, uint32_t, void *, void *, void *)(
        (void *)(uintptr_t)(event->stub + 0x120u),
        (void *)(uintptr_t)(pending_item + 0xcu), calc_after,
        (void *)(uintptr_t)(pending_item + 0x1e0u),
        (void *)(uintptr_t)(pending_item + 0x288u), route_result);
    /* Disarm stale Hermes1WorkerHighPrio callback so CHBStubBase::restoreContext
     * (CHBStubBase.cpp:173) is never double-called when the background task expires. */
    *(uint32_t *)(uintptr_t)(pending_item + 0x2e4u) = 0;

    uint32_t work_begin = *(uint32_t *)(uintptr_t)(route_data + 0x2c);
    uint32_t work_end = *(uint32_t *)(uintptr_t)(route_data + 0x30);
    uint32_t route_peer = *(uint32_t *)(uintptr_t)(route_data + 0x1c);
    uint32_t peer_request = route_peer >= 0x10000u ? *(uint32_t *)(uintptr_t)(route_peer + 0x10) : 0;
    uint32_t work_item = (work_end >= work_begin + 0xf0u && (work_end - work_begin) % 0xf0u == 0u)
                             ? (work_end - 0xf0u) : 0u;
    uint32_t work_token = work_item ? *(uint32_t *)(uintptr_t)(work_item + 4) : 0;
    uint32_t cb_owner = work_item ? *(uint32_t *)(uintptr_t)(work_item + 0xec) : 0;
    if (!work_token || work_token != peer_request || cb_owner != event->stub + 0x124u) {
        basic_log("SEMANTIC_BASIC calc_fail stage=route_data wb=%#x we=%#x tok=%u peer=%u cb=%#x\n",
                  work_begin, work_end, work_token, peer_request, cb_owner);
        return 0;
    }

    uint32_t *source_result = calloc(16, sizeof(*source_result));
    if (!source_result)
        return 0;
    source_result[0] = source_handle;
    *(uint32_t *)(uintptr_t)(route_data + 0x20) = work_token;
    FN(0x570b1c, void, void *, void *)((void *)(uintptr_t)route_data, source_result);

    uint32_t rb = *(uint32_t *)(uintptr_t)(event->stub + 0x15c);
    uint32_t re = *(uint32_t *)(uintptr_t)(event->stub + 0x160);
    uint32_t live_route_id = 0;
    if (rb >= 0x10000u && re >= rb + 4u && (re - rb) % 4u == 0u) {
        for (uint32_t p = re; p > rb; p -= 4u) {
            uint32_t route_obj = *(uint32_t *)(uintptr_t)(p - 4u);
            if (route_obj >= 0x10000u && route_obj < 0x80000000u) {
                live_route_id = *(uint32_t *)(uintptr_t)(route_obj + 0x18);
                if (live_route_id)
                    break;
            }
        }
    }
    if (!live_route_id) {
        basic_log("SEMANTIC_BASIC calc_fail stage=no_route_id rb=%#x re=%#x\n", rb, re);
        return 0;
    }
    __atomic_store_n(&created_route_id, live_route_id, __ATOMIC_RELEASE);
    return 1;
}

static uint32_t find_milca_list_for_route(uint32_t milca, uint32_t route_id) {
    uint32_t begin = *(const uint32_t *)(uintptr_t)(milca + 0x370);
    uint32_t end = *(const uint32_t *)(uintptr_t)(milca + 0x374);
    if (begin < 0x10000u || end < begin || (end - begin) % 24u != 0)
        return 0;
    uint32_t rows = (end - begin) / 24u;
    for (uint32_t i = 0; i < rows; ++i) {
        uint32_t row = begin + i * 24u;
        if (*(const uint32_t *)(uintptr_t)row == route_id) {
            uint32_t holder = *(const uint32_t *)(uintptr_t)(row + 0x14);
            if (holder >= 0x10000u && holder < 0x80000000u)
                return *(const uint32_t *)(uintptr_t)(holder + 4);
        }
    }
    return 0;
}

/* Lock-free seqlock carrying live car GPS captured from CRouteGuidance::onCarPositionUpdate. */
static volatile uint32_t g_gps_seq = 0u;
static struct semantic_car_gps g_live_gps;

static void store_live_gps(const struct semantic_car_gps *src) {
    uint32_t s = __atomic_load_n(&g_gps_seq, __ATOMIC_RELAXED);
    __atomic_store_n(&g_gps_seq, s + 1u, __ATOMIC_RELEASE);
    g_live_gps = *src;
    __atomic_store_n(&g_gps_seq, s + 2u, __ATOMIC_RELEASE);
}

static int load_live_gps(struct semantic_car_gps *out) {
    for (unsigned tries = 0u; tries < 4u; ++tries) {
        uint32_t s1 = __atomic_load_n(&g_gps_seq, __ATOMIC_ACQUIRE);
        if (s1 & 1u)
            continue;
        *out = g_live_gps;
        uint32_t s2 = __atomic_load_n(&g_gps_seq, __ATOMIC_ACQUIRE);
        if (s1 == s2)
            return out->valid;
    }
    memset(out, 0, sizeof(*out));
    return 0;
}

/* While our synthetic 2-link fixture route is actively guiding, live car GPS
 * ticks in CRouteGuidance::onCarPositionUpdate (rg+0x24c slot 3) spawn
 * CCarPositionUpdateJob, which computes distance=0 against the fixture links
 * and pushes dist=0 via onDistanceToManeuverCalculatedEvent (rg+0x308 slot 2).
 * Always capture live WGS84 car GPS for OEM NDS map lookup before suppressing. */
static void wrapped_rg_on_car_pos(void *subobj, const void *pos) {
    if (pos) {
        struct semantic_car_gps snap;
        memset(&snap, 0, sizeof(snap));
        semantic_nds_capture_car_pos(pos, mono_ms(), &snap);
        if (snap.valid)
            store_live_gps(&snap);
    }
    if (__atomic_load_n(&g_rg_gps_suppressed, __ATOMIC_ACQUIRE))
        return;
    FN(RG_CARPOS_ORIG, void, void *, const void *)(subobj, pos);
}

static void wrapped_rg_on_dtm_calc(void *subobj, const void *ev) {
    if (__atomic_load_n(&g_rg_gps_suppressed, __ATOMIC_ACQUIRE)) {
        uint32_t our_route = __atomic_load_n(&active_route_id, __ATOMIC_ACQUIRE);
        uint32_t ev_route = ev ? *(const uint32_t *)((const uint8_t *)ev + 0x1c) : 0u;
        if (!our_route || !ev_route || ev_route == our_route)
            return;
    }
    FN(RG_DTMCALC_ORIG, void, void *, const void *)(subobj, ev);
}

static void install_rg_gps_wrappers(uint32_t rg) {
    uint32_t carpos_addr = rg + RG_CARPOS_OFF;
    uint32_t carpos_vptr = *(uint32_t *)(uintptr_t)carpos_addr;
    int carpos_ok = (carpos_vptr == (uint32_t)(uintptr_t)&rg_carpos_vtable_copy[2]);
    if (!carpos_ok && carpos_vptr == RG_CARPOS_VPTR) {
        memcpy(rg_carpos_vtable_copy, (const void *)(uintptr_t)(RG_CARPOS_VPTR - 8u),
               sizeof(rg_carpos_vtable_copy));
        if (rg_carpos_vtable_copy[2 + 3] == RG_CARPOS_ORIG) {
            rg_carpos_vtable_copy[2 + 3] = (uint32_t)(uintptr_t)&wrapped_rg_on_car_pos;
            *(uint32_t *)(uintptr_t)carpos_addr = (uint32_t)(uintptr_t)&rg_carpos_vtable_copy[2];
            carpos_ok = 1;
        }
    }

    uint32_t dtmcalc_addr = rg + RG_DTMCALC_OFF;
    uint32_t dtmcalc_vptr = *(uint32_t *)(uintptr_t)dtmcalc_addr;
    int dtmcalc_ok = (dtmcalc_vptr == (uint32_t)(uintptr_t)&rg_dtmcalc_vtable_copy[2]);
    if (!dtmcalc_ok && dtmcalc_vptr == RG_DTMCALC_VPTR) {
        memcpy(rg_dtmcalc_vtable_copy, (const void *)(uintptr_t)(RG_DTMCALC_VPTR - 8u),
               sizeof(rg_dtmcalc_vtable_copy));
        if (rg_dtmcalc_vtable_copy[2 + 2] == RG_DTMCALC_ORIG) {
            rg_dtmcalc_vtable_copy[2 + 2] = (uint32_t)(uintptr_t)&wrapped_rg_on_dtm_calc;
            *(uint32_t *)(uintptr_t)dtmcalc_addr = (uint32_t)(uintptr_t)&rg_dtmcalc_vtable_copy[2];
            dtmcalc_ok = 1;
        }
    }
    basic_log("SEMANTIC_BASIC rg_gps_wrap carpos=%d dtmcalc=%d\n", carpos_ok, dtmcalc_ok);
}

static void publish_dtm(uint32_t rg, uint32_t route_id, uint32_t man_id,
                        uint32_t dist_dm, uint32_t norm_m,
                        uint32_t bargraph_flag, uint32_t bargraph_max_dm,
                        const void *locale) {
    _Alignas(8) uint8_t dtm[0x2c];
    memset(dtm, 0, sizeof(dtm));
    *(uint32_t *)(dtm + 0x00) = route_id;
    *(uint32_t *)(dtm + 0x04) = man_id;
    *(uint32_t *)(dtm + 0x08) = dist_dm;
    *(uint32_t *)(dtm + 0x0c) = bargraph_flag;
    *(uint32_t *)(dtm + 0x10) = bargraph_max_dm;
    FN(0x16c5f4, void, void *)(dtm + 0x14);
    FN(0x16ce4c, void, void *, const void *)(dtm + 0x14, locale);
    *(uint32_t *)(dtm + 0x24) = norm_m;
    *(uint32_t *)(dtm + 0x28) = 0u; /* UNIT_METERS */
    FN(0x86d0b8, void, void *, const void *)((void *)(uintptr_t)rg, dtm);
    FN(0x16cf50, void, void *)(dtm + 0x14);
}

static void reset_milca_active_ids(uint32_t list) {
    uint32_t id_beg = *(const uint32_t *)(uintptr_t)(list + 0x64);
    *(uint32_t *)(uintptr_t)(list + 0x68) = id_beg;
}

static void append_lane_boundary(uint8_t *point, uint32_t marking) {
    _Alignas(8) uint8_t lane[28];
    memset(lane, 0, sizeof(lane));
    FN(0x1d6580, void, void *)(lane);
    *(uint32_t *)(lane + 16) = marking;
    FN(0xcc59e0, void, void *, const void *)(point + 0x14c, lane);
    FN(0x1d57f8, void, void *)(lane);
}

static void append_point_element(uint8_t *point, uint32_t elem_type, float angle_deg) {
    _Alignas(8) uint8_t elem[24];
    FN(0x792cb8, void, void *)(elem);
    *(uint32_t *)(elem + 0x00) = elem_type;
    memcpy(elem + 0x14, &angle_deg, sizeof(angle_deg));
    uint32_t usage = 0;
    FN(0x8e6a6c, void, void *, const uint32_t *)(elem + 0x04, &usage);
    FN(0xcded58, void, void *, const void *)(point + 0x84, elem);
    FN(0x7928b0, void, void *)(elem);
}

static int insert_milca_maneuver(uint32_t list, uint32_t id, uint32_t sort_pos_dm,
                                 const struct semantic_basic_cue *cue) {
    uint8_t *rec = FN(0x11ad54, uint8_t *, unsigned)(0x188);
    if (!rec)
        return 0;
    FN(0x3edad4, void, void *)(rec);
    *(uint32_t *)(rec + 0x00) = id;
    *(uint32_t *)(rec + 0x04) = cue->action;
    *(uint32_t *)(rec + 0x08) = cue->direction;
    *(uint32_t *)(rec + 0xa4) = sort_pos_dm;

    if (cue->road[0]) {
        uint32_t entry[2] = {0, 0};
        FN(0x1185f8, void *, void *, const char *, unsigned)(
            &entry[0], cue->road, (unsigned)strlen(cue->road));
        FN(0x4a9730, void, void *, const void *)(rec + 0xe4, entry);
        FN(0x117710, void, void *)(&entry[0]);

        uint32_t country = 0;
        FN(0x1185f8, void *, void *, const char *, unsigned)(&country, "USA", 3u);
        FN(0x10dd20, void *, void *, const void *)(rec + 0x114, &country);
        FN(0x117710, void, void *)(&country);
    }

    _Alignas(8) uint8_t point[0x1a8];
    FN(0x3ed28c, void, void *)(point);
    *(uint32_t *)(point + 0x80) = 1u;
    *(uint32_t *)(point + 0x144) = 4u; /* NOW */
    *(uint32_t *)(point + 0x18c) = cue->dist_dm;

    append_point_element(point, cue->elem_type, cue->angle_deg);
    for (uint8_t k = 0u; k < cue->extra_arm_count; ++k)
        append_point_element(point, 32u, cue->extra_arm_deg[k]);

    if (cue->lane_count > 0u) {
        append_lane_boundary(point, semantic_lane_boundary_marking(0u, cue->lane_count));
        for (uint32_t i = cue->lane_count; i > 0u; --i) {
            const struct ng_lane data = cue->lanes[i - 1u];
            _Alignas(8) uint8_t lane[28];
            memset(lane, 0, sizeof(lane));
            FN(0x1d6580, void, void *)(lane);
            for (uint16_t bit = 1u; bit <= NG_LANE_UTURN_RIGHT; bit <<= 1) {
                if (!(data.arrows & bit))
                    continue;
                float deg = semantic_lane_degrees(bit);
                uint32_t status = (data.recommended & bit) ? 0u : 2u;
                _Alignas(4) uint8_t arrow[8];
                memcpy(arrow, &deg, 4);
                memcpy(arrow + 4, &status, 4);
                FN(0xcc5d00, void, void *, const void *)(lane, arrow);
            }
            *(uint32_t *)(lane + 16) = 1u; /* ARROWS */
            *(uint32_t *)(lane + 20) = 0u;
            FN(0xcc59e0, void, void *, const void *)(point + 0x14c, lane);
            FN(0x1d57f8, void, void *)(lane);
            uint32_t bidx = (uint32_t)cue->lane_count - i + 1u;
            append_lane_boundary(point, semantic_lane_boundary_marking(bidx, cue->lane_count));
        }
        *(uint32_t *)(point + 0x148) = 1u;
    }

    FN(0x3ed82c, void, void *, unsigned, const void *)(rec + 0x134, 1, point);
    FN(0x3ec444, void, void *)(point);

    uint32_t holder = 0;
    FN(0x8baba4, void *, void *, void *)(&holder, rec);
    if (!holder) {
        FN(0x3ec508, void, void *)(rec);
        FN(0x11ad44, void, void *)(rec);
        return 0;
    }
    FN(0x3ff604, void, void *, void *)((void *)(uintptr_t)list, &holder);
    FN(0x3f2ca4, void, void *)(&holder);
    return 1;
}

static void execute_hud_phase(const struct owner_event *event, unsigned context) {
    uint32_t rg = event->component;
    uint32_t milca = event->stub;
    uint32_t route_id = event->arg0;
    if (context != event->thread ||
        *(uint32_t *)(uintptr_t)rg != RG_VPTR ||
        *(uint32_t *)(uintptr_t)milca != MILCA_VPTR) {
        __atomic_store_n(&hud_dispatch_state, 2u, __ATOMIC_RELEASE);
        return;
    }
    if (event->phase == PHASE_ACTIVATE_GUIDANCE) {
        install_rg_gps_wrappers(rg);
        __atomic_store_n(&g_rg_gps_suppressed, 1u, __ATOMIC_RELEASE);
        uint32_t gs = 2u; /* GUIDANCE_NORMAL_ACTIVE */
        _Alignas(8) uint8_t ts[0xd0];
        FN(0x402a6c, void, void *, const uint32_t *)((void *)(uintptr_t)rg, &gs);
        FN(0x1d2b14, void *, void *, const void *)(ts, (const void *)(uintptr_t)(rg + 0x838));
        *(uint32_t *)(ts + 0x10) = 1u; /* ROLE_GUIDED_ROUTE */
        *(uint32_t *)(ts + 0xb8) = 0u;
        *(uint32_t *)(ts + 0xbc) = UINT32_MAX;
        *(uint32_t *)(ts + 0xc0) = 1u; /* GUIDANCE_ROUTE_STATUS_GUIDED */
        FN(0x4029cc, void, void *, const void *)((void *)(uintptr_t)rg, ts);
        FN(0x1d2b00, void, void *)(ts);
        __atomic_store_n(&hud_dispatch_state, 1u, __ATOMIC_RELEASE);
        return;
    }
    if (event->phase == PHASE_PUBLISH_CUE) {
        uint32_t list = find_milca_list_for_route(milca, route_id);
        if (list < 0x10000u || list >= 0x80000000u) {
            __atomic_store_n(&hud_dispatch_state, 2u, __ATOMIC_RELEASE);
            return;
        }
        const struct semantic_basic_cue *cue = &g_pending_cue;
        if (g_pending_dtm_only && g_active_man_id != 0u) {
            publish_dtm(rg, route_id, g_active_man_id, cue->dist_dm, cue->norm_m,
                        cue->bargraph_flag, cue->bargraph_max_dm,
                        (const void *)(uintptr_t)(list + 0x50));
            __atomic_store_n(&hud_dispatch_state, 1u, __ATOMIC_RELEASE);
            return;
        }
        reset_milca_active_ids(list);
        uint32_t id = g_next_man_id;
        g_next_man_id += 2u;
        g_active_man_id = id;
        uint32_t sort_pos_dm = (g_sort_counter++) * 10000u + cue->dist_dm;
        if (!insert_milca_maneuver(list, id, sort_pos_dm, cue)) {
            __atomic_store_n(&hud_dispatch_state, 2u, __ATOMIC_RELEASE);
            return;
        }
        if (!cue->is_arrival) {
            struct semantic_basic_cue trailing;
            memset(&trailing, 0, sizeof(trailing));
            trailing.action = 18u;
            trailing.direction = 3u;
            trailing.elem_type = 1u;
            trailing.dist_dm = cue->dist_dm + 2000u;
            trailing.norm_m = (cue->dist_dm + 2000u) / 10u;
            trailing.is_arrival = 1u;
            if (!insert_milca_maneuver(list, id + 1u, sort_pos_dm + 2000u, &trailing)) {
                __atomic_store_n(&hud_dispatch_state, 2u, __ATOMIC_RELEASE);
                return;
            }
        }
        FN(0x3f652c, void, void *, void *, void *, void *)(
            (void *)(uintptr_t)milca,
            (void *)(uintptr_t)(list + 0x70),
            (void *)(uintptr_t)(list + 0x50),
            (void *)(uintptr_t)(list + 0x60));
        FN(0x3f1fe8, void, void *, void *)((void *)(uintptr_t)milca,
                                           (void *)(uintptr_t)list);

        /* Pre-seed NavigationFully's mLastDTM cache (GVP+0x234) with the new
         * maneuver ID BEFORE emitting DRP so responseGetManeuverListStartingAtManeuver
         * populates maneuverData->dtm immediately instead of rendering -1 -> "0". */
        publish_dtm(rg, route_id, id, cue->dist_dm, cue->norm_m,
                    cue->bargraph_flag, cue->bargraph_max_dm,
                    (const void *)(uintptr_t)(list + 0x50));

        _Alignas(8) uint8_t drp[0x1c];
        memset(drp, 0, sizeof(drp));
        *(uint32_t *)(drp + 0x00) = route_id;
        *(uint32_t *)(drp + 0x04) = id;
        FN(0x16c5f4, void, void *)(drp + 0x0c);
        FN(0x16ce4c, void, void *, const void *)(drp + 0x0c,
                                                 (const void *)(uintptr_t)(list + 0x50));
        FN(0x1d28f0, void, void *, const void *)((void *)(uintptr_t)(rg + 0x54), drp);
        FN(0x86cf6c, void, void *, uint32_t)((void *)(uintptr_t)rg, 0xc0000000u);
        FN(0x16cf50, void, void *)(drp + 0x0c);

        publish_dtm(rg, route_id, id, cue->dist_dm, cue->norm_m,
                    cue->bargraph_flag, cue->bargraph_max_dm,
                    (const void *)(uintptr_t)(list + 0x50));
        __atomic_store_n(&hud_dispatch_state, 1u, __ATOMIC_RELEASE);
        return;
    }
    if (event->phase == PHASE_CLEAR_GUIDANCE) {
        __atomic_store_n(&g_rg_gps_suppressed, 0u, __ATOMIC_RELEASE);
        uint32_t gs = 5u; /* GUIDANCE_STOPPED_BY_USER_REQUEST */
        _Alignas(8) uint8_t ts[0xd0];
        FN(0x402a6c, void, void *, const uint32_t *)((void *)(uintptr_t)rg, &gs);
        FN(0x1d2b14, void *, void *, const void *)(ts, (const void *)(uintptr_t)(rg + 0x838));
        *(uint32_t *)(ts + 0x00) = 0;
        *(uint32_t *)(ts + 0x04) = 0;
        *(uint32_t *)(ts + 0x08) = 0;
        *(uint32_t *)(ts + 0x10) = 0;
        *(uint32_t *)(ts + 0xb8) = UINT32_MAX;
        *(uint32_t *)(ts + 0xbc) = UINT32_MAX;
        *(uint32_t *)(ts + 0xc0) = 0;
        FN(0x4029cc, void, void *, const void *)((void *)(uintptr_t)rg, ts);
        FN(0x1d2b00, void, void *)(ts);
        g_active_man_id = 0u;
        __atomic_store_n(&hud_dispatch_state, 1u, __ATOMIC_RELEASE);
        return;
    }
}

static void execute_event(void *raw, void *dispatcher) {
    struct owner_event *event = raw;
    (void)dispatcher;
    const unsigned context = FN(0x148d78, unsigned, void)();
    if (event->phase >= PHASE_ACTIVATE_GUIDANCE) {
        execute_hud_phase(event, context);
        return;
    }
    if (context != event->thread ||
        *(uint32_t *)(uintptr_t)event->component != COMPONENT_VPTR ||
        *(uint32_t *)(uintptr_t)event->stub != STUB_VPTR) {
        if (event->phase == PHASE_INSTALL_RESOLVER)
            __atomic_store_n(&resolver_wrapper_state, 2u, __ATOMIC_RELEASE);
        else
            __atomic_store_n(&owner_dispatch_state, 2u, __ATOMIC_RELEASE);
        return;
    }
    if (event->phase == PHASE_INSTALL_RESOLVER) {
        uint32_t resolver = event->component + 0x428u;
        uint32_t cur_vptr = *(uint32_t *)(uintptr_t)resolver;
        if (cur_vptr == (uint32_t)(uintptr_t)resolver_vtable_copy &&
            wrapped_resolver_addr == resolver && wrapped_stub_addr == event->stub) {
            __atomic_store_n(&resolver_wrapper_state, 1u, __ATOMIC_RELEASE);
            return;
        }
        if (cur_vptr != RESOLVER_VPTR) {
            __atomic_store_n(&resolver_wrapper_state, 2u, __ATOMIC_RELEASE);
            return;
        }
        memcpy(resolver_vtable_copy, (const void *)(uintptr_t)RESOLVER_VPTR,
               sizeof(resolver_vtable_copy));
        if (resolver_vtable_copy[3] != 0x56f5e0u || resolver_vtable_copy[4] != 0x56f970u) {
            __atomic_store_n(&resolver_wrapper_state, 2u, __ATOMIC_RELEASE);
            return;
        }
        resolver_vtable_copy[3] = (uint32_t)(uintptr_t)&wrapped_resolve_failed;
        resolver_vtable_copy[4] = (uint32_t)(uintptr_t)&wrapped_resolve_ok;
        wrapped_resolver_addr = resolver;
        wrapped_stub_addr = event->stub;
        *(uint32_t *)(uintptr_t)resolver = (uint32_t)(uintptr_t)resolver_vtable_copy;
        __atomic_store_n(&resolver_wrapper_state, 1u, __ATOMIC_RELEASE);
        return;
    }
    if (event->phase == PHASE_CALC_ROUTE) {
        uint32_t tour = 0u;
        if (scan_live_tours_direct(event->stub, &tour) == 0u || tour < 0x10000u) {
            __atomic_store_n(&owner_dispatch_state, 2u, __ATOMIC_RELEASE);
            return;
        }
        /* Exact X20511I CTour::getId() (0x574e9c / 0x574ea4): tour+0x18 / tour+0x1c. */
        uint32_t th = *(uint32_t *)(uintptr_t)(tour + 0x18u);
        uint32_t tl = *(uint32_t *)(uintptr_t)(tour + 0x1cu);
        tour_high_cached = th;
        tour_low_cached = tl;
        int ok = calculate_fixture_route(event, th, tl);
        __atomic_store_n(&owner_dispatch_state, ok ? 1u : 2u, __ATOMIC_RELEASE);
    }
}

static void *event_vtable[5] = {
    destroy_ev, delete_ev, cast_ev, send_self, execute_event
};

static uint32_t dispatch_owner(uint32_t comp, uint32_t stub, uint16_t tid,
                               uint16_t phase, uint32_t arg0, volatile uint32_t *state_var) {
    __atomic_store_n(state_var, 0u, __ATOMIC_RELEASE);
    struct owner_event *ev = calloc(1, sizeof(*ev));
    if (!ev) return 2u;
    FN(0x1458ac, void, void *)(ev);
    ev->vtable = event_vtable;
    ev->component = comp;
    ev->stub = stub;
    ev->thread = tid;
    ev->phase = phase;
    ev->arg0 = arg0;
    send_self(ev);
    for (unsigned i = 0; i < 150u; ++i) {
        uint32_t st = __atomic_load_n(state_var, __ATOMIC_ACQUIRE);
        if (st) return st;
        struct timespec w = {0, 20000000};
        nanosleep(&w, NULL);
    }
    return 0u;
}

static int open_dgram_socket(const char *path) {
    unlink(path);
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    fcntl(fd, F_SETFL, O_NONBLOCK);
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    struct qnx_sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    size_t len = strlen(path);
    if (len >= sizeof(addr.sun_path)) { close(fd); return -1; }
    addr.sun_family = AF_UNIX;
    addr.sun_len = (uint8_t)(offsetof(struct qnx_sockaddr_un, sun_path) + len + 1u);
    memcpy(addr.sun_path, path, len + 1u);
    if (bind(fd, (struct sockaddr *)&addr, addr.sun_len) != 0) {
        close(fd);
        return -1;
    }
    (void)qnx_chmod(path, 0600);
    return fd;
}

static void write_ng_reply(int fd, const struct sockaddr *peer, socklen_t peer_len,
                           uint32_t seq, uint32_t result) {
    uint8_t rep[16];
    memcpy(rep, "NGA1", 4);
    for (unsigned i = 0; i < 4u; ++i) rep[4u + i] = (uint8_t)(seq >> (8u * i));
    for (unsigned i = 0; i < 4u; ++i) rep[8u + i] = (uint8_t)(result >> (8u * i));
    for (unsigned i = 0; i < 4u; ++i) rep[12u + i] = (uint8_t)(NG_CAP_MODEL_ID >> (8u * i));
    (void)sendto(fd, rep, sizeof(rep), 0, peer, peer_len);
}

struct basic_runtime {
    int mem_fd;
    uint32_t component;
    uint32_t stub;
    uint16_t owner_tid;
    uint32_t rg;
    uint32_t milca;
    uint16_t rg_tid;

    int guidance_active;
    int has_pending;
    int has_published;
    struct ng_message pending_msg;
    struct semantic_basic_cue pending_cue;
    struct semantic_basic_cue published_cue;
    uint64_t session;
    uint64_t last_alive_ms;
    uint64_t next_calc_ms;
    uint64_t last_nds_query_ms;
    uint32_t calc_attempts;
    uint32_t last_seq;
    uint32_t minimap_seq;
    int has_active_minimap;
};

static int cue_eligible_for_minimap(const struct ng_message *msg,
                                    const struct semantic_basic_cue *cue) {
    if (!msg || !cue || cue->is_arrival)
        return 0;
    float dist_m = (float)msg->distance_mm * 0.001f;
    if (dist_m > 450.0f)
        return 0;
    /* Roundabouts (AA 32..38), turns/forks/ramps (anything non-straight/non-depart). */
    uint16_t aa = (msg->metadata_flags & NG_META_AA_TYPE) ? msg->aa_type : 0u;
    if (aa == 1u || aa == 2u || aa == 39u || aa == 40u || aa == 41u)
        return 0; /* DEPART, NAME_CHANGE, DESTINATION_* */
    if (aa == 0u && msg->turn == NG_STRAIGHT)
        return 0;
    return 1;
}

/* Returns 1 if a fresh valid minimap was just written to SHM, 0 otherwise. */
static int refresh_nds_minimap(struct basic_runtime *rt, int force) {
    uint64_t now = mono_ms();
    if (!force && now < rt->last_nds_query_ms + 600u)
        return 0;

    /* Test-only sentinel: lets a parked synthetic test drive /dev/shmem/aa-semantic-minimap
     * directly without NavigationBasic overwriting or clearing it on cue publish. */
    if (access("/dev/shmem/aa-semantic-minimap-lock", F_OK) == 0) {
        if (force)
            basic_log("SEMANTIC_BASIC nds_minimap locked (synthetic test override)\n");
        return 0;
    }

    if (!cue_eligible_for_minimap(&rt->pending_msg, &rt->pending_cue)) {
        if (rt->has_active_minimap || force) {
            semantic_minimap_shm_clear(NULL);
            rt->has_active_minimap = 0;
        }
        return 0;
    }

    struct semantic_car_gps gps;
    if (!load_live_gps(&gps) || now > gps.timestamp_ms + 5000u) {
        if (rt->has_active_minimap || force) {
            semantic_minimap_shm_clear(NULL);
            rt->has_active_minimap = 0;
        }
        if (force) {
            basic_log("SEMANTIC_BASIC nds_minimap skip=no_gps valid=%d age_ms=%llu\n",
                      gps.valid,
                      gps.valid ? (unsigned long long)(now - gps.timestamp_ms) : 0ULL);
        }
        return 0;
    }

    rt->last_nds_query_ms = now;
    float dist_m = (float)rt->pending_msg.distance_mm * 0.001f;
    float radius_m = dist_m + 120.0f;

    struct snds_graph *graph = calloc(1, sizeof(*graph));
    if (!graph) {
        semantic_minimap_shm_clear(NULL);
        rt->has_active_minimap = 0;
        return 0;
    }

    int links = semantic_nds_query_local_graph(rt->mem_fd, &gps, radius_m, graph);
    if (links <= 0) {
        free(graph);
        if (rt->has_active_minimap || force) {
            semantic_minimap_shm_clear(NULL);
            rt->has_active_minimap = 0;
        }
        if (force) {
            basic_log("SEMANTIC_BASIC nds_minimap skip=no_links lat=%.5f lon=%.5f hdg=%.1f r=%.0f\n",
                      gps.lat_deg, gps.lon_deg, (double)gps.heading_deg, (double)radius_m);
        }
        return 0;
    }

    struct snds_query q;
    memset(&q, 0, sizeof(q));
    q.car_x = 0.0f;
    q.car_y = 0.0f;
    q.car_heading_deg = gps.heading_deg;
    q.aa_type = (rt->pending_msg.metadata_flags & NG_META_AA_TYPE) ? rt->pending_msg.aa_type : 0u;
    q.dist_m = dist_m;
    if (rt->pending_msg.metadata_flags & NG_META_EXIT_NUMBER)
        q.exit_number = rt->pending_msg.exit_number;
    if (rt->pending_msg.metadata_flags & NG_META_EXIT_ANGLE)
        q.exit_angle = rt->pending_msg.exit_angle;
    if (rt->pending_cue.road[0]) {
        strncpy(q.road_name, rt->pending_cue.road, sizeof(q.road_name) - 1u);
    }

    struct snds_junction junc;
    memset(&junc, 0, sizeof(junc));
    int matched = snds_match_maneuver(graph, &q, &junc);
    free(graph);

    struct semantic_hud_minimap mm;
    memset(&mm, 0, sizeof(mm));
    if (matched && semantic_render_hud_minimap(&junc, g_next_man_id, 65u, &mm)) {
        rt->minimap_seq++;
        (void)semantic_minimap_shm_write(NULL, rt->session, rt->minimap_seq, &mm);
        rt->has_active_minimap = 1;
        basic_log("SEMANTIC_BASIC nds_minimap ok links=%d rb=%u conf=%u polys=%u dist_m=%.0f\n",
                  links, (unsigned)mm.is_roundabout, (unsigned)mm.confidence,
                  (unsigned)mm.poly_count, (double)dist_m);
        return 1;
    }

    if (rt->has_active_minimap || force) {
        semantic_minimap_shm_clear(NULL);
        rt->has_active_minimap = 0;
    }
    if (force) {
        basic_log("SEMANTIC_BASIC nds_minimap fallback links=%d matched=%d conf=%u dist_m=%.0f\n",
                  links, matched, (unsigned)junc.confidence, (double)dist_m);
    }
    return 0;
}

static int try_publish_pending(struct basic_runtime *rt) {
    if (!rt->has_pending || !rt->guidance_active)
        return 1;
    uint32_t route_id = __atomic_load_n(&active_route_id, __ATOMIC_ACQUIRE);
    if (!route_id)
        return 0;
    if (find_milca_list_for_route(rt->milca, route_id) == 0u)
        return 0;
    g_pending_cue = rt->pending_cue;
    g_pending_dtm_only = (rt->has_published &&
                          semantic_cue_is_distance_only(&rt->published_cue, &rt->pending_cue)) ? 1u : 0u;
    if (!g_pending_dtm_only) {
        (void)refresh_nds_minimap(rt, 1);
    } else if (!rt->has_active_minimap &&
               cue_eligible_for_minimap(&rt->pending_msg, &rt->pending_cue)) {
        /* Maneuver just entered <=450m approach range: upgrade to real NDS minimap once! */
        if (refresh_nds_minimap(rt, 0))
            g_pending_dtm_only = 0u;
    }
    if (dispatch_owner(rt->rg, rt->milca, rt->rg_tid, PHASE_PUBLISH_CUE, route_id, &hud_dispatch_state) != 1u)
        return 0;
    rt->published_cue = rt->pending_cue;
    rt->has_published = 1;
    rt->has_pending = 0;
    return 1;
}

static void clear_session(struct basic_runtime *rt) {
    uint32_t route_id = __atomic_load_n(&active_route_id, __ATOMIC_ACQUIRE);
    __atomic_store_n(&g_rg_gps_suppressed, 0u, __ATOMIC_RELEASE);
    semantic_minimap_shm_clear(NULL);
    if (rt->guidance_active && route_id) {
        (void)dispatch_owner(rt->rg, rt->milca, rt->rg_tid, PHASE_CLEAR_GUIDANCE, route_id, &hud_dispatch_state);
    }
    rt->guidance_active = 0;
    rt->has_pending = 0;
    rt->has_published = 0;
    rt->has_active_minimap = 0;
    rt->session = 0;
    rt->last_seq = 0;
    __atomic_store_n(&resolver_armed_flag, 0u, __ATOMIC_RELEASE);

    struct semantic_handoff h;
    memset(&h, 0, sizeof(h));
    h.state = SH_CLEARED;
    h.route_id = route_id;
    (void)semantic_handoff_write(semantic_handoff_default_path(), &h);
}

static void progress_handoff(struct basic_runtime *rt) {
    if (!rt->session || rt->guidance_active)
        return;

    uint32_t route_id = __atomic_load_n(&active_route_id, __ATOMIC_ACQUIRE);
    if (!route_id) {
        /* Wait for Controller's NewTrip to create a live (non-null) tour in Basic, then build 2-link route. */
        uint64_t now = mono_ms();
        if (rt->calc_attempts >= 5u || now < rt->next_calc_ms)
            return;
        uint32_t live_tour = 0u;
        if (scan_live_tours_mem(rt->mem_fd, rt->stub, &live_tour) > 0u && live_tour >= 0x10000u) {
            rt->calc_attempts++;
            rt->next_calc_ms = now + 500u;
            uint32_t st = dispatch_owner(rt->component, rt->stub, rt->owner_tid,
                                         PHASE_CALC_ROUTE, 0u, &owner_dispatch_state);
            route_id = (st == 1u) ? __atomic_load_n(&created_route_id, __ATOMIC_ACQUIRE) : 0u;
            basic_log("SEMANTIC_BASIC calc_route tour=%#x st=%u route_id=%u intercept=%u\n",
                      live_tour, st, route_id,
                      __atomic_load_n(&resolver_intercept_kind, __ATOMIC_ACQUIRE));
            if (route_id) {
                __atomic_store_n(&active_route_id, route_id, __ATOMIC_RELEASE);
                struct semantic_handoff h;
                memset(&h, 0, sizeof(h));
                h.state = SH_ROUTE_READY;
                h.route_id = route_id;
                h.session = rt->session;
                (void)semantic_handoff_write(semantic_handoff_default_path(), &h);
            }
        }
        return;
    }

    /* Check if Controller finished AttachRoute + StartGuidance for this session. */
    struct semantic_handoff h;
    if (semantic_handoff_read(semantic_handoff_default_path(), &h) &&
        h.session == rt->session && h.state == SH_GUIDANCE_ACTIVE && h.route_id == route_id) {
        uint32_t ast = dispatch_owner(rt->rg, rt->milca, rt->rg_tid, PHASE_ACTIVATE_GUIDANCE,
                                      route_id, &hud_dispatch_state);
        basic_log("SEMANTIC_BASIC activate route_id=%u st=%u\n", route_id, ast);
        if (ast == 1u) {
            rt->guidance_active = 1;
            int pub = try_publish_pending(rt);
            basic_log("SEMANTIC_BASIC publish_first route_id=%u ok=%d road='%s' dist_dm=%u\n",
                      route_id, pub, rt->published_cue.road, rt->published_cue.dist_dm);
        }
    }
}

static void *worker_main(void *administrator) {
    char proc[64];
    snprintf(proc, sizeof(proc), "/proc/%d/as", getpid());
    int mem_fd = open(proc, O_RDONLY);
    if (mem_fd < 0)
        return NULL;

    if (!verify_identity(mem_fd, administrator)) {
        int lfd = open("/dev/shmem/semantic-basic.log", O_WRONLY | O_CREAT | O_TRUNC, 0600);
        if (lfd >= 0) {
            const char msg[] = "SEMANTIC_BASIC refused identity\n";
            (void)write(lfd, msg, sizeof(msg) - 1u);
            close(lfd);
        }
        fprintf(stderr, "SEMANTIC_BASIC refused identity\n");
        close(mem_fd);
        return NULL;
    }

    struct basic_runtime rt;
    memset(&rt, 0, sizeof(rt));
    rt.mem_fd = mem_fd;

    /* Wait for NavigationBasic DataAccess + TourManager + CRouteGuidance + CMiLCa to initialize. */
    for (unsigned i = 0; i < 600u; ++i) {
        uint32_t ready = 0;
        if (word(mem_fd, DATA_ACCESS_READY, &ready) && (ready & 1u) &&
            find_all_mux_targets(mem_fd, &rt.component, &rt.owner_tid,
                                 &rt.rg, &rt.milca, &rt.rg_tid))
            break;
        struct timespec w = {0, 500000000};
        nanosleep(&w, NULL);
    }
    unsigned xar_patched = semantic_xar_patch_shm_in_place(mem_fd);
    if (!rt.component || !rt.rg || !rt.milca) {
        int lfd = open("/dev/shmem/semantic-basic.log", O_WRONLY | O_CREAT | O_TRUNC, 0600);
        if (lfd >= 0) {
            char msg[192];
            int ln = snprintf(msg, sizeof(msg),
                              "SEMANTIC_BASIC refused owner_timeout comp=%#x rg=%#x milca=%#x\n",
                              rt.component, rt.rg, rt.milca);
            if (ln > 0) (void)write(lfd, msg, (size_t)ln);
            close(lfd);
        }
        fprintf(stderr, "SEMANTIC_BASIC refused owner_timeout\n");
        close(mem_fd);
        return NULL;
    }
    rt.stub = rt.component + 0xacu;

    char main_path[96], alive_path[96];
    snprintf(main_path, sizeof(main_path), "%s/native-guidance-%d.sock", NG_SOCKET_ROOT, getpid());
    snprintf(alive_path, sizeof(alive_path), "%s/%s%d.sock", NG_SOCKET_ROOT, NG_ALIVE_ENDPOINT_PREFIX, getpid());
    int main_fd = open_dgram_socket(main_path);
    int alive_fd = open_dgram_socket(alive_path);
    if (main_fd < 0 || alive_fd < 0) {
        fprintf(stderr, "SEMANTIC_BASIC refused socket\n");
        if (main_fd >= 0) close(main_fd);
        if (alive_fd >= 0) close(alive_fd);
        close(mem_fd);
        return NULL;
    }
    {
        int lfd = open("/dev/shmem/semantic-basic.log", O_WRONLY | O_CREAT | O_TRUNC, 0600);
        if (lfd >= 0) {
            char msg[256];
            int ln = snprintf(msg, sizeof(msg),
                              "SEMANTIC_BASIC ready pid=%d comp=%#x owner_tid=%u rg=%#x milca=%#x rg_tid=%u xar_patched=%u\n",
                              getpid(), rt.component, (unsigned)rt.owner_tid, rt.rg, rt.milca,
                              (unsigned)rt.rg_tid, xar_patched);
            if (ln > 0) (void)write(lfd, msg, (size_t)ln);
            close(lfd);
        }
    }
    fprintf(stderr, "SEMANTIC_BASIC ready pid=%d rg=%#x milca=%#x xar_patched=%u\n",
            getpid(), rt.rg, rt.milca, xar_patched);

    struct pollfd pfds[2];
    pfds[0].fd = main_fd;  pfds[0].events = POLLIN;
    pfds[1].fd = alive_fd; pfds[1].events = POLLIN;
    uint64_t next_xar_scan_ms = mono_ms() + 2000u;

    for (;;) {
        (void)poll(pfds, 2, 50);
        if (xar_patched < 2u && mono_ms() >= next_xar_scan_ms) {
            xar_patched = semantic_xar_patch_shm_in_place(mem_fd);
            next_xar_scan_ms = mono_ms() + 5000u;
        }
        uint64_t now = mono_ms();

        /* Handle liveness pings */
        if (pfds[1].revents & POLLIN) {
            uint8_t buf[NG_ALIVE_BYTES + 4u];
            struct qnx_sockaddr_un peer;
            socklen_t peer_len = sizeof(peer);
            ssize_t n = recvfrom(alive_fd, buf, sizeof(buf), 0, (struct sockaddr *)&peer, &peer_len);
            if (n > 0) {
                struct ng_alive_message am;
                unsigned res = ng_alive_decode(buf, (size_t)n, &am);
                struct ng_alive_ack ack;
                memset(&ack, 0, sizeof(ack));
                ack.message = am;
                if (res == NG_ALIVE_ACCEPTED && rt.session && am.session == rt.session) {
                    rt.last_alive_ms = now;
                    ack.result = NG_ALIVE_ACCEPTED;
                    ack.flags = NG_ALIVE_ONLY;
                    ack.accepted_ms = now >= am.created_ms ? now : am.created_ms;
                } else {
                    ack.result = (res == NG_ALIVE_ACCEPTED) ? NG_ALIVE_SESSION : res;
                }
                uint8_t out[NG_ALIVE_ACK_BYTES];
                ng_alive_ack_encode(out, &ack);
                (void)sendto(alive_fd, out, sizeof(out), 0, (struct sockaddr *)&peer, peer_len);
            }
        }

        /* Handle NGV1..5 + NGC1 requests */
        if (pfds[0].revents & POLLIN) {
            uint8_t buf[NG_MAX_FRAME_BYTES + 8u];
            struct qnx_sockaddr_un peer;
            socklen_t peer_len = sizeof(peer);
            ssize_t n = recvfrom(main_fd, buf, sizeof(buf), 0, (struct sockaddr *)&peer, &peer_len);
            if (n > 0) {
                uint32_t cap_seq = ng_cap_query(buf, (size_t)n);
                if (cap_seq) {
                    uint8_t crep[NG_CAP_BYTES];
                    ng_cap_response(crep, cap_seq, NG_CAP_SUPPORTED);
                    (void)sendto(main_fd, crep, sizeof(crep), 0, (struct sockaddr *)&peer, peer_len);
                } else {
                    struct ng_message msg;
                    enum ng_result r = ng_decode(buf, (size_t)n, &msg);
                    if (r != NG_OK) {
                        write_ng_reply(main_fd, (struct sockaddr *)&peer, peer_len, 0u, (uint32_t)r);
                    } else if (msg.operation == NG_PROBE) {
                        write_ng_reply(main_fd, (struct sockaddr *)&peer, peer_len, msg.sequence, NG_OK);
                    } else if (msg.operation == NG_STOP) {
                        if (rt.session && msg.session == rt.session)
                            clear_session(&rt);
                        write_ng_reply(main_fd, (struct sockaddr *)&peer, peer_len, msg.sequence, NG_OK);
                    } else if (msg.operation == NG_BEGIN || msg.operation == NG_UPDATE) {
                        struct semantic_basic_cue cue;
                        if (!semantic_map_ng_message(&msg, &cue)) {
                            write_ng_reply(main_fd, (struct sockaddr *)&peer, peer_len,
                                           msg.sequence, NG_BAD_VALUE);
                        } else if (msg.operation == NG_BEGIN) {
                            if (xar_patched < 2u)
                                xar_patched = semantic_xar_patch_shm_in_place(rt.mem_fd);
                            if (rt.session && rt.session != msg.session)
                                clear_session(&rt);
                            rt.session = msg.session;
                            rt.last_seq = msg.sequence;
                            rt.last_alive_ms = now;
                            rt.next_calc_ms = 0u;
                            rt.calc_attempts = 0u;
                            rt.pending_msg = msg;
                            rt.pending_cue = cue;
                            rt.has_pending = 1;
                            rt.has_published = 0;

                            uint32_t existing_route = __atomic_load_n(&active_route_id, __ATOMIC_ACQUIRE);
                            if (existing_route && !route_exists_in_stub(rt.mem_fd, rt.stub, existing_route)) {
                                basic_log("SEMANTIC_BASIC stale_route=%u cleared (absent from stub)\n", existing_route);
                                __atomic_store_n(&active_route_id, 0u, __ATOMIC_RELEASE);
                                existing_route = 0u;
                            }
                            if (!existing_route) {
                                /* First route (or after OEM nav cleared our old route): arm resolver wrapper for Controller's NewTrip. */
                                __atomic_store_n(&resolver_intercept_kind, 0u, __ATOMIC_RELEASE);
                                __atomic_store_n(&resolver_armed_flag, 1u, __ATOMIC_RELEASE);
                                uint32_t wst = dispatch_owner(rt.component, rt.stub, rt.owner_tid,
                                                              PHASE_INSTALL_RESOLVER, 0u, &resolver_wrapper_state);
                                basic_log("SEMANTIC_BASIC begin session=%#llx wrap_st=%u tours=%u\n",
                                          (unsigned long long)rt.session, wst,
                                          scan_live_tours_mem(rt.mem_fd, rt.stub, NULL));
                            } else {
                                basic_log("SEMANTIC_BASIC begin session=%#llx reuse_route=%u\n",
                                          (unsigned long long)rt.session, existing_route);
                            }
                            struct semantic_handoff h;
                            memset(&h, 0, sizeof(h));
                            h.state = existing_route ? SH_ROUTE_READY : SH_BASIC_ARMED;
                            h.route_id = existing_route;
                            h.session = rt.session;
                            (void)semantic_handoff_write(semantic_handoff_default_path(), &h);
                            write_ng_reply(main_fd, (struct sockaddr *)&peer, peer_len, msg.sequence, NG_OK);
                        } else {
                            /* NG_UPDATE */
                            if (!rt.session || msg.session != rt.session) {
                                write_ng_reply(main_fd, (struct sockaddr *)&peer, peer_len,
                                               msg.sequence, NG_BAD_SESSION);
                            } else {
                                rt.last_seq = msg.sequence;
                                rt.last_alive_ms = now;
                                rt.pending_msg = msg;
                                rt.pending_cue = cue;
                                rt.has_pending = 1;
                                (void)try_publish_pending(&rt);
                                write_ng_reply(main_fd, (struct sockaddr *)&peer, peer_len, msg.sequence, NG_OK);
                            }
                        }
                    }
                }
            }
        }

        progress_handoff(&rt);
        if (rt.guidance_active && rt.has_pending)
            (void)try_publish_pending(&rt);

        /* Lease expiry: if nbt-aad disappears for >10 s, clear our owned guidance cleanly. */
        if (rt.session && rt.last_alive_ms && now > rt.last_alive_ms + NG_ALIVE_LEASE_MS) {
            fprintf(stderr, "SEMANTIC_BASIC lease expired; clearing owned guidance\n");
            clear_session(&rt);
        }
    }
    return NULL;
}

__attribute__((visibility("default"))) void registerCallbacks(void *administrator) {
    static unsigned entered;
    if (__atomic_exchange_n(&entered, 1u, __ATOMIC_ACQ_REL))
        return;
    semantic_xar_install_hooks();
    pthread_t worker;
    if (pthread_create(&worker, NULL, worker_main, administrator) == 0)
        pthread_detach(worker);
}
