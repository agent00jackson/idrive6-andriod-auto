// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
/* Production NavNBTEvoController semantic route-session addon (exact X20511I ARM QNX).
 * Speaks NCT1/NCS1 on NG_SOCKET_ROOT/native-functional-cue-<pid>.sock so nbt-aad
 * can start/reuse a native trip on Root context 0, attach NavigationBasic's route,
 * call StartGuidance, yield to factory BMW navigation, and retire cleanly.
 * Emulator-only MoCCA timer clamping hooks are intentionally excluded. */

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "native_model_wire/protocol.h"
#include "queue-pins.h"
#include "semantic_ctrl_wire.h"
#include "semantic_handoff.h"
#include "sha256.h"
#include "source-pins.h"

#if !defined(__QNXNTO__) || !defined(__arm__)
#error controller_addon.c targets ARM QNX NavNBTEvoController
#endif

#define FN(address, result, ...) ((result (*)(__VA_ARGS__))(uintptr_t)(address))

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

enum {
    MANAGER_VPTR = 0x00f09620,
    TASK_VPTR = 0x00f0bc70,
    ATTACH_TASK_VPTR = 0x00f0ac08,
    START_GUIDANCE_VPTR = 0x00f18ba0,
    CORE_VPTR = 0x00fe7020,
    DEFAULT_STRING = 0x0110c94c,
};

struct owner_event {
    void **vtable;
    uint32_t native_prefix[5];
    uint32_t manager;
    uint32_t request;
    uint32_t trip_segment;
    uint32_t tour_segment;
    uint16_t thread;
    uint16_t action; /* 0=NewTrip, 1=AttachRoute, 2=StartGuidance */
};
_Static_assert(sizeof(void *) == 4, "Exact ARM32 framework ABI");

static void *framework_handle;
static int memory_fd = -1;
static pthread_t worker;
static volatile unsigned stopping, worker_started, registered, executed;
static volatile unsigned trip_callback_state;
static volatile unsigned attach_callback_state;
static volatile unsigned guidance_callback_state;
static unsigned short target_thread = 0xffff;
static uint32_t live_manager;
static uint32_t owned_trip_handle;
static uint32_t owned_route_id;

static void (*event_construct)(void *);
static void (*event_destruct)(void *);
static void (*event_send)(void *, unsigned short);
static void *(*get_config)(void);
static unsigned short (*current_context)(void);

static int file_pin(const char *path, const unsigned char expected[32], unsigned size) {
    unsigned char buffer[4096], digest[32];
    unsigned total = 0;
    ssize_t count;
    map_sha256 hash;
    int fd = open(path, O_RDONLY);
    if (fd < 0) return 0;
    map_sha_init(&hash);
    while ((count = read(fd, buffer, sizeof(buffer))) > 0) {
        if ((unsigned)count > size - total) { close(fd); return 0; }
        total += (unsigned)count;
        map_sha_update(&hash, buffer, (unsigned)count);
    }
    close(fd);
    if (count < 0 || total != size) return 0;
    map_sha_final(&hash, digest);
    return memcmp(expected, digest, sizeof(digest)) == 0;
}

static int read_at(uint32_t address, void *out, size_t size) {
    return address >= 0x10000u && address < 0x80000000u && size <= 4096u &&
           size <= 0x80000000u - address &&
           lseek(memory_fd, (off_t)address, SEEK_SET) == (off_t)address &&
           read(memory_fd, out, size) == (ssize_t)size;
}

static int word_at(uint32_t address, uint32_t *value) {
    return read_at(address, value, sizeof(*value));
}

static int lookup_thread(void) {
    uint32_t config[9], row[3];
    char name[64];
    unsigned found = 0;
    void *raw = get_config();
    if (!raw || !read_at((uint32_t)(uintptr_t)raw, config, sizeof(config)) ||
        !config[8] || !config[7] || config[7] > 64u)
        return 0;
    for (unsigned i = 0; i < config[7]; ++i) {
        if (!read_at(config[8] + i * 12u, row, sizeof(row)) || !row[0])
            return 0;
        unsigned j;
        for (j = 0; j < sizeof(name); ++j) {
            if (!read_at(row[0] + j, name + j, 1u)) return 0;
            if (!name[j]) break;
        }
        if (j == sizeof(name)) return 0;
        if (strcmp(name, "RootThread") == 0) {
            target_thread = 0;
            ++found;
        }
    }
    return found == 1u;
}

static int typed_manager(uint32_t manager) {
    uint32_t core, task, backlink, table, receiver, first, last, cap;
    if (manager < 0x1000000u + 0xdb0u || manager > 0x2000000u - 0x17cu)
        return 0;
    core = manager - 0xdb0u;
    task = manager + 0x17cu;
    table = manager + 0x74u;
    return word_at(core, &backlink) && backlink == CORE_VPTR &&
           word_at(task, &backlink) && backlink == TASK_VPTR &&
           word_at(task + 0xcu, &backlink) && backlink == table &&
           word_at(task + 0x18u, &backlink) && backlink == manager &&
           word_at(task + 0x10u, &receiver) && receiver >= 0x10000u &&
           word_at(table + 8u, &first) && word_at(table + 12u, &last) &&
           word_at(table + 16u, &cap) && first <= last && last <= cap &&
           (!first || (last - first) % 4u == 0u) && (last - first) / 4u <= 1024u;
}

static uint32_t find_manager(void) {
    uint32_t words[1024], candidate = 0, count = 0;
    for (uint32_t base = 0x1000000u; base < 0x2000000u; base += sizeof(words)) {
        if (!read_at(base, words, sizeof(words)))
            continue;
        for (uint32_t i = 0; i < 1024u; ++i) {
            if (words[i] != MANAGER_VPTR) continue;
            uint32_t addr = base + i * 4u;
            if (typed_manager(addr)) {
                candidate = addr;
                ++count;
            }
        }
    }
    return count == 1u ? candidate : 0u;
}

static void destroy_event(void *raw) { event_destruct(raw); }
static void delete_event(void *raw) {
    event_destruct(raw);
    free(raw);
    __atomic_store_n(&executed, 1u, __ATOMIC_RELEASE);
}
static void *cast_event(void *raw, const void *type) { (void)raw; (void)type; return NULL; }
static void send_event(void *raw) { event_send(raw, target_thread); }

static void generic_completed(void *ctx, int ok) {
    if (ctx)
        __atomic_store_n((volatile unsigned *)ctx, ok ? 1u : 2u, __ATOMIC_RELEASE);
}

static void execute_event(void *raw, void *dispatcher) {
    const struct owner_event *event = raw;
    uint32_t manager = event->manager;
    uint32_t *request = (uint32_t *)(uintptr_t)event->request;
    (void)dispatcher;
    if (__atomic_load_n(&stopping, __ATOMIC_ACQUIRE) || current_context() != event->thread)
        return;

    if (event->action == 1u) {
        uint32_t attach_task = manager + 0x20cu;
        if (*(uint32_t *)(uintptr_t)attach_task != ATTACH_TASK_VPTR ||
            *(uint32_t *)(uintptr_t)(attach_task + 8u) != manager + 0x74u)
            return;
        FN(0x42bb08, void, void *, void *)((void *)(uintptr_t)attach_task, request);
        return;
    }
    if (event->action == 2u) {
        uint32_t start_task = manager + 0x1af8u;
        if (*(uint32_t *)(uintptr_t)start_task != START_GUIDANCE_VPTR ||
            *(uint32_t *)(uintptr_t)(start_task + 0x14u) != manager)
            return;
        FN(0x4689cc, void, void *, void *)((void *)(uintptr_t)start_task, request);
        return;
    }

    uint32_t task = manager + 0x17cu;
    uint32_t *trip = (uint32_t *)(uintptr_t)event->trip_segment;
    uint32_t *segment = (uint32_t *)(uintptr_t)event->tour_segment;
    uint32_t *description = request + 8;
    if (*(uint32_t *)(uintptr_t)manager != MANAGER_VPTR ||
        *(uint32_t *)(uintptr_t)task != TASK_VPTR ||
        *(uint32_t *)(uintptr_t)(task + 0xcu) != manager + 0x74u ||
        *(uint32_t *)(uintptr_t)(task + 0x18u) != manager)
        return;
    if (*(uint32_t *)(uintptr_t)(manager + 0x7cu) !=
        *(uint32_t *)(uintptr_t)(manager + 0x80u))
        return; /* Never create a second trip. */

    FN(0x2cdf84, void, void *)(description);
    FN(0x2cd2c4, void, void *)(segment);
    FN(0x2cdf2c, void, void *)(segment + 0x7c / 4);
    if (segment[0x14 / 4] != DEFAULT_STRING ||
        segment[0xc4 / 4] != 0xffffu || segment[0xc8 / 4] != 99u)
        return;

    request[1] = (uint32_t)(uintptr_t)&trip_callback_state;
    request[2] = (uint32_t)(uintptr_t)&generic_completed;
    FN(0x128160, void, void *, const void *)(request + 0x98 / 4, segment + 0x14 / 4);
    description[1] = (uint32_t)(uintptr_t)segment;
    description[2] = description[1] + 0xccu;
    description[3] = description[2];
    trip[0x10 / 4] = 1u;
    request[0x14 / 4] = (uint32_t)(uintptr_t)trip;
    request[0x18 / 4] = request[0x14 / 4] + 0x24u;
    request[0x1c / 4] = request[0x18 / 4];
    FN(0x430f38, unsigned, void *, void *)((void *)(uintptr_t)task, request);
}

static void *event_vtable[5] = {
    destroy_event, delete_event, cast_event, send_event, execute_event
};

static int inspect_trip_table(uint32_t *out_entries, uint32_t *out_attached_route) {
    uint32_t first = 0, last = 0;
    *out_entries = 0;
    *out_attached_route = 0;
    if (!live_manager || !word_at(live_manager + 0x7cu, &first) ||
        !word_at(live_manager + 0x80u, &last) || last < first)
        return 0;
    *out_entries = (last - first) / 4u;
    if (*out_entries == 1u && first >= 0x10000u) {
        uint32_t entry = 0;
        if (word_at(first, &entry) && entry >= 0x10000u)
            (void)word_at(entry + 0xa8u, out_attached_route);
    }
    return 1;
}

static int ctrl_can_select(void *ctx, uint32_t *out_foreign_active) {
    (void)ctx;
    uint32_t entries = 0, attached = 0;
    *out_foreign_active = 0;
    if (!inspect_trip_table(&entries, &attached))
        return 0;
    if (entries == 0u)
        return 1;
    /* Reuse our own trip across back-to-back AA routes; yield if a foreign BMW trip exists. */
    if (entries == 1u && owned_trip_handle && owned_route_id && attached == owned_route_id)
        return 1;
    *out_foreign_active = 1u;
    return 0;
}

static int dispatch_start_guidance(uint32_t trip_handle) {
    uint32_t *start_req = calloc(1, 0x40);
    struct owner_event *sev = calloc(1, sizeof(*sev));
    if (!start_req || !sev) { free(start_req); free(sev); return 0; }
    __atomic_store_n(&guidance_callback_state, 0u, __ATOMIC_RELEASE);
    start_req[0] = (uint32_t)(uintptr_t)&guidance_callback_state;
    start_req[1] = (uint32_t)(uintptr_t)&generic_completed;
    start_req[3] = trip_handle;
    start_req[4] = 0u;          /* discard_attached_route = 0 */
    start_req[5] = UINT32_MAX;  /* dest_index = -1 */
    event_construct(sev);
    sev->vtable = event_vtable;
    sev->manager = live_manager;
    sev->request = (uint32_t)(uintptr_t)start_req;
    sev->thread = target_thread;
    sev->action = 2u;
    send_event(sev);
    for (unsigned w = 0; w < 200u && !__atomic_load_n(&stopping, __ATOMIC_ACQUIRE); ++w) {
        unsigned st = __atomic_load_n(&guidance_callback_state, __ATOMIC_ACQUIRE);
        if (st) return st == 1u;
        struct timespec d = {0, 25000000};
        nanosleep(&d, NULL);
    }
    return 0;
}

static int ctrl_start_route(void *ctx, uint64_t session) {
    (void)ctx;
    uint32_t entries = 0, attached = 0;
    if (!inspect_trip_table(&entries, &attached))
        return 0;

    uint32_t route_id = 0;
    if (entries == 1u && owned_trip_handle && owned_route_id && attached == owned_route_id) {
        route_id = owned_route_id;
    } else if (entries == 0u) {
        /* 1. Run NewTrip on Root context 0 */
        uint8_t *request = calloc(1, 0x100);
        uint8_t *trip = calloc(1, 0x24);
        uint8_t *segment = calloc(1, 0xcc);
        struct owner_event *ev = calloc(1, sizeof(*ev));
        if (!request || !trip || !segment || !ev)
            return 0;
        __atomic_store_n(&trip_callback_state, 0u, __ATOMIC_RELEASE);
        __atomic_store_n(&executed, 0u, __ATOMIC_RELEASE);
        event_construct(ev);
        ev->vtable = event_vtable;
        ev->manager = live_manager;
        ev->request = (uint32_t)(uintptr_t)request;
        ev->trip_segment = (uint32_t)(uintptr_t)trip;
        ev->tour_segment = (uint32_t)(uintptr_t)segment;
        ev->thread = target_thread;
        ev->action = 0u;
        send_event(ev);

        uint32_t handle = 0;
        for (unsigned w = 0; w < 200u && !__atomic_load_n(&stopping, __ATOMIC_ACQUIRE); ++w) {
            handle = __atomic_load_n((uint32_t *)request, __ATOMIC_ACQUIRE);
            unsigned cb = __atomic_load_n(&trip_callback_state, __ATOMIC_ACQUIRE);
            if (handle || cb == 2u) break;
            struct timespec d = {0, 25000000};
            nanosleep(&d, NULL);
        }
        if (!handle)
            return 0;
        owned_trip_handle = handle;

        /* 2. Wait for NavigationBasic addon to publish RouteId for this session */
        for (unsigned w = 0; w < 200u && !__atomic_load_n(&stopping, __ATOMIC_ACQUIRE); ++w) {
            struct semantic_handoff h;
            if (semantic_handoff_read(semantic_handoff_default_path(), &h) &&
                h.session == session && h.state == SH_ROUTE_READY && h.route_id) {
                route_id = h.route_id;
                break;
            }
            struct timespec d = {0, 25000000};
            nanosleep(&d, NULL);
        }
        if (!route_id)
            return 0;

        /* 3. AttachRoute(owned_trip_handle, route_id) on Root context 0 */
        uint32_t *attach_req = calloc(1, 0x40);
        struct owner_event *aev = calloc(1, sizeof(*aev));
        if (!attach_req || !aev) return 0;
        __atomic_store_n(&attach_callback_state, 0u, __ATOMIC_RELEASE);
        attach_req[0] = (uint32_t)(uintptr_t)&attach_callback_state;
        attach_req[1] = (uint32_t)(uintptr_t)&generic_completed;
        attach_req[4] = owned_trip_handle;
        attach_req[5] = route_id;
        event_construct(aev);
        aev->vtable = event_vtable;
        aev->manager = live_manager;
        aev->request = (uint32_t)(uintptr_t)attach_req;
        aev->thread = target_thread;
        aev->action = 1u;
        send_event(aev);
        for (unsigned w = 0; w < 200u && !__atomic_load_n(&stopping, __ATOMIC_ACQUIRE); ++w) {
            unsigned st = __atomic_load_n(&attach_callback_state, __ATOMIC_ACQUIRE);
            if (st) break;
            struct timespec d = {0, 25000000};
            nanosleep(&d, NULL);
        }
        if (__atomic_load_n(&attach_callback_state, __ATOMIC_ACQUIRE) != 1u)
            return 0;
        owned_route_id = route_id;
    } else {
        return 0; /* Foreign BMW route active */
    }

    /* 4. StartGuidance(owned_trip_handle) on Root context 0 */
    if (!dispatch_start_guidance(owned_trip_handle))
        return 0;

    struct semantic_handoff h;
    memset(&h, 0, sizeof(h));
    h.state = SH_GUIDANCE_ACTIVE;
    h.route_id = route_id;
    h.trip_handle = owned_trip_handle;
    h.session = session;
    (void)semantic_handoff_write(semantic_handoff_default_path(), &h);
    return 1;
}

static int ctrl_stop_route(void *ctx, uint64_t session) {
    (void)ctx;
    (void)session;
    /* NavigationBasic addon performs the clean RG_GuidanceState=00 clear on NG_STOP. */
    return 1;
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

static void *run(void *unused) {
    (void)unused;
    for (unsigned i = 0; i < 600u && !__atomic_load_n(&stopping, __ATOMIC_ACQUIRE); ++i) {
        if (lookup_thread() && (live_manager = find_manager()))
            break;
        struct timespec delay = {0, 500000000};
        nanosleep(&delay, NULL);
    }
    if (!live_manager || __atomic_load_n(&stopping, __ATOMIC_ACQUIRE))
        return NULL;

    char sock_path[96];
    snprintf(sock_path, sizeof(sock_path), "%s/native-functional-cue-%d.sock",
             NG_SOCKET_ROOT, getpid());
    int fd = open_dgram_socket(sock_path);
    if (fd < 0)
        return NULL;

    fprintf(stderr, "SEMANTIC_CTRL ready pid=%d manager=%#x\n", getpid(), live_manager);

    struct sc_machine machine;
    sc_machine_init(&machine, mono_ms() ^ ((uint64_t)getpid() << 16), 10000u);
    struct sc_ops ops = {
        .can_select = ctrl_can_select,
        .start_route = ctrl_start_route,
        .stop_route = ctrl_stop_route,
        .ctx = NULL,
    };

    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    while (!__atomic_load_n(&stopping, __ATOMIC_ACQUIRE)) {
        (void)poll(&pfd, 1, 100);
        uint64_t now = mono_ms();
        if (pfd.revents & POLLIN) {
            uint8_t buf[SC_WIRE_BYTES + 4u];
            struct qnx_sockaddr_un peer;
            socklen_t peer_len = sizeof(peer);
            ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&peer, &peer_len);
            if (n > 0) {
                struct sc_request req;
                if (sc_decode_request(buf, (size_t)n, &req)) {
                    struct sc_reply rep;
                    sc_machine_handle(&machine, &req, now, &ops, &rep);
                    uint8_t wire[SC_WIRE_BYTES];
                    sc_encode_reply(wire, &rep);
                    (void)sendto(fd, wire, sizeof(wire), 0, (struct sockaddr *)&peer, peer_len);
                }
            }
        }
        (void)sc_machine_tick(&machine, now, &ops);
    }

    close(fd);
    unlink(sock_path);
    return NULL;
}

static void stop_cb(void *context) {
    (void)context;
    __atomic_store_n(&stopping, 1u, __ATOMIC_RELEASE);
    if (worker_started)
        pthread_join(worker, NULL);
}

#define SYMBOL(type, name) ((type)dlsym(framework_handle, name))
__attribute__((visibility("default"))) void registerCallbacks(void *administrator) {
    typedef void *(*instance_fn)(void);
    typedef void (*register_fn)(void *, unsigned, void (*)(void *), void *);
    const char *stage = "duplicate-entry";
    char proc[64];

    if (__atomic_exchange_n(&registered, 1u, __ATOMIC_ACQ_REL))
        goto refused;
    stage = "controller-source";
    if (!file_pin("/opt/nav/eceusa/bin/NavNBTEvoController", controller_sha, NC_CONTROLLER_BYTES))
        goto refused;
    stage = "framework-source";
    if (!file_pin("/opt/sys/lib/libSysMoCCAFrameworkSharedSo.cpp-ne.so.11", framework_sha, NC_FRAMEWORK_BYTES))
        goto refused;
    stage = "framework-load";
    framework_handle = dlopen("/opt/sys/lib/libSysMoCCAFrameworkSharedSo.cpp-ne.so.11", RTLD_NOW);
    if (!framework_handle)
        goto refused;

    instance_fn instance = SYMBOL(instance_fn, "_ZN21CHBApplicationManager11getInstanceEv");
    register_fn callback = SYMBOL(register_fn, "_ZN21CHBApplicationManager16registerCallbackENS_14ExecutionPointEPFvPvES1_");
    event_construct = SYMBOL(void (*)(void *), "_ZN8CHBEventC2Ev");
    event_destruct = SYMBOL(void (*)(void *), "_ZN8CHBEventD2Ev");
    event_send = SYMBOL(void (*)(void *, unsigned short), "_ZN8CHBEvent4sendEt");
    get_config = SYMBOL(void *(*)(void), "_ZN24IHBApplicationController15getSystemConfigEv");
    current_context = SYMBOL(unsigned short (*)(void), "_ZN9CHBThread16currentContextIDEv");
    stage = "framework-symbols";
    if (!instance || !callback || !event_construct || !event_destruct || !event_send ||
        !get_config || !current_context)
        goto refused;
    stage = "mapped-code";
    if (memcmp((const void *)instance, get_instance_pin, 16) ||
        memcmp((const void *)callback, register_shutdown_pin, 16) ||
        memcmp((const void *)event_construct, event_construct_pin, 16) ||
        memcmp((const void *)event_destruct, event_destruct_pin, 16) ||
        memcmp((const void *)event_send, event_send_pin, 16) ||
        memcmp((const void *)get_config, get_config_pin, 16) ||
        memcmp((const void *)current_context, current_context_pin, 16) ||
        memcmp((const void *)(uintptr_t)0x430cec, "\xf0\x41\x2d\xe9", 4) ||
        memcmp((const void *)(uintptr_t)0x42bb08, "\xf0\x41\x2d\xe9", 4) ||
        memcmp((const void *)(uintptr_t)0x4689cc, "\x38\x33\x9f\xe5", 4) ||
        memcmp((const void *)(uintptr_t)0x128160, "\x0f\xc6\x8f\xe2", 4) ||
        memcmp((const void *)(uintptr_t)0x2cdf84, "\x10\x40\x2d\xe9", 4) ||
        memcmp((const void *)(uintptr_t)0x2cd2c4, "\x10\x40\x2d\xe9", 4) ||
        memcmp((const void *)(uintptr_t)0x2cdf2c, "\x70\x40\x2d\xe9", 4))
        goto refused;
    stage = "administrator-identity";
    if (!administrator || instance() != administrator)
        goto refused;
    snprintf(proc, sizeof(proc), "/proc/%d/as", getpid());
    memory_fd = open(proc, O_RDONLY);
    stage = "own-memory-readonly";
    if (memory_fd < 0)
        goto refused;
    callback(administrator, 4u, stop_cb, NULL);
    stage = "worker";
    if (pthread_create(&worker, NULL, run, NULL))
        goto refused;
    worker_started = 1u;
    puts("SEMANTIC_CTRL registered exact_source=1");
    fflush(NULL);
    return;
refused:
    printf("SEMANTIC_CTRL refused stage=%s\n", stage);
    fflush(NULL);
}
