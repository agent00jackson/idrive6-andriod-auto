// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Host unit tests for the Semantic Navigation runtime:
//   - semantic_cue_map (all verified AA maneuver families, lanes, roundabouts, arrivals, distance-only diff)
//   - semantic_handoff (atomic shared-memory coordination record)
//   - semantic_ctrl_wire (NCT1/NCS1 codec, Probe/Arm/Heartbeat/Stop/Rearm, BMW factory-nav yield, lease expiry)
//   - live end-to-end loopback with nbt::kombinav::NativeDisplay + nbt::native_model::Backend

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "cluster/native_display.hpp"
#include "native_controller_protocol.hpp"
#include "native_model_protocol.hpp"
#include "native_model_wire/alive_wire.h"
#include "native_model_wire/capabilities.h"
#include "../../tools/native-semantic-addon/semantic_ctrl_wire.h"
#include "../../tools/native-semantic-addon/semantic_cue_map.h"
#include "../../tools/native-semantic-addon/semantic_handoff.h"
extern "C" {
#include "../../tools/native-semantic-addon/semantic_ctrl_wire.c"
#include "../../tools/native-semantic-addon/semantic_cue_map.c"
#include "../../tools/native-semantic-addon/semantic_handoff.c"
}

static uint64_t nowMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

static nbt::kombinav::NavView makeView(int aaType, double metres, const char* road = "",
                                       bool hasAngle = false, int angle = 0) {
    nbt::kombinav::NavView v;
    v.aa_type = aaType;
    v.has_step_m = true;
    v.step_m = (int64_t)(metres + 0.5);
    v.aa_unit = 1;
    if (road && *road) v.road = road;
    v.has_exit_angle = hasAngle;
    v.exit_angle = angle;
    return v;
}

static void testCueMappings() {
    struct {
        int aa_type;
        double dist_m;
        bool has_angle;
        int exit_angle;
        uint32_t exp_action;
        uint32_t exp_dir;
        uint32_t exp_elem;
        float exp_angle;
        uint8_t exp_arrival;
    } cases[] = {
        { 7,  45.0, false,   0,  2, 2,  7, 270.0f, 0 }, // left
        { 8,  60.0, false,   0,  2, 1,  7,  90.0f, 0 }, // right
        { 4,  50.0, false,   0,  2, 1,  7,  45.0f, 0 }, // slight right
        { 3,  50.0, false,   0,  2, 2,  7, 315.0f, 0 }, // slight left
        { 9,  50.0, false,   0,  2, 2,  7, 225.0f, 0 }, // sharp left
        { 10, 50.0, false,   0,  2, 1,  7, 135.0f, 0 }, // sharp right
        { 25, 50.0, false,   0, 14, 2, 12, 315.0f, 0 }, // fork left
        { 26, 50.0, false,   0, 14, 1, 12,  45.0f, 0 }, // fork right
        { 33, 70.0, true,   90,  2, 1, 15, 270.0f, 0 }, // CW roundabout 90 -> OEM 270
        { 33, 70.0, true,  180,  2, 1, 15,   0.0f, 0 }, // CW roundabout 180 -> OEM 0
        { 33, 70.0, true,  270,  2, 1, 15,  90.0f, 0 }, // CW roundabout 270 -> OEM 90
        { 35, 70.0, true,   90,  2, 2, 16,  90.0f, 0 }, // CCW roundabout 90 -> OEM 90
        { 23, 80.0, false,   0, 16, 2, 10, 315.0f, 0 }, // off-ramp left
        { 24, 80.0, false,   0, 16, 1, 10,  45.0f, 0 }, // off-ramp right
        { 11, 45.0, false,   0,  3, 2, 19, 270.0f, 0 }, // u-turn left
        { 12, 45.0, false,   0,  3, 1, 19,  90.0f, 0 }, // u-turn right
        { 36,  0.0, false,   0,  1, 3,  1,   0.0f, 0 }, // straight 0 m
        { 2,  30.0, false,   0,  1, 3,  1,   0.0f, 0 }, // name change
        { 39,  0.0, false,   0, 18, 3,  1,   0.0f, 1 }, // arrival zero
        { 41, 15.0, false,   0, 18, 2,  1, 270.0f, 1 }, // arrival left
        { 42, 15.0, false,   0, 18, 1,  1,  90.0f, 1 }, // arrival right
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        nbt::kombinav::NavView v = makeView(cases[i].aa_type, cases[i].dist_m, "Spring Road",
                                            cases[i].has_angle, cases[i].exit_angle);
        nbt::native_model::Message m;
        assert(nbt::native_model::fromNav(v, m, NG_CAP_SUPPORTED));
        ng_message wire = nbt::native_model::nativeMessage(m);
        semantic_basic_cue cue;
        assert(semantic_map_ng_message(&wire, &cue));
        assert(cue.action == cases[i].exp_action);
        assert(cue.direction == cases[i].exp_dir);
        assert(cue.elem_type == cases[i].exp_elem);
        assert(fabsf(cue.angle_deg - cases[i].exp_angle) < 0.01f);
        assert(cue.is_arrival == cases[i].exp_arrival);
        assert(cue.dist_dm == (uint32_t)(cases[i].dist_m * 10.0 + 0.5));
        assert(cue.norm_m == (uint32_t)(cases[i].dist_m + 0.5));
        assert(strcmp(cue.road, "Spring Road") == 0);
    }

    // Distance-only vs street/lane change detection (verifies lane-only & street-only removal trigger rotation!)
    ng_message a, b;
    memset(&a, 0, sizeof(a));
    a.operation = NG_BEGIN; a.turn = NG_LEFT; a.direction_centunits = 6400;
    a.distance_mm = 45000; a.metadata_flags = NG_META_AA_TYPE | NG_META_ROAD;
    a.aa_type = 7; a.road_length = 11; memcpy(a.road, "Spring Road", 11);
    a.lane_count = 2;
    a.lanes[0] = { NG_LANE_LEFT, NG_LANE_LEFT };
    a.lanes[1] = { NG_LANE_RIGHT, 0 };
    b = a;
    b.distance_mm = 35000;

    semantic_basic_cue ca, cb;
    assert(semantic_map_ng_message(&a, &ca));
    assert(semantic_map_ng_message(&b, &cb));
    assert(semantic_cue_is_distance_only(&ca, &cb) == 1);

    // Move lane recommendation LEFT -> RIGHT: must NOT be distance-only
    b.lanes[0].recommended = 0; b.lanes[1].recommended = NG_LANE_RIGHT;
    assert(semantic_map_ng_message(&b, &cb));
    assert(semantic_cue_is_distance_only(&ca, &cb) == 0);

    // Lane-only removal: must NOT be distance-only
    b = a; b.lane_count = 0;
    assert(semantic_map_ng_message(&b, &cb));
    assert(semantic_cue_is_distance_only(&ca, &cb) == 0);

    // Street-only removal: must NOT be distance-only
    b = a; b.metadata_flags = NG_META_AA_TYPE; b.road_length = 0; memset(b.road, 0, sizeof(b.road));
    assert(semantic_map_ng_message(&b, &cb));
    assert(semantic_cue_is_distance_only(&ca, &cb) == 0);

    // Verify lane degrees
    assert(semantic_lane_degrees(NG_LANE_STRAIGHT) == 0.0f);
    assert(semantic_lane_degrees(NG_LANE_LEFT) == 270.0f);
    assert(semantic_lane_degrees(NG_LANE_RIGHT) == 90.0f);
    assert(semantic_lane_degrees(NG_LANE_UTURN_LEFT) == 181.0f);
    assert(semantic_lane_degrees(NG_LANE_UTURN_RIGHT) == 179.0f);
}

static void testHandoff() {
    char tmpl[] = "/tmp/nbt-handoff-XXXXXX";
    assert(mkdtemp(tmpl) != NULL);
    std::string path = std::string(tmpl) + "/handoff";
    semantic_handoff w, r;
    memset(&w, 0, sizeof(w));
    w.state = SH_ROUTE_READY;
    w.route_id = 2118;
    w.trip_handle = 1;
    w.session = 0x123456789abcdef0ULL;
    assert(semantic_handoff_write(path.c_str(), &w) == 1);
    assert(semantic_handoff_read(path.c_str(), &r) == 1);
    assert(r.state == SH_ROUTE_READY && r.route_id == 2118 && r.trip_handle == 1 && r.session == w.session);
    unlink(path.c_str());
    rmdir(tmpl);
}

struct MockCtrlOps {
    int foreign_active;
    int starts;
    int stops;
};
static int mockCanSelect(void* ctx, uint32_t* out_foreign) {
    MockCtrlOps* o = (MockCtrlOps*)ctx;
    *out_foreign = (uint32_t)o->foreign_active;
    return !o->foreign_active;
}
static int mockStart(void* ctx, uint64_t) { ++((MockCtrlOps*)ctx)->starts; return 1; }
static int mockStop(void* ctx, uint64_t) { ++((MockCtrlOps*)ctx)->stops; return 1; }

static void testControllerStateMachine() {
    sc_machine m;
    sc_machine_init(&m, 42u, 5000u);
    MockCtrlOps mock = {0, 0, 0};
    sc_ops ops = { mockCanSelect, mockStart, mockStop, &mock };

    // 1. Probe -> Idle + Eligible
    uint8_t wire[SC_WIRE_BYTES], repWire[SC_WIRE_BYTES];
    nbt::native_model::controller::encode(wire, nbt::native_model::controller::kProbe, 1, 0, 0, 0, 0, 0, 1000);
    sc_request req;
    assert(sc_decode_request(wire, sizeof(wire), &req));
    sc_reply rep;
    sc_machine_handle(&m, &req, 1000, &ops, &rep);
    sc_encode_reply(repWire, &rep);
    nbt::native_model::controller::Reply decoded;
    assert(nbt::native_model::controller::decode(repWire, sizeof(repWire), 1, decoded));
    assert(decoded.phase == nbt::native_model::controller::kIdle);
    assert(decoded.nonce == 42u);
    assert((decoded.flags & SC_FLAG_PREFLIGHT_ELIGIBLE) != 0);

    // 2. Arm -> Armed
    nbt::native_model::controller::encode(wire, nbt::native_model::controller::kArm, 2, 42u, 99u, 123, 1, 777u, 1000);
    assert(sc_decode_request(wire, sizeof(wire), &req));
    sc_machine_handle(&m, &req, 1000, &ops, &rep);
    assert(rep.result == 0 && rep.phase == SC_PHASE_ARMED && mock.starts == 1);

    // 3. Heartbeat -> Selected
    nbt::native_model::controller::encode(wire, nbt::native_model::controller::kHeartbeat, 3, 42u, 99u, 123, 2, 777u, 2000);
    assert(sc_decode_request(wire, sizeof(wire), &req));
    sc_machine_handle(&m, &req, 2000, &ops, &rep);
    assert(rep.result == 0 && rep.phase == SC_PHASE_SELECTED);

    // 4. Stop -> EmptyReturned + CompletedRearm
    nbt::native_model::controller::encode(wire, nbt::native_model::controller::kStop, 4, 42u, 99u, 123, 2, 777u, 2500);
    assert(sc_decode_request(wire, sizeof(wire), &req));
    sc_machine_handle(&m, &req, 2500, &ops, &rep);
    assert(rep.result == 0 && rep.phase == SC_PHASE_EMPTY_RETURNED && mock.stops == 1);
    assert((rep.flags & SC_FLAG_COMPLETED_REARM) != 0);

    // 5. Rearm -> Idle with incremented nonce
    nbt::native_model::controller::encode(wire, nbt::native_model::controller::kRearm, UINT32_MAX, 42u, 99u, 0, 0, 0, 2600);
    assert(sc_decode_request(wire, sizeof(wire), &req));
    sc_machine_handle(&m, &req, 2600, &ops, &rep);
    assert(rep.result == 0 && rep.phase == SC_PHASE_IDLE && rep.nonce == 43u && rep.epoch == 0);

    // 6. Foreign BMW route active -> Probe reports ineligible, Arm yields without starting
    mock.foreign_active = 1;
    nbt::native_model::controller::encode(wire, nbt::native_model::controller::kProbe, 1, 0, 0, 0, 0, 0, 3000);
    assert(sc_decode_request(wire, sizeof(wire), &req));
    sc_machine_handle(&m, &req, 3000, &ops, &rep);
    assert((rep.flags & SC_FLAG_PREFLIGHT_ELIGIBLE) == 0);
    nbt::native_model::controller::encode(wire, nbt::native_model::controller::kArm, 2, 43u, 100u, 123, 1, 888u, 3000);
    assert(sc_decode_request(wire, sizeof(wire), &req));
    sc_machine_handle(&m, &req, 3000, &ops, &rep);
    assert(rep.result != 0 && rep.phase == SC_PHASE_YIELDED && mock.starts == 1);

    // 7. Rearm + Arm + Lease expiry -> retires cleanly
    mock.foreign_active = 0;
    nbt::native_model::controller::encode(wire, nbt::native_model::controller::kRearm, UINT32_MAX, 43u, 0, 0, 0, 0, 3100);
    assert(sc_decode_request(wire, sizeof(wire), &req));
    sc_machine_handle(&m, &req, 3100, &ops, &rep);
    assert(rep.phase == SC_PHASE_IDLE && rep.nonce == 44u);
    nbt::native_model::controller::encode(wire, nbt::native_model::controller::kArm, 2, 44u, 101u, 123, 1, 999u, 3200);
    assert(sc_decode_request(wire, sizeof(wire), &req));
    sc_machine_handle(&m, &req, 3200, &ops, &rep);
    assert(rep.phase == SC_PHASE_ARMED && mock.starts == 2);
    assert(sc_machine_tick(&m, 9000, &ops) == 1);
    assert(m.phase == SC_PHASE_EMPTY_RETURNED && mock.stops == 2);
}

// End-to-end loopback with real nbt::kombinav::NativeDisplay + nbt::native_model::Backend
struct LoopbackServer {
    int basicFd, aliveFd, ctrlFd;
    volatile int quit;
    int begins, updates, stops, ctrlStarts, ctrlStops;
    semantic_basic_cue lastCue;
    pthread_t th;
};

static int bindUnix(const std::string& path) {
    unlink(path.c_str());
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    assert(fd >= 0);
    fcntl(fd, F_SETFL, O_NONBLOCK);
    sockaddr_un a;
    memset(&a, 0, sizeof(a));
    a.sun_family = AF_UNIX;
    strncpy(a.sun_path, path.c_str(), sizeof(a.sun_path) - 1);
    assert(bind(fd, (sockaddr*)&a, sizeof(a)) == 0);
    assert(chmod(path.c_str(), 0600) == 0);
    return fd;
}

static int lbCanSelect(void*, uint32_t* f) { *f = 0; return 1; }
static int lbStart(void* c, uint64_t) { ++((LoopbackServer*)c)->ctrlStarts; return 1; }
static int lbStop(void* c, uint64_t) { ++((LoopbackServer*)c)->ctrlStops; return 1; }

static void* loopbackThread(void* arg) {
    LoopbackServer* s = (LoopbackServer*)arg;
    sc_machine cm;
    sc_machine_init(&cm, 100u, 10000u);
    sc_ops cops = { lbCanSelect, lbStart, lbStop, s };
    uint64_t curSession = 0;

    pollfd pfds[3] = {
        { s->basicFd, POLLIN, 0 },
        { s->aliveFd, POLLIN, 0 },
        { s->ctrlFd,  POLLIN, 0 },
    };
    while (!s->quit) {
        poll(pfds, 3, 10);
        uint64_t now = nowMs();
        if (pfds[1].revents & POLLIN) {
            uint8_t buf[64];
            sockaddr_un peer; socklen_t plen = sizeof(peer);
            ssize_t n = recvfrom(s->aliveFd, buf, sizeof(buf), 0, (sockaddr*)&peer, &plen);
            if (n > 0) {
                ng_alive_message am;
                unsigned r = ng_alive_decode(buf, (size_t)n, &am);
                ng_alive_ack ack; memset(&ack, 0, sizeof(ack));
                ack.message = am;
                if (r == NG_ALIVE_ACCEPTED && am.session == curSession) {
                    ack.result = NG_ALIVE_ACCEPTED;
                    ack.flags = NG_ALIVE_ONLY;
                    ack.accepted_ms = now >= am.created_ms ? now : am.created_ms;
                } else {
                    ack.result = NG_ALIVE_SESSION;
                }
                uint8_t out[NG_ALIVE_ACK_BYTES];
                ng_alive_ack_encode(out, &ack);
                sendto(s->aliveFd, out, sizeof(out), 0, (sockaddr*)&peer, plen);
            }
        }
        if (pfds[0].revents & POLLIN) {
            uint8_t buf[256];
            sockaddr_un peer; socklen_t plen = sizeof(peer);
            ssize_t n = recvfrom(s->basicFd, buf, sizeof(buf), 0, (sockaddr*)&peer, &plen);
            if (n > 0) {
                uint32_t cseq = ng_cap_query(buf, (size_t)n);
                if (cseq) {
                    uint8_t crep[NG_CAP_BYTES];
                    ng_cap_response(crep, cseq, NG_CAP_SUPPORTED);
                    sendto(s->basicFd, crep, sizeof(crep), 0, (sockaddr*)&peer, plen);
                } else {
                    ng_message msg;
                    assert(ng_decode(buf, (size_t)n, &msg) == NG_OK);
                    if (msg.operation == NG_BEGIN) {
                        curSession = msg.session;
                        assert(semantic_map_ng_message(&msg, &s->lastCue));
                        ++s->begins;
                    } else if (msg.operation == NG_UPDATE) {
                        assert(semantic_map_ng_message(&msg, &s->lastCue));
                        ++s->updates;
                    } else if (msg.operation == NG_STOP) {
                        ++s->stops;
                        curSession = 0;
                    }
                    uint8_t rep[16];
                    memcpy(rep, "NGA1", 4);
                    nbt::native_model::wr32(rep + 4, msg.sequence);
                    nbt::native_model::wr32(rep + 8, 0);
                    nbt::native_model::wr32(rep + 12, NG_CAP_MODEL_ID);
                    sendto(s->basicFd, rep, sizeof(rep), 0, (sockaddr*)&peer, plen);
                }
            }
        }
        if (pfds[2].revents & POLLIN) {
            uint8_t buf[SC_WIRE_BYTES + 4];
            sockaddr_un peer; socklen_t plen = sizeof(peer);
            ssize_t n = recvfrom(s->ctrlFd, buf, sizeof(buf), 0, (sockaddr*)&peer, &plen);
            if (n > 0) {
                sc_request req;
                if (sc_decode_request(buf, (size_t)n, &req)) {
                    sc_reply rep;
                    sc_machine_handle(&cm, &req, now, &cops, &rep);
                    uint8_t wire[SC_WIRE_BYTES];
                    sc_encode_reply(wire, &rep);
                    sendto(s->ctrlFd, wire, sizeof(wire), 0, (sockaddr*)&peer, plen);
                }
            }
        }
    }
    return NULL;
}

static void testEndToEndLoopback() {
    char dir[] = "/tmp/nbt-sem-lb-XXXXXX";
    assert(mkdtemp(dir) != NULL);
    std::string pBasic = std::string(dir) + "/native-guidance-4242.sock";
    std::string pAlive = std::string(dir) + "/native-guidance-alive-4242.sock";
    std::string pCtrl  = std::string(dir) + "/native-functional-cue-4243.sock";

    LoopbackServer srv;
    memset(&srv, 0, sizeof(srv));
    srv.basicFd = bindUnix(pBasic);
    srv.aliveFd = bindUnix(pAlive);
    srv.ctrlFd  = bindUnix(pCtrl);
    assert(pthread_create(&srv.th, NULL, loopbackThread, &srv) == 0);

    {
        nbt::kombinav::NativeDisplay disp(dir, true);
        assert(disp.start());
        nbt::kombinav::NavView left = makeView(7, 45.0, "Cumberland Blvd");
        assert(disp.showManeuver(left, 1.0, true, true) == nbt::kombinav::ClusterDisplay::kOk);
        for (int i = 0; i < 100 && srv.ctrlStarts == 0; ++i) usleep(20000);
        assert(srv.begins == 1 && srv.ctrlStarts == 1);
        assert(srv.lastCue.action == 2 && srv.lastCue.direction == 2 && strcmp(srv.lastCue.road, "Cumberland Blvd") == 0);

        // Next maneuver: destination arrival (requires NG_CAP_ARRIVAL negotiated!)
        nbt::kombinav::NavView arr = makeView(41, 15.0, "");
        assert(disp.showManeuver(arr, 2.0, true, false) == nbt::kombinav::ClusterDisplay::kOk);
        for (int i = 0; i < 100 && srv.updates == 0; ++i) usleep(20000);
        assert(srv.updates >= 1 && srv.lastCue.is_arrival == 1 && srv.lastCue.direction == 2);

        // Clear route -> Stop on both Basic and Controller
        std::string detail;
        disp.clear(&detail);
        for (int i = 0; i < 100 && (srv.stops == 0 || srv.ctrlStops == 0); ++i) usleep(20000);
        assert(srv.stops == 1 && srv.ctrlStops == 1);
    }

    srv.quit = 1;
    pthread_join(srv.th, NULL);
    close(srv.basicFd); close(srv.aliveFd); close(srv.ctrlFd);
    unlink(pBasic.c_str()); unlink(pAlive.c_str()); unlink(pCtrl.c_str());
    rmdir(dir);
}

int main() {
    testCueMappings();
    testHandoff();
    testControllerStateMachine();
    testEndToEndLoopback();
    printf("test_semantic_addon: all checks passed\n");
    return 0;
}
