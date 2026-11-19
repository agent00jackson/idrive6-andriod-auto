// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#include "kombi_nav_test_util.hpp"
#include "cluster/native_display.hpp"
#include "cluster/nav_core.hpp"
#include "native_controller_protocol.hpp"
#include "native_model_wire/alive_wire.h"
#include <poll.h>
#include <assert.h>
#include <atomic>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>
using namespace nbt::native_model;
using namespace nbt::kombinav;
using namespace kntest;
static NavView phone(int type, const std::vector<TLane> &lanes, int64_t metres) {
    NavMessages m;
    Bytes b = status(1);
    m.store(0x8003, b.data(), b.size());
    b = navStateLanes(type, "OWN AA ROAD", lanes);
    m.store(0x8006, b.data(), b.size());
    b = curPos(metres, "150", 6);
    m.store(0x8007, b.data(), b.size());
    assert(m.guidanceActive());
    NavView v;
    assert(viewFromMessages(m, v));
    return v;
}
int main(int argc, char** argv) {
    const bool typed = argc == 2 && !strcmp(argv[1], "--typed");
    const bool controllerPreflight = argc == 2 && !strcmp(argv[1], "--controller-preflight");
    const bool routeSession = typed || controllerPreflight || (argc == 2 && !strcmp(argv[1], "--controller"));
    if (routeSession) assert(!setenv("NBT_NATIVE_ROUTE_SESSION", "1", 1));
    else assert(!unsetenv("NBT_NATIVE_ROUTE_SESSION"));
    std::vector<TLane> lanes;
    lanes.push_back(lane(4, false));
    lanes.push_back(lane2(1, false, 5, true));
    NavView v = phone(8, std::vector<TLane>(), 46);
    Message m;
    assert(fromNav(v, m));
    assert(m.distanceMm == 46000 && m.turn == 3 && m.direction == 19200 && m.laneCount == 0);
    uint32_t selectedTurn=77; uint16_t selectedDirection=55;
    for(int type=-1;type<=256;++type) {
        const bool roundabout=(type>=32&&type<=35)||(type>=43&&type<=46);
        const bool offRampOrFork=type>=21&&type<=26;
        const bool arrival=type>=39&&type<=42;
        if((type<2||type>12)&&type!=36&&!roundabout&&!offRampOrFork&&!arrival){
            assert(!turnDirection(type,selectedTurn,selectedDirection));
            assert(selectedTurn==77&&selectedDirection==55);
        }
    }
    Message rejected;
    assert(fromNav(phone(7, std::vector<TLane>(), 46), rejected));
    assert(rejected.turn == 2 && rejected.direction == 6400);
    for(int type=0;type<=256;++type)if(type!=2&&type!=7&&type!=8&&type!=36)
        assert(!fromNav(phone(type, std::vector<TLane>(), 46), rejected)); // no implicit straight/depart/unknown mapping
    assert(fromNav(phone(2, {}, 46), rejected));
    assert(rejected.turn == NG_STRAIGHT && rejected.direction == 0);
    assert(fromNav(phone(2, {}, 46), rejected, NG_CAP_SUPPORTED));
    assert(rejected.aaType == 2 && rejected.roadLength == 11);
    const uint16_t offRampAngles[] = {3200, 22400, 6400, 19200};
    for (int type = 21; type <= 24; ++type) {
        const NavView upcomingExit = phone(type, {}, 46);
        Message exit;
        assert(fromNav(upcomingExit, exit, NG_CAP_SUPPORTED));
        assert(exit.aaType == type && exit.direction == offRampAngles[type - 21]);
        assert(!adaptToCapabilities(exit, NG_CAP_TYPED | NG_CAP_EXTENDED_FIELDS));
        assert(!fromNav(upcomingExit, rejected, NG_CAP_SUPPORTED & ~NG_CAP_EXTENDED_FIELDS));
        assert(!fromNav(upcomingExit, rejected, NG_CAP_SUPPORTED & ~NG_CAP_OFF_RAMPS));
    }
    for (int type = 25; type <= 26; ++type) {
        const NavView upcomingFork = phone(type, {}, 46);
        const bool left = type == 25;
        Message fork, decoded;
        assert(fromNav(upcomingFork, fork, NG_CAP_SUPPORTED));
        assert(fork.aaType == type && fork.turn == (left ? NG_LEFT : NG_RIGHT) &&
               fork.direction == (left ? 6400 : 19200));
        assert(!adaptToCapabilities(fork, NG_CAP_SUPPORTED & ~NG_CAP_FORKS));
        assert(!fromNav(upcomingFork, rejected, NG_CAP_SUPPORTED & ~NG_CAP_EXTENDED_FIELDS));
        assert(!fromNav(upcomingFork, rejected, NG_CAP_SUPPORTED & ~NG_CAP_FORKS));
        fork.operation=Begin;fork.sequence=1;fork.session=1;fork.maneuver=1;
        uint8_t wire[kMaxFrameBytes];
        assert(encode(fork,wire) && decode(wire,frameSize(fork),decoded) && samePayload(fork,decoded));
    }
    const uint32_t arrivalTurns[] = {NG_STRAIGHT, NG_STRAIGHT, NG_LEFT, NG_RIGHT};
    const uint16_t arrivalAngles[] = {0, 0, 6400, 19200};
    for (int type = 39; type <= 42; ++type) {
        const NavView destination = phone(type, {}, 0);
        Message arrival, decoded;
        assert(fromNav(destination, arrival, NG_CAP_SUPPORTED));
        assert(arrival.aaType == type && arrival.turn == arrivalTurns[type - 39] &&
               arrival.direction == arrivalAngles[type - 39] && arrival.distanceMm == 0);
        assert(!adaptToCapabilities(arrival, NG_CAP_BASE));
        assert(!fromNav(destination, rejected, NG_CAP_BASE));
        assert(!fromNav(destination, rejected, NG_CAP_SUPPORTED & ~NG_CAP_EXTENDED_FIELDS));
        arrival.operation=Begin;arrival.sequence=1;arrival.session=1;arrival.maneuver=1;
        uint8_t wire[kMaxFrameBytes];
        assert(encode(arrival,wire) && decode(wire,frameSize(arrival),decoded) && samePayload(arrival,decoded));
    }
    assert(fromNav(phone(8, lanes, 46), rejected) && rejected.laneCount == 0);
    Message keep, slight;
    for (int type = 3; type <= 4; ++type) {
        assert(fromNav(phone(type, {}, 46), keep, NG_CAP_SUPPORTED));
        assert(fromNav(phone(type + 2, {}, 46), slight, NG_CAP_SUPPORTED));
        assert(keep.aaType == type && keep.turn == slight.turn && keep.direction == slight.direction);
    }
    assert(fromNav(phone(11, {lane2(8,false,9,true)}, 46), rejected, NG_CAP_TYPED));
    assert(rejected.turn == NG_UTURN && rejected.direction == 6400 && rejected.laneMode == 2);
    assert(rejected.lanes[0].arrows == (NG_LANE_UTURN_LEFT|NG_LANE_UTURN_RIGHT));
    assert(rejected.lanes[0].recommended == NG_LANE_UTURN_RIGHT);
    NavView roundabout=phone(35,std::vector<TLane>(),46);
    roundabout.has_exit_number=true;roundabout.exit_number=3;
    roundabout.has_exit_angle=true;roundabout.exit_angle=180;
    Message extended;
    assert(fromNav(roundabout,extended,NG_CAP_SUPPORTED));
    assert(extended.aaType==35&&extended.turn==NG_LEFT&&extended.direction==6400&&
           extended.metadataFlags==(NG_META_AA_TYPE|NG_META_ROAD|NG_META_EXIT_NUMBER|NG_META_EXIT_ANGLE)&&
           extended.exitNumber==3&&extended.exitAngle==180&&extended.roadLength==11&&
           !memcmp(extended.road,"OWN AA ROAD",11));
    extended.operation=Begin;extended.sequence=1;extended.session=1;extended.maneuver=1;
    uint8_t extendedWire[kMaxFrameBytes];
    assert(encode(extended,extendedWire)&&frameSize(extended)==NG_EXT_FRAME_BYTES);
    ng_message extendedDecoded;
    assert(ng_decode(extendedWire,frameSize(extended),&extendedDecoded)==NG_OK&&
           extendedDecoded.aa_type==35&&extendedDecoded.exit_number==3&&extendedDecoded.exit_angle==180&&
           extendedDecoded.road_length==11);
    assert(!fromNav(roundabout,rejected,NG_CAP_TYPED));
    // The manual display's blank name does not create a native street record.
    NavView named = v;
    named.road = " ";
    Message road;
    assert(fromNav(named, road, NG_CAP_SUPPORTED));
    assert(road.metadataFlags == NG_META_AA_TYPE && road.roadLength == 0);
    // Cap road bytes on a complete character even when Core's code point
    // limit leaves a longer UTF-8 name.
    named.road.clear();
    for (int i = 0; i < 22; ++i)
        named.road += "\xe5\xb1\xb1";
    named.road.insert(0, "A");
    assert(fromNav(named, road, NG_CAP_SUPPORTED));
    assert(road.metadataFlags == (NG_META_AA_TYPE | NG_META_ROAD));
    assert(road.roadLength == 61 && !memcmp(road.road, named.road.data(), road.roadLength));
    named.road += "\xff";
    assert(fromNav(named, road, NG_CAP_SUPPORTED));
    assert(road.metadataFlags == NG_META_AA_TYPE && road.roadLength == 0);
    // Codec preserves the native literal lane DTO independently of production qualification.
    m.laneCount = 2;
    m.lanes[0].arrows = 2;
    m.lanes[1].arrows = 40;
    m.lanes[1].recommended = 1;
    m.operation = Begin;
    m.sequence = 17;
    m.session = 0x102030405060708ULL;
    m.maneuver = 1;
    uint8_t b[kFrameBytes];
    assert(encode(m, b));
    // Decode with the EXACT native adapter's C codec, not a mirrored test parser.
    ng_message n;
    assert(ng_decode(b, sizeof(b), &n) == NG_OK && n.distance_mm == 46000 && n.direction_centunits == 19200 &&
           n.session == m.session);
    uint8_t ack[kReplyBytes] = {'N', 'G', 'A', '1'};
    wr32(ack + 4, 17);
    wr32(ack + 8, 0);
    wr32(ack + 12, kModelId);
    assert(replyAccepted(ack, sizeof(ack), 17));
    assert(!replyAccepted(ack, sizeof(ack), 18));
    assert(!replyAccepted(ack, sizeof(ack) - 1, 17));
    wr32(ack + 8, NG_BACKEND_REJECTED);
    assert(!replyAccepted(ack, sizeof(ack), 17));
    wr32(ack + 8, 0);
    wr32(ack + 12, kModelId + 1);
    assert(!replyAccepted(ack, sizeof(ack), 17));
    uint16_t savedDirection = m.direction;
    m.direction = 25600;
    assert(!encode(m, b));
    m.direction = savedDirection;
    assert(encode(m, b));
    Message back;
    assert(decode(b, sizeof(b), back) && back.lanes[1].arrows == 40);
    b[4] = 2;
    assert(!decode(b, sizeof(b), back));
    b[4] = 1;
    b[79] = 1;
    assert(!decode(b, sizeof(b), back));
    b[79] = 0;
    b[43] = 1;
    assert(!decode(b, sizeof(b), back));
    b[43] = 0;
    assert(!decode(b, sizeof(b) - 1, back));
    assert(!fromNav(phone(0, std::vector<TLane>(), 46), m));
    assert(!fromNav(phone(8, std::vector<TLane>(), -1), m));
    assert(!fromNav(phone(8, std::vector<TLane>(), 2000001), m));
    v = phone(8, std::vector<TLane>(), 0);
    assert(fromNav(v, m) && m.distanceMm == 0);
    v.has_step_m = false;
    v.dist_text = "150";
    v.aa_unit = 6;
    assert(fromNav(v, m) && m.distanceMm == 45720);
    v.dist_text = "nan";
    assert(!fromNav(v, m));
    v.dist_text = "150x";
    assert(!fromNav(v, m));
    v = phone(8, std::vector<TLane>(9, lane(1, true)), 46);
    assert(fromNav(v, m) && m.laneCount == 0);
    v = phone(8, std::vector<TLane>(1, lane(0, true)), 46);
    assert(fromNav(v, m) && m.laneCount == 0);
    if(argc==2&&!strcmp(argv[1],"--fields-only"))return 0;
    // Real local0600 datagrams: probe→begin→heartbeat/update→owned stop, with no DSI.
    char dir[] = "/private/tmp/aa-model-test-XXXXXX";
    assert(mkdtemp(dir));
    {
        // The native adapter can appear after the Android Auto daemon starts.
        Backend lateAdapter(dir);
        assert(lateAdapter.start() && lateAdapter.arm(UINT64_MAX));
        NavView left = phone(7, {}, 46);
        Message first;
        assert(fromNav(left, first) && lateAdapter.submit(left, first, true));
        for (int i = 0; i < 200 && lateAdapter.busy(); ++i)
            usleep(10000);
        assert(lateAdapter.failed() && !lateAdapter.busy());
        assert(lateAdapter.arm(UINT64_MAX));
        lateAdapter.stop();
    }
    std::string path = std::string(dir) + "/native-guidance-123.sock";
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    assert(fd >= 0);
    sockaddr_un a;
    memset(&a, 0, sizeof(a));
    a.sun_family = AF_UNIX;
    strcpy(a.sun_path, path.c_str());
    if (bind(fd, (sockaddr *)&a, sizeof(a))) {
        perror("test bind");
        return 2;
    }
    assert(!chmod(path.c_str(), 0600));
    std::string alivePath = std::string(dir) + "/native-guidance-alive-123.sock";
    int alive = socket(AF_UNIX, SOCK_DGRAM, 0);
    assert(alive >= 0);
    strcpy(a.sun_path, alivePath.c_str());
    assert(!bind(alive, (sockaddr*)&a, sizeof(a)) && !chmod(alivePath.c_str(), 0600));
    const std::string controllerPath = std::string(dir) + "/native-functional-cue-456.sock";
    const int control = routeSession ? socket(AF_UNIX, SOCK_DGRAM, 0) : -1;
    if (routeSession) {
        assert(control >= 0);
        strcpy(a.sun_path, controllerPath.c_str());
        assert(!bind(control, (sockaddr*)&a, sizeof(a)) && !chmod(controllerPath.c_str(), 0600));
    }
    std::atomic<unsigned> begins(0), updates(0), stops(0), probes(0), beats(0);
    std::atomic<unsigned> controllerArms(0), controllerStops(0), controllerRearms(0), controllerPulses(0);
    std::atomic<uint32_t> preflightFlags(0);
    std::atomic<bool> closerTurn(false), followingTurn(false), repeatedTurn(false), lateLanes(false);
    std::atomic<bool> done(false);
    std::thread server([&] {
        uint64_t currentSession = 0;
        uint32_t initialManeuver = 0;
        uint32_t latestSequence = 0;
        uint32_t followingManeuver = 0;
        uint32_t controllerPhase = controller::kIdle;
        uint32_t controllerProducer = 0;
        uint64_t controllerNonce = 99, controllerEpoch = 0;
        while (!done) {
            pollfd fds[] = {{fd, POLLIN, 0}, {alive, POLLIN, 0}, {control, POLLIN, 0}};
            if (poll(fds, 3, 100) <= 0) continue;
            bool heartbeat = (fds[1].revents & POLLIN) != 0;
            bool routeCommand = (fds[2].revents & POLLIN) != 0;
            int ready = routeCommand ? control : heartbeat ? alive : fd;
            uint8_t data[256];
            sockaddr_un peer;
            socklen_t len = sizeof(peer);
            ssize_t z = recvfrom(ready, data, sizeof(data), 0, (sockaddr *)&peer, &len);
            if (z <= 0)
                continue;
            if (routeCommand) {
                using namespace controller;
                assert(z == kBytes && !memcmp(data, "NCT1", 4));
                uint32_t op = rd32(data + 8), commandSequence = rd32(data + 12);
                assert(read64(data + 48));
                if (op == kRearm) {
                    assert(controllerPhase == kEmptyReturned && !currentSession);
                    assert(commandSequence == UINT32_MAX && read64(data + 16) == controllerNonce);
                    assert(read64(data + 24) == controllerEpoch && !rd32(data + 32) && !read64(data + 40));
                    ++controllerRearms;
                    ++controllerNonce;
                    controllerEpoch = 0;
                    controllerProducer = 0;
                    controllerPhase = kIdle;
                } else if (op != kProbe) {
                    assert(read64(data + 16) == controllerNonce && rd32(data + 32) == 123);
                    assert(read64(data + 40) == currentSession);
                    const uint32_t producer = rd32(data + 36);
                    if (op == kStop) {
                        // Full can commit an in-flight update after the AA stop.
                        assert(producer >= controllerProducer && producer <= latestSequence);
                    } else {
                        assert(producer == latestSequence);
                    }
                    controllerProducer = producer;
                    if (op == kArm) {
                        assert(controllerPhase == kIdle && currentSession);
                        controllerEpoch = read64(data + 24);
                        assert(controllerEpoch);
                        ++controllerArms;
                        controllerPhase = kArmed;
                    } else {
                        assert(read64(data + 24) == controllerEpoch);
                        if (op == kStop) {
                            controllerPhase = kEmptyReturned;
                            ++controllerStops;
                        } else {
                            assert(op == kHeartbeat && controllerPhase != kIdle);
                            ++controllerPulses;
                            controllerPhase = kSelected;
                        }
                    }
                }
                uint8_t reply[kBytes] = {};
                memcpy(reply, "NCS1", 4);
                reply[4] = 1;
                reply[6] = kBytes;
                wr32(reply + 8, controllerPhase);
                write64(reply + 16, controllerNonce);
                write64(reply + 24, controllerEpoch);
                wr32(reply + 32, commandSequence);
                uint32_t flags = kRunningReadyOwner | kRouteSession;
                if (op == kProbe) flags |= preflightFlags.load();
                if (controllerPhase == kEmptyReturned) flags |= kCompletedRearm;
                wr32(reply + 36, flags);
                write64(reply + 40, read64(data + 48) + 60000);
                wr32(reply + 56, kModelId);
                wr32(reply + 60, 3201);
                assert(sendto(control, reply, sizeof(reply), 0, (sockaddr*)&peer, len) == sizeof(reply));
                continue;
            }
            if (heartbeat) {
                ng_alive_message beat;
                assert(ng_alive_decode(data, z, &beat) == NG_ALIVE_ACCEPTED);
                assert(beat.session == currentSession && beat.semantic_sequence == latestSequence);
                ng_alive_ack ack = {beat, NG_ALIVE_ACCEPTED, NG_ALIVE_ONLY, beat.created_ms};
                uint8_t reply[NG_ALIVE_ACK_BYTES];
                ng_alive_ack_encode(reply, &ack);
                assert(sendto(alive, reply, sizeof(reply), 0, (sockaddr*)&peer, len) == sizeof(reply));
                ++beats;
                continue;
            }
            if (const auto query = ng_cap_query(data, size_t(z))) {
                uint8_t reply[NG_CAP_BYTES];
                if (typed) {
                    ng_cap_response(reply, query, NG_CAP_SUPPORTED|0x100);
                } else {
                    memcpy(reply, "NGA1", 4);
                    wr32(reply+4,0);
                    wr32(reply+8,NG_BAD_FRAME);
                    wr32(reply+12,kModelId);
                }
                assert(sendto(fd, reply, sizeof(reply), 0, (sockaddr*)&peer, len) == sizeof(reply));
                continue;
            }
            ng_message x;
            assert(ng_decode(data, z, &x) == NG_OK);
            if(x.operation==NG_BEGIN||x.operation==NG_UPDATE) {
                assert((x.turn==NG_LEFT&&x.direction_centunits==6400)||(x.turn==NG_RIGHT&&x.direction_centunits==19200)||
                       (typed&&x.turn==NG_UTURN&&x.direction_centunits==6400));
                assert(typed || (!x.lane_count && !x.lane_mode));
            }
            if (x.operation == NG_BEGIN) {
                assert(!currentSession && x.session && x.maneuver);
                currentSession = x.session;
                initialManeuver = x.maneuver;
                latestSequence = x.sequence;
                if (typed) {
                    assert(x.lane_mode == 2 && x.lane_count == 2 &&
                           x.lanes[1].recommended == NG_LANE_RIGHT);
                    assert(x.metadata_flags == (NG_META_AA_TYPE | NG_META_ROAD) &&
                           x.aa_type == 8 && x.road_length == 11 &&
                           !memcmp(x.road, "OWN AA ROAD", 11));
                } else {
                    assert(z == NG_FRAME_BYTES && !x.metadata_flags);
                }
                ++begins;
            } else if (x.operation == NG_UPDATE) {
                assert(x.session == currentSession && x.sequence > latestSequence);
                latestSequence = x.sequence;
                if (typed && x.distance_mm == 46000) {
                    assert(x.maneuver == initialManeuver && x.lane_count == 2 &&
                           x.lanes[1].recommended == NG_LANE_UTURN_RIGHT);
                    lateLanes = true;
                }
                if (x.distance_mm == 35000) {
                    assert(x.maneuver == initialManeuver && x.turn == NG_RIGHT);
                    closerTurn = true;
                }
                if (x.distance_mm == 28000) {
                    assert(x.maneuver != initialManeuver && x.turn == (typed ? NG_UTURN : NG_LEFT) && closerTurn);
                    followingManeuver = x.maneuver;
                    followingTurn = true;
                }
                if (x.distance_mm == 900000) {
                    assert(followingTurn && x.maneuver > followingManeuver && x.turn==(typed ? NG_UTURN : NG_LEFT));
                    repeatedTurn = true;
                }
                ++updates;
            } else if (x.operation == NG_STOP) {
                assert(x.session == currentSession && x.sequence > latestSequence);
                if (routeSession) assert(controllerStops == stops + 1);
                currentSession = 0;
                ++stops;
            } else if (x.operation == NG_PROBE) {
                ++probes;
            }
            uint8_t ack[16];
            memcpy(ack, "NGA1", 4);
            wr32(ack + 4, x.sequence);
            wr32(ack + 8, 0);
            wr32(ack + 12, kModelId);
            assert(sendto(fd, ack, sizeof(ack), 0, (sockaddr *)&peer, len) == 16);
            if (x.operation == NG_STOP && stops == 2)
                break;
        }
    });
    if (controllerPreflight) {
        using namespace controller;
        for (uint32_t flags : {kPreflightReset, kPreflightReset | kPreflightFresh}) {
            preflightFlags = flags;
            Backend refusedController(dir, true);
            assert(refusedController.start() && refusedController.arm(UINT64_MAX));
            NavView first = phone(8, lanes, 46);
            Message message;
            assert(fromNav(first, message) && refusedController.submit(first, message, true));
            for (int i=0; i<200 && refusedController.busy(); ++i) usleep(10000);
            assert(refusedController.failed() && !refusedController.busy());
            assert(begins == 0 && controllerArms == 0);
            refusedController.stop();
        }
        preflightFlags = kPreflightReset | kPreflightFresh | kPreflightEligible;
    }
    {
        NativeDisplay display(dir);
        assert(display.start());
        Settings settings;
        settings.approach = kApproachOff;
        Core core(display, settings);
        auto first = phone(8, lanes, 46);
        core.step(&first, 1.0);
        for (int i=0; i<200 && !beats; ++i) usleep(10000);
        assert(probes == (controllerPreflight ? 3 : 1) && begins == 1 && beats > 0);
        // The separate Full lease must keep the initial semantic version stable.
        for (int i=0; i<600 && beats < 3; ++i) usleep(10000);
        assert(beats >= 3 && updates == 0);
        if (typed) {
            auto changed = phone(8, {lane(4,false),lane2(1,false,9,true)}, 46);
            core.step(&changed, 1.5);
            for (int i=0; i<350 && !lateLanes; ++i) usleep(10000);
            assert(lateLanes);
        }
        auto closer = phone(8, lanes, 35);
        closer.dist_text = "115";
        core.step(&closer, 2.0);
        for (int i=0; i<350 && !closerTurn; ++i) usleep(10000);
        assert(closerTurn);
        auto following = phone(typed ? 11 : 7, {}, 28);
        core.step(&following, 3.0);
        for (int i=0; i<350 && !followingTurn; ++i) usleep(10000);
        assert(followingTurn && updates >= 2);
        auto nextSame = following;
        nextSame.step_m = 900;
        nextSame.dist_text = "0.6";
        nextSame.aa_unit = 4;
        ++nextSame.step_seq;
        core.step(&nextSame, 4.0);
        for (int i=0; i<350 && !repeatedTurn; ++i) usleep(10000);
        assert(repeatedTurn);
        core.step(NULL, 5.0, "reroute gap");
        assert(stops == 0);
        core.step(NULL, 14.0, "guidance expired");
        for (int i=0; i<300 && !stops; ++i) usleep(10000);
        assert(stops == 1);
        core.step(&first, 15.0);
        for (int i=0; i<200 && begins < 2; ++i) {
            usleep(10000);
            core.step(&first, 15.0 + (i+1)*0.01);
        }
        assert(begins == 2);
        if (routeSession) {
            for (int i=0; i<200 && controllerArms < 2; ++i) usleep(10000);
            assert(controllerArms == 2);
        }
        core.yieldToFactory();
        for (int i=0; i<200 && stops < 2; ++i) usleep(10000);
        assert(stops == 2);
        if (routeSession) assert(controllerArms == 2 && controllerStops == 2 &&
                                 controllerRearms == 1 && controllerPulses >= 2);
    }
    done = true;
    server.join();
    close(fd);
    close(alive);
    unlink(alivePath.c_str());
    if (routeSession) {
        close(control);
        unlink(controllerPath.c_str());
    }
    unlink(path.c_str());
    assert(!rmdir(dir));
    puts("PASS upstream policy and AA decoder to native semantic, lease and owned clear sockets");
}
