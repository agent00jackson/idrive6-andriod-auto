// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#include "kombi_nav_test_util.hpp"
#include "cluster/native_display.hpp"
#include "cluster/nav_core.hpp"
#include "native_model_wire/alive_wire.h"
#include <poll.h>
#include <assert.h>
#include <atomic>
#include <stdio.h>
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
int main() {
    std::vector<TLane> lanes;
    lanes.push_back(lane(4, false));
    lanes.push_back(lane2(1, false, 5, true));
    NavView v = phone(8, std::vector<TLane>(), 46);
    Message m;
    assert(fromNav(v, m));
    assert(m.distanceMm == 46000 && m.turn == 3 && m.direction == 19200 && m.laneCount == 0);
    uint32_t selectedTurn=77; uint16_t selectedDirection=55;
    for(int type=-1;type<=256;++type)if(type!=7&&type!=8&&type!=36){
        assert(!turnDirection(type,selectedTurn,selectedDirection));
        assert(selectedTurn==77&&selectedDirection==55);
    }
    Message rejected;
    assert(fromNav(phone(7, std::vector<TLane>(), 46), rejected));
    assert(rejected.turn == 2 && rejected.direction == 6400);
    for(int type=0;type<=256;++type)if(type!=7&&type!=8&&type!=36)
        assert(!fromNav(phone(type, std::vector<TLane>(), 46), rejected)); // no implicit straight/depart/unknown mapping
    assert(fromNav(phone(8, lanes, 46), rejected) && rejected.laneCount == 0);
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
    // Real local0600 datagrams: probe→begin→heartbeat/update→owned stop, with no DSI.
    char dir[] = "/private/tmp/aa-model-test-XXXXXX";
    assert(mkdtemp(dir));
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
    std::atomic<unsigned> begins(0), updates(0), stops(0), probes(0), beats(0);
    std::atomic<bool> closerTurn(false), followingTurn(false), repeatedTurn(false);
    std::atomic<bool> done(false);
    std::thread server([&] {
        uint64_t currentSession = 0;
        uint32_t initialManeuver = 0;
        uint32_t latestSequence = 0;
        uint32_t followingManeuver = 0;
        while (!done) {
            pollfd fds[] = {{fd, POLLIN, 0}, {alive, POLLIN, 0}};
            if (poll(fds, 2, 100) <= 0) continue;
            bool heartbeat = (fds[1].revents & POLLIN) != 0;
            int ready = heartbeat ? alive : fd;
            uint8_t data[256];
            sockaddr_un peer;
            socklen_t len = sizeof(peer);
            ssize_t z = recvfrom(ready, data, sizeof(data), 0, (sockaddr *)&peer, &len);
            if (z <= 0)
                continue;
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
            ng_message x;
            assert(ng_decode(data, z, &x) == NG_OK);
            if(x.operation==NG_BEGIN||x.operation==NG_UPDATE)
                assert((x.turn==NG_LEFT&&x.direction_centunits==6400)||(x.turn==NG_RIGHT&&x.direction_centunits==19200));
            if (x.operation == NG_BEGIN) {
                assert(!currentSession && x.session && x.maneuver);
                currentSession = x.session;
                initialManeuver = x.maneuver;
                latestSequence = x.sequence;
                ++begins;
            } else if (x.operation == NG_UPDATE) {
                assert(x.session == currentSession && x.sequence > latestSequence);
                latestSequence = x.sequence;
                if (x.distance_mm == 35000) {
                    assert(x.maneuver == initialManeuver && x.turn == NG_RIGHT);
                    closerTurn = true;
                }
                if (x.distance_mm == 28000) {
                    assert(x.maneuver != initialManeuver && x.turn == NG_LEFT && closerTurn);
                    followingManeuver = x.maneuver;
                    followingTurn = true;
                }
                if (x.distance_mm == 900000) {
                    assert(followingTurn && x.maneuver > followingManeuver && x.turn==NG_LEFT);
                    repeatedTurn = true;
                }
                ++updates;
            } else if (x.operation == NG_STOP) {
                assert(x.session == currentSession && x.sequence > latestSequence);
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
    {
        NativeDisplay display(dir);
        assert(display.start());
        Settings settings;
        settings.approach = kApproachOff;
        Core core(display, settings);
        auto first = phone(8, lanes, 46);
        core.step(&first, 1.0);
        for (int i=0; i<200 && !beats; ++i) usleep(10000);
        assert(probes == 1 && begins == 1 && beats > 0);
        auto closer = phone(8, lanes, 35);
        closer.dist_text = "115";
        core.step(&closer, 2.0);
        for (int i=0; i<350 && !closerTurn; ++i) usleep(10000);
        assert(closerTurn);
        auto following = phone(7, {}, 28);
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
        core.yieldToFactory();
        for (int i=0; i<200 && stops < 2; ++i) usleep(10000);
        assert(stops == 2);
    }
    done = true;
    server.join();
    close(fd);
    close(alive);
    unlink(alivePath.c_str());
    unlink(path.c_str());
    rmdir(dir);
    puts("PASS upstream policy and AA decoder to native semantic, lease and owned clear sockets");
}
