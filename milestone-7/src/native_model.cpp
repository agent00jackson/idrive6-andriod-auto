// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#include "native_model.hpp"
#include "native_model_wire/alive_wire.h"
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#if !defined(__QNXNTO__)
#include <sys/un.h>
#endif
#include <stddef.h>
#include <time.h>
#include <unistd.h>
namespace nbt {
namespace native_model {
namespace {
#if defined(__QNXNTO__)
// Same BSD/QNX local-address layout used by the native adapter SDK.
struct LocalAddress {
    uint8_t sun_len, sun_family;
    char sun_path[104];
};
#else
typedef sockaddr_un LocalAddress;
#endif
// The qnx-free SDK declares these POSIX APIs but its link manifest omits them.
// Resolve genuine libc exports; never substitute filesystem/identity results.
struct LocalApi {
    int (*lstatFn)(const char *, struct stat *);
    int (*chmodFn)(const char *, mode_t);
    int (*rmdirFn)(const char *);
    uid_t (*geteuidFn)();
    LocalApi() : lstatFn(NULL), chmodFn(NULL), rmdirFn(NULL), geteuidFn(NULL) {
#if defined(__QNXNTO__)
        void *h = dlopen("libc.so.3", RTLD_NOW | RTLD_LOCAL);
#else
        void *h = RTLD_DEFAULT;
#endif
        if (!h && h != RTLD_DEFAULT)
            return;
        lstatFn = reinterpret_cast<int (*)(const char *, struct stat *)>(dlsym(h, "lstat"));
        chmodFn = reinterpret_cast<int (*)(const char *, mode_t)>(dlsym(h, "chmod"));
        rmdirFn = reinterpret_cast<int (*)(const char *)>(dlsym(h, "rmdir"));
        geteuidFn = reinterpret_cast<uid_t (*)()>(dlsym(h, "geteuid"));
    }
    bool ready() const { return lstatFn && chmodFn && rmdirFn && geteuidFn; }
};
LocalApi &api() {
    static LocalApi value;
    return value;
}
uint64_t millis() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return uint64_t(t.tv_sec) * 1000 + t.tv_nsec / 1000000;
}
uint64_t nonce() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t n = (uint64_t(t.tv_sec) << 32) ^ uint64_t(t.tv_nsec) ^ (uint64_t(getpid()) << 16);
    return n ? n : 1;
}
bool address(const std::string &path, LocalAddress &a) {
    if (path.empty() || path.size() >= sizeof(a.sun_path))
        return false;
    memset(&a, 0, sizeof(a));
    a.sun_family = AF_UNIX;
#if defined(__QNXNTO__) || defined(__APPLE__)
    a.sun_len = sizeof(a);
#endif
    memcpy(a.sun_path, path.c_str(), path.size() + 1);
    return true;
}
bool endpoint(const std::string &p) {
    struct stat s;
    return !api().lstatFn(p.c_str(), &s) && (s.st_mode & S_IFMT) == S_IFSOCK && (s.st_mode & 0777) == 0600 &&
           s.st_uid == api().geteuidFn();
}
bool nameValid(const char *n) {
    const char *prefix = "native-guidance-";
    size_t k = strlen(prefix);
    if (strncmp(n, prefix, k))
        return false;
    const char *p = n + k;
    if (*p < '1' || *p > '9')
        return false;
    while (*p >= '0' && *p <= '9')
        ++p;
    return !strcmp(p, ".sock");
}
} // namespace
Backend::Backend(const char *d)
    : started_(false), closing_(false), armed_(false), cancel_(false), pending_(false), busy_(false), failed_(false),
      expiry_(0), generation_(0), haveNav_(false), maneuver_(0), directory_(d), fd_(-1), aliveFd_(-1),
      aliveDevice_(0), aliveInode_(0), heartbeatSequence_(0), sequence_(0), session_(0),
      attempted_(false) {
    pthread_mutex_init(&mutex_, NULL);
}
Backend::~Backend() {
    shutdown();
    pthread_mutex_destroy(&mutex_);
}
bool Backend::start() {
    if (started_)
        return true;
    if (!api().ready()) {
        printf("[Native model] native POSIX APIs unavailable\n");
        return false;
    }
    if (pthread_create(&thread_, NULL, entry, this))
        return false;
    started_ = true;
    return true;
}
bool Backend::arm(uint64_t expires) {
    pthread_mutex_lock(&mutex_);
    bool ok = started_ && !closing_ && !failed_ && !busy_ && !armed_ && expires > millis();
    if (ok) {
        armed_ = true;
        busy_ = true;
        cancel_ = false;
        pending_ = false;
        expiry_ = expires;
        ++generation_;
        haveNav_ = false;
        maneuver_ = 0;
    }
    pthread_mutex_unlock(&mutex_);
    return ok;
}
bool Backend::submit(const kombinav::NavView &v, bool forceNew) {
    Message m;
    if (!fromNav(v, m))
        return false;
    pthread_mutex_lock(&mutex_);
    bool ok = armed_ && !cancel_ && !closing_ && !failed_ && millis() < expiry_;
    if (ok) {
        if (forceNew || !haveNav_ || !v.sameManeuver(lastNav_)) {
            if (++maneuver_ == 0)
                ++maneuver_;
        }
        m.maneuver = maneuver_;
        latest_ = m;
        pending_ = true;
        lastNav_ = v;
        haveNav_ = true;
    }
    pthread_mutex_unlock(&mutex_);
    return ok;
}
void Backend::stop() {
    pthread_mutex_lock(&mutex_);
    cancel_ = true;
    armed_ = false;
    pending_ = false;
    pthread_mutex_unlock(&mutex_);
}
bool Backend::busy() {
    pthread_mutex_lock(&mutex_);
    bool b = busy_;
    pthread_mutex_unlock(&mutex_);
    return b;
}
bool Backend::failed() {
    pthread_mutex_lock(&mutex_);
    bool failed = failed_;
    pthread_mutex_unlock(&mutex_);
    return failed;
}
bool Backend::shutdown() {
    if (!started_)
        return true;
    pthread_mutex_lock(&mutex_);
    closing_ = true;
    cancel_ = true;
    armed_ = false;
    pthread_mutex_unlock(&mutex_);
    bool ok = !pthread_join(thread_, NULL);
    started_ = false;
    return ok;
}
bool Backend::revoked() {
    pthread_mutex_lock(&mutex_);
    bool b = cancel_ || closing_ || !armed_ || millis() >= expiry_;
    pthread_mutex_unlock(&mutex_);
    return b;
}
void Backend::publish(bool busy, bool failed) {
    pthread_mutex_lock(&mutex_);
    busy_ = busy;
    failed_ = failed_ || failed;
    pthread_mutex_unlock(&mutex_);
}
void *Backend::entry(void *p) {
    static_cast<Backend *>(p)->run();
    return NULL;
}
void Backend::closeSocket() {
    if (aliveFd_ >= 0) close(aliveFd_);
    aliveFd_ = -1;
    struct stat found;
    if (!alivePath_.empty() && !api().lstatFn(alivePath_.c_str(), &found) &&
        uint64_t(found.st_dev) == aliveDevice_ && uint64_t(found.st_ino) == aliveInode_)
        unlink(alivePath_.c_str());
    alivePath_.clear();
    heartbeatSequence_ = 0;
    if (fd_ >= 0)
        close(fd_);
    fd_ = -1;
    if (!clientPath_.empty())
        unlink(clientPath_.c_str());
    if (!clientDirectory_.empty())
        api().rmdirFn(clientDirectory_.c_str());
    clientPath_.clear();
    clientDirectory_.clear();
}
bool Backend::exchange(Message &m) {
    if (fd_ < 0 || sequence_ == 0xffffffffU)
        return false;
    m.sequence = ++sequence_;
    uint8_t bytes[kFrameBytes];
    if (!encode(m, bytes))
        return false;
    if (send(fd_, bytes, sizeof(bytes), 0) != ssize_t(sizeof(bytes)))
        return false;
    // Bound each wait by the corresponding Full owner-queue budget.
    const unsigned budget = m.operation == Probe ? 17000 : m.operation == Begin ? 6000 : 4000;
    uint64_t deadline = millis() + budget;
    while (millis() < deadline) {
        if (m.operation != Stop && revoked()) return false;
        uint8_t reply[64];
        ssize_t n = recv(fd_, reply, sizeof(reply), 0);
        if (n >= 0) {
            if (n >= 8 && rd32(reply + 4) != m.sequence)
                continue;
            return replyAccepted(reply, size_t(n), m.sequence);
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            return false;
        timespec pause = {0, 10000000};
        nanosleep(&pause, NULL);
    }
    return false;
}
bool Backend::discover() {
    closeSocket();
    char dir[256];
    snprintf(dir, sizeof(dir), "%s/aa-native-model-%ld-%llx", directory_.c_str(), long(getpid()),
             (unsigned long long)nonce());
    if (mkdir(dir, 0700))
        return false;
    clientDirectory_ = dir;
    clientPath_ = clientDirectory_ + "/client.sock";
    fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    LocalAddress a;
    if (fd_ < 0 || !address(clientPath_, a) || fcntl(fd_, F_SETFL, O_NONBLOCK) < 0 ||
        fcntl(fd_, F_SETFD, FD_CLOEXEC) < 0 || bind(fd_, reinterpret_cast<sockaddr *>(&a), sizeof(a)) ||
        api().chmodFn(clientPath_.c_str(), 0600)) {
        closeSocket();
        return false;
    }
    DIR *d = opendir(directory_.c_str());
    if (!d) {
        closeSocket();
        return false;
    }
    bool found = false;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (!nameValid(ent->d_name))
            continue;
        std::string p = directory_ + "/" + ent->d_name;
        if (!endpoint(p) || !address(p, a) || connect(fd_, reinterpret_cast<sockaddr *>(&a), sizeof(a)))
            continue;
        Message probe;
        probe.operation = Probe;
        if (exchange(probe) && endpoint(p)) {
            found = connectLiveness(std::string(ent->d_name + strlen("native-guidance-")));
            break;
        }
    }
    closedir(d);
    if (!found)
        closeSocket();
    return found;
}
bool Backend::connectLiveness(const std::string& suffix) {
    std::string peer = directory_ + "/" NG_ALIVE_ENDPOINT_PREFIX + suffix;
    struct stat state;
    if (api().lstatFn(peer.c_str(), &state)) return errno == ENOENT;
    if (!endpoint(peer)) return false;

    char endpointName[64];
    snprintf(endpointName, sizeof(endpointName), NG_ALIVE_CLIENT_PREFIX "%ld.sock", long(getpid()));
    std::string path = directory_ + "/" + endpointName;
    LocalAddress local, remote;
    if (!address(path, local) || !address(peer, remote)) return false;
    aliveFd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (aliveFd_ < 0 || fcntl(aliveFd_, F_SETFL, O_NONBLOCK) < 0 ||
        fcntl(aliveFd_, F_SETFD, FD_CLOEXEC) < 0 ||
        bind(aliveFd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) ||
        api().lstatFn(path.c_str(), &state) || (state.st_mode & S_IFMT) != S_IFSOCK)
        return false;
    alivePath_ = path;
    aliveDevice_ = uint64_t(state.st_dev);
    aliveInode_ = uint64_t(state.st_ino);
    return !api().chmodFn(path.c_str(), 0600) && endpoint(path) &&
           !connect(aliveFd_, reinterpret_cast<sockaddr*>(&remote), sizeof(remote)) && endpoint(peer);
}

bool Backend::heartbeat(uint32_t semanticSequence) {
    if (aliveFd_ < 0) return true; // old adapters refresh through NG_UPDATE
    if (heartbeatSequence_ == UINT32_MAX || revoked()) return false;
    const uint64_t start = millis();
    ng_alive_message message = {session_, semanticSequence, ++heartbeatSequence_, start};
    uint8_t wire[NG_ALIVE_BYTES];
    ng_alive_encode(wire, &message);
    if (!start || send(aliveFd_, wire, sizeof(wire), 0) != ssize_t(sizeof(wire))) return false;
    while (!revoked() && millis() - start < 1000) {
        uint8_t reply[NG_ALIVE_ACK_BYTES + 1];
        ssize_t n = recv(aliveFd_, reply, sizeof(reply), 0);
        if (n >= 0) {
            ng_alive_ack ack;
            return ng_alive_ack_decode(reply, size_t(n), &ack) && ng_alive_ack_matches(&message, &ack);
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return false;
        const timespec pause = {0,10000000};
        nanosleep(&pause, NULL);
    }
    return false;
}

void Backend::clearOwned() {
    bool confirmed = true;
    if (attempted_) {
        Message stop;
        stop.operation = Stop;
        stop.session = session_;
        confirmed = exchange(stop);
        printf("[Native model] owned clear %s (adapter ACK; HUD unqualified)\n",
               confirmed ? "acknowledged" : "unconfirmed");
    }
    attempted_ = false;
    closeSocket();
    publish(false, false);
}
void Backend::run() {
    uint64_t seen = 0, nextUpdate = 0, nextHeartbeat = 0;
    uint32_t accepted = 0;
    Message active;
    bool dirty = false;
    for (;;) {
        pthread_mutex_lock(&mutex_);
        bool closing = closing_, armed = armed_, cancel = cancel_;
        uint64_t gen = generation_, expiry = expiry_;
        bool pending = pending_;
        Message m = latest_;
        if (pending)
            pending_ = false;
        pthread_mutex_unlock(&mutex_);
        if (closing || cancel || !armed || millis() >= expiry) {
            if (busy())
                clearOwned();
            dirty = false;
            if (closing)
                break;
        } else {
            if (gen != seen) {
                seen = gen;
                session_ = nonce();
                sequence_ = 0;
                accepted = 0;
                dirty = false;
                nextUpdate = nextHeartbeat = 0;
                if (!discover()) {
                    printf("[Native model] no ready private adapter; trial failed\n");
                    if (!revoked()) publish(true, true);
                    stop();
                    clearOwned();
                    continue;
                }
            }
            if (pending) {
                active = m;
                dirty = true;
            }
            const uint64_t now = millis();
            const bool update = (dirty || (accepted && aliveFd_ < 0)) && now >= nextUpdate;
            const bool keepAlive = accepted && aliveFd_ >= 0 && now >= nextHeartbeat;
            bool ok = true;
            if (update && !revoked()) {
                active.operation = attempted_ ? Update : Begin;
                active.session = session_;
                // A lost ACK can leave an owned model; always attempt its scoped clear.
                attempted_ = true;
                publish(true, false);
                ok = (!accepted || heartbeat(accepted)) && exchange(active);
                if (ok) {
                    accepted = active.sequence;
                    heartbeatSequence_ = 0;
                    ok = heartbeat(accepted);
                }
                if (ok) {
                    dirty = false;
                    printf("[Native model] AA model accepted distance_mm=%u lanes=%u (HUD unqualified)\n",
                           active.distanceMm, unsigned(active.laneCount));
                    nextUpdate = nextHeartbeat = millis() + 2000;
                }
            } else if (keepAlive && !revoked()) {
                ok = heartbeat(accepted);
                nextHeartbeat = millis() + 2000;
            }
            if (!ok) {
                if (!revoked()) {
                    printf("[Native model] producer receipt unavailable; retiring owned model\n");
                    publish(true, true);
                }
                stop();
                clearOwned();
                dirty = false;
            }
        }
        timespec pause = {0, 20000000};
        nanosleep(&pause, NULL);
    }
    closeSocket();
}
} // namespace native_model
} // namespace nbt
