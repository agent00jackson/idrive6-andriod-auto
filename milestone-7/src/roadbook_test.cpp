// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
#include "roadbook_test.hpp"
#include "roadbook_models.hpp"
#include "usb_log.hpp"

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

namespace nbt {
namespace roadbook {
namespace {

const char kArmFile[] = "/dev/shmem/nbt-aad-roadbook-test.arm";
const size_t kMaxResponse = 96 * 1024;
const double kTestSeconds = 60;

double monotonic() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

bool ready(int fd, short events, double deadline) {
    struct pollfd p = {fd, events, 0};
    for (;;) {
        double remaining = deadline - monotonic();
        if (remaining <= 0) return false;
        int rc = poll(&p, 1, static_cast<int>(remaining * 1000) + 1);
        if (rc < 0 && errno == EINTR) continue;
        return rc > 0 && (p.revents & (events | POLLHUP)) != 0;
    }
}

std::string form(const std::string& models) {
    if (models.empty()) return std::string();
    std::string result = "transData=transData&models=";
    const char* hex = "0123456789ABCDEF";
    for (std::string::const_iterator it = models.begin(); it != models.end(); ++it) {
        unsigned char c = static_cast<unsigned char>(*it);
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.') result += c;
        else {
            result += '%';
            result += hex[c >> 4];
            result += hex[c & 15];
        }
    }
    return result;
}

bool exchange(int fd, int port, const std::string& body, std::string& reply, bool& sentRequest) {
    const double deadline = monotonic() + 1.5;
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return false;
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        if (errno != EINPROGRESS || !ready(fd, POLLOUT, deadline)) return false;
        int error = 0;
        socklen_t size = sizeof(error);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &size) < 0 || error) return false;
    }

    std::string request = "POST /Roadbook HTTP/1.0\r\nHost: 127.0.0.1\r\nConnection: close\r\n"
        "Content-Type: application/x-www-form-urlencoded\r\nContent-Length: "
        + kombinav::fmtInt(body.size()) + "\r\n\r\n" + body;
    size_t sent = 0;
    while (sent < request.size() && ready(fd, POLLOUT, deadline)) {
        const ssize_t n = send(fd, request.data() + sent, request.size() - sent, 0);
        if (n > 0) sent += n;
        else if (n == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) return false;
    }
    if (sent != request.size()) return false;
    sentRequest = true;

    char buffer[4096];
    while (reply.size() < kMaxResponse && ready(fd, POLLIN, deadline)) {
        const ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
        if (n == 0) return true;
        if (n > 0) reply.append(buffer, n);
        else if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) return false;
    }
    return false;
}

bool roadbookPost(int port, const std::string& models, unsigned long& commits, bool* requestSent = NULL) {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return false;
    std::string reply;
    bool sent = false;
    const bool received = exchange(fd, port, form(models), reply, sent);
    if (requestSent) *requestSent = sent;
    close(fd);
    if (!received || (reply.compare(0, 13, "HTTP/1.1 200 ") != 0 &&
                      reply.compare(0, 13, "HTTP/1.0 200 ") != 0)) return false;
    const char tag[] = "Commit counter: ";
    const size_t tagPosition = reply.find(tag);
    if (reply.find("<h1>Roadbook interface</h1>") == std::string::npos ||
        tagPosition == std::string::npos) return false;
    const char* value = reply.c_str() + tagPosition + sizeof(tag) - 1;
    if (*value < '0' || *value > '9') return false;
    char* end = NULL;
    errno = 0;
    commits = strtoul(value, &end, 10);
    return end != value && errno == 0;
}

} // namespace

ParkedTest::ParkedTest()
    : enabled_(false), port_(0), submitted_(false), expires_(0), nextUpdate_(0), commits_(0) {}

bool ParkedTest::start() {
    const char* setting = getenv("NBT_ROADBOOK_TEST");
    if (!setting || strcmp(setting, "1") != 0) return false;
    char huVersion[96];
    nbt::huVersion(huVersion, sizeof(huVersion));
    if (strcmp(huVersion, "NBTevo_X20511I") != 0) {
        printf("[Roadbook] parked test requires X20511I firmware\n");
        return false;
    }
    // Require the operator to arm after this daemon has started.
    if (unlink(kArmFile) < 0 && errno != ENOENT) {
        printf("[Roadbook] could not remove old arm file, parked test disabled\n");
        return false;
    }
    enabled_ = true;
    printf("[Roadbook] parked test available; arm with %s\n", kArmFile);
    return true;
}

bool ParkedTest::armed() const {
    return enabled_ && (expires_ != 0 || access(kArmFile, F_OK) == 0);
}

bool ParkedTest::preflight() {
    const int ports[] = {8080, 8081};
    for (size_t i = 0; i < sizeof(ports) / sizeof(ports[0]); ++i) {
        if (roadbookPost(ports[i], std::string(), commits_)) {
            port_ = ports[i];
            return true;
        }
    }
    printf("[Roadbook] diagnostic page unavailable on head unit loopback\n");
    return false;
}

void ParkedTest::step(const kombinav::NavView* nav, bool guidanceActive, bool sessionEnded, double now) {
    if (!armed()) return;
    if ((sessionEnded && expires_) || access(kArmFile, F_OK) != 0 ||
        (expires_ && now >= expires_) || (submitted_ && !guidanceActive)) {
        stop();
        return;
    }
    if (!expires_) {
        if (guidanceActive) {
            printf("[Roadbook] arm before starting Android Auto guidance\n");
            stop();
            return;
        }
        expires_ = now + kTestSeconds;
        printf("[Roadbook] parked test armed for 60 seconds\n");
    }
    if (!nav || now < nextUpdate_) return;
    const std::string models = guidanceModels(*nav, false);
    if (models.empty()) {
        if (submitted_) stop();
        return;
    }
    if (models == lastModels_) return;
    if (!port_ && !preflight()) {
        stop();
        return;
    }
    if (access(kArmFile, F_OK) != 0) {
        stop();
        return;
    }
    const bool first = !submitted_;
    unsigned long commits = 0;
    const std::string& batch = first ? guidanceModels(*nav, true) : models;
    bool sent = false;
    const bool received = roadbookPost(port_, batch, commits, &sent);
    submitted_ = submitted_ || sent;  // A complete POST with a lost reply can leave staged data behind.
    if (!received || commits <= commits_) {
        printf("[Roadbook] did not observe a model commit, ending parked test\n");
        stop();
        return;
    }
    commits_ = commits;
    lastModels_ = models;
    nextUpdate_ = now + 0.5;
    printf("[Roadbook] parked test commit %lu, AA maneuver type %d\n", commits_, nav->aa_type);
}

void ParkedTest::rejectArm() {
    printf("[Roadbook] turn off the cluster/HUD option before arming the parked test\n");
    stop();
}

void ParkedTest::stop() {
    if (!enabled_) return;
    if (unlink(kArmFile) < 0 && errno != ENOENT) {
        printf("[Roadbook] could not disarm, parked test disabled until daemon restart\n");
        enabled_ = false;
    }
    if (submitted_) {
        unsigned long ignored = 0;
        const bool cleared = roadbookPost(port_, clearModels(), ignored);
        printf("[Roadbook] clear %s; restart iDrive to restore factory Roadbook\n",
               cleared ? "submitted" : "failed");
    }
    submitted_ = false;
    expires_ = nextUpdate_ = 0;
    port_ = 0;
    lastModels_.clear();
}

} // namespace roadbook
} // namespace nbt
