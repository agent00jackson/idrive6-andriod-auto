// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#ifndef NBT_NATIVE_MODEL_HPP
#define NBT_NATIVE_MODEL_HPP
#include "native_model_protocol.hpp"
#include <pthread.h>
#include <string>
namespace nbt {
namespace native_model {
// Opt-in local producer. Native operations and ACK waits run only on this thread.
class Backend {
  public:
    explicit Backend(const char *directory = NG_SOCKET_ROOT);
    ~Backend();
    bool start();
    bool arm(uint64_t expiresMs);
    bool submit(const kombinav::NavView&, Message, bool forceNew);
    unsigned capabilities();
    void stop();
    void guidanceEnded(bool) { stop(); }
    void observeFactoryRoute() { stop(); }
    bool busy();
    bool failed();
    bool shutdown();

  private:
    static void *entry(void *);
    void run();
    bool discover();
    unsigned queryCapabilities();
    bool exchange(Message &);
    void closeSocket();
    bool connectLiveness(const std::string& suffix);
    bool heartbeat(uint32_t semanticSequence);
    void clearOwned();
    bool revoked();
    void publish(bool busy, bool failed);
    pthread_mutex_t mutex_;
    pthread_t thread_;
    bool started_, closing_, armed_, cancel_, pending_, busy_, failed_;
    uint64_t expiry_, generation_;
    Message latest_;
    kombinav::NavView lastNav_;
    bool haveNav_;
    uint32_t maneuver_;
    unsigned capabilities_;
    std::string directory_, clientDirectory_, clientPath_;
    int fd_, aliveFd_;
    std::string alivePath_;
    uint64_t aliveDevice_, aliveInode_;
    uint32_t heartbeatSequence_;
    uint32_t sequence_;
    uint32_t fullPid_;
    uint64_t session_;
    bool attempted_;
};
} // namespace native_model
} // namespace nbt
#endif
