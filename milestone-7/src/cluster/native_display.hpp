// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#ifndef CLUSTER_NATIVE_DISPLAY_HPP
#define CLUSTER_NATIVE_DISPLAY_HPP

#include "cluster/display.hpp"
#include "native_model.hpp"

namespace nbt {
namespace kombinav {
class NativeDisplay final : public ClusterDisplay {
public:
    explicit NativeDisplay(const char* directory = NG_SOCKET_ROOT) : backend_(directory) {}
    bool start() { return backend_.start(); }
    const char* name() const override { return "BMW native guidance"; }
    void setSettings(const Settings&) override {}
    void setLog(LogFn, void*) override {}
    bool sameManeuver(const NavView&, double) override { return false; }
    Result showManeuver(const NavView&, double, bool isNew, bool first) override;
    Result updateDistance(const NavView&, double) override;
    Result keepUp(const NavView&, double) override;
    void guidanceGap() override {}
    bool clear(std::string*) override;
    void forget() override;
    bool sentAny() const override { return active_; }

private:
    Result publish(const NavView&, bool newManeuver);
    native_model::Backend backend_;
    bool active_ = false;
    uint32_t lastDistanceMm_ = 0;
};
} // namespace kombinav
} // namespace nbt
#endif
