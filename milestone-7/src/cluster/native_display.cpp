// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#include "cluster/native_display.hpp"

namespace nbt {
namespace kombinav {
ClusterDisplay::Result NativeDisplay::publish(const NavView& view, bool newManeuver) {
    native_model::Message message;
    if (!native_model::fromNav(view, message)) {
        // Release this model while the unsupported maneuver is current.
        forget();
        return kOk;
    }
    if (!active_) {
        if (backend_.busy()) return kOk;  // old scoped STOP is still in progress
        if (!backend_.arm(UINT64_MAX)) return kLink;
        active_ = true;
        newManeuver = true;
    }
    if (!backend_.submit(view, newManeuver)) return kLink;
    lastDistanceMm_ = message.distanceMm;
    return kOk;
}

ClusterDisplay::Result NativeDisplay::showManeuver(const NavView& view, double, bool isNew, bool) {
    return publish(view, isNew);
}

ClusterDisplay::Result NativeDisplay::updateDistance(const NavView& view, double) {
    return publish(view, false);
}

ClusterDisplay::Result NativeDisplay::keepUp(const NavView& view, double) {
    if (backend_.failed()) return kLink;
    native_model::Message candidate;
    if (!active_ || (native_model::fromNav(view, candidate) && candidate.distanceMm != lastDistanceMm_))
        return publish(view, false);
    return kOk;
}

bool NativeDisplay::clear(std::string* detail) {
    const bool retiring = active_ || backend_.busy();
    forget();
    if (retiring && detail) *detail = "native worker owns STOP and its acknowledgment";
    return !retiring;
}

void NativeDisplay::forget() {
    if (!active_) return;
    active_ = false;
    backend_.stop();
}
} // namespace kombinav
} // namespace nbt
