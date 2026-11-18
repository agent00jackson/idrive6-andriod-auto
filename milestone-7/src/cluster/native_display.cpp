// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
#include "cluster/native_display.hpp"

namespace nbt {
namespace kombinav {
ClusterDisplay::Result NativeDisplay::publish(const NavView& view, bool newManeuver) {
    native_model::Message message;
    const unsigned capabilities = backend_.capabilities() & (lanes_ ? NG_CAP_TYPED : NG_CAP_TYPED_TURNS);
    if (!native_model::fromNav(view, message, capabilities)) {
        // Release this model while the unsupported maneuver is current.
        forget();
        return kOk;
    }
    if (active_ && !backend_.busy()) active_ = false;
    if (!active_) {
        if (backend_.busy()) return kOk;  // old scoped STOP is still in progress
        if (!backend_.arm(UINT64_MAX)) return kLink;
        active_ = true;
        newManeuver = true;
    }
    if (!backend_.submit(view, message, newManeuver)) return kLink;
    last_ = message;
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
    native_model::Message previous = last_;
    const unsigned advertised = backend_.capabilities();
    const unsigned capabilities = advertised & (lanes_ ? NG_CAP_TYPED : NG_CAP_TYPED_TURNS);
    if (!active_ || !backend_.busy() || !native_model::fromNav(view, candidate, capabilities) ||
        !native_model::adaptToCapabilities(previous, advertised) ||
        !native_model::samePayload(candidate, previous))
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
