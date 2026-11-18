# BMW native guidance integration

Choose `NBT_CLUSTER=native` with the normal cluster arrows setting. This
selects the local semantic publisher. The native controller does not share
the J29 or 6WA transport, and the existing display presets still select
their corresponding drivers.

The backend takes the upstream channel-10 view and its approach, repeated
maneuver, reroute and disconnect policy. It publishes straight, left and right
turns with distance to the separately loaded Full adapter. A per-session
capability query enables additional typed maneuvers and lanes when the
adapter advertises them. E114 verified left and right arrows, a distance refresh, and clear
on the original BMW emulator software. The ARM Android Auto integration has
a host replay and still needs an end-to-end BMW trial. Straight is covered
at the data boundary and still needs visible 6WB verification.

The Full socket acknowledges an accepted native model. The Controller
selects the matching view for the 6WB. E114 accepted two explicitly armed
cues in one boot. The experimental `NBT_NATIVE_ROUTE_SESSION=1` setting
requires a separate Controller endpoint advertising the route-session
protocol and a Full liveness endpoint. In that mode the AA worker arms
Controller after Full accepts the model, maintains the two leases, and
requests scoped Controller retirement before Full STOP. A completed
Controller view can rearm after an authoritative Full idle probe. The
mock sockets and the Controller owner fixtures cover this ordering.
The paired BMW runtime and a compatible car loader still need qualification.

The selected Full adapter can accept live session heartbeats on its companion
socket. Both sockets are local, mode 0600 and source-bound by the same
endpoint PID. Heartbeats keep an unchanged model's semantic sequence stable.
Older Full adapters without the companion retain the timed update fallback.
Updates use the same session until an owned STOP or expiry. A following
maneuver advances the maneuver ID and can publish immediately. Repeated
distance-only updates retain their existing cadence. Factory-route detection
retires this publisher's own model. Main thread or transport failures are
handled by the Full lease expiry.

The expanded contract adds slight turns, sharp turns and U-turns in either
direction. With lanes enabled, it includes the directions and individual
recommendations for each lane. A lane update can reuse the same maneuver ID
without waiting for a distance change. Older adapters receive basic turns
without lane data. Unsupported maneuvers retire the current model.
BMW scanout for the expanded inputs still needs qualification. Road names
and roundabout exit data still need their own OEM input contracts.
The updated workspace navigation plan tracks each outstanding display test.

The QNX installation tree produced by CMake is an integration artifact.
It contains no Full or Controller activation package. The QNX SDK stub
extensions are link interfaces for verified runtime exports and are excluded
from install.
