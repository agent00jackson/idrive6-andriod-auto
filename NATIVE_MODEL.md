# BMW native guidance integration

Choose `NBT_CLUSTER=native` with the normal cluster arrows setting. This
selects the local semantic publisher. The native controller does not share
the J29 or 6WA transport, and the existing display presets still select
their corresponding drivers.

The backend takes the upstream channel-10 view and its approach, repeated
maneuver, reroute and disconnect policy. It publishes typed straight, normal
left and normal right turns with distance to the separately loaded Full
adapter. E114 verified left and right arrows, a distance refresh, and clear
on the original BMW emulator software. The ARM Android Auto integration has
a host replay and still needs an end-to-end BMW trial. Straight is covered
at the data boundary and still needs visible 6WB verification.

The Full socket acknowledges an accepted native model. The separately
qualified Controller path is responsible for selecting it for the 6WB. The
E114 Controller accepts two explicitly armed trials in a boot. A continuous
route-mode Controller and compatible car loader are required for a car test.

The selected Full adapter can accept live session heartbeats on its companion
socket. Both sockets are local, mode 0600 and source-bound by the same
endpoint PID. Updates use the same session until an owned STOP or expiry.
A following maneuver advances the maneuver ID. Factory-route detection
retires this publisher's own model. Main thread or transport failures are
handled by the Full lease expiry.

The upstream parser still carries lanes, road names and roundabout exit
data. Native rendering of those fields requires a larger semantic contract
and source-qualified OEM behavior. Supported basic turns still work when
the phone supplies lanes. Unsupported maneuvers retire the current model.
The updated workspace navigation plan tracks each outstanding display test.

The QNX installation tree produced by CMake is an integration artifact.
It contains no Full or Controller activation package. The QNX SDK stub
extensions are link interfaces for verified runtime exports and are excluded
from install.
