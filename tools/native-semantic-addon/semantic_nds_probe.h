// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
/* Live OEM NDS map accessor bridge for NavigationBasic (exact X20511I ARM QNX).
 * Reads car GPS from CRouteGuidance::onCarPositionUpdate (TCarPosition) and
 * queries the car's live decrypted NDS database through CPsiAdasAccessor
 * (IAdasAccessor secondary vtable 0x0105d120). Safe no-op on mapless systems. */
#ifndef NBT_SEMANTIC_NDS_PROBE_H
#define NBT_SEMANTIC_NDS_PROBE_H

#include <stdint.h>

#include "semantic_nds_map.h"

#ifdef __cplusplus
extern "C" {
#endif

struct semantic_car_gps {
    double lon_deg;
    double lat_deg;
    float heading_deg;
    uint64_t timestamp_ms;
    int valid;
};

/* Extract live WGS84 lon/lat/heading from NavigationBasic TCarPosition*. */
void semantic_nds_capture_car_pos(const void *tcarpos_ptr,
                                  uint64_t now_ms,
                                  struct semantic_car_gps *out);

/* Query local decrypted NDS road links within `radius_m` of `gps` into `out_graph`.
 * Returns number of links loaded (>= 0), or 0 if no NDS map / IAdasAccessor is live. */
int semantic_nds_query_local_graph(int mem_fd,
                                   const struct semantic_car_gps *gps,
                                   float radius_m,
                                   struct snds_graph *out_graph);

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_NDS_PROBE_H */
