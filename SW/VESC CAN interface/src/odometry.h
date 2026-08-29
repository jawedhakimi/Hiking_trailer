#pragma once
//
// odometry.h — trip distance prefers the VESC's own tachometer (CAN Status
// 5), with Status 1 ERPM integration as an automatic fallback when Status 5
// is not being broadcast.
//

#include <stdint.h>

namespace Odometry {

// Call once per loop(). Status 5 gives the most accurate result; Status 1
// keeps distance working when Status 5 is disabled or becomes stale.
void update();

float tripDistanceM();
void resetTrip();
bool usingTachometer();
bool hasDistanceSource();

} // namespace Odometry
