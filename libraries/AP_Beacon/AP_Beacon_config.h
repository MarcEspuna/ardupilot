#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_BEACON_ENABLED
#define AP_BEACON_ENABLED 1
#endif

#ifndef AP_BEACON_MAX_BEACONS
#define AP_BEACON_MAX_BEACONS 8
#endif

#ifndef AP_BEACON_MAX_TDOA_MEASUREMENTS
#define AP_BEACON_MAX_TDOA_MEASUREMENTS ((AP_BEACON_MAX_BEACONS * (AP_BEACON_MAX_BEACONS - 1)) / 2)
#endif

#ifndef AP_BEACON_TIMEOUT_MS
#define AP_BEACON_TIMEOUT_MS 300
#endif

// Maximum age_ms a TDoA backend may report before the measurement is rejected
// at the driver boundary. Anything older than this is treated as stale on
// arrival; the value is intentionally tighter than AP_BEACON_TIMEOUT_MS so the
// fresh-but-poisoned case (e.g. age_ms = 295 ms) cannot reach the EKF.
#ifndef AP_BEACON_TDOA_MAX_AGE_MS
#define AP_BEACON_TDOA_MAX_AGE_MS 200
#endif

// Tolerance for non-monotonic per-pair TDoA timestamps. A measurement whose
// back-stamped time is more than this many ms BEFORE the previously accepted
// measurement for the same anchor pair is rejected — we treat that as a
// firmware clock glitch rather than a real solve.
#ifndef AP_BEACON_TDOA_BACKWARDS_TOLERANCE_MS
#define AP_BEACON_TDOA_BACKWARDS_TOLERANCE_MS 5
#endif

#ifndef AP_BEACON_BACKEND_DEFAULT_ENABLED
#define AP_BEACON_BACKEND_DEFAULT_ENABLED AP_BEACON_ENABLED
#endif

#ifndef AP_BEACON_MARVELMIND_ENABLED
#define AP_BEACON_MARVELMIND_ENABLED AP_BEACON_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BEACON_NOOPLOOP_ENABLED
#define AP_BEACON_NOOPLOOP_ENABLED AP_BEACON_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BEACON_POZYX_ENABLED
#define AP_BEACON_POZYX_ENABLED AP_BEACON_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BEACON_RTLSLINK_ENABLED
#define AP_BEACON_RTLSLINK_ENABLED AP_BEACON_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_BEACON_SITL_ENABLED
#define AP_BEACON_SITL_ENABLED (AP_BEACON_BACKEND_DEFAULT_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
