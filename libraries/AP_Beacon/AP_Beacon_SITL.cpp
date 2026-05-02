/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Beacon_SITL.h"

#if AP_BEACON_SITL_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

/*
 *  Define rectangular and cube beacon patterns relative to the beacon origin as defined by the following params:
 *
 * BCN_ALT - Height above the WGS-84 geoid (m)
 * BCN_LATITUDE - WGS-84 latitude (deg)
 * BCN_LONGITUDE - WGS-84 longitude (deg)
 */
#define RECT_BEACON_SPACING_NORTH 10.0f
#define RECT_BEACON_SPACING_EAST 20.0f

// The centroid of the pattern can be moved using using the following parameters:
#define RECT_ORIGIN_OFFSET_NORTH 2.5f // shifts beacon pattern centroid North (m)
#define RECT_ORIGIN_OFFSET_EAST 5.0f // shifts beacon pattern centroid East (m)

#define CUBE_BEACON_SPACING_NORTH 20.0f
#define CUBE_BEACON_SPACING_EAST 20.0f
#define CUBE_BEACON_SPACING_DOWN 20.0f

enum class BeaconMeasurementMode : uint8_t {
    RANGE = 0,
    TDOA = 1,
};

enum class SITLPositionEstimateMode : uint8_t {
    DISABLED = 0,
    CONTINUOUS = 1,
    STARTUP_ONLY = 2,
};

enum class BeaconGeometry : uint8_t {
    RECTANGLE_4 = 0,
    CUBE_8 = 1,
};

static uint8_t num_beacons_for_geometry(BeaconGeometry geometry)
{
    switch (geometry) {
    case BeaconGeometry::CUBE_8:
        return 8;
    case BeaconGeometry::RECTANGLE_4:
    default:
        return 4;
    }
}

static Vector3f beacon_position_ned(BeaconGeometry geometry, uint8_t beacon_id)
{
    if (geometry == BeaconGeometry::CUBE_8) {
        const float north = (beacon_id & 0x01) ? -CUBE_BEACON_SPACING_NORTH * 0.5f : CUBE_BEACON_SPACING_NORTH * 0.5f;
        const float east = (beacon_id & 0x02) ? -CUBE_BEACON_SPACING_EAST * 0.5f : CUBE_BEACON_SPACING_EAST * 0.5f;
        const float down = (beacon_id & 0x04) ? CUBE_BEACON_SPACING_DOWN * 0.5f : -CUBE_BEACON_SPACING_DOWN * 0.5f;
        return Vector3f(north, east, down);
    }

    switch (beacon_id) {
    case 0:
        // NE corner
        return Vector3f(RECT_ORIGIN_OFFSET_NORTH + RECT_BEACON_SPACING_NORTH * 0.5f,
                        RECT_ORIGIN_OFFSET_EAST + RECT_BEACON_SPACING_EAST * 0.5f,
                        0.0f);
    case 1:
        // SE corner
        return Vector3f(RECT_ORIGIN_OFFSET_NORTH - RECT_BEACON_SPACING_NORTH * 0.5f,
                        RECT_ORIGIN_OFFSET_EAST + RECT_BEACON_SPACING_EAST * 0.5f,
                        0.0f);
    case 2:
        // SW corner
        return Vector3f(RECT_ORIGIN_OFFSET_NORTH - RECT_BEACON_SPACING_NORTH * 0.5f,
                        RECT_ORIGIN_OFFSET_EAST - RECT_BEACON_SPACING_EAST * 0.5f,
                        0.0f);
    case 3:
    default:
        // NW corner
        return Vector3f(RECT_ORIGIN_OFFSET_NORTH + RECT_BEACON_SPACING_NORTH * 0.5f,
                        RECT_ORIGIN_OFFSET_EAST - RECT_BEACON_SPACING_EAST * 0.5f,
                        0.0f);
    }
}

static void tdoa_pair_for_index(uint8_t pair_index, uint8_t num_beacons, uint8_t &anchor_id_a, uint8_t &anchor_id_b)
{
    uint8_t index = 0;
    for (uint8_t i = 0; i < num_beacons; i++) {
        for (uint8_t j = i + 1; j < num_beacons; j++) {
            if (index == pair_index) {
                anchor_id_a = i;
                anchor_id_b = j;
                return;
            }
            index++;
        }
    }

    anchor_id_a = 0;
    anchor_id_b = 1;
}

// constructor
AP_Beacon_SITL::AP_Beacon_SITL(AP_Beacon &frontend) :
    AP_Beacon_Backend(frontend),
    sitl(AP::sitl()),
    next_beacon(0),
    next_tdoa_pair(0),
    last_update_ms(0),
    tdoa_rng_state(1),
    rng_rng_state(1),
    last_tdoa_seed(0),
    last_rng_seed(0),
    tdoa_seed_initialized(false),
    rng_seed_initialized(false),
    startup_position_complete(false)
{
}

void AP_Beacon_SITL::reset_tdoa_rng_if_needed()
{
    const int32_t seed = get_sitl_tdoa_seed();
    if (tdoa_seed_initialized && seed == last_tdoa_seed) {
        return;
    }

    tdoa_seed_initialized = true;
    last_tdoa_seed = seed;
    tdoa_rng_state = seed == 0 ? 1U : (uint32_t)seed;
}

float AP_Beacon_SITL::tdoa_rand_float()
{
    tdoa_rng_state = 1664525U * tdoa_rng_state + 1013904223U;
    return ((tdoa_rng_state >> 8) & 0x00FFFFFFU) * (1.0f / 16777216.0f);
}

float AP_Beacon_SITL::tdoa_rand_normal()
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < 12; i++) {
        sum += tdoa_rand_float();
    }
    return sum - 6.0f;
}

void AP_Beacon_SITL::reset_rng_rng_if_needed()
{
    const int32_t seed = get_sitl_rng_seed();
    if (rng_seed_initialized && seed == last_rng_seed) {
        return;
    }

    rng_seed_initialized = true;
    last_rng_seed = seed;
    rng_rng_state = seed == 0 ? 1U : (uint32_t)seed;
}

float AP_Beacon_SITL::rng_rand_float()
{
    rng_rng_state = 1664525U * rng_rng_state + 1013904223U;
    return ((rng_rng_state >> 8) & 0x00FFFFFFU) * (1.0f / 16777216.0f);
}

float AP_Beacon_SITL::rng_rand_normal()
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < 12; i++) {
        sum += rng_rand_float();
    }
    return sum - 6.0f;
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_SITL::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_SITL::update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 10) {
        return;
    }

    const BeaconGeometry geometry = (BeaconGeometry)get_sitl_geometry();
    const uint8_t num_beacons = num_beacons_for_geometry(geometry);

    next_beacon %= num_beacons;
    uint8_t beacon_id = next_beacon;
    next_beacon++;

    // truth location of the flight vehicle
    Location current_loc;
    current_loc.lat = sitl->state.latitude * 1.0e7f;
    current_loc.lng = sitl->state.longitude * 1.0e7f;
    current_loc.alt = sitl->state.altitude * 1.0e2;

    // where the beacon system origin is located
    Location beacon_origin;
    beacon_origin.lat = get_beacon_origin_lat() * 1.0e7f;
    beacon_origin.lng = get_beacon_origin_lon() * 1.0e7f;
    beacon_origin.alt = get_beacon_origin_alt() * 1.0e2;

    const Vector2f veh_diff = beacon_origin.get_distance_NE(current_loc);

    Vector3f veh_pos3d(veh_diff.x, veh_diff.y, (beacon_origin.alt - current_loc.alt)*1.0e-2f);
    Vector3f beac_pos3d = beacon_position_ned(geometry, beacon_id);
    Vector3f beac_veh_offset = veh_pos3d - beac_pos3d;

    const BeaconMeasurementMode mode = (BeaconMeasurementMode)get_sitl_measurement_mode();
    if (mode == BeaconMeasurementMode::TDOA) {
        reset_tdoa_rng_if_needed();

        Vector3f beacon_pos[AP_BEACON_MAX_BEACONS];
        for (uint8_t i = 0; i < num_beacons; i++) {
            beacon_pos[i] = beacon_position_ned(geometry, i);
            set_beacon_position(i, beacon_pos[i]);
        }

        const uint8_t num_pairs = (num_beacons * (num_beacons - 1)) / 2;
        uint8_t anchor_id_a;
        uint8_t anchor_id_b;
        tdoa_pair_for_index(next_tdoa_pair, num_beacons, anchor_id_a, anchor_id_b);
        next_tdoa_pair = (next_tdoa_pair + 1) % num_pairs;

        const float distance_a = (veh_pos3d - beacon_pos[anchor_id_a]).length();
        const float distance_b = (veh_pos3d - beacon_pos[anchor_id_b]).length();
        const float dropout_pct = constrain_float(get_sitl_tdoa_dropout_pct(), 0.0f, 100.0f);
        if (tdoa_rand_float() * 100.0f >= dropout_pct) {
            const float noise_stddev = MAX(get_sitl_tdoa_noise(), 0.0f);
            float distance_diff = distance_b - distance_a + get_sitl_tdoa_bias();
            distance_diff += noise_stddev * tdoa_rand_normal();

            const float outlier_pct = constrain_float(get_sitl_tdoa_outlier_pct(), 0.0f, 100.0f);
            if (tdoa_rand_float() * 100.0f < outlier_pct) {
                const float outlier_sign = tdoa_rand_float() < 0.5f ? -1.0f : 1.0f;
                distance_diff += outlier_sign * MAX(get_sitl_tdoa_outlier_m(), 0.0f);
            }

            // Keep zero-noise simulator runs from implying a zero-variance EKF observation.
            // In-process backend: no transport latency, so age_ms = 0.
            set_tdoa_measurement(anchor_id_a, anchor_id_b, distance_diff, MAX(noise_stddev, 0.15f), 0);
        }
    } else {
        set_beacon_position(beacon_id, beac_pos3d);
        reset_rng_rng_if_needed();
        const float dropout_pct = constrain_float(get_sitl_rng_dropout_pct(), 0.0f, 100.0f);
        if (rng_rand_float() * 100.0f >= dropout_pct) {
            const float noise_stddev = MAX(get_sitl_rng_noise(), 0.0f);
            float distance = beac_veh_offset.length() + get_sitl_rng_bias();
            distance += noise_stddev * rng_rand_normal();

            const float outlier_pct = constrain_float(get_sitl_rng_outlier_pct(), 0.0f, 100.0f);
            if (rng_rand_float() * 100.0f < outlier_pct) {
                const float outlier_sign = rng_rand_float() < 0.5f ? -1.0f : 1.0f;
                distance += outlier_sign * MAX(get_sitl_rng_outlier_m(), 0.0f);
            }

            set_beacon_distance(beacon_id, MAX(distance, 0.0f));
        }
    }
    const SITLPositionEstimateMode position_mode = (SITLPositionEstimateMode)get_sitl_position_estimate_mode();
    bool publish_vehicle_position = position_mode == SITLPositionEstimateMode::CONTINUOUS;
    if (position_mode == SITLPositionEstimateMode::STARTUP_ONLY && !startup_position_complete) {
        const bool in_startup_altitude_band = fabsf(veh_pos3d.z) < 1.0f;
        publish_vehicle_position = in_startup_altitude_band;
        if (!in_startup_altitude_band) {
            startup_position_complete = true;
        }
    }
    if (publish_vehicle_position) {
        set_vehicle_position(veh_pos3d, 0.5f);
    }
    last_update_ms = now;
}

#endif // AP_BEACON_SITL_ENABLED
