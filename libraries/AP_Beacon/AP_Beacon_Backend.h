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
#pragma once

#include "AP_Beacon.h"

#if AP_BEACON_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

class AP_Beacon_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_Beacon_Backend(AP_Beacon &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    virtual bool healthy() = 0;

    // update
    virtual void update() = 0;

    // set vehicle position
    // pos should be in meters in NED frame from the beacon's local origin
    // accuracy_estimate is also in meters
    void set_vehicle_position(const Vector3f& pos, float accuracy_estimate);

    // set individual beacon distance from vehicle in meters in NED frame
    void set_beacon_distance(uint8_t beacon_instance, float distance);

    // set beacon's position
    // pos should be in meters in NED from the beacon's local origin
    void set_beacon_position(uint8_t beacon_instance, const Vector3f& pos);

    // set a TDoA range-difference measurement between two beacons.
    // distance_diff is distance(anchor_id_b) - distance(anchor_id_a), in meters.
    // anchor positions must be populated with set_beacon_position(), even for TDoA-only backends.
    // anchor order is normalized internally, with distance_diff sign adjusted to match.
    // age_ms is the time elapsed between the physical TDoA solve and this call;
    // backends with no transport latency (in-process SITL) pass 0.
    void set_tdoa_measurement(uint8_t anchor_id_a, uint8_t anchor_id_b, float distance_diff, float distance_diff_err, uint16_t age_ms);

    float get_beacon_origin_lat(void) const { return _frontend.origin_lat; }
    float get_beacon_origin_lon(void) const { return _frontend.origin_lon; }
    float get_beacon_origin_alt(void) const { return _frontend.origin_alt; }
#if AP_BEACON_SITL_ENABLED
    uint8_t get_sitl_measurement_mode(void) const { return _frontend.sitl_measurement_mode(); }
    float get_sitl_tdoa_noise(void) const { return _frontend.sitl_tdoa_noise(); }
    float get_sitl_tdoa_bias(void) const { return _frontend.sitl_tdoa_bias(); }
    float get_sitl_tdoa_dropout_pct(void) const { return _frontend.sitl_tdoa_dropout_pct(); }
    float get_sitl_tdoa_outlier_pct(void) const { return _frontend.sitl_tdoa_outlier_pct(); }
    float get_sitl_tdoa_outlier_m(void) const { return _frontend.sitl_tdoa_outlier_m(); }
    int32_t get_sitl_tdoa_seed(void) const { return _frontend.sitl_tdoa_seed(); }
    float get_sitl_rng_noise(void) const { return _frontend.sitl_rng_noise(); }
    float get_sitl_rng_bias(void) const { return _frontend.sitl_rng_bias(); }
    float get_sitl_rng_dropout_pct(void) const { return _frontend.sitl_rng_dropout_pct(); }
    float get_sitl_rng_outlier_pct(void) const { return _frontend.sitl_rng_outlier_pct(); }
    float get_sitl_rng_outlier_m(void) const { return _frontend.sitl_rng_outlier_m(); }
    int32_t get_sitl_rng_seed(void) const { return _frontend.sitl_rng_seed(); }
    uint8_t get_sitl_position_estimate_mode(void) const { return _frontend.sitl_position_estimate_mode(); }
    uint8_t get_sitl_geometry(void) const { return _frontend.sitl_geometry(); }
#endif

protected:

    // references
    AP_Beacon &_frontend;

    // yaw correction
    int16_t orient_yaw_deg; // cached version of orient_yaw parameter
    float orient_cos_yaw = 0.0f;
    float orient_sin_yaw = 1.0f;

    // yaw correction methods
    Vector3f correct_for_orient_yaw(const Vector3f &vector);

    AP_HAL::UARTDriver *uart;
};

#endif  // AP_BEACON_ENABLED
