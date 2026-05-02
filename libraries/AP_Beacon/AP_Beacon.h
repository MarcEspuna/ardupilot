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

#include "AP_Beacon_config.h"

#if AP_BEACON_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

class AP_Beacon_Backend;

class AP_Beacon
{
public:
    friend class AP_Beacon_Backend;

    AP_Beacon();

    // get singleton instance
    static AP_Beacon *get_singleton() { return _singleton; }

    // external position backend types (used by _TYPE parameter)
    enum class Type : uint8_t {
        None   = 0,
        Pozyx  = 1,
        Marvelmind = 2,
        Nooploop  = 3,
        RTLSLink = 4,
#if AP_BEACON_SITL_ENABLED
        SITL   = 10
#endif
    };

    // The AP_BeaconState structure is filled in by the backend driver
    struct BeaconState {
        uint16_t id;            // unique id of beacon
        bool     healthy;       // true if beacon is healthy
        float    distance;      // distance from vehicle to beacon (in meters)
        uint32_t distance_update_ms;    // system time of last update from this beacon
        Vector3f position;      // location of beacon as an offset from origin in NED in meters
    };

    // TDoA range-difference measurement between two anchors.
    // distance_diff is distance(anchor_id_b) - distance(anchor_id_a), in meters.
    struct TDoAState {
        uint8_t anchor_id_a;             // first anchor index
        uint8_t anchor_id_b;             // second anchor index
        bool healthy;                    // true if measurement is healthy
        float distance_diff;             // distance(anchor_b) - distance(anchor_a), in meters
        float distance_diff_err;         // 1-sigma error of distance difference, in meters
        uint32_t update_ms;              // system time of last update from this anchor pair
    };

    // initialise any available position estimators
    void init(void);

    // return true if beacon feature is enabled
    bool enabled(void) const;

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy(void) const;

    // update state of all beacons
    void update(void);

    // return origin of position estimate system in lat/lon
    bool get_origin(Location &origin_loc) const;

    // return vehicle position in NED from position estimate system's origin in meters
    bool get_vehicle_position_ned(Vector3f& pos, float& accuracy_estimate) const;

    // return the number of beacons
    uint8_t count() const;

    // methods to return beacon specific information

    // return all beacon data
    bool get_beacon_data(uint8_t beacon_instance, struct BeaconState& state) const;

    // return individual beacon's id
    uint8_t beacon_id(uint8_t beacon_instance) const;

    // return beacon health
    bool beacon_healthy(uint8_t beacon_instance) const;

    // return distance to beacon in meters
    float beacon_distance(uint8_t beacon_instance) const;

    // return NED position of beacon in meters relative to the beacon systems origin
    Vector3f beacon_position(uint8_t beacon_instance) const;

    // return last update time from beacon in milliseconds
    uint32_t beacon_last_update_ms(uint8_t beacon_instance) const;

    // return number of TDoA anchor-pair measurements
    uint8_t tdoa_count() const;

    // return TDoA data for an anchor pair measurement instance
    bool get_tdoa_data(uint8_t tdoa_instance, struct TDoAState& state) const;

#if AP_BEACON_SITL_ENABLED
    uint8_t sitl_measurement_mode() const { return (uint8_t)sitl_mode; }
    float sitl_tdoa_noise() const { return sitl_tdoa_noise_m_param; }
    float sitl_tdoa_bias() const { return sitl_tdoa_bias_m_param; }
    float sitl_tdoa_dropout_pct() const { return sitl_tdoa_dropout_pct_param; }
    float sitl_tdoa_outlier_pct() const { return sitl_tdoa_outlier_pct_param; }
    float sitl_tdoa_outlier_m() const { return sitl_tdoa_outlier_m_param; }
    int32_t sitl_tdoa_seed() const { return sitl_tdoa_seed_param; }
    float sitl_rng_noise() const { return sitl_rng_noise_m_param; }
    float sitl_rng_bias() const { return sitl_rng_bias_m_param; }
    float sitl_rng_dropout_pct() const { return sitl_rng_dropout_pct_param; }
    float sitl_rng_outlier_pct() const { return sitl_rng_outlier_pct_param; }
    float sitl_rng_outlier_m() const { return sitl_rng_outlier_m_param; }
    int32_t sitl_rng_seed() const { return sitl_rng_seed_param; }
    uint8_t sitl_position_estimate_mode() const { return constrain_int16(sitl_position_estimate_param, 0, 2); }
    uint8_t sitl_geometry() const { return sitl_geometry_param == 1 ? 1 : 0; }
#endif

    // update fence boundary array
    void update_boundary_points();

    // return fence boundary array
    const Vector2f* get_boundary_points(uint16_t& num_points) const;

    static const struct AP_Param::GroupInfo var_info[];

    // a method for vehicles to call to make onboard log messages:
    void log();

private:

    static AP_Beacon *_singleton;

    // check if device is ready
    bool device_ready(void) const;

    // find next boundary point from an array of boundary points given the current index into that array
    // returns true if a next point can be found
    //   current_index should be an index into the boundary_pts array
    //   start_angle is an angle (in radians), the search will sweep clockwise from this angle
    //   the index of the next point is returned in the next_index argument
    //   the angle to the next point is returned in the next_angle argument
    static bool get_next_boundary_point(const Vector2f* boundary, uint8_t num_points, uint8_t current_index, float start_angle, uint8_t& next_index, float& next_angle);

    // parameters
    AP_Enum<Type> _type;
    AP_Float origin_lat;
    AP_Float origin_lon;
    AP_Float origin_alt;
    AP_Int16 orient_yaw;
#if AP_BEACON_SITL_ENABLED
    AP_Int8 sitl_mode;
    AP_Float sitl_tdoa_noise_m_param;
    AP_Float sitl_tdoa_bias_m_param;
    AP_Float sitl_tdoa_dropout_pct_param;
    AP_Float sitl_tdoa_outlier_pct_param;
    AP_Float sitl_tdoa_outlier_m_param;
    AP_Int32 sitl_tdoa_seed_param;
    AP_Float sitl_rng_noise_m_param;
    AP_Float sitl_rng_bias_m_param;
    AP_Float sitl_rng_dropout_pct_param;
    AP_Float sitl_rng_outlier_pct_param;
    AP_Float sitl_rng_outlier_m_param;
    AP_Int32 sitl_rng_seed_param;
    AP_Int8 sitl_position_estimate_param;
    AP_Int8 sitl_geometry_param;
#endif

    // external references
    AP_Beacon_Backend *_driver;

    // last known position
    Vector3f veh_pos_ned;
    float veh_pos_accuracy;
    uint32_t veh_pos_update_ms;

    // individual beacon data
    uint8_t num_beacons = 0;
    BeaconState beacon_state[AP_BEACON_MAX_BEACONS];
    uint8_t num_tdoa = 0;
    TDoAState tdoa_state[AP_BEACON_MAX_TDOA_MEASUREMENTS];

    // fence boundary
    Vector2f boundary[AP_BEACON_MAX_BEACONS+1]; // array of boundary points (used for fence)
    uint8_t boundary_num_points;                // number of points in boundary
    uint8_t boundary_num_beacons;               // total number of beacon points consumed while building boundary
};

namespace AP {
    AP_Beacon *beacon();
};

#endif  // AP_BEACON_ENABLED
