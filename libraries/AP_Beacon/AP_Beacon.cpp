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

#include "AP_Beacon.h"

#if AP_BEACON_ENABLED

#include "AP_Beacon_Backend.h"
#include "AP_Beacon_Pozyx.h"
#include "AP_Beacon_Marvelmind.h"
#include "AP_Beacon_Nooploop.h"
#include "AP_Beacon_RTLSLink.h"
#include "AP_Beacon_SITL.h"

#include <AP_Common/Location.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

#ifndef AP_BEACON_MINIMUM_FENCE_BEACONS
#define AP_BEACON_MINIMUM_FENCE_BEACONS 3
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Beacon::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Beacon based position estimation device type
    // @Description: What type of beacon based position estimation device is connected
    // @Values: 0:None,1:Pozyx,2:Marvelmind,3:Nooploop,4:RTLSLink,10:SITL
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE",    0, AP_Beacon, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _LATITUDE
    // @DisplayName: Beacon origin's latitude
    // @Description: Beacon origin's latitude
    // @Units: deg
    // @Increment: 0.000001
    // @Range: -90 90
    // @User: Advanced
    AP_GROUPINFO("_LATITUDE", 1, AP_Beacon, origin_lat, 0),

    // @Param: _LONGITUDE
    // @DisplayName: Beacon origin's longitude
    // @Description: Beacon origin's longitude
    // @Units: deg
    // @Increment: 0.000001
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("_LONGITUDE", 2, AP_Beacon, origin_lon, 0),

    // @Param: _ALT
    // @DisplayName: Beacon origin's altitude above sealevel in meters
    // @Description: Beacon origin's altitude above sealevel in meters
    // @Units: m
    // @Increment: 1
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("_ALT", 3, AP_Beacon, origin_alt, 0),

    // @Param: _ORIENT_YAW
    // @DisplayName: Beacon systems rotation from north in degrees
    // @Description: Beacon systems rotation from north in degrees
    // @Units: deg
    // @Increment: 1
    // @Range: -180 +180
    // @User: Advanced
    AP_GROUPINFO("_ORIENT_YAW", 4, AP_Beacon, orient_yaw, 0),

#if AP_BEACON_SITL_ENABLED
    // @Param: _SITL_MODE
    // @DisplayName: SITL beacon measurement mode
    // @Description: Selects range or TDoA measurements for the SITL beacon backend
    // @Values: 0:Range,1:TDoA
    // @User: Advanced
    AP_GROUPINFO("_SITL_MODE", 5, AP_Beacon, sitl_mode, 0),

    // @Param: _TDOA_NOISE
    // @DisplayName: SITL TDoA noise
    // @Description: SITL TDoA range-difference Gaussian noise standard deviation. This corrupts generated TDoA samples; estimator weighting still uses the reported TDoA measurement error and EK3_BCN_M_NSE minimum variance handling.
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_TDOA_NOISE", 6, AP_Beacon, sitl_tdoa_noise_m_param, 0),

    // @Param: _TDOA_BIAS
    // @DisplayName: SITL TDoA bias
    // @Description: SITL TDoA range-difference constant bias
    // @Units: m
    // @Range: -100 100
    // @User: Advanced
    AP_GROUPINFO("_TDOA_BIAS", 7, AP_Beacon, sitl_tdoa_bias_m_param, 0),

    // @Param: _TDOA_DROPOUT
    // @DisplayName: SITL TDoA dropout rate
    // @Description: SITL TDoA measurement dropout percentage
    // @Units: %
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_TDOA_DROPOUT", 8, AP_Beacon, sitl_tdoa_dropout_pct_param, 0),

    // @Param: _TDOA_OUT_PCT
    // @DisplayName: SITL TDoA outlier rate
    // @Description: SITL TDoA outlier percentage
    // @Units: %
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_TDOA_OUT_PCT", 9, AP_Beacon, sitl_tdoa_outlier_pct_param, 0),

    // @Param: _TDOA_OUT_MAG
    // @DisplayName: SITL TDoA outlier magnitude
    // @Description: SITL TDoA absolute outlier magnitude
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_TDOA_OUT_MAG", 10, AP_Beacon, sitl_tdoa_outlier_m_param, 0),

    // @Param: _TDOA_SEED
    // @DisplayName: SITL TDoA random seed
    // @Description: SITL TDoA deterministic random seed
    // @User: Advanced
    AP_GROUPINFO("_TDOA_SEED", 11, AP_Beacon, sitl_tdoa_seed_param, 1),

    // @Param: _RNG_NOISE
    // @DisplayName: SITL range noise
    // @Description: SITL range Gaussian noise standard deviation. This corrupts generated range samples; range beacon estimator weighting comes from EK3_BCN_M_NSE.
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_RNG_NOISE", 12, AP_Beacon, sitl_rng_noise_m_param, 0),

    // @Param: _RNG_BIAS
    // @DisplayName: SITL range bias
    // @Description: SITL range constant bias
    // @Units: m
    // @Range: -100 100
    // @User: Advanced
    AP_GROUPINFO("_RNG_BIAS", 13, AP_Beacon, sitl_rng_bias_m_param, 0),

    // @Param: _RNG_DROPOUT
    // @DisplayName: SITL range dropout rate
    // @Description: SITL range measurement dropout percentage
    // @Units: %
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_RNG_DROPOUT", 14, AP_Beacon, sitl_rng_dropout_pct_param, 0),

    // @Param: _RNG_OUT_PCT
    // @DisplayName: SITL range outlier rate
    // @Description: SITL range outlier percentage
    // @Units: %
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_RNG_OUT_PCT", 15, AP_Beacon, sitl_rng_outlier_pct_param, 0),

    // @Param: _RNG_OUT_MAG
    // @DisplayName: SITL range outlier magnitude
    // @Description: SITL range absolute outlier magnitude
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_RNG_OUT_MAG", 16, AP_Beacon, sitl_rng_outlier_m_param, 0),

    // @Param: _RNG_SEED
    // @DisplayName: SITL range random seed
    // @Description: SITL range deterministic random seed
    // @User: Advanced
    AP_GROUPINFO("_RNG_SEED", 17, AP_Beacon, sitl_rng_seed_param, 1),

    // @Param: _SITL_POS
    // @DisplayName: SITL vehicle position estimate
    // @Description: Controls whether the SITL beacon backend publishes a vehicle position estimate. StartupOnly publishes near the origin altitude for initial alignment and then latches off until reboot.
    // @Values: 0:Disabled,1:Continuous,2:StartupOnly
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("_SITL_POS", 18, AP_Beacon, sitl_position_estimate_param, 1),

    // @Param: _SITL_GEOM
    // @DisplayName: SITL beacon geometry
    // @Description: Controls the SITL beacon anchor geometry. Cube8 is a 20m cube centered on the beacon origin, with anchors at +/-10m N/E/D.
    // @Values: 0:Rectangle4,1:Cube8
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("_SITL_GEOM", 19, AP_Beacon, sitl_geometry_param, 0),
#endif

    AP_GROUPEND
};

AP_Beacon::AP_Beacon()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Fence must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the AP_Beacon class
void AP_Beacon::init(void)
{
    if (_driver != nullptr) {
        // init called a 2nd time?
        return;
    }

    // create backend
    switch ((Type)_type) {
    case Type::Pozyx:
        _driver = NEW_NOTHROW AP_Beacon_Pozyx(*this);
        break;
    case Type::Marvelmind:
        _driver = NEW_NOTHROW AP_Beacon_Marvelmind(*this);
        break;
    case Type::Nooploop:
        _driver = NEW_NOTHROW AP_Beacon_Nooploop(*this);
        break;
#if AP_BEACON_RTLSLINK_ENABLED
    case Type::RTLSLink:
        _driver = NEW_NOTHROW AP_Beacon_RTLSLink(*this);
        break;
#endif
#if AP_BEACON_SITL_ENABLED
    case Type::SITL:
        _driver = NEW_NOTHROW AP_Beacon_SITL(*this);
        break;
#endif
    case Type::None:
        break;
    }
}

// return true if beacon feature is enabled
bool AP_Beacon::enabled(void) const
{
    return (_type != Type::None);
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon::healthy(void) const
{
    if (!device_ready()) {
        return false;
    }
    return _driver->healthy();
}

// update state. This should be called often from the main loop
void AP_Beacon::update(void)
{
    if (!device_ready()) {
        return;
    }
    _driver->update();

    const uint32_t now_ms = AP_HAL::millis();
    for (uint8_t i = 0; i < num_tdoa; i++) {
        auto &tdoa = tdoa_state[i];
        if (tdoa.healthy && now_ms - tdoa.update_ms > AP_BEACON_TIMEOUT_MS) {
            tdoa.healthy = false;
        }
    }

    // update boundary for fence
    update_boundary_points();
}

// return origin of position estimate system
bool AP_Beacon::get_origin(Location &origin_loc) const
{
    if (!device_ready()) {
        return false;
    }

    // check for un-initialised origin
    if (is_zero(origin_lat) && is_zero(origin_lon) && is_zero(origin_alt)) {
        return false;
    }

    // return origin
    origin_loc = {};
    origin_loc.lat = origin_lat * 1.0e7f;
    origin_loc.lng = origin_lon * 1.0e7f;
    origin_loc.alt = origin_alt * 100;

    return true;
}

// return position in NED from position estimate system's origin in meters
bool AP_Beacon::get_vehicle_position_ned(Vector3f &position, float& accuracy_estimate) const
{
    if (!device_ready()) {
        return false;
    }

    // check for timeout
    if (AP_HAL::millis() - veh_pos_update_ms > AP_BEACON_TIMEOUT_MS) {
        return false;
    }

    // return position
    position = veh_pos_ned;
    accuracy_estimate = veh_pos_accuracy;
    return true;
}

// return the number of beacons
uint8_t AP_Beacon::count() const
{
    if (!device_ready()) {
        return 0;
    }
    return num_beacons;
}

// return all beacon data
bool AP_Beacon::get_beacon_data(uint8_t beacon_instance, struct BeaconState& state) const
{
    if (!device_ready() || beacon_instance >= num_beacons) {
        return false;
    }
    state = beacon_state[beacon_instance];
    return true;
}

// return individual beacon's id
uint8_t AP_Beacon::beacon_id(uint8_t beacon_instance) const
{
    if (beacon_instance >= num_beacons) {
        return 0;
    }
    return beacon_state[beacon_instance].id;
}

// return beacon health
bool AP_Beacon::beacon_healthy(uint8_t beacon_instance) const
{
    if (beacon_instance >= num_beacons) {
        return false;
    }
    return beacon_state[beacon_instance].healthy;
}

// return distance to beacon in meters
float AP_Beacon::beacon_distance(uint8_t beacon_instance) const
{
    if ( beacon_instance >= num_beacons || !beacon_state[beacon_instance].healthy) {
        return 0.0f;
    }
    return beacon_state[beacon_instance].distance;
}

// return beacon position in meters
Vector3f AP_Beacon::beacon_position(uint8_t beacon_instance) const
{
    if (!device_ready() || beacon_instance >= num_beacons) {
        Vector3f temp = {};
        return temp;
    }
    return beacon_state[beacon_instance].position;
}

// return last update time from beacon in milliseconds
uint32_t AP_Beacon::beacon_last_update_ms(uint8_t beacon_instance) const
{
    if (_type == Type::None || beacon_instance >= num_beacons) {
        return 0;
    }
    return beacon_state[beacon_instance].distance_update_ms;
}

// return number of TDoA anchor-pair measurements
uint8_t AP_Beacon::tdoa_count() const
{
    if (!device_ready()) {
        return 0;
    }
    return num_tdoa;
}

// return TDoA data for an anchor pair measurement instance
bool AP_Beacon::get_tdoa_data(uint8_t tdoa_instance, struct TDoAState& state) const
{
    if (!device_ready() || tdoa_instance >= num_tdoa) {
        return false;
    }
    state = tdoa_state[tdoa_instance];
    return true;
}

// create fence boundary points
void AP_Beacon::update_boundary_points()
{
    // we need three beacons at least to create boundary fence.
    // update boundary fence if number of beacons changes
    if (!device_ready() || num_beacons < AP_BEACON_MINIMUM_FENCE_BEACONS || boundary_num_beacons == num_beacons) {
        return;
    }

    // record number of beacons so we do not repeat calculations
    boundary_num_beacons = num_beacons;

    // accumulate beacon points
    Vector2f beacon_points[AP_BEACON_MAX_BEACONS];
    for (uint8_t index = 0; index < num_beacons; index++) {
        const Vector3f& point_3d = beacon_position(index);
        beacon_points[index].x = point_3d.x;
        beacon_points[index].y = point_3d.y;
    }

    // create polygon around boundary points using the following algorithm
    //     set the "current point" as the first boundary point
    //     loop through all the boundary points looking for the point which creates a vector (from the current point to this new point) with the lowest angle
    //     check if point is already in boundary
    //       - no: add to boundary, move current point to this new point and repeat the above
    //       - yes: we've completed the bounding box, delete any boundary points found earlier than the duplicate

    Vector2f boundary_points[AP_BEACON_MAX_BEACONS+1];  // array of boundary points
    uint8_t curr_boundary_idx = 0;                      // index into boundary_sorted index.  always points to the highest filled in element of the array
    uint8_t curr_beacon_idx = 0;                        // index into beacon_point array.  point indexed is same point as curr_boundary_idx's

    // initialise first point of boundary_sorted with first beacon's position (this point may be removed later if it is found to not be on the outer boundary)
    boundary_points[curr_boundary_idx] = beacon_points[curr_beacon_idx];

    bool boundary_success = false;  // true once the boundary has been successfully found
    bool boundary_failure = false;  // true if we fail to build the boundary
    float start_angle = 0.0f;       // starting angle used when searching for next boundary point, on each iteration this climbs but never climbs past PI * 2
    while (!boundary_success && !boundary_failure) {
        // look for next outer point
        uint8_t next_idx;
        float next_angle;
        if (get_next_boundary_point(beacon_points, num_beacons, curr_beacon_idx, start_angle, next_idx, next_angle)) {
            // add boundary point to boundary_sorted array
            curr_boundary_idx++;
            boundary_points[curr_boundary_idx] = beacon_points[next_idx];
            curr_beacon_idx = next_idx;
            start_angle = next_angle;

            // check if we have a complete boundary by looking for duplicate points within the boundary_sorted
            uint8_t dup_idx = 0;
            bool dup_found = false;
            while (dup_idx < curr_boundary_idx && !dup_found) {
                dup_found = (boundary_points[dup_idx] == boundary_points[curr_boundary_idx]);
                if (!dup_found) {
                    dup_idx++;
                }
            }
            // if duplicate is found, remove all boundary points before the duplicate because they are inner points
            if (dup_found) {
                // note that the closing/duplicate point is not
                // included in the boundary points.
                const uint8_t num_pts = curr_boundary_idx - dup_idx;
                if (num_pts >= AP_BEACON_MINIMUM_FENCE_BEACONS) { // we consider three points to be a polygon
                    // success, copy boundary points to boundary array and convert meters to cm
                    for (uint8_t j = 0; j < num_pts; j++) {
                        boundary[j] = boundary_points[j+dup_idx] * 100.0f;
                    }
                    boundary_num_points = num_pts;
                    boundary_success = true;
                } else {
                    // boundary has too few points
                    boundary_failure = true;
                }
            }
        } else {
            // failed to create boundary - give up!
            boundary_failure = true;
        }
    }

    // clear boundary on failure
    if (boundary_failure) {
        boundary_num_points = 0;
    }
}

// find next boundary point from an array of boundary points given the current index into that array
// returns true if a next point can be found
//   current_index should be an index into the boundary_pts array
//   start_angle is an angle (in radians), the search will sweep clockwise from this angle
//   the index of the next point is returned in the next_index argument
//   the angle to the next point is returned in the next_angle argument
bool AP_Beacon::get_next_boundary_point(const Vector2f* boundary_pts, uint8_t num_points, uint8_t current_index, float start_angle, uint8_t& next_index, float& next_angle)
{
    // sanity check
    if (boundary_pts == nullptr || current_index >= num_points) {
        return false;
    }

    // get current point
    Vector2f curr_point = boundary_pts[current_index];

    // search through all points for next boundary point in a clockwise direction
    float lowest_angle = M_PI_2;
    float lowest_angle_relative = M_PI_2;
    bool lowest_found = false;
    uint8_t lowest_index = 0;
    for (uint8_t i=0; i < num_points; i++) {
        if (i != current_index) {
            Vector2f vec = boundary_pts[i] - curr_point;
            if (!vec.is_zero()) {
                float angle = wrap_2PI(atan2f(vec.y, vec.x));
                float angle_relative = wrap_2PI(angle - start_angle);
                if ((angle_relative < lowest_angle_relative) || !lowest_found) {
                    lowest_angle = angle;
                    lowest_angle_relative = angle_relative;
                    lowest_index = i;
                    lowest_found = true;
                }
            }
        }
    }

    // return results
    if (lowest_found) {
        next_index = lowest_index;
        next_angle = lowest_angle;
    }
    return lowest_found;
}

// return fence boundary array
const Vector2f* AP_Beacon::get_boundary_points(uint16_t& num_points) const
{
    if (!device_ready()) {
        num_points = 0;
        return nullptr;
    }

    num_points = boundary_num_points;
    return boundary;
}

// check if the device is ready
bool AP_Beacon::device_ready(void) const
{
    return ((_driver != nullptr) && (_type != Type::None));
}

#if HAL_LOGGING_ENABLED
// Write beacon sensor (position) data
void AP_Beacon::log()
{
    if (!enabled()) {
        return;
    }
    // position
    Vector3f pos;
    float accuracy = 0.0f;
    get_vehicle_position_ned(pos, accuracy);

    const struct log_Beacon pkt_beacon{
       LOG_PACKET_HEADER_INIT(LOG_BEACON_MSG),
       time_us         : AP_HAL::micros64(),
       health          : (uint8_t)healthy(),
       count           : (uint8_t)count(),
       dist0           : beacon_distance(0),
       dist1           : beacon_distance(1),
       dist2           : beacon_distance(2),
       dist3           : beacon_distance(3),
       posx            : pos.x,
       posy            : pos.y,
       posz            : pos.z
    };
    AP::logger().WriteBlock(&pkt_beacon, sizeof(pkt_beacon));

    for (uint8_t i = 0; i < num_tdoa; i++) {
        const auto &tdoa = tdoa_state[i];
        const struct log_BeaconTDoA pkt_tdoa{
            LOG_PACKET_HEADER_INIT(LOG_BEACON_TDOA_MSG),
            time_us            : AP_HAL::micros64(),
            anchor_id_a        : tdoa.anchor_id_a,
            anchor_id_b        : tdoa.anchor_id_b,
            distance_diff      : tdoa.distance_diff,
            distance_diff_err  : tdoa.distance_diff_err,
            age_ms             : tdoa.age_ms,
            healthy            : (uint8_t)tdoa.healthy
        };
        AP::logger().WriteBlock(&pkt_tdoa, sizeof(pkt_tdoa));
    }
}
#endif

// singleton instance
AP_Beacon *AP_Beacon::_singleton;

namespace AP {

AP_Beacon *beacon()
{
    return AP_Beacon::get_singleton();
}

}

#endif
