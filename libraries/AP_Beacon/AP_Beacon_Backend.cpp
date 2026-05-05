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

#include "AP_Beacon_Backend.h"

#if AP_BEACON_ENABLED

// debug
#include <stdio.h>
#include <AP_SerialManager/AP_SerialManager.h>

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Beacon_Backend::AP_Beacon_Backend(AP_Beacon &frontend) :
    _frontend(frontend)
{
    const AP_SerialManager &serialmanager = AP::serialmanager();
    uart = serialmanager.find_serial(AP_SerialManager::SerialProtocol_Beacon, 0);
    if (uart == nullptr) {
        return;
    }

    uart->begin(serialmanager.find_baudrate(AP_SerialManager::SerialProtocol_Beacon, 0));
}

// set vehicle position
// pos should be in meters in NED frame from the beacon's local origin
// accuracy_estimate is also in meters
void AP_Beacon_Backend::set_vehicle_position(const Vector3f& pos, float accuracy_estimate)
{
    _frontend.veh_pos_update_ms = AP_HAL::millis();
    _frontend.veh_pos_accuracy = accuracy_estimate;
    _frontend.veh_pos_ned = correct_for_orient_yaw(pos);
}

// set individual beacon distance from vehicle in meters in NED frame
void AP_Beacon_Backend::set_beacon_distance(uint8_t beacon_instance, float distance)
{
    // sanity check instance
    if (beacon_instance >= AP_BEACON_MAX_BEACONS) {
        return;
    }

    // setup new beacon
    if (beacon_instance >= _frontend.num_beacons) {
        _frontend.num_beacons = beacon_instance+1;
    }

    _frontend.beacon_state[beacon_instance].distance_update_ms = AP_HAL::millis();
    _frontend.beacon_state[beacon_instance].distance = distance;
    _frontend.beacon_state[beacon_instance].healthy = true;
}

// set beacon's position
// pos should be in meters in NED from the beacon's local origin
void AP_Beacon_Backend::set_beacon_position(uint8_t beacon_instance, const Vector3f& pos)
{
    // sanity check instance
    if (beacon_instance >= AP_BEACON_MAX_BEACONS) {
        return;
    }

    // setup new beacon
    if (beacon_instance >= _frontend.num_beacons) {
        _frontend.num_beacons = beacon_instance+1;
    }

    // set position after correcting yaw
    _frontend.beacon_state[beacon_instance].position = correct_for_orient_yaw(pos);
}

void AP_Beacon_Backend::set_beacon_origin(const Location& origin)
{
    _frontend.backend_origin = origin;
    _frontend.backend_origin_valid = true;
}

void AP_Beacon_Backend::clear_beacon_origin()
{
    _frontend.backend_origin_valid = false;
    _frontend.backend_origin = {};
}

// set a TDoA range-difference measurement between two beacons.
// Hardening contract for the age_ms field:
//   - age_ms > AP_BEACON_TDOA_MAX_AGE_MS is rejected outright.
//   - The back-stamped time is clamped at boot wraparound (age_ms > millis).
//   - A new measurement whose back-stamped time is more than
//     AP_BEACON_TDOA_BACKWARDS_TOLERANCE_MS BEFORE the previously accepted
//     measurement for the same anchor pair is rejected (per-pair monotonicity).
// Together these protect the EKF buffer recall from firmware clock glitches,
// time-travel-backwards, and stale-but-just-under-timeout samples.
bool AP_Beacon_Backend::set_tdoa_measurement(uint8_t anchor_id_a, uint8_t anchor_id_b, float distance_diff, float distance_diff_err, uint16_t age_ms)
{
    if (anchor_id_a >= AP_BEACON_MAX_BEACONS ||
        anchor_id_b >= AP_BEACON_MAX_BEACONS ||
        anchor_id_a == anchor_id_b ||
        !isfinite(distance_diff) ||
        !isfinite(distance_diff_err)) {
        return false;
    }

    // Reject measurements the firmware has already let go stale. Anything older
    // than AP_BEACON_TDOA_MAX_AGE_MS has either been queued too long inside the
    // UWB MCU or is the symptom of a clock glitch; either way it is unsafe to
    // back-stamp into the EKF delay buffer.
    if (age_ms > AP_BEACON_TDOA_MAX_AGE_MS) {
        return false;
    }

    if (anchor_id_b < anchor_id_a) {
        const uint8_t anchor_id_tmp = anchor_id_a;
        anchor_id_a = anchor_id_b;
        anchor_id_b = anchor_id_tmp;
        distance_diff = -distance_diff;
    }

    uint8_t instance = _frontend.num_tdoa;
    for (uint8_t i = 0; i < _frontend.num_tdoa; i++) {
        const auto &state = _frontend.tdoa_state[i];
        if (state.anchor_id_a == anchor_id_a && state.anchor_id_b == anchor_id_b) {
            instance = i;
            break;
        }
    }

    if (instance >= AP_BEACON_MAX_TDOA_MEASUREMENTS) {
        return false;
    }

    // back-stamp the measurement to when the UWB solver actually produced it.
    // Boot wraparound: u32 underflow is handled by clamping to 0.
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t meas_time_ms = (age_ms > now_ms) ? 0 : (now_ms - age_ms);

    // Per-pair monotonicity guard: reject a measurement that claims to be
    // older than the previous accepted one for the same pair, beyond a small
    // jitter tolerance. Cast to int32_t for wrap-safe signed comparison: the
    // difference (meas_time_ms - prev.update_ms) is interpreted as signed and
    // a strongly-negative result means time-travel-backwards.
    if (instance < _frontend.num_tdoa) {
        const auto &prev = _frontend.tdoa_state[instance];
        if (prev.healthy) {
            const int32_t time_delta = (int32_t)(meas_time_ms - prev.update_ms);
            if (time_delta < -(int32_t)AP_BEACON_TDOA_BACKWARDS_TOLERANCE_MS) {
                return false;
            }
        }
    }

    if (instance >= _frontend.num_tdoa) {
        _frontend.num_tdoa = instance + 1;
    }

    auto &state = _frontend.tdoa_state[instance];
    state.anchor_id_a = anchor_id_a;
    state.anchor_id_b = anchor_id_b;
    state.distance_diff = distance_diff;
    state.distance_diff_err = MAX(distance_diff_err, 0.0f);
    state.healthy = true;
    state.update_ms = meas_time_ms;
    state.age_ms = age_ms;
    return true;
}

// rotate vector (meters) to correct for beacon system yaw orientation
Vector3f AP_Beacon_Backend::correct_for_orient_yaw(const Vector3f &vector)
{
    // exit immediately if no correction
    if (_frontend.orient_yaw == 0) {
        return vector;
    }

    // check for change in parameter value and update constants
    if (orient_yaw_deg != _frontend.orient_yaw) {
        _frontend.orient_yaw.set(wrap_180(_frontend.orient_yaw.get()));

        // calculate rotation constants
        orient_yaw_deg = _frontend.orient_yaw;
        orient_cos_yaw = cosf(radians(orient_yaw_deg));
        orient_sin_yaw = sinf(radians(orient_yaw_deg));
    }

    // rotate x,y by -orient_yaw
    Vector3f vec_rotated;
    vec_rotated.x = vector.x*orient_cos_yaw - vector.y*orient_sin_yaw;
    vec_rotated.y = vector.x*orient_sin_yaw + vector.y*orient_cos_yaw;
    vec_rotated.z = vector.z;
    return vec_rotated;
}

#endif  // AP_BEACON_ENABLED
