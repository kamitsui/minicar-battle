/*
 * SteeringController.cpp
 *
 * Pure Pursuit on the GapFinder target.
 *
 * Formula: steering = atan2(2 * L * sin(alpha), Ld)
 *   L:     wheelbase (mm)
 *   alpha: angle to the target point (rad)
 *   Ld:    lookahead distance (mm) = front sensor distance - LOOKAHEAD_OFFSET_MM
 *
 * Side-wall repel (Config.h WALL_REPEL_*): in straight mode, steering away
 * from a near side wall is added after Pure Pursuit.
 */

#include "SteeringController.h"

#include <math.h>

SteeringController::SteeringController() : _lastWallRepel(0.0f) {}

void SteeringController::begin() {
    // Pure Pursuit is stateless; nothing to initialize.
}

float SteeringController::_wallRepel(const SensorData* sensorData) const {
    float left_push = 0.0f;   // 0..1, closest left-side wall
    float right_push = 0.0f;  // 0..1, closest right-side wall

    for (int i = 0; i < NUM_SENSORS; ++i) {
        float start_mm = WALL_REPEL_START_MM[i];
        if (start_mm <= 0.0f) continue;

        const SensorData& s = sensorData[i];
        float d;
        if (s.valid) {
            d = s.distance;
        } else if (s.status == VL53L1X::RangeValid &&
                   s.distance < MIN_VALID_DISTANCE) {
            d = 0.0f;  // closer than the valid range: treat as contact
        } else {
            continue;  // failed or disabled reading
        }
        if (d >= start_mm) continue;

        float push = (start_mm - d) / start_mm;
        if (SENSOR_ANGLES[i] < 0.0f) {
            left_push = fmaxf(left_push, push);
        } else {
            right_push = fmaxf(right_push, push);
        }
    }

    // Left wall near -> steer right (+); right wall near -> steer left (-).
    return (left_push - right_push) * WALL_REPEL_MAX_DEG;
}

float SteeringController::calculate(const GapResult& gap, const SensorData* sensorData) {
    // Treat GapFinder's output as the polar coordinates of the target point.
    // alpha: angle to the target.
    // Ld:    lookahead distance, derived from the front sensor.

    float alpha_deg = gap.target_angle;

    // Subtract the offset from the front sensor distance to get Ld.
    float Ld_mm = sensorData[FRONT_SENSOR_INDEX].valid
                  ? sensorData[FRONT_SENSOR_INDEX].distance - LOOKAHEAD_OFFSET_MM
                  : 1000.0f;  // fallback when the front sensor is invalid

    // Avoid divide-by-zero / extreme values.
    if (Ld_mm < 50.0f) Ld_mm = 50.0f;

    float alpha_rad = alpha_deg * DEG_TO_RAD;

    // Pure Pursuit: delta = atan2(2 * L * sin(alpha), Ld)
    float steering_rad = atan2(2.0f * WHEELBASE_MM * sin(alpha_rad), Ld_mm);

    float steering_deg = steering_rad * RAD_TO_DEG;

    _lastWallRepel = (WALL_REPEL_ENABLED && gap.mode == 0)
                     ? _wallRepel(sensorData)
                     : 0.0f;
    steering_deg += _lastWallRepel;

    // Clamp to the mechanical limit.
    steering_deg = constrain(steering_deg, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

    return steering_deg;
}
