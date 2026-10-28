/*
 * GapFinder.cpp
 *
 * Algorithm:
 * 1. Pick the farthest valid sensor (with hysteresis to avoid flapping).
 * 2. Look at its left/right neighbors when available.
 * 3. With both neighbors: weight each neighbor by the area of the triangle
 *    it forms with the farthest sensor, and take the area-weighted angle.
 *    With one neighbor (or none): fall back to a distance-weighted angle.
 *
 * Triangle areas (the 1/2 cancels in the ratio, so it is omitted):
 *   area_left  = d_left  * d_farthest * sin(angle gap to left)
 *   area_right = d_right * d_farthest * sin(angle gap to right)
 *   target = (angle_left * area_left + angle_right * area_right) /
 *            (area_left + area_right)
 *
 * Front gate (Config.h GAP_LOGIC): before step 1, the median-filtered front
 * sensor selects straight or corner mode, which changes the distances used
 * for picking the farthest sensor (steps 1-2 only; step 3 uses raw
 * distances). LOGIC_GATE_BIAS also favors the left side for a short time
 * after a left corner.
 *
 * Note: Ld (the lookahead distance) is computed in SteeringController from
 * the front sensor reading, not here.
 */

#include "GapFinder.h"

#include <math.h>

GapFinder::GapFinder()
    : _lastFarthestIdx(FRONT_SENSOR_INDEX),
      _turnDir(0),
      _frontCount(0),
      _frontPos(0),
      _frontFiltered(0),
      _enterCount(0),
      _approachBalance(0.0f),
      _biasActive(false),
      _biasUntilMs(0) {}

// ============================================================================
// Private
// ============================================================================

void GapFinder::_updateFrontFilter(const SensorData* data) {
    const SensorData& front = data[FRONT_SENSOR_INDEX];
    if (!front.valid) return;  // keep the previous median

    _frontHist[_frontPos] = front.distance;
    _frontPos = (_frontPos + 1) % FRONT_MEDIAN_WINDOW;
    if (_frontCount < FRONT_MEDIAN_WINDOW) ++_frontCount;

    // Insertion sort on a copy; the window is tiny.
    uint16_t sorted[FRONT_MEDIAN_WINDOW];
    for (uint8_t i = 0; i < _frontCount; ++i) {
        uint16_t v = _frontHist[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > v) {
            sorted[j + 1] = sorted[j];
            --j;
        }
        sorted[j + 1] = v;
    }
    _frontFiltered = sorted[_frontCount / 2];
}

void GapFinder::_updateMode(const SensorData* data) {
    _updateFrontFilter(data);  // also logged under LOGIC_BASELINE

    if (!FRONT_GATE_ENABLED) {
        _turnDir = 0;
        return;
    }
    if (_frontFiltered == 0) return;  // no front data yet

    if (_biasActive && (long)(millis() - _biasUntilMs) >= 0) {
        _biasActive = false;
    }

    if (_turnDir == 0) {
        // Approach: accumulate which side is more open while the front
        // closes in, so the decision reflects the corner ahead rather than
        // the last frame before entry.
        if (_frontFiltered < APPROACH_FRONT_MM) {
            for (int i = 0; i < NUM_SENSORS; ++i) {
                if (!data[i].valid) continue;
                if (SENSOR_ANGLES[i] < 0.0f) _approachBalance -= data[i].distance;
                if (SENSOR_ANGLES[i] > 0.0f) _approachBalance += data[i].distance;
            }
        } else {
            _approachBalance = 0.0f;
        }

        _enterCount = (_frontFiltered < CORNER_ENTER_FRONT_MM) ? _enterCount + 1 : 0;
        if (_enterCount >= CORNER_ENTER_DEBOUNCE) {
            // Ties go left: the course is counter-clockwise.
            _turnDir = (_approachBalance > 0.0f) ? 1 : -1;
            _enterCount = 0;
            _approachBalance = 0.0f;
            _biasActive = false;
        }
    } else if (_frontFiltered > CORNER_EXIT_FRONT_MM) {
        if (_turnDir < 0 && POST_CORNER_BIAS_ENABLED) {
            _biasActive = true;
            _biasUntilMs = millis() + POST_CORNER_BIAS_MS;
        }
        _turnDir = 0;
    }
}

float GapFinder::_effectiveDistance(const SensorData* data, int idx) const {
    if (!data[idx].valid) return 0.0f;
    if (!FRONT_GATE_ENABLED) return data[idx].distance;

    if (_turnDir == 0) {
        // Straight: discount the side sensors (more on the right right
        // after a left corner).
        const float* weight = _biasActive ? LEFT_BIAS_WEIGHT : STRAIGHT_SIDE_WEIGHT;
        return data[idx].distance * weight[idx];
    }
    // Corner: ignore sensors on the side opposite to the locked turn.
    if (SENSOR_ANGLES[idx] * _turnDir < 0.0f) return 0.0f;
    return data[idx].distance;
}

int GapFinder::_findFarthestSensor(const SensorData* data,
                                  float& outDistance) const {
    int farthest_idx = -1;
    float farthest_dist = 0.0f;

    for (int i = 0; i < NUM_SENSORS; ++i) {
        float d = _effectiveDistance(data, i);
        if (d > farthest_dist) {
            farthest_dist = d;
            farthest_idx = i;
        }
    }

    outDistance = farthest_dist;
    return farthest_idx;
}

int GapFinder::_applyHysteresis(const SensorData* data, int candidateIdx,
                               float candidateDist, float& outDistance) {
    int result_idx = candidateIdx;

    // Keep the previous farthest sensor if the new candidate is not enough farther.
    // Both sides of the comparison use effective distances.
    if (_lastFarthestIdx >= 0 && _lastFarthestIdx < NUM_SENSORS &&
        candidateIdx != _lastFarthestIdx) {
        float last_dist = _effectiveDistance(data, _lastFarthestIdx);
        if (last_dist > 0.0f &&
            candidateDist - last_dist < FARTHEST_HYSTERESIS) {
            result_idx = _lastFarthestIdx;
        }
    }

    // Update state for the next call.
    _lastFarthestIdx = result_idx;
    outDistance = data[result_idx].distance;  // raw distance for step 3
    return result_idx;
}

float GapFinder::_calculateTargetAngle(const SensorData* data, int farthestIdx,
                                      float farthestDist) const {
    int left_idx = (farthestIdx > 0) ? farthestIdx - 1 : -1;
    int right_idx = (farthestIdx < NUM_SENSORS - 1) ? farthestIdx + 1 : -1;

    bool has_left = (left_idx >= 0 && data[left_idx].valid);
    bool has_right = (right_idx >= 0 && data[right_idx].valid);

    if (has_left && has_right) {
        // Both neighbors valid: triangle-area weighting.
        float angle_diff_left =
            fabs(SENSOR_ANGLES[farthestIdx] - SENSOR_ANGLES[left_idx]);
        float angle_diff_right =
            fabs(SENSOR_ANGLES[right_idx] - SENSOR_ANGLES[farthestIdx]);

        // Use the precomputed sin values (15 deg or 20 deg).
        float sin_left = (angle_diff_left < 17.5f) ? SIN_15_DEG : SIN_20_DEG;
        float sin_right = (angle_diff_right < 17.5f) ? SIN_15_DEG : SIN_20_DEG;

        float area_left = data[left_idx].distance * farthestDist * sin_left;
        float area_right = farthestDist * data[right_idx].distance * sin_right;

        float total_area = area_left + area_right;
        if (total_area < 0.001f) {
            // Avoid divide-by-zero; fall back to the farthest sensor's angle.
            return SENSOR_ANGLES[farthestIdx];
        }
        return (SENSOR_ANGLES[left_idx] * area_left +
                SENSOR_ANGLES[right_idx] * area_right) / total_area;
    } else {
        // One side or zero neighbors: distance-weighted average.
        float weighted_sum = SENSOR_ANGLES[farthestIdx] * farthestDist;
        float weight_total = farthestDist;

        if (has_left) {
            weighted_sum += SENSOR_ANGLES[left_idx] * data[left_idx].distance;
            weight_total += data[left_idx].distance;
        }
        if (has_right) {
            weighted_sum += SENSOR_ANGLES[right_idx] * data[right_idx].distance;
            weight_total += data[right_idx].distance;
        }

        if (weight_total < 0.001f) {
            // Avoid divide-by-zero; fall back to the farthest sensor's angle.
            return SENSOR_ANGLES[farthestIdx];
        }
        return weighted_sum / weight_total;
    }
}

// ============================================================================
// Public
// ============================================================================

GapResult GapFinder::find(const SensorData* data) {
    GapResult result = {0.0f, 0, false, 0};

    // Step 0: straight/corner mode from the front sensor.
    _updateMode(data);
    result.mode = _turnDir;
    result.left_bias = _biasActive && _turnDir == 0;
    result.front_filtered = _frontFiltered;

    // Step 1: pick the farthest sensor.
    float farthest_dist;
    int farthest_idx = _findFarthestSensor(data, farthest_dist);

    // No valid sensor -> drive straight.
    if (farthest_idx < 0) {
        return result;
    }

    // Step 2: hysteresis.
    farthest_idx = _applyHysteresis(data, farthest_idx, farthest_dist,
                                   farthest_dist);

    // Step 3: target angle.
    result.target_angle = _calculateTargetAngle(data, farthest_idx,
                                               farthest_dist);

    return result;
}
