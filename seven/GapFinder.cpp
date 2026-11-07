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
 * after a left corner. In corner mode the turn direction comes from
 * Config.h TURN_DIR_SOURCE, and step 3 can drop the opposite-side neighbor
 * (CORNER_ONE_SIDED_TARGET).
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
      _exitCount(0),
      _approachBalance(0.0f),
      _biasActive(false),
      _biasUntilMs(0),
      _cornerCount(0),
      _edgeNearAge(255),
      _edgeFarCount(0),
      _edgePending(-1),
      _edgeEntry(false),
      _exitArmed(true),
      _cornerFrames(0),
      _seqStart(CORNER_SEQUENCE_START),
      _cornerIdx(0),
      _exitInnerSeen(false) {
    for (uint8_t i = 0; i < CORNER_SEQUENCE_LEN; ++i) _cp[i] = CORNER_PARAMS[i];
}

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

        // Hairpin edge entry (EXP-006): the inner wall ended.
        if (_hairpinEdge(data) && _edgePending < 0) _edgePending = _cp[_nextIdx()].delay;
        if (_edgePending == 0) {
            _enterCorner(true);
            return;
        }
        if (_edgePending > 0) --_edgePending;

        _enterCount = (_frontFiltered < _cp[_nextIdx()].gate) ? _enterCount + 1 : 0;
        if (_enterCount >= CORNER_ENTER_DEBOUNCE) _enterCorner(false);
    } else {
        if (_cornerFrames < 255) ++_cornerFrames;
        // After an edge entry the front is still far; count the exit only
        // once it has closed in (or after EDGE_EXIT_ARM_FRAMES).
        if (!_exitArmed && (_frontFiltered < CORNER_EXIT_FRONT_MM ||
                            _cornerFrames >= EDGE_EXIT_ARM_FRAMES)) {
            _exitArmed = true;
        }
        bool open = _exitArmed && _frontFiltered > CORNER_EXIT_FRONT_MM;
        _exitCount = open ? _exitCount + 1 : 0;
        // Per-corner exitIn (EXP-007): the inner-side sensor must also see
        // the divider again during this run, unless the run gets long.
        uint16_t exitIn = _cp[_cornerIdx].exitIn;
        const SensorData& inner = data[_turnDir < 0 ? 0 : NUM_SENSORS - 1];
        if (!open) {
            _exitInnerSeen = false;
        } else if (exitIn == 0 || (inner.valid && inner.distance < exitIn)) {
            _exitInnerSeen = true;
        }
        if (_exitCount >= CORNER_EXIT_DEBOUNCE &&
            (_exitInnerSeen || _exitCount >= CORNER_EXIT_FALLBACK)) {
            if (_turnDir < 0 && POST_CORNER_BIAS_ENABLED) {
                _biasActive = true;
                _biasUntilMs = millis() + POST_CORNER_BIAS_MS;
            }
            _turnDir = 0;
            _exitCount = 0;
            _edgeEntry = false;
        }
    }
}

void GapFinder::_enterCorner(bool byEdge) {
    _cornerIdx = _nextIdx();
    _exitInnerSeen = false;
    if (TURN_DIR_SOURCE == TURN_DIR_SEQUENCE) {
        _turnDir = _nextCornerDir();
    } else {
        // Ties go left: the course is counter-clockwise.
        _turnDir = (_approachBalance > 0.0f) ? 1 : -1;
    }
    if (_cornerCount < 255) ++_cornerCount;
    _enterCount = 0;
    _exitCount = 0;
    _approachBalance = 0.0f;
    _biasActive = false;
    _edgeEntry = byEdge;
    _exitArmed = !byEdge;
    _cornerFrames = 0;
    _edgePending = -1;
    _edgeNearAge = 255;
    _edgeFarCount = 0;
}

bool GapFinder::_hairpinEdge(const SensorData* data) {
    if (TURN_DIR_SOURCE != TURN_DIR_SEQUENCE) return false;

    // Inner side of the next corner: S0 for a left turn, S6 for a right turn.
    const SensorData& s = data[_nextCornerDir() < 0 ? 0 : NUM_SENSORS - 1];
    if (s.valid && s.distance < EDGE_NEAR_MM) {
        _edgeNearAge = 0;
    } else if (_edgeNearAge < 255) {
        ++_edgeNearAge;
    }
    if (s.valid && s.distance > EDGE_FAR_MM) {
        if (_edgeFarCount < 255) ++_edgeFarCount;
    } else {
        _edgeFarCount = 0;
    }

    // Fires once, on the EDGE_FAR_FRAMES-th far frame, if the side was near
    // within EDGE_NEAR_WINDOW frames before the far run started.
    uint16_t edgeMax = _cp[_nextIdx()].edgeMax;
    return edgeMax > 0 && _edgeFarCount == EDGE_FAR_FRAMES &&
           _edgeNearAge <= EDGE_NEAR_WINDOW + EDGE_FAR_FRAMES - 1 &&
           _frontFiltered < edgeMax;
}

uint8_t GapFinder::_nextIdx() const {
    return (_seqStart + _cornerCount) % CORNER_SEQUENCE_LEN;
}

int8_t GapFinder::_nextCornerDir() const {
    return CORNER_SEQUENCE[_nextIdx()];
}

float GapFinder::_effectiveDistance(const SensorData* data, int idx) const {
    if (!data[idx].valid) return 0.0f;
    if (!FRONT_GATE_ENABLED) return data[idx].distance;

    if (_turnDir == 0) {
        // Straight: discount the side sensors (more on the right right
        // after a left corner).
        const float* weight = _biasActive ? LEFT_BIAS_WEIGHT : STRAIGHT_SIDE_WEIGHT;
        float d = data[idx].distance * weight[idx];
        if (TURN_DIR_SOURCE == TURN_DIR_SEQUENCE &&
            SENSOR_ANGLES[idx] * _nextCornerDir() < 0.0f) {
            d *= NEXT_CORNER_OPPOSITE_WEIGHT;  // away from the next corner
        }
        return d;
    }
    // Corner: ignore sensors on the side opposite to the locked turn (and
    // the front sensor, which opens up before the turn is finished).
    float side = SENSOR_ANGLES[idx] * _turnDir;
    if (side < 0.0f || (CORNER_EXCLUDE_FRONT && side == 0.0f)) return 0.0f;
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

    // Corner mode: drop the neighbor on the side opposite to the turn.
    if (CORNER_ONE_SIDED_TARGET && _turnDir < 0) has_right = false;
    if (CORNER_ONE_SIDED_TARGET && _turnDir > 0) has_left = false;

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

void GapFinder::reset(uint8_t seqStart) {
    CornerParams cp[CORNER_SEQUENCE_LEN];
    memcpy(cp, _cp, sizeof(cp));  // keep values set at runtime
    *this = GapFinder();
    memcpy(_cp, cp, sizeof(cp));
    _seqStart = seqStart % CORNER_SEQUENCE_LEN;
}

bool GapFinder::setParam(uint8_t idx, const String& key, long value) {
    if (idx >= CORNER_SEQUENCE_LEN) return false;
    CornerParams& p = _cp[idx];
    if (key == "gate" && value >= 300 && value < CORNER_EXIT_FRONT_MM) {
        p.gate = (uint16_t)value;
    } else if (key == "edgemax" && (value == 0 || (value >= 500 && value <= 4000))) {
        p.edgeMax = (uint16_t)value;
    } else if (key == "delay" && value >= 0 && value <= 20) {
        p.delay = (uint8_t)value;
    } else if (key == "exitin" && (value == 0 || (value >= 200 && value <= 3000))) {
        p.exitIn = (uint16_t)value;
    } else {
        return false;
    }
    return true;
}

GapResult GapFinder::find(const SensorData* data) {
    GapResult result = {0.0f, 0, false, 0, 0, false};

    // Step 0: straight/corner mode from the front sensor.
    _updateMode(data);
    result.mode = _turnDir;
    result.left_bias = _biasActive && _turnDir == 0;
    result.front_filtered = _frontFiltered;
    result.corner_count = _cornerCount;
    result.edge_entry = _edgeEntry && _turnDir != 0;

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
