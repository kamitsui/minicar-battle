/*
 * GapFinder.h
 *
 * Finds the steering target by combining the farthest sensor with its neighbors.
 * The target angle is computed by triangle-area weighting when both neighbors
 * are valid, and falls back to distance weighting otherwise. See GapFinder.cpp
 * for details.
 *
 * State:
 * - _lastFarthestIdx: the previous farthest-sensor index. Used by the
 *   hysteresis check so that a new sensor only wins when it is at least
 *   FARTHEST_HYSTERESIS (mm) farther than the previous one.
 * - _turnDir: straight/corner mode for the front gate (see Config.h).
 *   0 = straight, -1 = corner locked left, +1 = corner locked right.
 * - _frontHist: recent valid front readings for the gate's median filter.
 * - _approachBalance: (right - left) distances summed while approaching a
 *   corner; its sign is the locked turn direction.
 * - _biasUntilMs: end of the post-left-corner bias window.
 */

#ifndef GAP_FINDER_H
#define GAP_FINDER_H

#include <Arduino.h>

#include "Config.h"
#include "SensorReader.h"

// Output of GapFinder::find.
struct GapResult {
    float target_angle;  // target heading (deg); positive = right, negative = left
    int8_t mode;         // 0 = straight, -1 = corner left, +1 = corner right
    bool left_bias;      // post-left-corner bias active (straight mode only)
    uint16_t front_filtered;  // median-filtered front distance used by the gate (mm)
};

class GapFinder {
   private:
    int _lastFarthestIdx;  // previous farthest-sensor index (for hysteresis)
    int8_t _turnDir;       // front-gate mode (see GapResult::mode)

    uint16_t _frontHist[FRONT_MEDIAN_WINDOW];  // ring buffer of front readings
    uint8_t _frontCount;   // number of filled entries
    uint8_t _frontPos;     // next write position
    uint16_t _frontFiltered;  // latest median (0 = no data yet)

    uint8_t _enterCount;     // consecutive frames below CORNER_ENTER_FRONT_MM
    float _approachBalance;  // summed (right - left) during the approach
    bool _biasActive;
    unsigned long _biasUntilMs;

    // Push the front reading and update the median.
    void _updateFrontFilter(const SensorData* data);

    // Switch between straight and corner mode from the filtered front sensor.
    void _updateMode(const SensorData* data);

    // Distance used for farthest-sensor selection (0 = not selectable).
    float _effectiveDistance(const SensorData* data, int idx) const;

    // Pick the farthest sensor before applying hysteresis.
    int _findFarthestSensor(const SensorData* data, float& outDistance) const;

    // Apply hysteresis and return the final farthest-sensor index.
    int _applyHysteresis(const SensorData* data, int candidateIdx,
                         float candidateDist, float& outDistance);

    // Compute the target angle from the farthest sensor and its neighbors.
    float _calculateTargetAngle(const SensorData* data, int farthestIdx,
                                float farthestDist) const;

   public:
    GapFinder();

    // Main entry point: turn sensor data into a target angle.
    GapResult find(const SensorData* sensorData);
};

#endif  // GAP_FINDER_H
