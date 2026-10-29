/*
 * SteeringController.h
 *
 * Steering control via Pure Pursuit on top of the Follow-the-Gap target.
 */

#ifndef STEERING_CONTROLLER_H
#define STEERING_CONTROLLER_H

#include <Arduino.h>

#include "Config.h"
#include "GapFinder.h"

class SteeringController {
   private:
    float _lastWallRepel;  // side-wall repel of the last call (deg)

    // Steering added to keep off a side wall (deg, + = right).
    float _wallRepel(const SensorData* sensorData) const;

   public:
    SteeringController();

    // Init (no-op; included for symmetry with the other modules).
    void begin();

    // Compute the steering angle (deg) using Pure Pursuit.
    float calculate(const GapResult& gap, const SensorData* sensorData);

    // Side-wall repel included in the last calculate() result (deg).
    float lastWallRepel() const { return _lastWallRepel; }
};

#endif  // STEERING_CONTROLLER_H
