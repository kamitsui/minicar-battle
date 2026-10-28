/*
 * SensorReader.h
 *
 * Reads VL53L1X sensors through the TCA9548A I2C multiplexer.
 */

#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

#include "Config.h"

// One sensor reading.
struct SensorData {
    uint16_t distance;  // measured distance (mm)
    bool valid;         // true when the reading is within the trusted range
    uint8_t status;     // VL53L1X range_status (0 = RangeValid); logged only
};

class SensorReader {
   private:
    VL53L1X _sensors[NUM_SENSORS];
    SensorData _sensorData[NUM_SENSORS];

    // Select the active TCA9548A channel.
    void _selectChannel(uint8_t channel);

    // True when SENSOR_ENABLE_MASK lets sensor idx range.
    bool _isEnabled(uint8_t idx) const;

   public:
    SensorReader();

    // Initialize the multiplexer and all sensors.
    bool begin();

    // Read every sensor once.
    void readAll();

    // Get the cached readings.
    const SensorData* getAllData() const;
};

#endif  // SENSOR_READER_H
