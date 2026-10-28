/*
 * SensorReader.cpp
 *
 * VL53L1X reads through the TCA9548A multiplexer.
 */

#include "SensorReader.h"

#include "Logger.h"

SensorReader::SensorReader() {
    // Zero-initialize the cached readings.
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        _sensorData[i].distance = 0;
        _sensorData[i].valid = false;
        _sensorData[i].status = VL53L1X::None;
    }
}

bool SensorReader::_isEnabled(uint8_t idx) const {
    return (SENSOR_ENABLE_MASK >> idx) & 1;
}

void SensorReader::_selectChannel(uint8_t channel) {
    if (channel > 7) return;

    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

bool SensorReader::begin() {
    Wire.begin();
    Wire.setClock(400000);  // I2C fast mode (400 kHz)

    Logger::println("=== VL53L1X Sensor Initialization ===");

    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        _selectChannel(SENSOR_CHANNELS[i]);
        delay(10);  // settle after switching channels

        Logger::print("Sensor ");
        Logger::print(i);
        Logger::print(" (Ch");
        Logger::print(SENSOR_CHANNELS[i]);
        Logger::print(", ");
        Logger::print(SENSOR_ANGLES[i]);
        Logger::print("deg)...");

        _sensors[i].setTimeout(500);
        if (!_sensors[i].init()) {
            Logger::println(" FAILED!");
            return false;
        }

        // Long range mode, configurable timing budget
        _sensors[i].setDistanceMode(VL53L1X::Long);
        _sensors[i].setMeasurementTimingBudget(L1X_TIMING_BUDGET_US);
        if (L1X_ROI_SIZE[i] < 16) {
            _sensors[i].setROISize(L1X_ROI_SIZE[i], L1X_ROI_SIZE[i]);
        }

        if (!_isEnabled(i)) {
            // Left idle (no emission) for interference tests.
            Logger::println(" OK (disabled)");
            continue;
        }

        // Continuous ranging
        _sensors[i].startContinuous(L1X_INTER_MEASUREMENT_MS);

        Logger::print(" OK");
        if (L1X_ROI_SIZE[i] < 16) {
            Logger::print(" ROI ");
            Logger::print(L1X_ROI_SIZE[i]);
        }
        Logger::println();
    }

    Logger::println("=== All VL53L1X sensors initialized ===");
    return true;
}

void SensorReader::readAll() {
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        if (!_isEnabled(i)) continue;  // stays invalid (constructor values)

        _selectChannel(SENSOR_CHANNELS[i]);

        // Read in continuous mode
        _sensors[i].read();

        _sensorData[i].distance = _sensors[i].ranging_data.range_mm;
        // Recorded for analysis only; validity below does not use it yet.
        _sensorData[i].status = _sensors[i].ranging_data.range_status;

        if (_sensorData[i].distance > RELIABLE_RANGE) {
            // Clamp to the upper trusted bound
            _sensorData[i].distance = RELIABLE_RANGE;
        }
        _sensorData[i].valid =
            (_sensorData[i].distance >= MIN_VALID_DISTANCE) &&
            (_sensorData[i].distance <= RELIABLE_RANGE);
    }
}

const SensorData* SensorReader::getAllData() const { return _sensorData; }
