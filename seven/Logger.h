/*
 * Logger.h
 *
 * Header-only debug logger.
 * Serial output is gated on RUN_MODE (see Config.h).
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

#include "Config.h"

// ============================================================================
// Serial output, conditional on the run mode.
// ============================================================================
class Logger {
   public:
    // Open the serial port.
    static void begin(unsigned long baud = 9600) {
#if ENABLE_SERIAL
        Serial.begin(baud);
        while (!Serial) {
            delay(10);
        }  // wait for the serial connection
#endif
    }

    // Generic print, no newline.
    template <typename T>
    static void print(const T& value) {
#if ENABLE_SERIAL
        Serial.print(value);
#endif
    }

    // Float print with explicit precision.
    static void print(float value, int decimals) {
#if ENABLE_SERIAL
        Serial.print(value, decimals);
#endif
    }

    // Generic println.
    template <typename T>
    static void println(const T& value) {
#if ENABLE_SERIAL
        Serial.println(value);
#endif
    }

    // Bare newline.
    static void println() {
#if ENABLE_SERIAL
        Serial.println();
#endif
    }

    // Compact one-sensor reading.
    // A non-zero VL53L1X range_status is appended as "/<status>"
    // (e.g. "S3:1485/1" = SigmaFail); RangeValid (0) prints nothing extra.
    static void printSensorData(uint8_t channel, uint16_t distance,
                                bool valid, uint8_t status) {
        print("S");
        print(channel);
        print(":");
        if (valid) {
            print(distance);
        } else {
            print("---");
        }
        if (status != 0) {
            print("/");
            print(status);
        }
    }

    // Gap-detection result.
    // mode: 0 = straight (S, or SL with the post-left-corner bias),
    //       -1 = corner left (CL), +1 = corner right (CR).
    // F: median-filtered front distance used by the gate (mm).
    static void printGapResult(float targetAngle, float targetDistance,
                               int8_t mode, bool leftBias,
                               uint16_t frontFiltered) {
        print(" | T:");
        print(targetAngle, 1);
        print("° Ld:");
        print((int)targetDistance);
        print(" M:");
        if (mode == 0) {
            print(leftBias ? "SL" : "S");
        } else {
            print(mode < 0 ? "CL" : "CR");
        }
        print(" F:");
        print(frontFiltered);
    }

    // Steering angle, with a small visual indicator.
    static void printSteering(float angle) {
        print(" St:");
        print(angle, 1);
        print(" ");

        if (angle < -5.0) {
            // Hard left.
            int bars = constrain((int)(-angle / 5), 1, 6);
            for (int i = 0; i < bars; ++i) print("L");
        } else if (angle > 5.0) {
            // Hard right.
            int bars = constrain((int)(angle / 5), 1, 6);
            for (int i = 0; i < bars; ++i) print("R");
        } else {
            // Roughly centered.
            print("|");
        }
    }

    // Actuator state.
    static void printActuator(const char* name, uint16_t pulse_us) {
        print(" [");
        print(name);
        print(":");
        print(pulse_us);
        print("]");
    }

    // Loop timing in microseconds.
    static void printLoopTiming(unsigned long loop_us, unsigned long sensor_us) {
        print(" | T:");
        print(loop_us);
        print("us(S:");
        print(sensor_us);
        print("us)");
    }

    // Timing-config summary.
    static void printTimingConfig() {
        println("--- Timing Configuration ---");
        print("  L1X Timing Budget: ");
        print(L1X_TIMING_BUDGET_US);
        println(" us");
        print("  L1X Inter-Measurement: ");
        print(L1X_INTER_MEASUREMENT_MS);
        println(" ms");
        print("  Loop Interval: ");
        print(MEASUREMENT_INTERVAL);
        println(" ms");
        print("  Sensor Count: ");
        print(NUM_SENSORS);
        println(" (continuous mode, ~5ms/sensor)");
        println("----------------------------");
    }
};

#endif  // LOGGER_H
