/*
 * seven.ino
 *
 * Follow-the-Gap + Pure Pursuit control over 7 VL53L1X sensors.
 *
 * Wiring:
 * - TCA9548A I2C multiplexer (address 0x70)
 * - VL53L1X sensors connect to multiplexer channels 0-6
 * - Steering servo on pin 9
 * - ESC on pin 10
 *
 * Usage:
 * 1. Open this sketch in the Arduino IDE.
 * 2. Install the required libraries:
 *    - VL53L1X (Pololu)
 *    - Servo (Arduino built-in)
 * 3. Set RUN_MODE in Config.h:
 *    - MODE_DEBUG: debug only (no PWM, serial enabled)
 *    - MODE_PRODUCTION: race run (PWM enabled, serial disabled)
 *    - MODE_DEBUG_RUN: debug run (PWM enabled, serial enabled)
 * 4. Flash to an Arduino Nano R4.
 */

#include "Actuator.h"
#include "Config.h"
#include "GapFinder.h"
#include "Logger.h"
#include "SensorReader.h"
#include "SteeringController.h"

// ============================================================================
// Globals
// ============================================================================
SensorReader sensorReader;
GapFinder gapFinder;
SteeringController steeringController;
Actuator actuator;
uint16_t speedUs = SPEED_US;  // cruise pulse; "speed" command

// One corner's values, as in the banner and the "get" reply.
static String cornerLine(uint8_t i) {
    const CornerParams& p = gapFinder.params(i);
    return "c" + String(i) + " " + CORNER_NAMES[i] + " gate=" + String(p.gate) +
           " edgemax=" + String(p.edgeMax) + " delay=" + String(p.delay) +
           " exitin=" + String(p.exitIn);
}

// ============================================================================
// Serial commands (see Config.h)
// ============================================================================
#if ENABLE_SERIAL
// Into the log (both outputs), and to USB when the log does not go there.
static void reply(const String& msg) {
    Logger::println(msg);
    if (LOG_TO_USB == USB_LOG_OFF) Serial.println(msg);
}

// "set N KEY V": per-corner value (CORNER_PARAMS).
static bool setCommand(const String& arg) {
    int a = arg.indexOf(' ');
    int b = a < 0 ? -1 : arg.indexOf(' ', a + 1);
    if (a < 0 || b < 0 || !isDigit(arg[0])) return false;
    long idx = arg.substring(0, a).toInt();
    String key = arg.substring(a + 1, b);
    String val = arg.substring(b + 1);
    val.trim();
    if (val.length() == 0 || !isDigit(val[0]) || idx >= CORNER_SEQUENCE_LEN) return false;
    if (!gapFinder.setParam((uint8_t)idx, key, val.toInt())) return false;
    reply("# SET " + cornerLine((uint8_t)idx));
    return true;
}

static void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    int sp = cmd.indexOf(' ');
    String name = sp < 0 ? cmd : cmd.substring(0, sp);
    String arg = sp < 0 ? "" : cmd.substring(sp + 1);
    arg.trim();
    long v = arg.toInt();
    bool num = arg.length() > 0 && (isDigit(arg[0]) || arg[0] == '-');

    if (name == "start" && num && v >= 0 && v < CORNER_SEQUENCE_LEN) {
        gapFinder.reset((uint8_t)v);
        reply("# SET start=" + String(v));
    } else if (name == "reset" && arg.length() == 0) {
        gapFinder.reset(gapFinder.sequenceStart());
        reply("# SET reset start=" + String(gapFinder.sequenceStart()));
    } else if (name == "speed" && num && v >= CMD_SPEED_MIN_US && v <= CMD_SPEED_MAX_US) {
        speedUs = (uint16_t)v;
        reply("# SET speed=" + String(speedUs));
    } else if (name == "set" && setCommand(arg)) {
        // replied in setCommand
    } else if (name == "help" && arg.length() == 0) {
        reply("# HELP start N    next corner = N (0 h1, 1 h2, 2 h3, 3 90L-1, 4 90L-2), back to boot state");
        reply("# HELP reset      back to boot state, same start corner (repeat a test)");
        reply("# HELP speed US   cruise pulse " + String(CMD_SPEED_MIN_US) + "-" +
              String(CMD_SPEED_MAX_US) + " (1500 = stop)");
        reply("# HELP set N KEY V  corner N: gate MM (enter when F < MM) | edgemax MM (edge entry while F < MM, 0 off)");
        reply("# HELP              | delay F (frames edge -> turn) | exitin MM (exit needs inner side < MM, 0 off)");
        reply("# HELP get        show settings only: start, speed, servo, per-corner values");
        reply("# HELP Not saved: a reboot restores Config.h. Changes are logged as # SET.");
    } else if (name == "get" && arg.length() == 0) {
        reply("# GET start=" + String(gapFinder.sequenceStart()) +
              " speed=" + String(speedUs) +
              " servo=" + String(SERVO_MIN) + "/" + String(SERVO_CENTER) + "/" + String(SERVO_MAX));
        for (uint8_t i = 0; i < CORNER_SEQUENCE_LEN; ++i) reply("# GET " + cornerLine(i));
    } else {
        reply("# ERR '" + cmd + "' (start 0-4 | reset | speed " + String(CMD_SPEED_MIN_US) +
              "-" + String(CMD_SPEED_MAX_US) + " | set N KEY V | get | help)");
    }
}

// Non-blocking: collect characters, run a command at the end of a line.
// Longer lines are cut at 40 characters (and then rejected as # ERR).
static void pollStream(Stream& in, String& buf) {
    while (in.available() > 0) {
        char c = (char)in.read();
        if (c == '\n' || c == '\r') {
            handleCommand(buf);
            buf = "";
        } else if (buf.length() < 40) {
            buf += c;
        }
    }
}

// USB serial monitor, and the M5 logger on Serial1 (CMD_FROM_UART).
static void pollCommands() {
    static String usbBuf, uartBuf;
    pollStream(Serial, usbBuf);
    if (CMD_FROM_UART && LOG_TO_UART) pollStream(Serial1, uartBuf);
}
#endif

// ============================================================================
// setup
// ============================================================================
void setup() {
    // Logger init (115200 baud for detailed telemetry)
    Logger::begin(115200);

    Logger::println("==============================================");
    Logger::println("  VL53L1X Follow the Gap + Pure Pursuit");
    Logger::println("==============================================");
    Logger::print("Run Mode: ");
#if RUN_MODE == MODE_DEBUG
    Logger::println("DEBUG (No PWM, Serial ON)");
#elif RUN_MODE == MODE_PRODUCTION
    Logger::println("PRODUCTION (PWM ON, Serial OFF)");
#else
    Logger::println("DEBUG_RUN (PWM ON, Serial ON)");
#endif
    Logger::print("Gap Logic: ");
#if GAP_LOGIC == LOGIC_BASELINE
    Logger::println("BASELINE (farthest sensor)");
#elif GAP_LOGIC == LOGIC_GATE
    Logger::println("GATE (straight/corner modes)");
#else
    Logger::println("GATE_BIAS (modes + post-left-corner bias)");
#endif
    Logger::print("Sensor Enable Mask: 0x");
    Logger::println(String(SENSOR_ENABLE_MASK, HEX));
    Logger::print("Sensor ROI (S0-S6):");
    for (int i = 0; i < NUM_SENSORS; ++i) {
        Logger::print(" ");
        Logger::print(L1X_ROI_SIZE[i]);
    }
    Logger::println();
    Logger::print("Turn Direction: ");
    Logger::println(TURN_DIR_SOURCE == TURN_DIR_SEQUENCE ? "SEQUENCE" : "BALANCE");
    Logger::print("Corner Sequence Start: ");
    Logger::println(CORNER_SEQUENCE_START);
    Logger::print("Commands: start N | reset | speed US | set N KEY V | get | help (USB");
    Logger::println(CMD_FROM_UART && LOG_TO_UART ? ", M5 UART)" : ")");
    Logger::print("Corner One-Sided Target: ");
    Logger::println(CORNER_ONE_SIDED_TARGET ? "ON" : "OFF");
    Logger::print("Corner Hold: exclude S3 ");
    Logger::print(CORNER_EXCLUDE_FRONT ? "ON" : "OFF");
    Logger::print(", Ld ");
    Logger::print(CORNER_LD_MM, 0);
    Logger::print(", exit debounce ");
    Logger::println(CORNER_EXIT_DEBOUNCE);
    Logger::print("Next-Corner Opposite Weight: ");
    Logger::print(NEXT_CORNER_OPPOSITE_WEIGHT, 2);
    Logger::println();
    Logger::print("Hairpin Edge Entry: corners");
    for (int i = 0; i < CORNER_SEQUENCE_LEN; ++i) {
        if (CORNER_PARAMS[i].edgeMax > 0) {
            Logger::print(" ");
            Logger::print(i + 1);
        }
    }
    Logger::print(", near ");
    Logger::print(EDGE_NEAR_MM);
    Logger::print(" far ");
    Logger::print(EDGE_FAR_MM);
    Logger::println(" (front max and delay per corner)");
    for (uint8_t i = 0; i < CORNER_SEQUENCE_LEN; ++i) {
        Logger::print("Corner Param: ");
        Logger::println(cornerLine(i));
    }
    Logger::print("Straight Ld Min: ");
    Logger::print(STRAIGHT_LD_MIN_MM, 0);
    Logger::println();
    Logger::print("Emergency Keep Steer In Corner: ");
    Logger::println(EMERGENCY_KEEP_STEER_IN_CORNER ? "ON" : "OFF");
    Logger::print("Speed: ");
    Logger::print(SPEED_US);
    Logger::println(" us");
    Logger::print("Corner Gate: enter < gate (per corner) x");
    Logger::print(CORNER_ENTER_DEBOUNCE);
    Logger::print(", exit > ");
    Logger::print(CORNER_EXIT_FRONT_MM, 0);
    Logger::print(" x");
    Logger::print(CORNER_EXIT_DEBOUNCE);
    Logger::print(" (+ exitin, fallback x");
    Logger::print(CORNER_EXIT_FALLBACK);
    Logger::println(")");
    Logger::print("Lookahead Offset: ");
    Logger::print(LOOKAHEAD_OFFSET_MM, 0);
    Logger::println();
    Logger::print("Emergency Front: ");
    Logger::println(EMERGENCY_FRONT_THRESHOLD);
    Logger::print("Log Output: USB ");
    Logger::print(LOG_TO_USB == USB_LOG_FULL ? "FULL"
                  : LOG_TO_USB == USB_LOG_INFO ? "INFO (no frames)" : "OFF");
    Logger::print(", UART ");
    if (LOG_TO_UART) {
        Logger::print("ON ");
        Logger::println(LOG_UART_BAUD);
    } else {
        Logger::println("OFF");
    }
    Logger::print("Servo MIN/CENTER/MAX: ");
    Logger::print(SERVO_MIN);
    Logger::print("/");
    Logger::print(SERVO_CENTER);
    Logger::print("/");
    Logger::println(SERVO_MAX);
    Logger::print("Wall Repel: ");
    Logger::println(WALL_REPEL_ENABLED ? "ON (straight mode)" : "OFF");
    Logger::print("Measurement Interval: ");
    Logger::print(MEASUREMENT_INTERVAL);
    Logger::println("ms");
    Logger::print("Wheelbase: ");
    Logger::print(WHEELBASE_MM, 0);
    Logger::println("mm");
    Logger::println("Lookahead: Front sensor distance");
    Logger::println();

    // Print timing config summary
    Logger::printTimingConfig();
    Logger::println();

    // Sensor init
    if (!sensorReader.begin()) {
        Logger::println("ERROR: Sensor initialization failed!");
        while (1) {
            delay(100);
        }
    }

    // Steering controller init
    steeringController.begin();

    // Actuator init
    actuator.begin();

    // Wait for ESC arming
    Logger::println("Waiting 3 seconds for ESC arming...");
    delay(3000);

    Logger::println();
    Logger::println("System ready!");
    Logger::println();
}

// ============================================================================
// loop
// ============================================================================
void loop() {
    static unsigned long lastMeasurement = 0;
    static unsigned long frame = 0;
    static unsigned long startTime = 0;
    static bool firstRun = true;
    unsigned long currentTime = millis();

    // Record start time on the first iteration
    if (firstRun) {
        startTime = currentTime;
        firstRun = false;
    }

    // Auto-stop on timeout
    if (AUTO_STOP_SECONDS > 0 &&
        (currentTime - startTime) >= (AUTO_STOP_SECONDS * 1000UL)) {
        actuator.setSteering(0.0);
        actuator.stop();
        Logger::println("AUTO STOP: Timeout reached");
        while (1) {
            delay(1000);
        }
    }

#if ENABLE_SERIAL
    // Serial commands, between frames (their reply lines never split a frame).
    pollCommands();
#endif

    // Run measurement and control on a fixed cadence
    if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
        lastMeasurement += MEASUREMENT_INTERVAL;

        // Start loop timer
        unsigned long loopStartUs = micros();

        // =========================================================================
        // Phase 1: read sensors
        // =========================================================================
        unsigned long sensorStartUs = micros();
        sensorReader.readAll();
        unsigned long sensorEndUs = micros();
        unsigned long sensorElapsedUs = sensorEndUs - sensorStartUs;

        const SensorData* sensorData = sensorReader.getAllData();

        // Debug: print sensor readings
        Logger::beginFrame();
        for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
            Logger::printSensorData(SENSOR_CHANNELS[i], sensorData[i].distance,
                                    sensorData[i].valid, sensorData[i].status);
            if (i < NUM_SENSORS - 1) {
                Logger::print(" | ");
            }
        }

        // =========================================================================
        // Phase 2: emergency stop check (front obstacle)
        // =========================================================================
        bool emergency_stop = false;
        if (sensorData[FRONT_SENSOR_INDEX].valid &&
            sensorData[FRONT_SENSOR_INDEX].distance < EMERGENCY_FRONT_THRESHOLD) {
            emergency_stop = true;
            Logger::print(" | EMERGENCY!");
        }

        // =========================================================================
        // Phase 3: gap detection (farthest sensor + neighbors)
        // =========================================================================
        GapResult gap = gapFinder.find(sensorData);

        // Debug: print gap result (Ld is derived from the front sensor distance)
        float front_distance = sensorData[FRONT_SENSOR_INDEX].valid
                               ? sensorData[FRONT_SENSOR_INDEX].distance
                               : 0.0f;
        Logger::printGapResult(gap.target_angle, front_distance, gap.mode,
                               gap.left_bias, gap.front_filtered,
                               gap.corner_count, gap.edge_entry);

        // =========================================================================
        // Phase 4: steering angle (Pure Pursuit)
        // =========================================================================
        float steering_angle = steeringController.calculate(gap, sensorData);

        // Debug: print target/steering angles
        Logger::printSteering(steering_angle, steeringController.lastWallRepel());

        // =========================================================================
        // Phase 5: actuators
        // =========================================================================
        if (emergency_stop) {
            // Emergency stop: cut power. Center the steering, except in
            // corner mode (EMERGENCY_KEEP_STEER_IN_CORNER).
            bool keep = EMERGENCY_KEEP_STEER_IN_CORNER && gap.mode != 0;
            actuator.setSteering(keep ? steering_angle : 0.0);
            actuator.stop();
        } else {
            // Normal cruise.
            actuator.setSteering(steering_angle);
            actuator.setSpeed(speedUs);
        }

        // Loop timer end + print
        unsigned long loopEndUs = micros();
        unsigned long loopElapsedUs = loopEndUs - loopStartUs;
        Logger::printLoopTiming(loopElapsedUs, sensorElapsedUs);
        Logger::printFrame(frame++);

        Logger::println("");
        Logger::endFrame();
    }
}
