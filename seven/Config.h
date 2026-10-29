/*
 * Config.h
 *
 * Centralized configuration values.
 * All constants are named here to avoid magic numbers.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Run mode
// ============================================================================
// Mode definitions:
//   0: MODE_DEBUG      - debug only (no PWM, serial enabled)
//   1: MODE_PRODUCTION - race run (PWM enabled, serial disabled)
//   2: MODE_DEBUG_RUN  - debug run (PWM enabled, serial enabled)
#define MODE_DEBUG 0
#define MODE_PRODUCTION 1
#define MODE_DEBUG_RUN 2

#define RUN_MODE MODE_DEBUG // select run mode here

// Feature toggles derived from RUN_MODE
#define ENABLE_SERIAL (RUN_MODE == MODE_DEBUG || RUN_MODE == MODE_DEBUG_RUN)
#define ENABLE_PWM (RUN_MODE == MODE_PRODUCTION || RUN_MODE == MODE_DEBUG_RUN)

// ============================================================================
// Hardware
// ============================================================================
// TCA9548A I2C multiplexer
const uint8_t TCA9548A_ADDR = 0x70;

// Sensor layout
const uint8_t NUM_SENSORS = 7;
const uint8_t SENSOR_CHANNELS[NUM_SENSORS] = {0, 1, 2, 3, 4, 5, 6};
// Physical mounting is +/-60deg, but these tuned values produced the winning 17.5s final (3 laps).
const float SENSOR_ANGLES[NUM_SENSORS] = {-50.0, -30.0, -15.0, 0.0, 15.0, 30.0, 50.0};
const uint8_t FRONT_SENSOR_INDEX = 3;  // index of the front-facing sensor (0 deg)

// PWM output pins
const uint8_t SERVO_PIN = 9;  // steering servo
const uint8_t ESC_PIN = 10;   // ESC (motor)

// ============================================================================
// Sensor parameters
// ============================================================================
const uint16_t MIN_VALID_DISTANCE = 50;  // minimum valid range (mm)
const uint16_t RELIABLE_RANGE = 4000;    // reliable measurement range (mm); VL53L1X max is 4m
// VL53L1X timing
const uint32_t L1X_TIMING_BUDGET_US = 33000;   // measurement budget (us)
const uint32_t L1X_INTER_MEASUREMENT_MS = 40;  // inter-measurement period (ms)

// ============================================================================
// Loop timing
// 40ms is the floor and maximizes measurements per second.
// 40-60ms is the VL53L1X medium-mode range (~3m effective).
// ============================================================================
const unsigned long MEASUREMENT_INTERVAL = 40;  // must be >= L1X inter-measurement period


// ============================================================================
// Steering
// ============================================================================
const float MAX_STEERING_ANGLE = 30.0;  // max steering angle (deg)

// ============================================================================
// Gap detection
// Tested 0-200; no clear effect on yaw oscillation.
// ============================================================================
const float FARTHEST_HYSTERESIS = 100.0;  // hysteresis for farthest-sensor switching (mm); tested 0-300

// ============================================================================
// Gap logic selection (EXP-001, see docs/experiments/)
//   LOGIC_BASELINE:  original farthest-sensor GapFinder.
//   LOGIC_GATE:      front-gated straight/corner modes (below).
//   LOGIC_GATE_BIAS: LOGIC_GATE + left bias for a short time after a left
//                    corner (picks the left lane at the board after hairpin 3).
// ============================================================================
#define LOGIC_BASELINE 0
#define LOGIC_GATE 1
#define LOGIC_GATE_BIAS 2

#define GAP_LOGIC LOGIC_GATE_BIAS  // select gap logic here

// ============================================================================
// Front gate (LOGIC_GATE, LOGIC_GATE_BIAS)
// The front sensor (S3), median-filtered, tells straights from corners:
//   Straight mode (until filtered S3 < CORNER_ENTER_FRONT_MM for
//     CORNER_ENTER_DEBOUNCE frames): side sensors are discounted by
//     STRAIGHT_SIDE_WEIGHT so wall openings (shortcut gaps) do not beat
//     the road ahead.
//   Corner mode (until filtered S3 > CORNER_EXIT_FRONT_MM): the turn
//     direction is locked on entry and sensors on the opposite side are
//     ignored, so the car does not turn back toward the straight it came
//     from after running wide.
// Turn direction = sign of (right - left) distances summed over the
// approach (filtered S3 < APPROACH_FRONT_MM). Deciding from the frame of
// entry alone picked the wrong side at hairpin (1) in hand-push logs.
// History (offline replay of hand-push logs, 2026-10-28):
//   ENTER/EXIT 1500/1800, raw S3: rejected (159 switches per lap; the ~1.4 m
//     reading S3 gives with no near wall caused false corners).
//   ENTER 1300: rejected (false corners at start/goal remained).
//   Side weight 0.7: rejected (side sensor still won 9 frames at the shortcut).
// ============================================================================
const bool FRONT_GATE_ENABLED = (GAP_LOGIC != LOGIC_BASELINE);
const uint8_t FRONT_MEDIAN_WINDOW = 5;       // frames (~200 ms); gate input only
const uint8_t CORNER_ENTER_DEBOUNCE = 3;     // consecutive frames below ENTER
const float CORNER_ENTER_FRONT_MM = 1200.0;  // provisional; from replay
const float CORNER_EXIT_FRONT_MM = 2000.0;   // provisional; must be > ENTER
const float APPROACH_FRONT_MM = 2000.0;      // start summing the turn direction
const float STRAIGHT_SIDE_WEIGHT[NUM_SENSORS] = {0.4, 0.6, 1.0, 1.0, 1.0, 0.6, 0.4};  // provisional

// ============================================================================
// Post-left-corner bias (LOGIC_GATE_BIAS)
// For POST_CORNER_BIAS_MS after leaving a left corner, straight mode uses
// LEFT_BIAS_WEIGHT instead of STRAIGHT_SIDE_WEIGHT. It only changes which
// direction is aimed at; it does not control the distance to the left wall.
// ============================================================================
const bool POST_CORNER_BIAS_ENABLED = (GAP_LOGIC == LOGIC_GATE_BIAS);
const unsigned long POST_CORNER_BIAS_MS = 1000;  // provisional
const float LEFT_BIAS_WEIGHT[NUM_SENSORS] = {0.4, 0.6, 1.0, 1.0, 0.7, 0.3, 0.2};  // provisional

// ============================================================================
// Sensor test aids (static tests; keep defaults for runs)
// L1X_ROI_SIZE: ROI width/height in SPADs (4-16). 16 = full ~27deg field of
//   view; smaller narrows it (e.g. 6 for S3 so side walls in a 700 mm lane
//   stay out of view).
// SENSOR_ENABLE_MASK: bit i = sensor i ranges. Disabled sensors stay idle
//   (no emission) and read as invalid. 0x08 = S3 only, 0x1C = S2-S4.
// ============================================================================
const uint8_t L1X_ROI_SIZE[NUM_SENSORS] = {16, 16, 16, 16, 16, 16, 16};
const uint8_t SENSOR_ENABLE_MASK = 0x7F;

// ============================================================================
// Pure Pursuit
// Formula: steering_angle = atan2(2 * L * sin(alpha), Ld)
//   L:     wheelbase (mm)
//   alpha: angle to the target point (rad)
//   Ld:    lookahead distance (mm) = front sensor distance - body length
// ============================================================================
const float WHEELBASE_MM = 210.0;       // wheelbase (mm), MF-01X
// Manually tuned. Sweep history: SPEED 1640 -> 900, 1660 -> 1000, 1680 -> 1100, > 1680 -> 1200.
const float LOOKAHEAD_OFFSET_MM = 1100.0;

// ============================================================================
// Side-wall repel (EXP-003)
// Pure Pursuit only aims at the most open direction, so nothing keeps the
// car off a side wall on a straight (with the S3 ROI at 6, Ld grew from
// ~100 to ~1400 mm and the steering became too weak to recover). When a
// side sensor reads closer than WALL_REPEL_START_MM, steer away by up to
// WALL_REPEL_MAX_DEG: 0 at the start distance, the max at contact. The
// closest sensor on each side counts, and the two sides push against each
// other. Straight mode only (corner mode turns along the inside wall).
// 0 in WALL_REPEL_START_MM = sensor not used. Start distances are below the
// lane-center readings in a 700 mm lane (2026-10-29: S0 ~330, S1 ~520,
// S5 ~580, S6 ~330).
// ============================================================================
const bool WALL_REPEL_ENABLED = true;
const float WALL_REPEL_START_MM[NUM_SENSORS] = {250, 400, 0, 0, 0, 400, 250};  // provisional
const float WALL_REPEL_MAX_DEG = 20.0;  // provisional

// ============================================================================
// Safety
// ============================================================================
const uint16_t EMERGENCY_FRONT_THRESHOLD = 400;  // front emergency-stop threshold (mm)

// Auto-stop timeout (debug aid).
// 0 disables it; otherwise the car stops after this many seconds.
const unsigned long AUTO_STOP_SECONDS = 0;
//const unsigned long AUTO_STOP_SECONDS = 20;

// ============================================================================
// Servo / ESC pulse widths
// ============================================================================
// Steering servo. New servo nominal range is 1200(R)-1500(C)-1800(L);
// values below are calibrated for the current install.
const uint16_t SERVO_CENTER = 1415;  // center (us)
const uint16_t SERVO_MIN = 1115;     // min pulse (us) = full right
const uint16_t SERVO_MAX = 1715;     // max pulse (us) = full left

// ESC
const uint16_t ESC_STOP_US = 1500;   // neutral / stop (us)
const uint16_t ESC_MIN_US = 1000;    // min ESC pulse (us)
const uint16_t ESC_MAX_US = 2000;    // max ESC pulse (us)
// Cruise pulse history:
//   1680: C1 (2025 final course), 3 laps in 17.5s, 1st place.
//   1600: C2 (2026 hairpin course), testing since 2026-10 (slower for the
//         three hairpins). LOOKAHEAD_OFFSET_MM is still the 1680 value.
const uint16_t SPEED_US = 1600;      // cruise pulse (us)

// ============================================================================
// Math constants
// ============================================================================
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295f  // PI / 180.0
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232f  // 180.0 / PI
#endif

// Precomputed sin values for the angular gaps between sensors
const float SIN_15_DEG = 0.2588190451f;  // sin(15deg)
const float SIN_20_DEG = 0.3420201433f;  // sin(20deg)

#endif  // CONFIG_H
