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

#define RUN_MODE MODE_DEBUG_RUN // select run mode here

// Feature toggles derived from RUN_MODE
#define ENABLE_SERIAL (RUN_MODE == MODE_DEBUG || RUN_MODE == MODE_DEBUG_RUN)
#define ENABLE_PWM (RUN_MODE == MODE_PRODUCTION || RUN_MODE == MODE_DEBUG_RUN)

// ============================================================================
// Log outputs (only when ENABLE_SERIAL)
// USB: the PC serial monitor. UART: Serial1 (D1 TX) to the M5StickC Plus2
// logger for untethered debug runs (5 V -> 3.3 V divider on the line).
// LOG_TO_USB:
//   USB_LOG_FULL  everything, frame lines included
//   USB_LOG_INFO  boot banner and command replies only; frame lines go to
//                 the M5 only (debug runs: settings visible on the PC)
//   USB_LOG_OFF   nothing on USB, no wait for a PC at boot (race)
// USB_WAIT_MS: how long setup() waits for a PC on USB (FULL / INFO); 0 = wait
// forever (tethered runs, so the banner is not missed). Untethered runs need
// > 0, or the car never starts.
// ============================================================================
#define USB_LOG_OFF 0
#define USB_LOG_INFO 1
#define USB_LOG_FULL 2
const uint8_t LOG_TO_USB = USB_LOG_INFO;
const bool LOG_TO_UART = true;
const unsigned long LOG_UART_BAUD = 1000000;  // exact divider on RA4M1 and ESP32
const unsigned long USB_WAIT_MS = 3000;

// ============================================================================
// Serial commands (one per line; only when ENABLE_SERIAL)
// From the USB serial monitor, and from the M5 logger on Serial1 RX (D0)
// when CMD_FROM_UART (phone / PC -> WiFi -> M5 -> Nano). Race build:
// CMD_FROM_UART = false (no external control while running).
//   start N   next corner = CORNER_SEQUENCE[N] (0 h1, 1 h2, 2 h3, 3 90L-1,
//             4 90L-2), and back to the boot state (no corners counted,
//             straight mode): set up a partial-course test without flashing
//   reset     back to the boot state, same start corner (repeat a test)
//   speed US  cruise pulse, CMD_SPEED_MIN_US..CMD_SPEED_MAX_US
//   set N KEY V  per-corner value for corner N (CORNER_PARAMS):
//             gate | edgemax | delay | exitin
//   get       print the current settings only (start, speed, servo, the
//             per-corner table), no sensor data
//   help      print this list
// Changes are not saved: a reboot goes back to the values in this file.
// Each change is logged as "# SET ..." (tools/plot_log.py and the M5 screen
// restart the section count there).
// ============================================================================
const uint16_t CMD_SPEED_MIN_US = 1500;  // ESC neutral: no reverse from a typo
const uint16_t CMD_SPEED_MAX_US = 1750;
const bool CMD_FROM_UART = true;

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
//   Straight mode (until filtered S3 < the next corner's gate (CORNER_PARAMS) for
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
const float CORNER_EXIT_FRONT_MM = 2000.0;   // provisional; must be > every gate
const float APPROACH_FRONT_MM = 2000.0;      // start summing the turn direction
const float STRAIGHT_SIDE_WEIGHT[NUM_SENSORS] = {0.4, 0.6, 1.0, 1.0, 1.0, 0.6, 0.4};  // provisional

// ============================================================================
// Turn direction source (EXP-004)
//   TURN_DIR_BALANCE:  sign of the approach balance (above). Rejected for C2:
//     it depends on the lateral position; entering on the inside line picked
//     the wrong side at hairpins (1) and (2) (EXP-002 run 9, EXP-003 run 1).
//   TURN_DIR_SEQUENCE: the n-th corner entry after boot uses
//     CORNER_SEQUENCE[(CORNER_SEQUENCE_START + n) % CORNER_SEQUENCE_LEN].
//     Risk: a missed or false corner entry shifts every later corner.
// C2 order: hairpin (1) L, hairpin (2) R, hairpin (3) L, 90deg L, 90deg L.
// CORNER_SEQUENCE_START: index of the first corner after boot (0 = from the
// start line; set it when a steerpush starts mid-course, e.g. 1 = before
// hairpin (2)).
// ============================================================================
#define TURN_DIR_BALANCE 0
#define TURN_DIR_SEQUENCE 1
#define TURN_DIR_SOURCE TURN_DIR_SEQUENCE  // select turn direction source here

const uint8_t CORNER_SEQUENCE_LEN = 5;
const int8_t CORNER_SEQUENCE[CORNER_SEQUENCE_LEN] = {-1, 1, -1, -1, -1};
const uint8_t CORNER_SEQUENCE_START = 0;

// Corner-mode target (EXP-004): in corner mode, the target angle uses only
// the farthest sensor and its neighbor on the turn side. With both
// neighbors, S3 as the farthest got pulled toward the opposite side by its
// raw neighbor, and with Ld clamped near the wall the car steered +8 to +15
// deg (right) in a left hairpin (EXP-003 run 2).
const bool CORNER_ONE_SIDED_TARGET = true;

// ============================================================================
// Corner hold (EXP-005)
// Mid-turn, S3 starts to see down the next straight before the car has
// finished turning. EXP-004 hairpin (1): S3 jumped ~900 -> 2000-2500 mm,
// the target fell to ~-5 deg, Ld (S3 - offset) grew to ~1400 mm, St dropped
// from -30 to -1.7 deg and the corner exited, so the car ran wide.
//   CORNER_EXCLUDE_FRONT:   S3 is not a target candidate in corner mode.
//   CORNER_LD_MM:           fixed Ld in corner mode (0 = use S3 as usual).
//   CORNER_EXIT_DEBOUNCE:   consecutive frames above CORNER_EXIT_FRONT_MM
//                           needed to leave corner mode.
// ============================================================================
const bool CORNER_EXCLUDE_FRONT = true;
const float CORNER_LD_MM = 300.0;           // provisional
const uint8_t CORNER_EXIT_DEBOUNCE = 8;     // provisional (~320 ms)

// ============================================================================
// Next-corner side bias (EXP-005, TURN_DIR_SEQUENCE only)
// In straight mode, sensors on the side opposite to the next corner in
// CORNER_SEQUENCE are multiplied by NEXT_CORNER_OPPOSITE_WEIGHT, on top of
// STRAIGHT_SIDE_WEIGHT / LEFT_BIAS_WEIGHT. EXP-004: after hairpin (2) (next
// corner left), S4 saw 2400-3500 mm through the shortcut exit while S3 read
// ~1700 and the car turned into it. 1.0 = off.
// ============================================================================
const float NEXT_CORNER_OPPOSITE_WEIGHT = 0.4;  // provisional

// ============================================================================
// Hairpin edge entry (EXP-006, TURN_DIR_SEQUENCE only)
// The front gate (gate in CORNER_PARAMS) fires once the outer wall is
// ~1.1 m ahead, well after the inner divider of a hairpin has ended, so the
// car turned too late and hit the outer wall (EXP-005 hairpins (1) and (3):
// the inner side opened 35 / 37 frames before the gate fired).
// For corners with edgeMax > 0 (CORNER_PARAMS), straight mode also enters the
// corner when the inner-side sensor (S0 for a left turn, S6 for a right
// turn) goes from near to far:
//   near: read < EDGE_NEAR_MM within the last EDGE_NEAR_WINDOW frames;
//   far:  read > EDGE_FAR_MM for EDGE_FAR_FRAMES frames in a row;
//   and the filtered front is below that corner's edgeMax.
// That corner's delay frames later the corner starts (0 = at once).
// After an edge entry the front is still above CORNER_EXIT_FRONT_MM, so the
// exit only counts once F has dropped below it, or after
// EDGE_EXIT_ARM_FRAMES frames in the corner.
// No edge entry (edgeMax 0): hairpin (2) (the shortcut opening sits on its inner side) and
// the 90-deg corners (an S0 edge fired 56 frames early before 90L-2).
// If no edge shows up (different line), the front gate still enters.
// Replay of the EXP-005 logs: hairpin (1) 35 / 39 frames earlier, hairpin
// (3) 37 frames earlier in one run and no edge (gate fallback) in the other;
// no false edges on the flagged corners.
// ============================================================================
const uint16_t EDGE_NEAR_MM = 600;          // provisional
// History: EDGE_FAR_MM 1000 -> 1300 (EXP-007): S0 hovering at 1014-1032 on
//   the mock straight fired a false edge once edgeMax was raised (b0012);
//   real edges jump to 1550-2300 (10/29 h1 / h3, mock b0021).
const uint16_t EDGE_FAR_MM = 1300;          // provisional
const uint8_t EDGE_NEAR_WINDOW = 10;        // frames
const uint8_t EDGE_FAR_FRAMES = 2;          // frames
const uint8_t EDGE_EXIT_ARM_FRAMES = 25;    // frames (~1 s)

// ============================================================================
// Per-corner values (EXP-007), index = CORNER_SEQUENCE index
// The hairpins differ (divider end to outer wall: h1 220 cm, h2 190 cm,
// h3 185 cm; h1 has a curtain behind, the others a 120 mm wall), so the
// distance decisions are per corner. Runtime: "set N KEY V" (serial).
//   gate:    front gate, enter when F < gate for CORNER_ENTER_DEBOUNCE frames
//   edgeMax: hairpin edge entry (above) only while F < edgeMax; 0 = no edge
//            entry for this corner (was HAIRPIN_EDGE_ENTRY / EDGE_MAX_FRONT_MM)
//   delay:   frames from the edge to the corner start (EDGE_ENTRY_DELAY);
//            raise if the inner rear wheel cuts the divider tip
//   exitIn:  exit needs, besides F > CORNER_EXIT_FRONT_MM for
//            CORNER_EXIT_DEBOUNCE frames, the inner-side sensor (S0 left,
//            S6 right) valid and < exitIn in one of those frames: the
//            divider is beside the car again. 0 = off. Without it, after
//            CORNER_EXIT_FALLBACK frames above CORNER_EXIT_FRONT_MM the
//            corner exits anyway.
// Why exitIn: on the outer line the front opens to 2200-2400 at about
// 90 deg into a hairpin and the corner exited there (1107 mock b0019,
// b0020, b0022: S0 2000-2300 at that point); the real exit (inner line,
// b0021) had S0 700-950. Not for h2: the shortcut opening is on its inner
// side, and the mock h2 had S6 > 2000 at its exit.
// History:
//   edgeMax 2500 (h1, h3): raised (mock h1 edge at F 2690 did not fire;
//     10/29 venue h1 2074-2189, h3 2358-2380: little margin at h3).
//   edgeMax 3000: rejected (replay: false edges on the mock straights at
//     F 2863 / 2884). 2700 until tuned at the venue.
// ============================================================================
struct CornerParams {
    uint16_t gate;     // mm
    uint16_t edgeMax;  // mm, 0 = off
    uint8_t delay;     // frames
    uint16_t exitIn;   // mm, 0 = off
};
const CornerParams CORNER_PARAMS[CORNER_SEQUENCE_LEN] = {
    // gate  edgeMax  delay  exitIn
    {1200,   2700,    0,     1200},  // 0 h1  (left)
    {1200,   0,       0,     0},     // 1 h2  (right)
    {1200,   2700,    0,     1200},  // 2 h3  (left)
    {1200,   0,       0,     0},     // 3 90L-1
    {1200,   0,       0,     0},     // 4 90L-2
};
const char* const CORNER_NAMES[CORNER_SEQUENCE_LEN] = {"h1", "h2", "h3", "90L-1", "90L-2"};
const uint8_t CORNER_EXIT_FALLBACK = 30;  // frames (~1.2 s); mock early exits had ~20

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
const uint8_t L1X_ROI_SIZE[NUM_SENSORS] = {16, 16, 16, 6, 16, 16, 16};
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

// Straight-mode floor for Ld (EXP-006). Near a wall, S3 - offset drops to
// the 50 mm clamp and a small target becomes a large steer: EXP-005 frame
// 1012, T +2.3 deg with S3 1135 gave St +18.5 (right) just before hairpin (3).
// With 300 the same frame gives ~+3 deg. Corner mode uses CORNER_LD_MM.
const float STRAIGHT_LD_MIN_MM = 300.0;  // provisional

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
// Emergency stop in corner mode keeps the computed steering instead of
// centering it (the drive is cut either way). EXP-005 hairpin (1): S3 < 400
// mid-turn centered the servo and the car went straight into the outer wall.
const bool EMERGENCY_KEEP_STEER_IN_CORNER = true;

// Auto-stop timeout (debug aid).
// 0 disables it; otherwise the car stops after this many seconds.
const unsigned long AUTO_STOP_SECONDS = 0;
//const unsigned long AUTO_STOP_SECONDS = 20;

// ============================================================================
// Servo / ESC pulse widths
// ============================================================================
// Steering servo. New servo nominal range is 1200(R)-1500(C)-1800(L).
// History:
//   1115(R)-1415(C)-1715(L): rejected 2026-10-29 (C2 steerpush drifted right
//     with the steering near center).
//   1200(R)-1500(C)-1800(L): in use. EXP-003 drifted left with St +0.7 to
//     +1 deg, but that run also had the corner steering bug.
//   1180(R)-1480(C)-1780(L): tried 2026-10-29 (EXP-004); no clear drift
//     either way, reverted to the nominal range.
//   1115(R)-1415(C)-1715(L): back in use 2026-11-07 (the center seemed off
//     with 1500 in a static check).
// Steering maps onto MIN..MAX, so shift all three together when trimming.
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
const uint16_t SPEED_US = 1500;      // cruise pulse (us)

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
