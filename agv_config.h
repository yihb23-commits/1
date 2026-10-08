#ifndef AGV_CONFIG_H
#define AGV_CONFIG_H
/* Confirmed by the user's saved parameter sheet (2026-10-03):
 * Wheeltech C03C STM32F103C8T6 board, external 8MHz crystal, PC13 LED low-active.
 * Both motors: WHEELTECHH MG513XP28, rated 12V. Gear ratio/counts still unknown;
 * do not infer gear ratio from the model suffix "P28".
 * Gray module: 5V supply, analog AND digital outputs, L1..L8 left-to-right,
 * 8mm probe pitch, nominal width 60mm, listed working height 5..60mm.
 * Output voltage and actual mounting height are NOT yet confirmed.
 * Existing normalized equal-spacing weights also apply to an 8mm pitch. */
/* Supplied SI dimensions. 0.060m caster spacing is NOT the drive wheel base. */
#define AGV_WHEEL_RADIUS_M 0.033f
#define AGV_WHEEL_BASE_M   0.1192f
#define AGV_ENCODER_PPR    13.0f
/* TODO: encoder axis, gear ratio and definition of PPR are unknown.
 * TIM2/TIM3 use quadrature x4. If 13 is motor-shaft single-channel PPR:
 * counts/wheel = 13 * gear_ratio * 4. If it already includes x4, do NOT x4 again.
 * Best: measure actual counts for one wheel revolution and enter that total.
 * 0 deliberately prevents launch; never silently assume 13 counts/wheel. */
#ifndef AGV_COUNTS_PER_WHEEL
#define AGV_COUNTS_PER_WHEEL 0.0f
#endif
/* TODO: confirm proposed pin map, logic levels and polarity before setting 1. */
#define AGV_HARDWARE_CONFIRMED 0
#define AGV_ENCODER_LEFT_SIGN  1
#define AGV_ENCODER_RIGHT_SIGN 1
#define AGV_MOTOR_LEFT_SIGN    1
#define AGV_MOTOR_RIGHT_SIGN   1
/* Select the module's analog outputs for this default. TODO: verify output
 * stays within ADC range before wiring; 5V supply does not specify output level.
 * Set 0 and wire the module's digital outputs to use comparator mode instead.
 * Do not connect both outputs to the same MCU pin. */
#ifndef AGV_GRAY_ANALOG
#define AGV_GRAY_ANALOG       1
#endif
#define AGV_GRAY_BLACK_LOW    1 /* digital mode only */
#define AGV_GRAY_REVERSE      0 /* channel 0 = leftmost when viewed from behind */
#define AGV_GRAY_THRESHOLD    500U
#define AGV_GRAY_MIN_SPAN     200 /* minimum ADC black-white difference */
#define AGV_PERIOD_MS         10U
#define AGV_DT                ((float)AGV_PERIOD_MS / 1000.0f)
/* TODO: tune corner/capture signatures after observing eight-channel data.
 * Bit 0 = leftmost probe; bit 7 = rightmost probe. */
#define AGV_CORNER_RIGHT_MASK  0xC0U
#define AGV_CENTER_MASK        0x18U
#define AGV_CORNER_MIN_BLACK   4U
#define AGV_CAPTURE_OUTER_MASK 0xC3U
#define AGV_CAPTURE_MAX_BLACK  4U
/* INITIAL tuning values; must tune on actual car and track. */
#define AGV_CRUISE_MPS        0.22f
#define AGV_SLOW_MPS          0.10f
#define AGV_APPROACH_MPS      0.08f
#define AGV_TURN_MPS          0.09f
#define AGV_MAX_WHEEL_MPS     0.34f
#define AGV_LINE_KP           2.0f   /* rad/s per normalized line error */
#define AGV_LINE_KD           0.025f
#define AGV_SPEED_KP          0.8f   /* duty fraction per m/s */
#define AGV_SPEED_KI          1.5f
#define AGV_SPEED_FF          2.5f
#define AGV_SPEED_FILTER      0.3f
#define AGV_PWM_LIMIT         0.85f
/* TODO: measure sensor-row to driving-axle offset, default 60mm.
 * Advance axle onto vertex before pivoting; overhead view must confirm that
 * chassis projection always overlaps track. Counts cannot prove compliance. */
#define AGV_CORNER_ADVANCE_M  0.060f
/* TODO: stopping displacement from first corner detection; default 0.
 * Account for body length, array position and braking travel. */
#define AGV_STOP_ADVANCE_M    0.000f
#define AGV_CORNER_ARM_M      0.65f
#define AGV_SLOW_AFTER_M      0.78f
#define AGV_SEGMENT_MAX_M     1.20f
#define AGV_CORNER_TICKS      3U
#define AGV_CAPTURE_TICKS     4U
#define AGV_LOST_TICKS        8U
#define AGV_TURN_MIN_RAD      1.05f /* about 60deg: reject old edge */
#define AGV_TURN_MAX_RAD      1.92f /* about 110deg: fail */
#define AGV_TURN_TIMEOUT_MS   2500U
#define AGV_APPROACH_TIMEOUT_MS 1800U
#define AGV_AB_TIMEOUT_MS     10000U
#define AGV_LAP_TIMEOUT_MS    30000U
#endif
