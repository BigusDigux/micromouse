#include "motion.h"
#include "encoder.h"
#include "vl6180x.h"
#include "drv8833.h"
#include "MPU.h"
#include <math.h>
#include <stdlib.h>

/* ==================== Configuration Constants ==================== */
#define MOTOR_R_GAIN        1.10f   // Right motor gain for symmetry
#define MIN_FWD_SPEED       140     // Minimum forward speed
#define TOF_ALPHA           0.5f    // Low-pass filter smoothing for ToF (0..1)
#define KP_SIDE             1.0f    // Proportional gain for side difference (mm)
#define KD_SIDE             3.0f    // Derivative gain for side difference (mm/loop)
#define KENC                1.0f    // Encoder balance gain (ticks)
#define E_BAD_THRESH        12.0f   // Large lateral error threshold (mm)
#define DE_BAD_THRESH       20.0f   // Large angle rate threshold (mm/loop)
#define CENTER_MM_DEFAULT   100.0f  // Default single-wall target distance (mm)
#define CENTER_ALPHA        0.05f   // Low-pass filter for centerline (slower = smoother)
#define FRONT_WALL_THRESH   80.0f   // Front wall distance threshold (mm)
#define CORNER_THRESH       20.0f   // Sudden distance drop for corner detection (mm)
#define DE_MAX              15.0f   // Max derivative to limit twitchiness (mm/loop)

/* ==================== Configuration for Curve Turn ==================== */
#define KP_YAW              15.0f    // Proportional gain for yaw control
#define BASE_CURVE_SPEED    200     // Base speed for outer wheel during curve
#define MIN_CURVE_SPEED     50      // Minimum speed for inner wheel
#define YAW_TOLERANCE       0.5f    // Yaw error tolerance for completion (degrees)

/* ==================== Helper Functions ==================== */
static inline int clamp_int(int value, int min, int max) {
    return (value < min) ? min : (value > max) ? max : value;
}

static inline float clamp_float(float value, float min, float max) {
    return (value < min) ? min : (value > max) ? max : value;
}

static inline int get_base_speed(void) {
    extern int selectedSpeedIndex; // Defined in main.c
    return (selectedSpeedIndex == 0) ? SPEED_MEDIUM : SPEED_FAST;
}

static inline bool is_tof_valid(uint8_t distance) {
    return (distance > 0) && (distance <= SENSOR_FRONT_LIMIT);
}

static inline float low_pass_filter(float previous, float current, float alpha) {
    return previous + alpha * (current - previous);
}

/* ==================== Motion Control Functions ==================== */
void reset_motion(void) {
    DRV8833_Brake(&motorL);
    DRV8833_Brake(&motorR);
    DRV8833_SetSpeed(&motorL, 0);
    DRV8833_SetSpeed(&motorR, 0);
    ENCODER_ResetLeft();
    ENCODER_ResetRight();
}

/**
 * Drives forward for a specified number of cells, centering between walls using ToF sensors.
 * Fuses lateral error (left-right distance) with its derivative and encoder balance.
 * Realigns when a front wall is detected and mitigates twitchiness at corners.
 */
void driveForward(int cells) {
    if (cells <= 0) return;

    // Adjust ticks per cell based on speed profile
    int ticks_per_cell = TICKS_PER_CELL;
    extern int selectedSpeedIndex;
    if (selectedSpeedIndex == 1) {
        ticks_per_cell = (TICKS_PER_CELL - TICK_FAST) < 0 ? 0 : (TICKS_PER_CELL - TICK_FAST);
    }

    const int target_ticks = ticks_per_cell * cells;
    const int base_speed = get_base_speed();

    ENCODER_ResetLeft();
    ENCODER_ResetRight();

    // Initialize filter states
    float left_filtered = 0.0f, right_filtered = 0.0f;
    float left_prev_raw = 0.0f, right_prev_raw = 0.0f;
    float center_mm = CENTER_MM_DEFAULT;
    float prev_error = 0.0f;
    bool is_initialized = false;

    uint32_t last_update = HAL_GetTick();
    uint32_t turn_cooldown_end = HAL_GetTick() + 500;

    while (true) {
        if (HAL_GetTick() - last_update < LOOP_DT_MS) continue;
        last_update = HAL_GetTick();

        // Read encoder counts
        int left_count = ENCODER_GetLeft();
        int right_count = ENCODER_GetRight();
        int avg_count = (left_count + right_count) / 2;

        // Read ToF sensors
        uint8_t left_raw = VL6180X_ReadRange(&tofLeft);
        uint8_t right_raw = VL6180X_ReadRange(&tofRight);
        uint8_t front_raw = VL6180X_ReadRange(&tofFront);
        bool left_valid = is_tof_valid(left_raw);
        bool right_valid = is_tof_valid(right_raw);
        bool front_valid = is_tof_valid(front_raw) && front_raw <= FRONT_WALL_THRESH;

        // Detect corners (sudden distance changes)
        bool is_corner = false;
        if (left_valid && is_initialized && fabsf((float)left_raw - left_prev_raw) > CORNER_THRESH) {
            is_corner = true;
        }
        if (right_valid && is_initialized && fabsf((float)right_raw - right_prev_raw) > CORNER_THRESH) {
            is_corner = true;
        }
        left_prev_raw = left_valid ? (float)left_raw : left_prev_raw;
        right_prev_raw = right_valid ? (float)right_raw : right_prev_raw;

        // Low-pass filter ToF readings
        if (!is_initialized) {
            if (left_valid) left_filtered = left_raw;
            if (right_valid) right_filtered = right_raw;
            is_initialized = true;
        } else {
            if (left_valid) left_filtered = low_pass_filter(left_filtered, (float)left_raw, is_corner ? 0.2f : TOF_ALPHA);
            if (right_valid) right_filtered = low_pass_filter(right_filtered, (float)right_raw, is_corner ? 0.2f : TOF_ALPHA);
        }

        // Calculate centering error
        float error = 0.0f;
        bool have_both_sides = left_valid && right_valid;
        if (front_valid && (left_valid || right_valid)) {
            error = left_valid ? (left_filtered - center_mm) : (center_mm - right_filtered);
        } else if (have_both_sides) {
            error = left_filtered - right_filtered;
            center_mm = low_pass_filter(center_mm, (left_filtered + right_filtered) * 0.5f, CENTER_ALPHA);
        } else if (left_valid) {
            error = left_filtered - center_mm;
        } else if (right_valid) {
            error = center_mm - right_filtered;
        }

        float error_derivative = clamp_float(error - prev_error, -DE_MAX, DE_MAX);
        prev_error = error;

        // Adjust forward speed
        int forward_speed = base_speed;
        float slow_scale = 1.0f;
        if (is_corner || (front_valid && (left_valid || right_valid))) {
            slow_scale = 0.5f;
        } else if (fabsf(error) > E_BAD_THRESH || fabsf(error_derivative) > DE_BAD_THRESH) {
            slow_scale = 0.55f;
        } else if (have_both_sides) {
            slow_scale = 0.85f + 0.15f * (1.0f - clamp_float(fabsf(error) / E_BAD_THRESH, 0.0f, 1.0f));
        }
        forward_speed = (int)(forward_speed * slow_scale);
        if (forward_speed < MIN_FWD_SPEED) forward_speed = MIN_FWD_SPEED;

        // Apply steering correction
        float kp = is_corner ? (KP_SIDE * 0.5f) : KP_SIDE;
        float kd = is_corner ? (KD_SIDE * 0.5f) : KD_SIDE;
        float enc_correction = KENC * (float)(left_count - right_count);
        float side_correction = (left_valid || right_valid) ? (kp * error + kd * error_derivative) : 0.0f;
        float correction = enc_correction + side_correction;

        int left_cmd = clamp_int((int)(forward_speed - correction), SPEED_MIN, SPEED_MAX);
        int right_cmd = clamp_int((int)((forward_speed + correction) * MOTOR_R_GAIN), SPEED_MIN, SPEED_MAX);

        DRV8833_SetSpeed(&motorL, left_cmd);
        DRV8833_SetSpeed(&motorR, right_cmd);

        // Check stop conditions
        extern int selectedTurnIndex;
        if (avg_count >= target_ticks) {
            //reset_motion();
            break;
        }

        if (HAL_GetTick() > turn_cooldown_end && is_tof_valid(front_raw)) {
            if ((selectedTurnIndex == 0 && front_raw <= 90) ||
                (selectedTurnIndex == 1 && front_raw <= 150)) {
                reset_motion();
                break;
            }
        }
    }
}

/**
 * Performs a pivot turn to a specified angle using MPU yaw feedback.
 */
void turn_pivot(float target_deg) {
    if (fabsf(target_deg) < 0.1f) return;

    reset_motion();
    MPU_Update();

    float desired_yaw = MPU_GetYaw() + target_deg;
    if (desired_yaw > 180.0f) desired_yaw -= 360.0f;
    if (desired_yaw < -180.0f) desired_yaw += 360.0f;

    int direction = (target_deg > 0) ? 1 : -1;
    int expected_ticks = (int)(TICKS_PER_TURN * (fabsf(target_deg) / 90.0f));

    ENCODER_ResetLeft();
    ENCODER_ResetRight();
    uint32_t last_update = HAL_GetTick();

    while (true) {
        if (HAL_GetTick() - last_update < LOOP_DT_MS) continue;
        last_update = HAL_GetTick();

        MPU_Update();
        float error = desired_yaw - MPU_GetYaw();
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        if (fabsf(error) < 0.1f) {
            reset_motion();
            break;
        }

        int base_turn_speed = clamp_int((int)(2.0f * fabsf(error)), 100, TURN_BASE_SPEED);
        if (fabsf(error) < 45.0f) {
            base_turn_speed = clamp_int((int)(base_turn_speed * fabsf(error) / 45.0f), 100, base_turn_speed);
        }

        DRV8833_SetSpeed(&motorL, -direction * base_turn_speed);
        DRV8833_SetSpeed(&motorR, direction * (int)(base_turn_speed * 1.2f));

        if (abs(ENCODER_GetLeft() - ENCODER_GetRight()) / 2 > (int)(expected_ticks * 1.2f)) {
            reset_motion();
            break;
        }
    }
}

/**
 * Performs a curved turn to a specified angle using differential wheel speeds and MPU yaw feedback.
 */
void turn_curve(float target_deg) {
    if (fabsf(target_deg) < 0.1f) return;

    reset_motion();
    MPU_Update();

    ENCODER_ResetLeft();
    ENCODER_ResetRight();

    float desired_yaw = MPU_GetYaw() + target_deg;
    if (desired_yaw > 180.0f) desired_yaw -= 360.0f;
    if (desired_yaw < -180.0f) desired_yaw += 360.0f;

    int direction = (target_deg > 0) ? 1 : -1; // Positive for left turn, negative for right

    ENCODER_ResetLeft();
    ENCODER_ResetRight();
    uint32_t last_update = HAL_GetTick();

    while (true) {
        if (HAL_GetTick() - last_update < LOOP_DT_MS) continue;
        last_update = HAL_GetTick();

        MPU_Update();
        float error = desired_yaw - MPU_GetYaw();
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        if (fabsf(error) < YAW_TOLERANCE) {
            reset_motion();
            break;
        }

        // Proportional control for smooth turning
        float correction = KP_YAW * error;
        int outer_speed = BASE_CURVE_SPEED;
        int inner_speed = clamp_int((int)(BASE_CURVE_SPEED - fabsf(correction)), MIN_CURVE_SPEED, BASE_CURVE_SPEED);

        // Apply speeds based on turn direction
        if (direction > 0) { // Left turn: left wheel slower
            DRV8833_SetSpeed(&motorL, inner_speed);
            DRV8833_SetSpeed(&motorR, (int)(outer_speed * MOTOR_R_GAIN));
        } else { // Right turn: right wheel slower
            DRV8833_SetSpeed(&motorL, outer_speed);
            DRV8833_SetSpeed(&motorR, (int)(inner_speed * MOTOR_R_GAIN));
        }
    }
}

/**
 * Placeholder for diagonal movement.
 */
void turn_diagonal(void) {
    reset_motion();
    int base_speed = get_base_speed();
    DRV8833_SetSpeed(&motorL, base_speed);
    DRV8833_SetSpeed(&motorR, (int)(base_speed * MOTOR_R_GAIN));
    HAL_Delay(400);
    reset_motion();
}

/**
 * Wrapper for 90-degree turns based on selected turn mode.
 */
void turn90(bool left) {
    extern int selectedTurnIndex;
    switch (selectedTurnIndex) {
        case 0: turn_pivot(left ? 90.0f : -90.0f); break;
        case 1: turn_curve(left ? 90.0f : -90.0f); break;
        case 2: turn_diagonal(); break;
        default: break;
    }
}

/**
 * Performs a 180-degree pivot turn.
 */
void turn180(void) {
    turn_pivot(180.0f);
}
