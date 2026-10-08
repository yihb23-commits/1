#include "agv_control.h"
#include <string.h>
/* Volatile read keeps the deliberately unset configuration a runtime guard. */
const volatile float agv_counts_per_wheel = AGV_COUNTS_PER_WHEEL;
static float clamp(float v, float lo, float hi)
{ return v < lo ? lo : (v > hi ? hi : v); }
static float absf(float v) { return v < 0 ? -v : v; }
void Agv_Init(AgvControl *c)
{ memset(c, 0, sizeof(*c)); c->state = AGV_IDLE; }
void Agv_Stop(AgvControl *c, AgvFault reason)
{
    c->state = reason == AGV_OK ? AGV_DONE : AGV_FAULT;
    c->fault = reason;
    c->target_left = c->target_right = 0;
    c->pwm_left = c->pwm_right = 0;
    c->integral_left = c->integral_right = 0;
}
void Agv_Start(AgvControl *c, uint8_t lap)
{
    Agv_Init(c); c->lap = lap != 0;
    if (agv_counts_per_wheel <= 0) { Agv_Stop(c, AGV_BAD_CONFIG); return; }
    c->state = AGV_FOLLOW;
}
static void enter(AgvControl *c, AgvState state)
{
    c->state = state; c->state_ms = 0;
    c->integral_left = c->integral_right = 0;
    c->corner_ticks = c->capture_ticks = c->lost_ticks = 0;
    c->derivative = 0; c->line_valid = 0;
}
static float pi(float target, float speed, float *integral)
{
    float error, next, u;
    error = target - speed;
    next = clamp(*integral + AGV_SPEED_KI * error * AGV_DT, -0.4f, 0.4f);
    u = AGV_SPEED_FF * target + AGV_SPEED_KP * error + next;
    if ((u <= AGV_PWM_LIMIT && u >= -AGV_PWM_LIMIT) ||
        (u > AGV_PWM_LIMIT && error < 0) ||
        (u < -AGV_PWM_LIMIT && error > 0)) *integral = next;
    return clamp(AGV_SPEED_FF * target + AGV_SPEED_KP * error + *integral,
                 -AGV_PWM_LIMIT, AGV_PWM_LIMIT);
}
void Agv_Update(AgvControl *c, const AgvInput *in)
{
    unsigned int i, count = 0;
    float total = 0, weighted = 0, dl, dr, distance, v, omega, scale, advance;
    uint8_t mask = 0, corner, center, final_corner;
    if (c->state == AGV_IDLE || c->state == AGV_DONE || c->state == AGV_FAULT) return;
    scale = agv_counts_per_wheel;
    if (scale <= 0) { Agv_Stop(c, AGV_BAD_CONFIG); return; }
    scale = 6.283185307f * AGV_WHEEL_RADIUS_M / scale;
    dl = (float)in->delta_left * scale; dr = (float)in->delta_right * scale;
    distance = (dl + dr) * 0.5f;
    c->speed_left += AGV_SPEED_FILTER * (dl / AGV_DT - c->speed_left);
    c->speed_right += AGV_SPEED_FILTER * (dr / AGV_DT - c->speed_right);
    c->elapsed_ms += AGV_PERIOD_MS; c->state_ms += AGV_PERIOD_MS;
    if (c->elapsed_ms >= (c->lap ? AGV_LAP_TIMEOUT_MS : AGV_AB_TIMEOUT_MS)) {
        Agv_Stop(c, AGV_TIMEOUT); return;
    }
    for (i = 0; i < 8; ++i) {
        if (in->gray[i] >= AGV_GRAY_THRESHOLD) {
            mask |= (uint8_t)(1U << i); ++count;
            total += (float)in->gray[i];
            weighted += (float)in->gray[i] * ((float)i * 2 - 7) / 7;
        }
    }
    if (total > 0) c->error = weighted / total;
    /* TODO: validate this signature against actual array pitch and track.
     * Right crossbar + center black, only armed after sufficient travel.
     * Full-black can also qualify; do not add broad black markers. */
    corner = (mask & AGV_CORNER_RIGHT_MASK) == AGV_CORNER_RIGHT_MASK &&
             (mask & AGV_CENTER_MASK) != 0 && count >= AGV_CORNER_MIN_BLACK;
    center = (mask & AGV_CENTER_MASK) != 0 &&
             (mask & AGV_CAPTURE_OUTER_MASK) == 0 && count <= AGV_CAPTURE_MAX_BLACK;
    if (c->state == AGV_FOLLOW) {
        c->segment_m += distance;
        if (c->segment_m > AGV_SEGMENT_MAX_M) { Agv_Stop(c, AGV_CORNER_MISSED); return; }
        if (c->segment_m >= AGV_CORNER_ARM_M && corner) ++c->corner_ticks;
        else c->corner_ticks = 0;
        if (c->corner_ticks >= AGV_CORNER_TICKS) {
            c->advance_m = 0; enter(c, AGV_APPROACH);
            c->target_left = c->target_right = AGV_APPROACH_MPS;
        } else {
            if (!mask) {
                c->line_valid = 0; c->derivative = 0;
                if (++c->lost_ticks >= AGV_LOST_TICKS) { Agv_Stop(c, AGV_LOST); return; }
            } else {
                c->lost_ticks = 0;
                if (c->line_valid) c->derivative += 0.25f *
                    ((c->error - c->last_error) / AGV_DT - c->derivative);
                else c->derivative = 0;
                c->last_error = c->error; c->line_valid = 1;
            }
            v = c->segment_m >= AGV_SLOW_AFTER_M || !mask ? AGV_SLOW_MPS : AGV_CRUISE_MPS;
            v /= 1 + 1.5f * absf(c->error);
            omega = clamp(AGV_LINE_KP*c->error + AGV_LINE_KD*c->derivative, -2, 2);
            c->target_left = v + omega*AGV_WHEEL_BASE_M*0.5f;
            c->target_right = v - omega*AGV_WHEEL_BASE_M*0.5f;
        }
    } else if (c->state == AGV_APPROACH) c->advance_m += distance;
    else if (c->state == AGV_TURN) c->turn_rad += (dl - dr) / AGV_WHEEL_BASE_M;
    /* Process transitions this tick, including zero-distance stop. */
    if (c->state == AGV_APPROACH) {
        final_corner = !c->lap || c->corners == 3U;
        advance = final_corner ? AGV_STOP_ADVANCE_M : AGV_CORNER_ADVANCE_M;
        if (c->advance_m >= advance) {
            if (final_corner) { Agv_Stop(c, AGV_OK); return; }
            c->turn_rad = 0; enter(c, AGV_TURN);
            c->target_left = AGV_TURN_MPS; c->target_right = -AGV_TURN_MPS;
        } else if (c->state_ms >= AGV_APPROACH_TIMEOUT_MS) {
            Agv_Stop(c, AGV_BAD_APPROACH); return;
        }
    }
    if (c->state == AGV_TURN) {
        if (c->turn_rad > AGV_TURN_MAX_RAD || c->turn_rad < -0.2f ||
            c->state_ms >= AGV_TURN_TIMEOUT_MS) { Agv_Stop(c, AGV_BAD_TURN); return; }
        if (c->turn_rad >= AGV_TURN_MIN_RAD && center) ++c->capture_ticks;
        else c->capture_ticks = 0;
        if (c->capture_ticks >= AGV_CAPTURE_TICKS) {
            ++c->corners; c->segment_m = 0; enter(c, AGV_FOLLOW);
            c->target_left = c->target_right = AGV_SLOW_MPS;
        }
    }
    c->target_left = clamp(c->target_left, -AGV_MAX_WHEEL_MPS, AGV_MAX_WHEEL_MPS);
    c->target_right = clamp(c->target_right, -AGV_MAX_WHEEL_MPS, AGV_MAX_WHEEL_MPS);
    c->pwm_left = pi(c->target_left, c->speed_left, &c->integral_left);
    c->pwm_right = pi(c->target_right, c->speed_right, &c->integral_right);
}
