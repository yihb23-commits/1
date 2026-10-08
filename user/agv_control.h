#ifndef AGV_CONTROL_H
#define AGV_CONTROL_H
#include <stdint.h>
#include "agv_config.h"
typedef enum { AGV_IDLE, AGV_FOLLOW, AGV_APPROACH, AGV_TURN,
               AGV_DONE, AGV_FAULT } AgvState;
typedef enum { AGV_OK, AGV_BAD_CONFIG, AGV_BAD_SENSOR, AGV_LOST,
    AGV_CORNER_MISSED, AGV_BAD_TURN, AGV_BAD_APPROACH, AGV_TIMEOUT,
    AGV_OVERRUN, AGV_USER_STOP } AgvFault;
typedef struct {
    uint16_t gray[8]; /* left to right, 0 white .. 1000 black */
    uint16_t gray_raw[8]; /* left to right, ADC 0..4095 or digital 0/1 */
    int16_t delta_left, delta_right; /* signed wheel counts over 10ms */
} AgvInput;
typedef struct {
    AgvState state;
    AgvFault fault;
    uint8_t lap, corners, corner_ticks, capture_ticks, lost_ticks, line_valid;
    uint32_t elapsed_ms, state_ms;
    float segment_m, advance_m, turn_rad;
    float speed_left, speed_right, target_left, target_right;
    float integral_left, integral_right;
    float error, last_error, derivative, pwm_left, pwm_right;
} AgvControl;
extern const volatile float agv_counts_per_wheel;
void Agv_Init(AgvControl *c);
void Agv_Start(AgvControl *c, uint8_t lap);
void Agv_Stop(AgvControl *c, AgvFault reason);
void Agv_Update(AgvControl *c, const AgvInput *in);
#endif
