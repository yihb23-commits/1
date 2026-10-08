#ifndef AGV_BOARD_H
#define AGV_BOARD_H
#include "agv_control.h"
void Board_Init(void);
uint32_t Board_Millis(void);
uint8_t Board_Read(AgvInput *in);
uint8_t Board_Buttons(void);
uint8_t Board_LapMode(void);
uint8_t Board_Ready(void);
void Board_CalibrateWhite(void);
void Board_CalibrateBlack(void);
void Board_Motor(float left, float right);
void Board_Status(AgvState state);
void Board_Tick(void);
#endif
