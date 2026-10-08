#include "stm32f10x.h"
#include "agv_board.h"
#include "agv_control.h"

/* Keil Watch: agv_debug, agv_input_debug. No launch after power-on. */
volatile AgvControl agv_debug;
volatile AgvInput agv_input_debug;

int main(void)
{
    AgvInput input;
    AgvControl car;
    uint32_t last, now;
    uint8_t buttons, pressed, previous = 0;
    Board_Init();
    Agv_Init(&car);
    last = Board_Millis();
    for (;;) {
        now = Board_Millis();
        if ((uint32_t)(now-last) < AGV_PERIOD_MS) continue;
        if ((uint32_t)(now-last) >= AGV_PERIOD_MS*2U) {
            Agv_Stop(&car,AGV_OVERRUN);
            last = now;
        } else last += AGV_PERIOD_MS;
        buttons = Board_Buttons();
        pressed = (uint8_t)(buttons & (uint8_t)~previous);
        previous = buttons;
        if (!Board_Read(&input)) Agv_Stop(&car,AGV_BAD_SENSOR);
        else {
            agv_input_debug = input;
            if (car.state==AGV_IDLE || car.state==AGV_DONE || car.state==AGV_FAULT) {
                if (pressed & 2U) Board_CalibrateWhite();
                if (pressed & 4U) Board_CalibrateBlack();
                if (pressed & 1U) {
                    if (Board_Ready()) Agv_Start(&car,Board_LapMode());
                    else Agv_Stop(&car,AGV_BAD_CONFIG);
                }
            } else if (pressed & 1U) Agv_Stop(&car,AGV_USER_STOP);
            Agv_Update(&car,&input);
        }
        Board_Motor(car.pwm_left,car.pwm_right);
        Board_Status(car.state);
        agv_debug = car;
    }
}
