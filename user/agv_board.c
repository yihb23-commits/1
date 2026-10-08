#include "stm32f10x.h"
#include "agv_board.h"
static volatile uint32_t milliseconds;
static uint16_t previous_left, previous_right;
#if AGV_GRAY_ANALOG
static uint16_t raw[8], white[8], black[8];
static uint8_t white_set, black_set;
static const uint8_t channels[8] = {0,1,2,3,4,5,8,9};
#endif
void Board_Tick(void) { ++milliseconds; }
uint32_t Board_Millis(void) { return milliseconds; }
static void gpio(GPIO_TypeDef *port, uint16_t pins, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef g;
    g.GPIO_Pin=pins; g.GPIO_Mode=mode; g.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(port, &g);
}
static void encoder(TIM_TypeDef *timer)
{
    TIM_TimeBaseInitTypeDef t;
    TIM_ICInitTypeDef ic;
    TIM_TimeBaseStructInit(&t); t.TIM_Period=65535;
    TIM_TimeBaseInit(timer, &t);
    TIM_EncoderInterfaceConfig(timer, TIM_EncoderMode_TI12,
                              TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&ic); ic.TIM_ICFilter=6;
    ic.TIM_Channel=TIM_Channel_1; TIM_ICInit(timer, &ic);
    ic.TIM_Channel=TIM_Channel_2; TIM_ICInit(timer, &ic);
    TIM_SetCounter(timer, 0); TIM_Cmd(timer, ENABLE);
}
void Board_Init(void)
{
    TIM_TimeBaseInitTypeDef t;
    TIM_OCInitTypeDef oc;
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock;
#if AGV_GRAY_ANALOG
    ADC_InitTypeDef a;
    uint32_t timeout;
#endif
    SystemCoreClockUpdate();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|
        RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); /* retain SWD */
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); /* PA15,PB3 */
    gpio(GPIOA, GPIO_Pin_15|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_IPU);
    gpio(GPIOB, GPIO_Pin_3, GPIO_Mode_IPU);
    gpio(GPIOB, GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10, GPIO_Mode_Out_PP);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); /* standby */
    gpio(GPIOB, GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_AF_PP);
    gpio(GPIOB, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15, GPIO_Mode_IPU);
    gpio(GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);
    encoder(TIM2); encoder(TIM3);
    RCC_GetClocksFreq(&clocks); timer_clock=clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) timer_clock *= 2U;
    TIM_TimeBaseStructInit(&t);
    t.TIM_Prescaler=(uint16_t)(timer_clock/2000000U-1U);
    t.TIM_Period=99; /* 20kHz PWM */
    TIM_TimeBaseInit(TIM4, &t);
    TIM_OCStructInit(&oc); oc.TIM_OCMode=TIM_OCMode_PWM1;
    oc.TIM_OutputState=TIM_OutputState_Enable; oc.TIM_Pulse=0;
    TIM_OC1Init(TIM4, &oc); TIM_OC2Init(TIM4, &oc);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); TIM_Cmd(TIM4, ENABLE);
#if AGV_GRAY_ANALOG
    gpio(GPIOA, 0x003F, GPIO_Mode_AIN); gpio(GPIOB, GPIO_Pin_0|GPIO_Pin_1, GPIO_Mode_AIN);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_StructInit(&a); a.ADC_Mode=ADC_Mode_Independent;
    a.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
    a.ADC_DataAlign=ADC_DataAlign_Right; a.ADC_NbrOfChannel=1;
    ADC_Init(ADC1, &a); ADC_Cmd(ADC1, ENABLE);
    for (timeout=0; timeout<1000U; ++timeout) __NOP();
    ADC_ResetCalibration(ADC1); timeout=100000U;
    while (ADC_GetResetCalibrationStatus(ADC1) && --timeout) {}
    if (timeout) {
        ADC_StartCalibration(ADC1); timeout=100000U;
        while (ADC_GetCalibrationStatus(ADC1) && --timeout) {}
    }
    if (!timeout) ADC_Cmd(ADC1, DISABLE);
#else
    gpio(GPIOA, 0x003F, GPIO_Mode_IPU); gpio(GPIOB, GPIO_Pin_0|GPIO_Pin_1, GPIO_Mode_IPU);
#endif
    if (SysTick_Config(SystemCoreClock/1000U)) for (;;) {}
}
static int16_t delta(uint16_t current, uint16_t previous, int sign)
{
    int32_t d=(int32_t)current-(int32_t)previous;
    if (d>32767) d-=65536;
    if (d< -32768) d+=65536;
    return (int16_t)(d*sign);
}
uint8_t Board_Read(AgvInput *in)
{
    uint16_t l=(uint16_t)TIM_GetCounter(TIM2), r=(uint16_t)TIM_GetCounter(TIM3);
    unsigned int i, destination;
#if AGV_GRAY_ANALOG
    uint32_t timeout;
    int32_t span, normalized;
#else
    uint8_t high;
#endif
    in->delta_left=delta(l,previous_left,AGV_ENCODER_LEFT_SIGN);
    in->delta_right=delta(r,previous_right,AGV_ENCODER_RIGHT_SIGN);
    previous_left=l; previous_right=r;
    for (i=0; i<8; ++i) {
        destination=AGV_GRAY_REVERSE ? 7U-i : i;
#if AGV_GRAY_ANALOG
        timeout=10000U;
        normalized=0;
        if (!(ADC1->CR2 & ADC_CR2_ADON)) return 0;
        ADC_RegularChannelConfig(ADC1,channels[i],1,ADC_SampleTime_55Cycles5);
        ADC_ClearFlag(ADC1,ADC_FLAG_EOC); ADC_SoftwareStartConvCmd(ADC1,ENABLE);
        while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) && --timeout) {}
        if (!timeout) return 0;
        raw[i]=ADC_GetConversionValue(ADC1);
        in->gray_raw[destination]=raw[i];
        span=(int32_t)black[i]-white[i];
        if (white_set && black_set && (span>=AGV_GRAY_MIN_SPAN || span<= -AGV_GRAY_MIN_SPAN))
            normalized=((int32_t)raw[i]-white[i])*1000/span;
        if (normalized<0) normalized=0;
        if (normalized>1000) normalized=1000;
        in->gray[destination]=(uint16_t)normalized;
#else
        high=i<6 ? GPIO_ReadInputDataBit(GPIOA,(uint16_t)(1U<<i)) :
                           GPIO_ReadInputDataBit(GPIOB,(uint16_t)(1U<<(i-6U)));
        in->gray_raw[destination]=high;
        in->gray[destination]=(high != (AGV_GRAY_BLACK_LOW != 0)) ? 1000U : 0U;
#endif
    }
    return 1;
}
void Board_CalibrateWhite(void)
{
#if AGV_GRAY_ANALOG
    unsigned int i; for (i=0;i<8;++i) white[i]=raw[i]; white_set=1;
#endif
}
void Board_CalibrateBlack(void)
{
#if AGV_GRAY_ANALOG
    unsigned int i; for (i=0;i<8;++i) black[i]=raw[i]; black_set=1;
#endif
}
uint8_t Board_Ready(void)
{
#if AGV_GRAY_ANALOG
    unsigned int i;
    int32_t span;
    if (!white_set || !black_set) return 0;
    for (i=0;i<8;++i) {
        span=(int32_t)black[i]-white[i];
        if (span<AGV_GRAY_MIN_SPAN && span> -AGV_GRAY_MIN_SPAN) return 0;
    }
#endif
    return AGV_HARDWARE_CONFIRMED && AGV_COUNTS_PER_WHEEL>0;
}
uint8_t Board_Buttons(void)
{
    static uint8_t candidate,stable,ticks;
    uint8_t sample=(uint8_t)((~GPIO_ReadInputData(GPIOB)>>12)&7U);
    if (sample!=candidate) { candidate=sample; ticks=0; }
    else if (ticks<3U) { if (++ticks==3U) stable=sample; }
    return stable;
}
uint8_t Board_LapMode(void) { return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0; }
static void motor(float duty, uint16_t p1, uint16_t p2, uint8_t left)
{
    uint16_t pulse;
    if (duty>AGV_PWM_LIMIT) duty=AGV_PWM_LIMIT;
    if (duty< -AGV_PWM_LIMIT) duty= -AGV_PWM_LIMIT;
    if (left) TIM_SetCompare1(TIM4,0); else TIM_SetCompare2(TIM4,0);
    if (duty>0) { GPIO_ResetBits(GPIOB,p2); GPIO_SetBits(GPIOB,p1); }
    else if (duty<0) { GPIO_ResetBits(GPIOB,p1); GPIO_SetBits(GPIOB,p2); }
    else GPIO_SetBits(GPIOB,p1|p2); /* short brake */
    pulse=(uint16_t)((duty<0 ? -duty : duty)*100);
    if (left) TIM_SetCompare1(TIM4,pulse); else TIM_SetCompare2(TIM4,pulse);
}
void Board_Motor(float left, float right)
{
    if (!Board_Ready()) { GPIO_ResetBits(GPIOB,GPIO_Pin_10); return; }
    motor(left*AGV_MOTOR_LEFT_SIGN,GPIO_Pin_4,GPIO_Pin_5,1);
    motor(right*AGV_MOTOR_RIGHT_SIGN,GPIO_Pin_8,GPIO_Pin_9,0);
    GPIO_SetBits(GPIOB,GPIO_Pin_10);
}
void Board_Status(AgvState state)
{
    uint8_t on;
    if (state==AGV_FAULT) on=(milliseconds/150U)&1U;
    else if (state==AGV_IDLE) on=(milliseconds/600U)&1U;
    else on=state!=AGV_DONE;
    if (on) GPIO_ResetBits(GPIOC,GPIO_Pin_13); else GPIO_SetBits(GPIOC,GPIO_Pin_13);
}
