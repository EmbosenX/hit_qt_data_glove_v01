/*******************************************************************************
 * @file        adc_sample.c
 * @brief       Y0~Y34输出控制 + AS模拟开关 + PALM多通道ADC采集底层驱动
 * @author      
 * @date        2026-07-21
 * @version     V1.1  对齐原厂DDL规范优化ADC底层
 ******************************************************************************/
#include "adc_sample.h"
#include <string.h>
#include "app_tac_config.h"
#include "main.h"

/* ADC基础配置 */
#define ADC_UNIT       			(CM_ADC1)
#define ADC_PERIPH_CLK      (FCG3_PERIPH_ADC1)
#define ADC_SEQ        			ADC_SEQ_A
#define ADC_EOC_FLAG   			ADC_FLAG_EOCA

/* ADC通道引脚定义 */
#define AD0_ADC_PORT       GPIO_PORT_A
#define AD0_ADC_PIN        GPIO_PIN_00
#define AD0_ADC_CH         ADC_CH0
#define AD0_ADC_PGA_PIN    ADC_PGA_PIN_ADC1_PA0

#define AD1_ADC_PORT       GPIO_PORT_A
#define AD1_ADC_PIN        GPIO_PIN_01
#define AD1_ADC_CH         ADC_CH1
#define AD1_ADC_PGA_PIN    ADC_PGA_PIN_ADC1_PA1

#define AD2_ADC_PORT       GPIO_PORT_A
#define AD2_ADC_PIN        GPIO_PIN_02
#define AD2_ADC_CH         ADC_CH2
#define AD2_ADC_PGA_PIN    ADC_PGA_PIN_ADC1_PA2

#define AD3_ADC_PORT       GPIO_PORT_A
#define AD3_ADC_PIN        GPIO_PIN_03
#define AD3_ADC_CH         ADC_CH3
#define AD3_ADC_PGA_PIN    ADC_PGA_PIN_ADC1_PA3

#define AD4_ADC_PORT       GPIO_PORT_A
#define AD4_ADC_PIN        GPIO_PIN_04
#define AD4_ADC_CH         ADC_CH4
#define AD4_ADC_PGA_PIN    ADC_PGA_PIN_ADC1_PA4

#define AD5_ADC_PORT       GPIO_PORT_A
#define AD5_ADC_PIN        GPIO_PIN_05
#define AD5_ADC_CH         ADC_CH5
#define AD5_ADC_PGA_PIN    ADC_PGA_PIN_ADC1_PA5

#define AD6_ADC_PORT       GPIO_PORT_A
#define AD6_ADC_PIN        GPIO_PIN_06
#define AD6_ADC_CH         ADC_CH6
#define AD6_ADC_PGA_PIN    ADC_PGA_PIN_ADC1_PA6

#define AD_ADC_PGA_UNIT     (ADC_PGA1)
#define AD_ADC_PGA_GAIN     (ADC_PGA_GAIN_4)
#define AD_ADC_PGA_VSS      (ADC_PGA_VSS_AVSS)

/* ADC参数配置 */
#define ADC_VREF            (3.3F)
#define ADC_ACCURACY        (1UL << 12U)
#define ADC_TIMEOUT_VAL     (1000U)
#define ADC_SAMPLE_TIME     (0x20U)    /* 采样周期，加长适配高阻传感器 */

#define SAMPLER_MUX_CHANNELS   (8U)

/* ====================== 静态内部函数声明 ====================== */
static void GPIO_Y_Init(void);
static void GPIO_AS_Init(void);
static void ADC_Palm_Init(void);
static void AdcClockConfig(void);
static void SetSinglePin(uint8_t u8Port, uint16_t u16Pin, en_pin_level_t enLevel);
static void sampler_switch_cols(uint8_t AREA,uint8_t N);
static void Y_Channel_AllLow(void);
static void AS_Chip_Enable(en_as_chip_t enChip);
static void AS_Channel_Select(en_as_channel_t enCh);
static void AS_AllChip_Disable(void);
static int32_t AdcPollingSample(uint8_t adx, uint16_t *pu16AdcVal);

/* ====================== 管脚映射表：Y编号 -> PORT/PIN ====================== */
typedef struct
{
    uint8_t 			pstPort;
    uint16_t      u16Pin;
} stc_y_pin_map_t;

static const stc_y_pin_map_t m_stYPinMap[] =
{
		// 扫描顺序：Y6(0) → Y5(1) → Y4(2) → Y3(3) → Y2(4) → Y1(5) → Y0(6)
		{PIN_TY6_PORT,  PIN_TY6_PIN},  // Y6 0
		{PIN_TY5_PORT,  PIN_TY5_PIN},  // Y5 1
		{PIN_TY4_PORT,  PIN_TY4_PIN},  // Y4 2 
		{PIN_TY3_PORT,  PIN_TY3_PIN},  // Y3 3
		{PIN_TY2_PORT,  PIN_TY2_PIN},  // Y2 4
		{PIN_TY1_PORT,  PIN_TY1_PIN},  // Y1 5
		{PIN_TY0_PORT,  PIN_TY0_PIN},  // Y0 6
		
    {PIN_IY0_PORT,  PIN_IY0_PIN},  // Y0 13
    {PIN_IY1_PORT,  PIN_IY1_PIN},  // Y1 12 
    {PIN_IY2_PORT,  PIN_IY2_PIN},  // Y2 11
    {PIN_IY3_PORT,  PIN_IY3_PIN},  // Y3 10
    {PIN_IY4_PORT,  PIN_IY4_PIN},  // Y4 9
    {PIN_IY5_PORT,  PIN_IY5_PIN},  // Y5 8
    {PIN_IY6_PORT,  PIN_IY6_PIN},  // Y6 7
		
    {PIN_RY0_PORT,  PIN_RY0_PIN},  // Y0 20
    {PIN_RY1_PORT,  PIN_RY1_PIN},  // Y1 19
    {PIN_RY2_PORT,  PIN_RY2_PIN},  // Y2 18
    {PIN_RY3_PORT,  PIN_RY3_PIN},  // Y3 17 
    {PIN_RY4_PORT,  PIN_RY4_PIN},  // Y4 16
    {PIN_RY5_PORT,  PIN_RY5_PIN},  // Y5 15
    {PIN_RY6_PORT,  PIN_RY6_PIN},  // Y6 14
		
    {PIN_MY6_PORT,  PIN_MY6_PIN},  // Y6 27
    {PIN_MY5_PORT,  PIN_MY5_PIN},  // Y5 26
    {PIN_MY4_PORT,  PIN_MY4_PIN},  // Y4 25
    {PIN_MY3_PORT,  PIN_MY3_PIN},  // Y3 24
    {PIN_MY2_PORT,  PIN_MY2_PIN},  // Y2 23
    {PIN_MY1_PORT,  PIN_MY1_PIN},  // Y1 22
    {PIN_MY0_PORT,  PIN_MY0_PIN},  // Y0 21
		
    {PIN_LY6_PORT,  PIN_LY6_PIN},  // Y6 34
    {PIN_LY5_PORT,  PIN_LY5_PIN},  // Y5 33 
    {PIN_LY4_PORT,  PIN_LY4_PIN},  // Y4 32 
    {PIN_LY3_PORT,  PIN_LY3_PIN},  // Y3 31 
    {PIN_LY2_PORT,  PIN_LY2_PIN},  // Y2 30 
    {PIN_LY1_PORT,  PIN_LY1_PIN},  // Y1 29 
    {PIN_LY0_PORT,  PIN_LY0_PIN},  // Y0 28 

    {PIN_PY17_PORT,  PIN_PY17_PIN},// Y17 35  掌心从上到下从左到右
    {PIN_PY16_PORT,  PIN_PY16_PIN},// Y16 
    {PIN_PY15_PORT,  PIN_PY15_PIN},// Y15 
    {PIN_PY14_PORT,  PIN_PY14_PIN},// Y14 
    {PIN_PY13_PORT,  PIN_PY13_PIN},// Y13 
    {PIN_PY12_PORT,  PIN_PY12_PIN},// Y12 
    {PIN_PY11_PORT,  PIN_PY11_PIN},// Y11 
    {PIN_PY10_PORT,  PIN_PY10_PIN},// Y10 
    {PIN_PY9_PORT,   PIN_PY9_PIN}, // Y9  
    {PIN_PY8_PORT,   PIN_PY8_PIN}, // Y8  44
		
    {PIN_PY7_PORT,   PIN_PY7_PIN}, // Y7  45
    {PIN_PY6_PORT,   PIN_PY6_PIN}, // Y6  46
    {PIN_PY5_PORT,   PIN_PY5_PIN}, // Y5  47
    {PIN_PY4_PORT,   PIN_PY4_PIN}, // Y4  48
    {PIN_PY3_PORT,   PIN_PY3_PIN}, // Y3  49
    {PIN_PY2_PORT,   PIN_PY2_PIN}, // Y2  50
    {PIN_PY1_PORT,   PIN_PY1_PIN}, // Y1  51
    {PIN_PY0_PORT,   PIN_PY0_PIN}, // Y0  52

//			// Y0 ~ Y17 正序：Y0(索引0), Y1(索引1) ... Y17(索引17)
//			{PIN_PY0_PORT,   PIN_PY0_PIN}, // Y0 35
//			{PIN_PY1_PORT,   PIN_PY1_PIN}, // Y1
//			{PIN_PY2_PORT,   PIN_PY2_PIN}, // Y2
//			{PIN_PY3_PORT,   PIN_PY3_PIN}, // Y3
//			{PIN_PY4_PORT,   PIN_PY4_PIN}, // Y4
//			{PIN_PY5_PORT,   PIN_PY5_PIN}, // Y5
//			{PIN_PY6_PORT,   PIN_PY6_PIN}, // Y6
//			{PIN_PY7_PORT,   PIN_PY7_PIN}, // Y7 42
//			
//			{PIN_PY8_PORT,   PIN_PY8_PIN}, // Y8 43
//			{PIN_PY9_PORT,   PIN_PY9_PIN}, // Y9
//			{PIN_PY10_PORT,  PIN_PY10_PIN},// Y10
//			{PIN_PY11_PORT,  PIN_PY11_PIN},// Y11
//			{PIN_PY12_PORT,  PIN_PY12_PIN},// Y12
//			{PIN_PY13_PORT,  PIN_PY13_PIN},// Y13
//			{PIN_PY14_PORT,  PIN_PY14_PIN},// Y14
//			{PIN_PY15_PORT,  PIN_PY15_PIN},// Y15
//			{PIN_PY16_PORT,  PIN_PY16_PIN},// Y16
//			{PIN_PY17_PORT,  PIN_PY17_PIN},// Y17 52
};
#define Y_CH_TOTAL_NUM  (sizeof(m_stYPinMap)/sizeof(stc_y_pin_map_t))

/**
 * @brief 底层驱动总初始化：Y输出IO + AS控制IO + PALM ADC
 */
void ADC_SAMPLE_Init(void)
{
		/* 释放PB3(PY10/Y10) JTDO，PB4(PY11/Y11) NJTRST，PA15(Y6) JTDI，保留SWD(PA13 PA14)可用 */
		GPIO_SetDebugPort(GPIO_PIN_TDO, DISABLE);   /* PB3  */
		GPIO_SetDebugPort(GPIO_PIN_TRST, DISABLE);  /* PB4  */
		GPIO_SetDebugPort(GPIO_PIN_TDI, DISABLE);   /* PA15  */
    GPIO_Y_Init();
    GPIO_AS_Init();
    ADC_Palm_Init();

    /* 上电默认全部关闭 */
    Y_Channel_AllLow();
    AS_AllChip_Disable();
}

/**
 * @brief Y0~Y34全部推挽输出初始化，默认低电平
 */
static void GPIO_Y_Init(void)
{
    stc_gpio_init_t stcGpioInit;
    uint8_t u8Idx;
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
		stcGpioInit.u16PullUp = PIN_PU_ON;
		stcGpioInit.u16PinOutputType = PIN_OUT_TYPE_CMOS;

    for(u8Idx = 0; u8Idx < Y_CH_TOTAL_NUM; u8Idx++)
    {
        GPIO_Init(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin, &stcGpioInit);
				GPIO_OutputCmd(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin,ENABLE);
				SetSinglePin(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin, PIN_LOW);
    }
}

/**
 * @brief AS模拟开关控制管脚初始化（AS_EN1~7 / AS_S1~3 推挽输出）
 */
static void GPIO_AS_Init(void)
{
    stc_gpio_init_t stcGpioInit;
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
		stcGpioInit.u16PinOutputType = PIN_OUT_TYPE_CMOS;
	
    /* AS_EN */
		GPIO_Init(AS_EN0_PORT, AS_EN0_PIN, &stcGpioInit);
    GPIO_Init(AS_EN1_PORT, AS_EN1_PIN, &stcGpioInit);
    GPIO_Init(AS_EN2_PORT, AS_EN2_PIN, &stcGpioInit);
    GPIO_Init(AS_EN3_PORT, AS_EN3_PIN, &stcGpioInit);
    GPIO_Init(AS_EN4_PORT, AS_EN4_PIN, &stcGpioInit);
		GPIO_Init(AS_EN5_PORT, AS_EN5_PIN, &stcGpioInit);
		GPIO_Init(AS_EN6_PORT, AS_EN6_PIN, &stcGpioInit);
    /* AS_S 通道选择 */
    GPIO_Init(AS_S1_PORT, AS_S1_PIN, &stcGpioInit);
    GPIO_Init(AS_S2_PORT, AS_S2_PIN, &stcGpioInit);
    GPIO_Init(AS_S3_PORT, AS_S3_PIN, &stcGpioInit);

    /* 默认全部关闭 */
    AS_AllChip_Disable();
}

/**
 * @brief ADC时钟配置（对齐原厂例程规范）
 * @note  适配200MHz系统主频，PCLK2=50MHz(ADC模拟时钟)，PCLK4=100MHz(数字接口)，比例2:1符合规格
 */
static void AdcClockConfig(void)
{
    /* 选择系统PCLK作为ADC时钟源 */
    CLK_SetPeriClockSrc(CLK_PERIPHCLK_PCLK);
}

/**
 * @brief PALM多通道ADC初始化（对齐原厂DDL标准流程）
 */
static void ADC_Palm_Init(void)
{
    stc_adc_init_t stcAdcInit;
    stc_gpio_init_t stcGpioInit;
    uint8_t u8Ch;

    /* 1. ADC时钟配置 */
    AdcClockConfig();
    
    /* 2. 使能ADC外设时钟 */
    FCG_Fcg3PeriphClockCmd(ADC_PERIPH_CLK, ENABLE);

    /* 3. ADC内核初始化 */
    (void)ADC_StructInit(&stcAdcInit);
    (void)ADC_Init(ADC_UNIT, &stcAdcInit);

    /* 4. 配置所有模拟输入引脚 */
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    GPIO_Init(AD0_ADC_PORT, AD0_ADC_PIN, &stcGpioInit);
    GPIO_Init(AD1_ADC_PORT, AD1_ADC_PIN, &stcGpioInit);
    GPIO_Init(AD2_ADC_PORT, AD2_ADC_PIN, &stcGpioInit);
    GPIO_Init(AD3_ADC_PORT, AD3_ADC_PIN, &stcGpioInit);
    GPIO_Init(AD4_ADC_PORT, AD4_ADC_PIN, &stcGpioInit);
    GPIO_Init(AD5_ADC_PORT, AD5_ADC_PIN, &stcGpioInit);
    GPIO_Init(AD6_ADC_PORT, AD6_ADC_PIN, &stcGpioInit);
		ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD0_ADC_CH, DISABLE);
		ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD1_ADC_CH, DISABLE);
		ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD2_ADC_CH, DISABLE);
		ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD3_ADC_CH, DISABLE);
		ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD4_ADC_CH, DISABLE);
		ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD5_ADC_CH, DISABLE);
		ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD6_ADC_CH, DISABLE);
    /* 5. 统一配置所有通道的采样时间 + 硬件平均（全局配置，通道开关不影响） */
    for (u8Ch = AD0_ADC_CH; u8Ch <= AD6_ADC_CH; u8Ch++) {
        ADC_SetSampleTime(ADC_UNIT, u8Ch, ADC_SAMPLE_TIME);
    }
    ADC_PGA_Config(ADC_UNIT, AD_ADC_PGA_UNIT, AD_ADC_PGA_GAIN, AD_ADC_PGA_VSS);
    ADC_PGA_Cmd(ADC_UNIT, AD_ADC_PGA_UNIT, ENABLE);
}

/* ====================== Y通道操作函数 ====================== */
static void SetSinglePin(uint8_t u8Port, uint16_t u16Pin, en_pin_level_t enLevel)
{
    if(PIN_HIGH == enLevel)
    {
        GPIO_SetPins(u8Port, u16Pin);
    }
    else
    {
        GPIO_ResetPins(u8Port, u16Pin);
    }
}

static void sampler_switch_cols(uint8_t AREA,uint8_t N)
{
    uint8_t i;

		switch(AREA)
		{
				case TAC_AREA_THUMB_DIP:
				case TAC_AREA_THUMB_MCP:
				case TAC_AREA_INDEX_DIP:
				case TAC_AREA_INDEX_PIP:
				case TAC_AREA_INDEX_MCP:
				case TAC_AREA_MIDDLE_DIP:
				case TAC_AREA_MIDDLE_PIP:
				case TAC_AREA_MIDDLE_MCP:
				case TAC_AREA_RING_DIP:
				case TAC_AREA_RING_PIP:
				case TAC_AREA_RING_MCP:
				case TAC_AREA_LITTLE_DIP:
				case TAC_AREA_LITTLE_PIP:
				case TAC_AREA_LITTLE_MCP:
				case TAC_AREA_PALM1:
				case TAC_AREA_PALM2:
				for (i = TacAreaRange[AREA].col_start; i <= TacAreaRange[AREA].col_end; ++i) 
				{
						SetSinglePin(m_stYPinMap[i].pstPort,m_stYPinMap[i].u16Pin,((i == N) ? PIN_HIGH : PIN_LOW));
				}
				break;
				default:break;
		}
}

static void Y_Channel_AllLow(void)
{
    uint8_t u8Idx;
    for(u8Idx = 0; u8Idx < Y_CH_TOTAL_NUM; u8Idx++)
    {
        GPIO_ResetPins(m_stYPinMap[u8Idx].pstPort, m_stYPinMap[u8Idx].u16Pin);
    }
}

/* ====================== AS模拟开关操作函数 ====================== */
static void AS_Chip_Enable(en_as_chip_t enChip)
{
    en_pin_level_t enPin = PIN_LOW;
    switch(enChip)
    {
				case AS_CHIP_0: SetSinglePin(AS_EN0_PORT, AS_EN0_PIN, enPin); break;
        case AS_CHIP_1: SetSinglePin(AS_EN1_PORT, AS_EN1_PIN, enPin); break;
        case AS_CHIP_2: SetSinglePin(AS_EN2_PORT, AS_EN2_PIN, enPin); break;
        case AS_CHIP_3: SetSinglePin(AS_EN3_PORT, AS_EN3_PIN, enPin); break;
        case AS_CHIP_4: SetSinglePin(AS_EN4_PORT, AS_EN4_PIN, enPin); break;
				case AS_CHIP_5: SetSinglePin(AS_EN5_PORT, AS_EN5_PIN, enPin); break;
				case AS_CHIP_6: SetSinglePin(AS_EN6_PORT, AS_EN6_PIN, enPin); break;
        default: break;
    }
}

static void AS_Channel_Select(en_as_channel_t enCh)
{
    /* S3 S2 S1 三位控制位 */
    SetSinglePin(AS_S1_PORT, AS_S1_PIN, (enCh & 0x01U) ? PIN_HIGH : PIN_LOW);
    SetSinglePin(AS_S2_PORT, AS_S2_PIN, (enCh & 0x02U) ? PIN_HIGH : PIN_LOW);
    SetSinglePin(AS_S3_PORT, AS_S3_PIN, (enCh & 0x04U) ? PIN_HIGH : PIN_LOW);
}

static void AS_AllChip_Disable(void)
{
		SetSinglePin(AS_EN0_PORT, AS_EN0_PIN, PIN_HIGH);
    SetSinglePin(AS_EN1_PORT, AS_EN1_PIN, PIN_HIGH);
    SetSinglePin(AS_EN2_PORT, AS_EN2_PIN, PIN_HIGH);
    SetSinglePin(AS_EN3_PORT, AS_EN3_PIN, PIN_HIGH);
    SetSinglePin(AS_EN4_PORT, AS_EN4_PIN, PIN_HIGH);
		SetSinglePin(AS_EN5_PORT, AS_EN5_PIN, PIN_HIGH);
		SetSinglePin(AS_EN6_PORT, AS_EN6_PIN, PIN_HIGH);
}

/* ====================== PALM ADC采集函数 ====================== */
/**
 * @brief 轮询ADC单通道采集（完全对齐原厂例程逻辑）
 * @param  adx         ADC通道号
 * @param  pu16AdcVal  采样结果输出
 * @retval LL_OK：成功，LL_ERR：超时失败
 */
static int32_t AdcPollingSample(uint8_t adx, uint16_t *pu16AdcVal)
{
    int32_t iRet = LL_ERR;
    __IO uint32_t u32TimeCount = 0UL;
    static uint8_t u8CurEnabledCh = 0xFFU; /* 缓存当前已使能的通道，0xFF表示无 */

    /* 通道切换：仅当目标通道与当前使能通道不同时，重新配置 */
    if (adx != u8CurEnabledCh) {
				ADC_PGA_SelectInputSrc(ADC_UNIT, (uint32_t)(1<<adx));
        /* 关闭所有7个通道 */
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD0_ADC_CH, DISABLE);
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD1_ADC_CH, DISABLE);
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD2_ADC_CH, DISABLE);
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD3_ADC_CH, DISABLE);
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD4_ADC_CH, DISABLE);
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD5_ADC_CH, DISABLE);
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, AD6_ADC_CH, DISABLE);
        /* 仅使能目标通道 */
        ADC_ChCmd(ADC_UNIT, ADC_SEQ, adx, ENABLE);
        u8CurEnabledCh = adx;
        /* 通道切换后加1us稳定延时，确保多路开关建立完成 */
        DDL_DelayUS(1);
    }
    /* 启动序列A转换 */
    ADC_Start(ADC_UNIT);
    do
    {
        if(SET == ADC_GetStatus(ADC_UNIT, ADC_EOC_FLAG))
        {
            ADC_ClearStatus(ADC_UNIT, ADC_EOC_FLAG);
            *pu16AdcVal = ADC_GetValue(ADC_UNIT, adx);
            iRet = LL_OK;
            break;
        }
    } while (u32TimeCount++ < ADC_TIMEOUT_VAL);

    if(LL_ERR == iRet)
    {
        ADC_Stop(ADC_UNIT);
    }
    return iRet;
}

/**
 * @brief X通道模拟开关选择
 */
static void sampler_select_mux(uint8_t AREA, uint8_t u8XIndex)
{
    en_as_chip_t mux_index;
    en_as_channel_t channel;
		
    if (u8XIndex >= ALL_ROW) {
        AS_AllChip_Disable();
        return;
    }
		switch(AREA)
		{
			case TAC_AREA_THUMB_DIP:
			case TAC_AREA_THUMB_MCP:
				mux_index = AS_CHIP_0;
				channel = (en_as_channel_t)(u8XIndex % SAMPLER_MUX_CHANNELS);
				break;
			case TAC_AREA_INDEX_DIP:
			case TAC_AREA_INDEX_PIP:
			case TAC_AREA_INDEX_MCP:
				mux_index = AS_CHIP_1;
				channel = (en_as_channel_t)(u8XIndex % SAMPLER_MUX_CHANNELS);
				break;
			case TAC_AREA_MIDDLE_DIP:
			case TAC_AREA_MIDDLE_PIP:
			case TAC_AREA_MIDDLE_MCP:
				mux_index = AS_CHIP_2;
				channel = (en_as_channel_t)(u8XIndex % SAMPLER_MUX_CHANNELS);
				break;
			case TAC_AREA_RING_DIP:
			case TAC_AREA_RING_PIP:
			case TAC_AREA_RING_MCP:
				mux_index = AS_CHIP_3;
				channel = (en_as_channel_t)(u8XIndex % SAMPLER_MUX_CHANNELS);
				break;
			case TAC_AREA_LITTLE_DIP:
			case TAC_AREA_LITTLE_PIP:
			case TAC_AREA_LITTLE_MCP:
				mux_index = AS_CHIP_4;
				channel = (en_as_channel_t)(u8XIndex % SAMPLER_MUX_CHANNELS);
				break;
			case TAC_AREA_PALM1:
				mux_index = AS_CHIP_5;
				channel = (en_as_channel_t)((4-u8XIndex) % SAMPLER_MUX_CHANNELS);
				break;
			case TAC_AREA_PALM2:
				mux_index = AS_CHIP_6;
				channel = (en_as_channel_t)((5-u8XIndex) % SAMPLER_MUX_CHANNELS);
				break;
			default:break;
		}
    AS_AllChip_Disable();
    AS_Channel_Select(channel);
		AS_Chip_Enable(mux_index);
}

/* ====================== 对外采集函数 ====================== */
/**
 * @brief 轮询采集指定区域的完整触觉帧
 * @param  AREA   区域编号
 * @param  row    区域行数
 * @param  col    区域列数
 * @param  buffer 输出二维数组缓冲区
 * @retval 采集到的总数据字节数
 */
#if(DATA_FRAME_BYTE_NUM)
uint16_t SampleGetFrame(uint8_t AREA, uint8_t row, uint8_t col, uint16_t buffer[][col])
{
    int i;
    int j;
		uint16_t length=0;
		const tac_data_cfg_t* pTacMaskData = GetTacMaskData();
		int32_t iRet = LL_ERR;
    for (i = TacAreaRange[AREA].col_start; i <= TacAreaRange[AREA].col_end; i++) {
        sampler_switch_cols(AREA,(uint8_t)i);
        for (j = TacAreaRange[AREA].row_start; j <= TacAreaRange[AREA].row_end; j++) {
            uint16_t value=0;
						sampler_select_mux(AREA,(uint8_t)j);
						DDL_DelayUS(1);
						switch(AREA)
						{
							case TAC_AREA_THUMB_DIP:
								if(pTacMaskData->thumb_dip[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD0_ADC_CH,&value);
								}
								break;
							case TAC_AREA_THUMB_MCP:
								if(pTacMaskData->thumb_mcp[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD0_ADC_CH,&value);
								}
								break;
							case TAC_AREA_INDEX_DIP:
								if(pTacMaskData->index_dip[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD1_ADC_CH,&value);
								}
								break;
							case TAC_AREA_INDEX_PIP:
								if(pTacMaskData->index_pip[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD1_ADC_CH,&value);
								}
								break;
							case TAC_AREA_INDEX_MCP:
								if(pTacMaskData->index_mcp[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD1_ADC_CH,&value);
								}
								break;
							case TAC_AREA_MIDDLE_DIP:
								if(pTacMaskData->middle_dip[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD2_ADC_CH,&value);
								}
								break;
							case TAC_AREA_MIDDLE_PIP:
								if(pTacMaskData->middle_pip[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD2_ADC_CH,&value);
								}
								break;
							case TAC_AREA_MIDDLE_MCP:
								if(pTacMaskData->middle_mcp[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD2_ADC_CH,&value);
								}
								break;
							case TAC_AREA_RING_DIP:
								if(pTacMaskData->ring_dip[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD3_ADC_CH,&value);
								}
								break;
							case TAC_AREA_RING_PIP:
								if(pTacMaskData->ring_pip[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD3_ADC_CH,&value);
								}
								break;
							case TAC_AREA_RING_MCP:
								if(pTacMaskData->ring_mcp[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD3_ADC_CH,&value);
								}
								break;
							case TAC_AREA_LITTLE_DIP:
								if(pTacMaskData->little_dip[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD4_ADC_CH,&value);
								}
								break;
							case TAC_AREA_LITTLE_PIP:
								if(pTacMaskData->little_pip[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD4_ADC_CH,&value);
								}
								break;
							case TAC_AREA_LITTLE_MCP:
								if(pTacMaskData->little_mcp[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD4_ADC_CH,&value);
								}
								break;
							case TAC_AREA_PALM1:
								if(pTacMaskData->palm1[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD5_ADC_CH,&value);
								}
								break;
							case TAC_AREA_PALM2:
								if(pTacMaskData->palm2[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start]==0x0000)
								{
									value=0;
								}
								else{
									iRet = AdcPollingSample(AD6_ADC_CH,&value);
								}
								break;
							default:break;
						}
						if(iRet==LL_ERR)
						{
							buffer[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start] = 0;
						}
						else{
							uint8_t p=0,q=0;
							p = j-TacAreaRange[AREA].row_start;
							q = i-TacAreaRange[AREA].col_start;
							buffer[p][q] = (value);
						}
        }
    }
		length = col*row;
    Y_Channel_AllLow();
    AS_AllChip_Disable();
		return length;
}
#else
uint16_t SampleGetFrame(uint8_t AREA, uint8_t row, uint8_t col, uint8_t buffer[][col])
{
    int i;
    int j;
		uint16_t length=0;
		int32_t iRet = LL_ERR;
    for (i = TacAreaRange[AREA].col_start; i <= TacAreaRange[AREA].col_end; i++) {
        sampler_switch_cols(AREA,(uint8_t)i);
        for (j = TacAreaRange[AREA].row_start; j <= TacAreaRange[AREA].row_end; j++) {
            uint16_t value=0;
						sampler_select_mux(AREA,(uint8_t)j);
						DDL_DelayUS(1);
						switch(AREA)
						{
							case TAC_AREA_THUMB_DIP:
							case TAC_AREA_THUMB_MCP:
								iRet = AdcPollingSample(AD0_ADC_CH,&value);
								break;
							case TAC_AREA_INDEX_DIP:
							case TAC_AREA_INDEX_PIP:
							case TAC_AREA_INDEX_MCP:
								iRet = AdcPollingSample(AD1_ADC_CH,&value);
								break;
							case TAC_AREA_MIDDLE_DIP:
							case TAC_AREA_MIDDLE_PIP:
							case TAC_AREA_MIDDLE_MCP:
								iRet = AdcPollingSample(AD2_ADC_CH,&value);
								break;
							case TAC_AREA_RING_DIP:
							case TAC_AREA_RING_PIP:
							case TAC_AREA_RING_MCP:
								iRet = AdcPollingSample(AD3_ADC_CH,&value);
								break;
							case TAC_AREA_LITTLE_DIP:
							case TAC_AREA_LITTLE_PIP:
							case TAC_AREA_LITTLE_MCP:
								iRet = AdcPollingSample(AD4_ADC_CH,&value);
								break;
							case TAC_AREA_PALM1:
								iRet = AdcPollingSample(AD5_ADC_CH,&value);
								break;
							case TAC_AREA_PALM2:
								iRet = AdcPollingSample(AD6_ADC_CH,&value);
								break;
							default:break;
						}
						if(iRet==LL_ERR)
						{
							buffer[j-TacAreaRange[AREA].row_start][i-TacAreaRange[AREA].col_start] = 0;
						}
						else{
							uint8_t p=0,q=0;
							p = j-TacAreaRange[AREA].row_start;
							q = i-TacAreaRange[AREA].col_start;
							buffer[p][q] = (value>>4)&0XFF;
						}
        }
    }
		length = col*row;
    Y_Channel_AllLow();
    AS_AllChip_Disable();
		return length;
}
#endif


