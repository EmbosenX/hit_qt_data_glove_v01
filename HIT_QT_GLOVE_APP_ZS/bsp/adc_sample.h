/*******************************************************************************
 * @file        adc_sample.h
 * @brief       Y0~Y34 IO输出控制 + AS模拟开关使能/通道选择 + PALM_RP_AD01 ADC采集驱动
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 * @note        适配HC32F460PEHB-VFBGA100原理图，单路ADC轮询多路模拟开关采集
 ******************************************************************************/
#ifndef __ADC_SAMPLE_H__
#define __ADC_SAMPLE_H__

#include "hc32_ll.h"
#include "main.h"
/* ====================== 硬件管脚定义（严格匹配原理图） ====================== */
/* 大拇指通用输出Y管脚 */
#define PIN_TY0_PORT     GPIO_PORT_C
#define PIN_TY0_PIN      GPIO_PIN_02

#define PIN_TY1_PORT     GPIO_PORT_C
#define PIN_TY1_PIN      GPIO_PIN_15

#define PIN_TY2_PORT     GPIO_PORT_C
#define PIN_TY2_PIN      GPIO_PIN_14

#define PIN_TY3_PORT     GPIO_PORT_C
#define PIN_TY3_PIN      GPIO_PIN_13

#define PIN_TY4_PORT     GPIO_PORT_C
#define PIN_TY4_PIN      GPIO_PIN_12

#define PIN_TY5_PORT     GPIO_PORT_C
#define PIN_TY5_PIN      GPIO_PIN_11

#define PIN_TY6_PORT     GPIO_PORT_C
#define PIN_TY6_PIN      GPIO_PIN_10
/* 食指通用输出Y管脚 */
#define PIN_IY0_PORT     GPIO_PORT_E
#define PIN_IY0_PIN      GPIO_PIN_13

#define PIN_IY1_PORT     GPIO_PORT_E
#define PIN_IY1_PIN      GPIO_PIN_14

#define PIN_IY2_PORT     GPIO_PORT_E
#define PIN_IY2_PIN      GPIO_PIN_15

#define PIN_IY3_PORT     GPIO_PORT_E
#define PIN_IY3_PIN      GPIO_PIN_02

#define PIN_IY4_PORT     GPIO_PORT_E
#define PIN_IY4_PIN      GPIO_PIN_03

#define PIN_IY5_PORT     GPIO_PORT_E
#define PIN_IY5_PIN      GPIO_PIN_04

#define PIN_IY6_PORT     GPIO_PORT_E
#define PIN_IY6_PIN      GPIO_PIN_05
/* 无名指通用输出Y管脚 */
#define PIN_RY0_PORT     GPIO_PORT_D
#define PIN_RY0_PIN      GPIO_PIN_08

#define PIN_RY1_PORT     GPIO_PORT_D
#define PIN_RY1_PIN      GPIO_PIN_09

#define PIN_RY2_PORT     GPIO_PORT_E
#define PIN_RY2_PIN      GPIO_PIN_00

#define PIN_RY3_PORT     GPIO_PORT_E
#define PIN_RY3_PIN      GPIO_PIN_01

#define PIN_RY4_PORT     GPIO_PORT_E
#define PIN_RY4_PIN      GPIO_PIN_10

#define PIN_RY5_PORT     GPIO_PORT_E
#define PIN_RY5_PIN      GPIO_PIN_11

#define PIN_RY6_PORT     GPIO_PORT_E
#define PIN_RY6_PIN      GPIO_PIN_12

/* 中指通用输出Y管脚 */
#define PIN_MY0_PORT     GPIO_PORT_C  
#define PIN_MY0_PIN      GPIO_PIN_06  
                                      
#define PIN_MY1_PORT     GPIO_PORT_C  
#define PIN_MY1_PIN      GPIO_PIN_05  
                                      
#define PIN_MY2_PORT     GPIO_PORT_C  
#define PIN_MY2_PIN      GPIO_PIN_04  
                                      
#define PIN_MY3_PORT     GPIO_PORT_C  
#define PIN_MY3_PIN      GPIO_PIN_03  
                                      
#define PIN_MY4_PORT     GPIO_PORT_D  
#define PIN_MY4_PIN      GPIO_PIN_12  
                                      
#define PIN_MY5_PORT     GPIO_PORT_D  
#define PIN_MY5_PIN      GPIO_PIN_13  
                                      
#define PIN_MY6_PORT     GPIO_PORT_D  
#define PIN_MY6_PIN      GPIO_PIN_14  

/* 小指通用输出Y管脚 */
#define PIN_LY0_PORT       GPIO_PORT_D
#define PIN_LY0_PIN        GPIO_PIN_15
                           
#define PIN_LY1_PORT       GPIO_PORT_D
#define PIN_LY1_PIN        GPIO_PIN_02
                           
#define PIN_LY2_PORT       GPIO_PORT_D
#define PIN_LY2_PIN        GPIO_PIN_03
                           
#define PIN_LY3_PORT       GPIO_PORT_D
#define PIN_LY3_PIN        GPIO_PIN_04
                           
#define PIN_LY4_PORT       GPIO_PORT_D
#define PIN_LY4_PIN        GPIO_PIN_05
                           
#define PIN_LY5_PORT       GPIO_PORT_D
#define PIN_LY5_PIN        GPIO_PIN_06
                           
#define PIN_LY6_PORT       GPIO_PORT_D
#define PIN_LY6_PIN        GPIO_PIN_07
/* 手掌通用输出Y管脚 */
#define PIN_PY0_PORT     GPIO_PORT_C
#define PIN_PY0_PIN      GPIO_PIN_01

#define PIN_PY1_PORT     GPIO_PORT_E
#define PIN_PY1_PIN      GPIO_PIN_07

#define PIN_PY2_PORT     GPIO_PORT_E
#define PIN_PY2_PIN      GPIO_PIN_08

#define PIN_PY3_PORT     GPIO_PORT_E
#define PIN_PY3_PIN      GPIO_PIN_09

#define PIN_PY4_PORT     GPIO_PORT_H
#define PIN_PY4_PIN      GPIO_PIN_00

#define PIN_PY5_PORT     GPIO_PORT_H
#define PIN_PY5_PIN      GPIO_PIN_01

#define PIN_PY6_PORT     GPIO_PORT_H
#define PIN_PY6_PIN      GPIO_PIN_02

#define PIN_PY7_PORT     GPIO_PORT_B
#define PIN_PY7_PIN      GPIO_PIN_02

#define PIN_PY8_PORT     GPIO_PORT_A
#define PIN_PY8_PIN      GPIO_PIN_07

#define PIN_PY9_PORT     GPIO_PORT_B
#define PIN_PY9_PIN      GPIO_PIN_12

#define PIN_PY10_PORT     GPIO_PORT_B
#define PIN_PY10_PIN      GPIO_PIN_03

#define PIN_PY11_PORT     GPIO_PORT_B
#define PIN_PY11_PIN      GPIO_PIN_04

#define PIN_PY12_PORT     GPIO_PORT_B
#define PIN_PY12_PIN      GPIO_PIN_05

#define PIN_PY13_PORT     GPIO_PORT_B
#define PIN_PY13_PIN      GPIO_PIN_06

#define PIN_PY14_PORT     GPIO_PORT_B
#define PIN_PY14_PIN      GPIO_PIN_07

#define PIN_PY15_PORT     GPIO_PORT_B
#define PIN_PY15_PIN      GPIO_PIN_08

#define PIN_PY16_PORT     GPIO_PORT_B
#define PIN_PY16_PIN      GPIO_PIN_09

#define PIN_PY17_PORT     GPIO_PORT_C
#define PIN_PY17_PIN      GPIO_PIN_00

/* 模拟开关 AS 使能管脚 AS_EN1 ~ AS_EN4 */
#define AS_EN0_PORT     GPIO_PORT_D
#define AS_EN0_PIN      GPIO_PIN_11

#define AS_EN1_PORT     GPIO_PORT_D
#define AS_EN1_PIN      GPIO_PIN_10

#define AS_EN2_PORT     GPIO_PORT_D
#define AS_EN2_PIN      GPIO_PIN_01

#define AS_EN3_PORT     GPIO_PORT_D
#define AS_EN3_PIN      GPIO_PIN_00

#define AS_EN4_PORT     GPIO_PORT_C
#define AS_EN4_PIN      GPIO_PIN_09

#define AS_EN5_PORT     GPIO_PORT_C
#define AS_EN5_PIN      GPIO_PIN_08

#define AS_EN6_PORT     GPIO_PORT_C
#define AS_EN6_PIN      GPIO_PIN_07

/* 模拟开关通道选择 AS_S1 ~ AS_S3 */
#define AS_S1_PORT      GPIO_PORT_B
#define AS_S1_PIN       GPIO_PIN_10

#define AS_S2_PORT      GPIO_PORT_B
#define AS_S2_PIN       GPIO_PIN_01

#define AS_S3_PORT      GPIO_PORT_B
#define AS_S3_PIN       GPIO_PIN_00


/*所有的行列*/
#define ALL_ROW			53
#define ALL_COL 		56


/* 电平枚举 */
typedef enum
{
    PIN_LOW  = 0U,
    PIN_HIGH = 1U
} en_pin_level_t;

/* AS模拟开关通道选择编码 0~7 (S3 S2 S1 三位二进制) */
typedef enum
{
    AS_CH_0 = 0x00U,    // 000
    AS_CH_1 = 0x01U,    // 001
    AS_CH_2 = 0x02U,    // 010
    AS_CH_3 = 0x03U,    // 011
    AS_CH_4 = 0x04U,    // 100
    AS_CH_5 = 0x05U,    // 101
    AS_CH_6 = 0x06U,    // 110
    AS_CH_7 = 0x07U     // 111
} en_as_channel_t;

/* 模拟开关芯片编号 1~4 */
typedef enum
{
		AS_CHIP_0 = 0U,
    AS_CHIP_1 = 1U,
    AS_CHIP_2 = 2U,
    AS_CHIP_3 = 3U,
    AS_CHIP_4 = 4U,
		AS_CHIP_5 = 5U,
		AS_CHIP_6 = 6U,

} en_as_chip_t;

/* 对外API */
void ADC_SAMPLE_Init(void);
#if(DATA_FRAME_BYTE_NUM)
uint16_t SampleGetFrame(uint8_t AREA, uint8_t row, uint8_t col, uint16_t buffer[][col]);
#else
uint16_t SampleGetFrame(uint8_t AREA, uint8_t row, uint8_t col, uint8_t buffer[][col]);
#endif
/* ADC单次采集函数 */

#endif /* __ADC_SAMPLE_H__ */



