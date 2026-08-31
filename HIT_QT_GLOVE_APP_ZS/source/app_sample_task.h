/*******************************************************************************
 * @file        app_sample_task.h
 * @brief       多通道传感器ADC采集业务任务，FreeRTOS静态创建无堆分配
 * @author      
 * @date        2026-07-21
 * @version     V1.0
 * @note        适配触觉/手指多路ADC采集，消息队列异步调度，互斥锁保护数据缓存
 ******************************************************************************/
#ifndef __APP_SAMPLE_TASK_H__
#define __APP_SAMPLE_TASK_H__

#include "hc32_ll.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "adc_sample.h"
#include "main.h"
/* -------------------------- 任务配置宏 -------------------------- */
#define SAMPLE_TASK_STACK_SIZE      768U
#define SAMPLE_TASK_PRIORITY        (configMAX_PRIORITIES - 3)
#define SAMPLE_MSG_QUEUE_DEPTH      10U
#define SAMPLE_PERIOD_MS            10U    // 默认采集周期10ms

/* -------------------------- 采集缓存配置 -------------------------- */
#define SAMPLE_PLAT_NUM             (ALL_ROW*ALL_COL*DATA_FRAME_BYTE_NUM)     // 采集通道总数（手指触觉）

/* -------------------------- 采集完整数据帧结构体 -------------------------- */

/* -------------------------- 对外全局句柄与状态声明 -------------------------- */
extern TaskHandle_t        m_hSampleTask;
extern SemaphoreHandle_t   m_hSampleDataMutex;


/* -------------------------- 对外业务API -------------------------- */
void APP_Sample_TaskCreate(void);

#endif /* __APP_SAMPLE_TASK_H__ */


