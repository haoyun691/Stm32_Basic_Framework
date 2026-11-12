#include "bsp_dwt.h"
#include "system_stm32f1xx.h"
#include <stdint.h>


static uint32_t CYCCNT_LAST;
static uint32_t CYCCNT_RountCount;

static void DWT_CNT_Update(void)
{
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = DWT->CYCCNT;
        bit_locker = 0;
    }
}

/**
 *@brief 初始化DWT
*/
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     // 开启 TRC

    DWT->CYCCNT = (uint32_t)0u;                         // CYCCNT 计数器清零

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                // 开启 CYCCNT 计数器

    DWT_CNT_Update();
}

/**
 *@brief 获取时间间隔/us
 *@param cnt_last 上一时刻
 *@param dt 从两次执行该函数时的时间间隔
 *@note cnt_last初始默认为零，需执行一次该函数校准
 *
 *@return dt 单位为毫秒/us
*/
float DWT_GetDeltaT(uint32_t *cnt_last){
    volatile uint32_t cnt_now = DWT->CYCCNT;

    //以72MHz为例，每秒滴答72M 除以1M即为每72滴答记一次数，即为1us
    float dt = ((uint32_t)(cnt_now - *cnt_last)) * (SystemCoreClock / 1000000.0); 
    *cnt_last = cnt_now;

    DWT_CNT_Update();

    return dt;
}

/**
 *@brief DWT延时，单位毫秒/us
 *@attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
*/
void DWT_Delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - startTick) < delayTicks);
}