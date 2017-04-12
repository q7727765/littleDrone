#include "delay.h"
#include "stm32f103xb.h"


// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;


void delay_init()
{
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);
}
static void cycleCounterInit(void)
{
    usTicks = SystemCoreClock / 1000000;
}

void SysTick_Handler(void){
	sysTickUptime++;
}

uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;

        /*
         * If the SysTick timer expired during the previous instruction, we need to give it a little time for that
         * interrupt to be delivered before we can recheck sysTickUptime:
         */
        __asm volatile("\tnop\n");
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

void delay_us(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}

void delay_ms(uint32_t ms)
{
    while (ms--)
        delay_us(1000);
}
