#include "stdint.h"
#include "stm32f4xx.h"  // Needed for CoreDebug and DWT
void delay_ms(uint16_t us){
	for(int i=0;i<50*us;i++){}
}
void delayLCD(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);  // 1 us = N cycles

    while ((DWT->CYCCNT - start) < delayTicks);
}
