#ifdef __AVR__

#include "interrupt.h"

void GPIO_CORE_ISR_D2(){ GPIO_PTR[2]->HANDLE_ISR(); }
void GPIO_CORE_ISR_D3(){ GPIO_PTR[3]->HANDLE_ISR(); }

void (*GPIO_CORE_ISR[GPIO_CORE_ISR_PIN_COUNT])() = {
    GPIO_CORE_ISR_D2,
    GPIO_CORE_ISR_D3
};

#endif