#ifndef GPIO_CORE_ISR_H
#define GPIO_CORE_ISR_H

#include "GPIO_CORE.h"

extern GPIO_CORE *GPIO_PTR[GPIO_CORE_PIN_COUNT];

void GPIO_CORE_ISR_D2();
void GPIO_CORE_ISR_D3();

#endif