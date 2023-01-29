#ifndef GPIO_CORE_MCU_CONF_H
#define GPIO_CORE_MCU_CONF_H

#define GPIO_CORE_AVR

#define GPIO_CORE_PIN_COUNT       22
#define GPIO_CORE_VCC             5.0

// PWM
#define GPIO_CORE_PWM_COUNT       6
#define GPIO_CORE_PWM_RES         8  // resolution
#define GPIO_CORE_PWM_MAX         255
#define GPIO_CORE_PWM_HZ          500
#define GPIO_CORE_PWM_MAX_RES     8

// ADC
#define GPIO_CORE_ADC_RES         10
#define GPIO_CORE_ADC_MAX         1023

// INTERRUPT
#define GPIO_CORE_ISR_PIN_COUNT   2
#define GPIO_CORE_ISR_PIN_OFFSET -2

#endif