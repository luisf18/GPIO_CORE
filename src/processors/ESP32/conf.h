#ifndef GPIO_CORE_MCU_CONF_H
#define GPIO_CORE_MCU_CONF_H

#define GPIO_CORE_MCU_ESP

#define GPIO_CORE_PIN_COUNT 40
#define GPIO_CORE_VCC       3.3


// PWM
#if defined(CONFIG_IDF_TARGET_ESP32C3)
#define GPIO_CORE_PWM_COUNT      6
#else
#define GPIO_CORE_PWM_COUNT      16
#endif
#define GPIO_CORE_PWM_HZ         1000
#define GPIO_CORE_PWM_RES        10 // resolution
#define GPIO_CORE_PWM_MAX        1023
#define GPIO_CORE_PWM_MAX_RES    16

// ADC
#define GPIO_CORE_ADC_RES        12 // resolution
#define GPIO_CORE_ADC_MAX        4095

// INTERRUPT
#define GPIO_CORE_ISR_PIN_COUNT  40
#define GPIO_CORE_ISR_PIN_OFFSET 0


/*
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#elif defined (ESP32)
#elif defined (ARDUINO_ARCH_ESP8266)
#elif defined (STM32)
#elif defined(ARDUINO_ARCH_RP2040)
#else
#endif
*/

#endif