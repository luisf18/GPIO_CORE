#ifdef ESP8266

#include "interrupt.h"

void ICACHE_RAM_ATTR GPIO_CORE_ISR_D0() { GPIO_PTR[0]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D1() { GPIO_PTR[1]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D2() { GPIO_PTR[2]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D3() { GPIO_PTR[3]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D4() { GPIO_PTR[4]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D5() { GPIO_PTR[5]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D6() { GPIO_PTR[6]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D7() { GPIO_PTR[7]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D8() { GPIO_PTR[8]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D9() { GPIO_PTR[9]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D10() { GPIO_PTR[10]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D11() { GPIO_PTR[11]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D12() { GPIO_PTR[12]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D13() { GPIO_PTR[13]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D14() { GPIO_PTR[14]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D15() { GPIO_PTR[15]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D16() { GPIO_PTR[16]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D17() { GPIO_PTR[17]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D18() { GPIO_PTR[18]->HANDLE_ISR();}
void ICACHE_RAM_ATTR GPIO_CORE_ISR_D19() { GPIO_PTR[19]->HANDLE_ISR();}

void ICACHE_RAM_ATTR (*GPIO_CORE_ISR[GPIO_CORE_ISR_PIN_COUNT])() = {
	GPIO_CORE_ISR_D0,
	GPIO_CORE_ISR_D1,
	GPIO_CORE_ISR_D2,
	GPIO_CORE_ISR_D3,
	GPIO_CORE_ISR_D4,
	GPIO_CORE_ISR_D5,
	GPIO_CORE_ISR_D6,
	GPIO_CORE_ISR_D7,
	GPIO_CORE_ISR_D8,
	GPIO_CORE_ISR_D9,
	GPIO_CORE_ISR_D10,
	GPIO_CORE_ISR_D11,
	GPIO_CORE_ISR_D12,
	GPIO_CORE_ISR_D13,
	GPIO_CORE_ISR_D14,
	GPIO_CORE_ISR_D15,
	GPIO_CORE_ISR_D16,
	GPIO_CORE_ISR_D17,
	GPIO_CORE_ISR_D18,
	GPIO_CORE_ISR_D19
};

#endif