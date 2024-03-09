#include <stm32g030xx.h>

#include "librcc.h"

void SysClock_HWInit(void) {
	// 64MHz derived from HSI16
	// Update VOS to high performance
	RCC->APBENR1 |= RCC_APBENR1_PWREN; 
	__DSB(); 
	PWR->CR1 = PWR->CR1 & 0xFFFFD8FF | 0x00000300; 
	while(PWR->SR2 & 0x00000400); 
	// Update flash wait state
	RCC->AHBENR |= RCC_AHBENR_FLASHEN; 
	__DSB(); 
	FLASH->ACR |= 0x00000602; // TODO: enable prefetch?
	// Configure PLL: 16M / 8(M) * 64(N) / 2(R)
	RCC->PLLCFGR = 0x30004072; 
	RCC->CR |= 0x01000000; 
	while(!(RCC->CR & 0x02000000)); 
	// Switch clock to higher frequency
	RCC->CFGR = 0x00000002; 
	while((RCC->CFGR & 0x00000038) != 0x00000010); 
}
