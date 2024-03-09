#include <stm32g030xx.h>

#include "liblfp.h"
#include "liblfphw.h"

void LFP_HWInit(void) {
	// Remap PA9/10
	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN; 
	SYSCFG->CFGR1 |= 0x18; 
	// GPIO configuration
	// PA9:  USART1_TX, AF1PP
	// PA10: USART1_RX, AF1IN (Pu)
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN; 
	GPIOA->AFR[1] = GPIOA->AFR[1] & 0xFFFFF00F | 0x00000110; 
	GPIOA->PUPDR = GPIOA->PUPDR & 0xFFCFFFFF | 0x00100000; 
	GPIOA->MODER = GPIOA->MODER & 0xFFC3FFFF | 0x00280000; 
	// USART1 configuration
	// Mode: 8N1 @ 9600 (64MHz clock)
	// Rx IRQ on, Tx IRQ off
	RCC->APBENR2 |= RCC_APBENR2_USART1EN; 
	USART1->BRR = 6667; 
	USART1->CR3 = 0x00001000; // No ORE
	USART1->CR2 = 0x00000000; 
	USART1->CR1 = 0x0000002D; 
#define USART1_TXIRQ_ON()		USART1->CR1 |=  0x80
#define USART1_TXIRQ_OFF()	USART1->CR1 &= ~0x80
#define USART1_TXIRQ_ENABLED() (USART1->CR1 & 0x80)
	NVIC_EnableIRQ(USART1_IRQn); 
}

void LFP_TxIRQSetMask(int state) {
	if(state) USART1_TXIRQ_ON(); 
	else USART1_TXIRQ_OFF(); 
}

void USART1_IRQHandler(void) {
	uint32_t flag = USART1->ISR; 
	if(flag & USART_ISR_TXE_TXFNF && USART1_TXIRQ_ENABLED()) {
		USART1->TDR = LFP_TxIRQProducer(); 
	}
	if(flag & USART_ISR_RXNE_RXFNE) {
		LFP_RxIRQConsumer(USART1->RDR); 
	}
}
