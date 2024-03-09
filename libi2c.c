#include <stm32g030xx.h>

#include "libi2c.h"

void (*I2C_Callback)(int status); 

uint8_t I2C_Busy; 
I2C_Transaction_t I2C_Transaction; 

void I2C_HWInit(void) {
	// PB7: SDA (AF6)
	// PB6: SCL (AF6)
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN; 
	GPIOB->BSRR = 0x00C0; 
	GPIOB->AFR[0] = GPIOB->AFR[0] & 0x00FFFFFF | 0x66000000; 
	GPIOB->OTYPER |= 0x00C0; 
	GPIOB->MODER = GPIOB->MODER & 0xFFFF0FFF | 0x0000A000; 
	// I2C1@64MHz, 100kHz
	RCC->CCIPR = RCC->CCIPR & 0xFFFFCFFF; 
	RCC->APBENR1 |= RCC_APBENR1_I2C1EN; 
	__DSB(); 
	I2C1->CR1 = 0x0; 
	// Sample data from STM32126
//I2C1->TIMINGR = 0x00000103; // 100kHz @ 1MHz
//I2C1->TIMINGR = 0x30420F13; // 100kHz @16MHz
//I2C1->TIMINGR = 0x10320309; // 400kHz @16MHz
	I2C1->TIMINGR = 0x60302730; // 100kHz @64MHz
	// Enabled IRQ: TX/RXIE, NACKIE, TCIE, ERRIE, STOPF. DNF OFF, ANF ON. 
	I2C1->CR1 = 0x000000F7; 
	I2C1->ICR = 0x3F38; 
	// I2C IRQ Enable
	NVIC_ClearPendingIRQ(I2C1_IRQn); 
	NVIC_SetPriority(I2C1_IRQn, 3); 
	NVIC_EnableIRQ(I2C1_IRQn); 
}

#define ctrl_wr_noend(addr, N) 		((addr) & 0xFE) | 0x00002000 | (((N) & 0xFF) << 16)
#define ctrl_wr_autoend(addr, N) 	((addr) & 0xFE) | 0x02002000 | (((N) & 0xFF) << 16)
#define ctrl_rd_autoend(addr, N) 	((addr) & 0xFE) | 0x02002400 | (((N) & 0xFF) << 16)

void I2C_Start(void) {
	I2C_Busy = 1; 
	uint8_t addr = I2C_Transaction.addr << 1; 
	I2C_Transaction.index = 0; 
	if(I2C_Transaction.wrlen != 0) {
		if(I2C_Transaction.rdlen == 0) 
			I2C1->CR2 = ctrl_wr_autoend(addr, I2C_Transaction.wrlen); 
		else
			I2C1->CR2 = ctrl_wr_noend(addr, I2C_Transaction.wrlen); 
	}
	else I2C1->CR2 = ctrl_rd_autoend(addr, I2C_Transaction.rdlen); 
}

void I2C1_IRQHandler(void) {
	// Clear OVR/UDR/PEC/TIMEOUT/ALERT error (irrelevant error)
	// Clear BERR (errata)
	// Only error left is ARLO
#define call(arg) \
	if(I2C_Callback) { \
		void (*cb)(int) = I2C_Callback; \
		I2C_Callback = 0; \
		cb(arg); \
	} \
	I2C1->ICR = 0x3D00; 
	unsigned int flags = I2C1->ISR; 
	if(flags & 0x0200) { // ARLO
		I2C1->ICR = 0x0200; 
		I2C_Busy = 0; 
		call(I2C_ARLO); 
	}
	if(flags & 0x0010) { // NACK
		I2C1->ICR = 0x0010; 
		I2C_Busy = 0; 
		call(I2C_NACK); 
	}
	if(flags & 0x0020) { // STOPF
		I2C1->ICR = 0x0020; 
		I2C_Busy = 0; 
		call(I2C_OKAY); 
	}
	if(flags & 0x0040) { // TC (TCR not used)
		// Only WR->RD stage comes here
		I2C_Transaction.index = 0; 
		I2C1->CR2 = ctrl_rd_autoend(I2C_Transaction.addr << 1, I2C_Transaction.rdlen); 
	}
	if(flags & 0x0002) { // TXIS
		I2C1->TXDR = I2C_Transaction.wrbuf[I2C_Transaction.index++]; 
	}
	if(flags & 0x0004) { // RXNE
		I2C_Transaction.rdbuf[I2C_Transaction.index++] = I2C1->RXDR; 
	}
}
