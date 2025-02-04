#include <stm32g030xx.h>

#include "libgpio.h"

// IO driver
void IO_HWInit(void) {
	// PA4:  Servo PWM (TIM14_CH1, AF4)
	// PC14: LED1 (active low)
	// PC15: LED2 (active low)
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN; 
	GPIOA->AFR[0] = GPIOA->AFR[0] & 0xFFF0FFFF | 0x00040000; 
	GPIOA->MODER = GPIOA->MODER & 0xFFFFFCFF | 0x00000200; 
	GPIOC->BSRR = 0xC000; 
	GPIOC->MODER = GPIOC->MODER & 0x0FFFFFFF | 0x50000000; 
#define LED1_ON()		GPIOC->BRR = 0x4000
#define LED1_OFF()	GPIOC->BSRR = 0x4000
#define LED2_ON()		GPIOC->BRR = 0x8000
#define LED2_OFF()	GPIOC->BSRR = 0x8000
	// Timer 14: 64MHz, 50Hz, PWM resolution 1us
	RCC->APBENR2 |= RCC_APBENR2_TIM14EN; 
	TIM14->PSC = 64 - 1; // Resolution 1us
	TIM14->ARR = 20000 - 1; // Frequency 50Hz
	TIM14->CCR1 = TIM14->ARR;
	TIM14->CCMR1 = 0x78; // PWM mode 2
	TIM14->EGR = 0x1; 
	TIM14->CCER = 0x1; 
#define TIM14_START()	TIM14->CR1 = 0x1
#define TIM14_STOP()	TIM14->CR1 = 0x9 // One pulse mode stops the counter after current cycle
#define TIM14_WR_PULSE(us) TIM14->CCR1 = 20000 - us
}

void IO_LED1_On(void) {
	LED1_ON(); 
}

void IO_LED1_Off(void) {
	LED1_OFF(); 
}

void IO_LED2_On(void) {
	LED2_ON(); 
}

void IO_LED2_Off(void) {
	LED2_OFF(); 
}

void IO_Servo_Start(void) {
    // TODO: set a default pulse width?
	TIM14_START(); 
}

void IO_Servo_Stop(void) {
	TIM14_STOP(); 
}

void IO_Servo_Write(int us) {
    TIM14_START();
    TIM14_WR_PULSE(us);
}

#define SERVO_MIN_PULSE_WIDTH (1000)
#define SERVO_MAX_PULSE_WIDTH (2000)
#define SERVO_MIN_POSITION (-128)
#define SERVO_MAX_POSITION (127)
void IO_Servo_SetPos(uint8_t pos_two_comp) {
    // Pos param is 2's complement stored in an unsigned byte
    int8_t pos = (int8_t)pos_two_comp;

    int us = SERVO_MIN_PULSE_WIDTH +
            (int)(((int)(pos - SERVO_MIN_POSITION) * (SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH)) /
            (SERVO_MAX_POSITION - SERVO_MIN_POSITION));

    IO_Servo_Write(us);
}
