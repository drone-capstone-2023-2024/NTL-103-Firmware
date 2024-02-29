#include <stm32g030xx.h>

uint8_t LFP_CrcTable[] = {
	0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 
	0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB, 
	0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 
	0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76, 
	0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 
	0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C, 
	0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 
	0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1, 
	0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 
	0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8, 
	0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 
	0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65, 
	0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 
	0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F, 
	0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 
	0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2, 
	0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 
	0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D, 
	0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 
	0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50, 
	0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 
	0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A, 
	0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 
	0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7, 
	0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 
	0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E, 
	0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 
	0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43, 
	0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 
	0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09, 
	0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 
	0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4, 
}; 

#define LFP_MTU 63
#define LFP_MAXBUF 130 // maximum payload 65 bytes, all escaped

typedef struct {
	uint8_t service; 
	uint8_t length; 
	uint8_t payload[LFP_MTU]; 
} LFP_Frame_t; 

#define LFP_ENCODE_OKAY  0 // Encoding fine
#define LFP_ENCODE_MTUEX 1 // MTU exceeded
int LFP_Encode(const LFP_Frame_t * frame, uint8_t * stream, uint8_t * streamSize0) {
	static uint8_t workaround_buffer[LFP_MAXBUF]; // This will work but ugly
	uint8_t payloadLength = frame->length; 
	if(payloadLength > LFP_MTU) return LFP_ENCODE_MTUEX; 
	uint8_t header = frame->service << 6 | frame->length & 0x3F; 
	uint8_t crc = LFP_CrcTable[0xFF ^ header]; 
	stream[0] = header; 
	for(int i = 0; i < payloadLength; i++) {
		uint8_t ch = frame->payload[i]; 
		stream[i + 1] = ch; 
		crc = LFP_CrcTable[crc ^ ch]; 
	}
	stream[payloadLength + 1] = crc; 
	uint8_t packetLength = payloadLength + 2; 
	uint8_t streamSize = 0; 
	uint8_t pendingStuffByte; 
	uint8_t pendingStuffFlag = 0; 
	for(int i = 0; i < packetLength; i++) {
		if(pendingStuffFlag) {
			workaround_buffer[streamSize++] = pendingStuffByte; 
			pendingStuffFlag = 0; 
		}
		uint8_t ch = stream[i]; 
		switch(ch) {
			case 0x55: ch = 0xAA; pendingStuffFlag = 1; pendingStuffByte = 0x05; break; 
			case 0xAA: ch = 0xAA; pendingStuffFlag = 1; pendingStuffByte = 0x0A; break; 
		}
		workaround_buffer[streamSize++] = ch; 
	}
	*streamSize0 = streamSize; 
	for(int i = 0; i < streamSize; i++) {
		stream[i] = workaround_buffer[i]; 
	}
	return LFP_ENCODE_OKAY; 
}

#define LFP_DECODE_OKAY   0 // Decoding fine
#define LFP_DECODE_ESCERR 1 // Escape chracter error
#define LFP_DECODE_LENERR 2 // Length too short, too long, or does not match
#define LFP_DECODE_CRCERR 3 // CRC corrupted
int LFP_Decode(LFP_Frame_t * frame, uint8_t * stream, uint8_t streamSize) {
	int packetSize = 0; 
	int escapeFlag = 0; 
	for(int i = 0; i < streamSize; i++) {
		uint8_t ch = stream[i]; 
		if(escapeFlag) {
			switch(ch) {
				case 0x05: ch = 0x55; break; 
				case 0x0A: ch = 0xAA; break; 
				default: return LFP_DECODE_ESCERR; 
			}
			escapeFlag = 0; 
		}
		else if(ch == 0xAA) {
			escapeFlag = 1; 
			continue; 
		}
		stream[packetSize++] = ch; 
	}
	if(escapeFlag) return LFP_DECODE_ESCERR; 
	if(packetSize < 2) return LFP_DECODE_LENERR; 
	uint8_t header = stream[0]; 
	uint8_t payloadSize = header & 0x3F; 
	if(payloadSize != packetSize - 2) return LFP_DECODE_LENERR; 
	frame->length = payloadSize; 
	frame->service = header >> 6; 
	uint8_t crc = LFP_CrcTable[0xFF ^ header]; 
	for(int i = 0; i < payloadSize; i++) {
		uint8_t ch = stream[i + 1]; 
		frame->payload[i] = ch; 
		crc = LFP_CrcTable[crc ^ ch]; 
	}
	crc = LFP_CrcTable[crc ^ stream[packetSize - 1]]; 
	if(crc != 0) return LFP_DECODE_CRCERR; 
	return LFP_DECODE_OKAY; 
}

typedef struct {
	uint8_t full; 
	uint8_t index; 
	uint8_t length; 
	uint8_t buffer[LFP_MAXBUF]; 
} LFP_Linebuf_t; 

uint8_t LFP_TxLinebuf_SOF; 
LFP_Linebuf_t LFP_TxLinebuf; 
LFP_Linebuf_t LFP_RxLinebuf; 

extern void LFP_TxIRQSetMask(int state); 
//__weak void LFP_TxIRQSetMask(int state) {
//	return; 
//}

uint8_t LFP_TxIRQProducer(void) {
	if(LFP_TxLinebuf_SOF) {
		LFP_TxLinebuf_SOF = 0; 
		return 0x55; 
	}
	if(LFP_TxLinebuf.index >= LFP_TxLinebuf.length) {
		LFP_TxIRQSetMask(0); 
		LFP_TxLinebuf.full = 0; 
		return 0x55; 
	}
	return LFP_TxLinebuf.buffer[LFP_TxLinebuf.index++]; 
}

int LFP_BlockingTx(const LFP_Frame_t * frame) {
	while(LFP_TxLinebuf.full); 
	LFP_TxLinebuf.index = 0; 
	int status = LFP_Encode(frame, LFP_TxLinebuf.buffer, &LFP_TxLinebuf.length); 
	if(status != LFP_ENCODE_OKAY) {
		return status; 
	}
	LFP_TxLinebuf_SOF = 1; 
	LFP_TxLinebuf.full = 1; 
	LFP_TxIRQSetMask(1); 
	return status; 
}

void LFP_RxIRQConsumer(uint8_t ch) {
	if(LFP_RxLinebuf.full) return; // Overrun
	if(ch == 0x55) {
		if(LFP_RxLinebuf.length > 0) LFP_RxLinebuf.full = 1; 
		return; 
	}
	if(LFP_RxLinebuf.length >= LFP_MAXBUF) return; // Babbling
	else
		LFP_RxLinebuf.buffer[LFP_RxLinebuf.length++] = ch; 
}

int LFP_RxAvailable(void) {
	return !!LFP_RxLinebuf.full; 
}

int LFP_Rx(LFP_Frame_t * frame) {
	int status = LFP_Decode(frame, LFP_RxLinebuf.buffer, LFP_RxLinebuf.length); 
	LFP_RxLinebuf.length = 0; 
	LFP_RxLinebuf.full = 0; 
	return status; 
}

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

#undef USART1_TXIRQ_ON
#undef USART1_TXIRQ_OFF
#undef USART1_TXIRQ_ENABLED

#define I2C_WR_MAX 6
#define I2C_RD_MAX 4
typedef struct {
	uint8_t addr; 
	uint8_t index; 
	uint8_t wrlen; 
	uint8_t rdlen; 
	uint8_t wrbuf[I2C_WR_MAX]; 
	uint8_t rdbuf[I2C_RD_MAX]; 
} I2C_Transaction_t; 

uint8_t I2C_Busy; 
I2C_Transaction_t I2C_Transaction; 

#define I2C_OKAY 0
#define I2C_NACK 1
#define I2C_ARLO 2

void (*I2C_Callback)(int status); 

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
#undef call
}

#undef ctrl_wr_noend
#undef ctrl_wr_autoend
#undef ctrl_rd_autoend

void * I2CDRV_Callback; 

void I2CDRV_CB_R1(int st) {
	((void (*)(int, uint8_t))I2CDRV_Callback)(st, I2C_Transaction.rdbuf[0]); 
}

void I2CDRV_CB_R2(int st) {
	uint16_t rd = (uint16_t)I2C_Transaction.rdbuf[0] << 8 | I2C_Transaction.rdbuf[1]; 
	((void (*)(int, uint16_t))I2CDRV_Callback)(st, rd); 
}

void I2CDRV_CB_R3(int st) {
	uint16_t rd1 = (uint16_t)I2C_Transaction.rdbuf[0] << 8 | I2C_Transaction.rdbuf[1]; 
	uint8_t rd2 = I2C_Transaction.rdbuf[2]; 
	((void (*)(int, uint16_t, uint8_t))I2CDRV_Callback)(st, rd1, rd2); 
}

void I2CDRV_W3R0(uint8_t addr, uint8_t wr1, uint16_t wr2, void (*cb)(int st)) {
	I2C_Transaction.addr = addr; 
	I2C_Transaction.wrlen = 3; 
	I2C_Transaction.wrbuf[0] = wr1; 
	I2C_Transaction.wrbuf[1] = wr2 >> 8; 
	I2C_Transaction.wrbuf[2] = wr2; 
	I2C_Transaction.rdlen = 0; 
	I2C_Callback = cb; 
	I2C_Start(); 
}

void I2CDRV_W1R2(uint8_t addr, uint8_t wr, void (*cb)(int st, uint16_t rd)) {
	I2C_Transaction.addr = addr; 
	I2C_Transaction.wrlen = 1; 
	I2C_Transaction.wrbuf[0] = wr; 
	I2C_Transaction.rdlen = 2; 
	I2C_Callback = I2CDRV_CB_R2; 
	I2CDRV_Callback = cb; 
	I2C_Start(); 
}

void I2CDRV_W2R0(uint8_t addr, uint16_t wr, void (*cb)(int st)) {
	I2C_Transaction.addr = addr; 
	I2C_Transaction.wrlen = 2; 
	I2C_Transaction.wrbuf[0] = wr >> 8; 
	I2C_Transaction.wrbuf[1] = wr; 
	I2C_Transaction.rdlen = 0; 
	I2C_Callback = cb; 
	I2C_Start(); 
}

void I2CDRV_W2R3(uint8_t addr, uint16_t wr, void (*cb)(int st, uint16_t rd1, uint8_t rd2)) {
	I2C_Transaction.addr = addr; 
	I2C_Transaction.wrlen = 2; 
	I2C_Transaction.wrbuf[0] = wr >> 8; 
	I2C_Transaction.wrbuf[1] = wr; 
	I2C_Transaction.rdlen = 3; 
	I2C_Callback = I2CDRV_CB_R3; 
	I2CDRV_Callback = cb; 
	I2C_Start(); 
}

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
	TIM14->CCMR1 = 0x38; // PWM mode 2
	TIM14->EGR = 0x1; 
#define TIM14_START()	TIM14->CR1 = 0x1
#define TIM14_STOP()	TIM14->CR1 = 0x9 // One pulse mode stops the counter after current cycle
#define TIM14_WR_PULSE(us) TIM14->CCR1 = 20000 - us // TODO: validate this
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
	// TODO
}

#undef LED1_ON
#undef LED1_OFF
#undef LED2_ON
#undef LED2_OFF
#undef TIM14_START
#undef TIM14_STOP
#undef TIM14_WR_PULSE

// I2C device code
#define TMP1075N 0x49
#define INA236A 0x41
#define SHT30 0x44

typedef struct {
	int initialized; 
	float converterV; 
	float converterA; 
	float converterW; 
	float boardTemp; 
	int extSensorPresent; 
	float extSensorTemp; 
	float extSensorHumid; 
} I2C_Telemetry_t; 

I2C_Telemetry_t I2C_Telemetry; 

void I2CDEV_Init0(void); 
void I2CDEV_Init1(int st); 
void I2CDEV_Init2(int st); 
// I2C device initializations:
void I2CDEV_Init0(void) {
	// TMP1075N: No initialization required. Device defaults to 35ms continuous measurement mode. 
	// INA236A: Set 81.92mV shunt range, avg=16, ts=8.244ms (both), continuous
	I2CDRV_W3R0(INA236A, 0x00, 0x45FF, I2CDEV_Init1); 
}
void I2CDEV_Init1(int st) {
	if(st) {
		I2CDEV_Init0(); 
		return; 
	}
	// SHT30: Set continuous conversion mode 2Hz
	I2CDRV_W2R0(SHT30, 0x2236, I2CDEV_Init2); 
}
void I2CDEV_Init2(int st) {
	I2C_Telemetry.extSensorPresent = !st; 
	// Initialization complete
	I2C_Telemetry.initialized = 1; 
}

void I2CDEV_Act0(void); 
void I2CDEV_Act1(int st, uint16_t rd); 
void I2CDEV_Act2(int st, uint16_t rd); 
void I2CDEV_Act3(int st, uint16_t rd); 
// I2C device readings
void I2CDEV_Act0(void) {
	// TMP1075N: read temperature
	I2CDRV_W1R2(TMP1075N, 0x00, I2CDEV_Act1); 
}
void I2CDEV_Act1(int st, uint16_t rd) {
	if(!st) I2C_Telemetry.boardTemp = (float)(int16_t)rd * 3.90625e-3; 
	// INA236A: read voltage
	I2CDRV_W1R2(INA236A, 0x02, I2CDEV_Act2); 
}
void I2CDEV_Act2(int st, uint16_t rd) {
	if(!st) I2C_Telemetry.converterV = (float)rd * 1.6e-3; 
	// INA236A: read current (39mOhm)
	I2CDRV_W1R2(INA236A, 0x01, I2CDEV_Act3); 
}
void I2CDEV_Act3(int st, uint16_t rd) {
	if(!st) I2C_Telemetry.converterA = (float)(int16_t)rd * 6.41025641e-5; 
	// TODO: power with power register
	I2C_Telemetry.converterW = I2C_Telemetry.converterV * I2C_Telemetry.converterA; 
	// TODO: SHT30 readings
	if(I2C_Telemetry.extSensorPresent); 
}

void I2CDEV_Init(void) {
	I2C_Telemetry.initialized = 0; 
	I2C_Telemetry.extSensorPresent = 0; 
	float NaN; 
	*((uint32_t *)&NaN) = 0xFFFFFFFF; 
	I2C_Telemetry.boardTemp = NaN; 
	I2C_Telemetry.converterA = NaN; 
	I2C_Telemetry.converterV = NaN; 
	I2C_Telemetry.converterW = NaN; 
	I2C_Telemetry.extSensorTemp = NaN; 
	I2C_Telemetry.extSensorHumid = NaN; 
	I2CDEV_Init0(); 
	while(!I2C_Telemetry.initialized); 
}

void I2CDEV_Sample(void) {
	if(I2C_Busy) return; 
	I2CDEV_Act0(); 
}

void Util_WriteFloat(LFP_Frame_t * frame, int startIndex, float number) {
	uint8_t * arr = (uint8_t *)&number; 
	frame->payload[startIndex    ] = arr[3]; 
	frame->payload[startIndex + 1] = arr[2]; 
	frame->payload[startIndex + 2] = arr[1]; 
	frame->payload[startIndex + 3] = arr[0]; 
}

int main(void) {
	SysClock_HWInit(); 
	IO_HWInit(); 
	LFP_HWInit(); 
	I2C_HWInit(); 
	
	IO_LED1_On(); 
	I2CDEV_Init(); 
	IO_LED1_Off(); 
	
	// SysTick 20Hz
	SysTick->CTRL = 0x00; 
	SysTick->VAL = 0x00; 
	SysTick->LOAD = 400000 - 1; 
	SysTick->CTRL = 0x01; 
	
	uint8_t cntsec = 0; // Second counter
	
	// TODO: organize boilerplate code
	uint8_t cnt1val = 0; // Counter 1 value
	uint8_t cnt1rl = 2;  // Counter 1 reload
	uint8_t cnt2val = 0; // Counter 2 value
	uint8_t cnt2rl = 2;  // Counter 2 reload
	uint8_t cnt3val = 0; // Counter 3 value
	uint8_t cnt3rl = 2;  // Counter 3 reload
	
	while(1) {
		uint8_t telemetryBitmap = 0; // Env, Temp, Power
		uint8_t doSampling = 0; 
		
		LFP_Frame_t frame; 
		// Handle frame reception
		if(LFP_RxAvailable()) {
			int status = LFP_Rx(&frame); 
			if(status == LFP_DECODE_OKAY) {
				// Power telemetry
				if(frame.service == 0) {
					if(frame.length == 0) {
						// PacketOutPowerRequest
						telemetryBitmap |= 0x01; 
					}
					if(frame.length == 1) {
						// PacketOutPowerSetRate
						cnt1rl = frame.payload[0]; 
						cnt1val = cnt1rl; 
					}
				}
				// Temperature telemetry
				if(frame.service == 1) {
					if(frame.length == 0) {
						// PacketOutTempRequest
						telemetryBitmap |= 0x02; 
					}
					if(frame.length == 1) {
						// PacketOutPowerSetRate
						cnt2rl = frame.payload[0]; 
						cnt2val = cnt2rl; 
					}
				}
				// Environmental telemetry
				if(frame.service == 2) {
					if(frame.length == 0) {
						// PacketOutEnvRequest
						telemetryBitmap |= 0x04; 
					}
					if(frame.length == 1) {
						// PacketOutPowerSetRate
						cnt3rl = frame.payload[0]; 
						cnt3val = cnt3rl; 
					}
				}
			}
		}
		
		// Handle tick
		if(SysTick->CTRL & 0x00010000) { // 20Hz
			if(++cntsec >= 20) { // 1Hz
				cntsec = 0; 
				// Logic: I2C periodic sampling
				doSampling = 1; 
				// Logic: Counter 1 Power telemetry
				if(cnt1rl && ++cnt1val >= cnt1rl) {
					cnt1val = 0; 
					telemetryBitmap |= 0x01; 
				}
				// Logic: Counter 2 Temperature telemetry
				if(cnt2rl && ++cnt2val >= cnt2rl) {
					cnt2val = 0; 
					telemetryBitmap |= 0x02; 
				}
				// Logic: Counter 3 Environmental telemetry
				if(cnt3rl && ++cnt3val >= cnt3rl) {
					cnt3val = 0; 
					telemetryBitmap |= 0x04; 
				}
				// .. more logic
			}
			// Logic: periodic flashing
			if(cntsec == 1) IO_LED1_On(); 
			if(cntsec == 2) IO_LED1_Off(); 
			// Logic (temporary): low battery indicator
			if(I2C_Telemetry.converterV > 6.5 && I2C_Telemetry.converterV < 19.8) {
				if(cntsec & 0x1) IO_LED2_On(); 
				else IO_LED2_Off(); 
			}
			// .. more logic
		}
		
		// Telemetry writeback
		if(telemetryBitmap & 0x01) {
			// PacketInPowerTelemetry
			frame.service = 0; 
			frame.length = 12; 
			Util_WriteFloat(&frame, 0, I2C_Telemetry.converterV); 
			Util_WriteFloat(&frame, 4, I2C_Telemetry.converterA); 
			Util_WriteFloat(&frame, 8, I2C_Telemetry.converterW); 
			LFP_BlockingTx(&frame); 
		}
		if(telemetryBitmap & 0x02) {
			// PacketInTempTelemetry
			frame.service = 1; 
			frame.length = 20; 
			Util_WriteFloat(&frame, 0, I2C_Telemetry.boardTemp); 
			for(int i = 4; i < 20; i++) frame.payload[i] = 0xFF; 
			LFP_BlockingTx(&frame); 
		}
		if(telemetryBitmap & 0x04) {
			// PacketInEnvTelemetry
			frame.service = 2; 
			frame.length = 8; 
			if(I2C_Telemetry.extSensorPresent) {
				Util_WriteFloat(&frame, 0, I2C_Telemetry.extSensorTemp); 
				Util_WriteFloat(&frame, 4, I2C_Telemetry.extSensorHumid); 
			}
			else for(int i = 0; i < 8; i++) frame.payload[i] = 0xFF; 
			LFP_BlockingTx(&frame); 
		}
		
		// Trigger sampling at the very end
		if(doSampling) {
			I2CDEV_Sample(); 
		}
	}
}

