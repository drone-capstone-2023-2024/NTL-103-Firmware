#ifndef LIBLFP_H__
#define LIBLFP_H__

#include <stdint.h>

#define LFP_MTU 63
#define LFP_MAXBUF 130 // maximum payload 65 bytes, all escaped

typedef struct {
	uint8_t service; 
	uint8_t length; 
	uint8_t payload[LFP_MTU]; 
} LFP_Frame_t; 

typedef struct {
	uint8_t full; 
	uint8_t index; 
	uint8_t length; 
	uint8_t buffer[LFP_MAXBUF]; 
} LFP_Linebuf_t; 


extern void LFP_TxIRQSetMask(int state); // Externally declared

#define LFP_ENCODE_OKAY  0 // Encoding fine
#define LFP_ENCODE_MTUEX 1 // MTU exceeded
extern int LFP_Encode(const LFP_Frame_t * frame, uint8_t * stream, uint8_t * streamSize0); 

#define LFP_DECODE_OKAY   0 // Decoding fine
#define LFP_DECODE_ESCERR 1 // Escape chracter error
#define LFP_DECODE_LENERR 2 // Length too short, too long, or does not match
#define LFP_DECODE_CRCERR 3 // CRC corrupted
extern int LFP_Decode(LFP_Frame_t * frame, uint8_t * stream, uint8_t streamSize); 

// Hardware driver interface
extern uint8_t LFP_TxIRQProducer(void); 
extern void LFP_RxIRQConsumer(uint8_t ch); 

// Application level methods
extern int LFP_BlockingTx(const LFP_Frame_t * frame); 
extern int LFP_RxAvailable(void); 
extern int LFP_Rx(LFP_Frame_t * frame); 


#endif
