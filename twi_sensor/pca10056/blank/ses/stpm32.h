#ifndef __STPM32_H
#include <stdint.h>

typedef struct{
	char rxBuffer[20];
	char txBuffer[5];
	char *rxBuffer_ptr;
	char vrms[2];
	char irms[2];
	char pha[2];
	char state;
}t_stpm32;

static t_stpm32 _stpm32;

void stpm32_init(void);
void stpm32_manager(void);
//void stpm32_sendBreak(void);
void stpm32_write(char *buffer, char length);
void stpm32_read(uint8_t *buffer, int length);
void stpm32_readVI(void);
void stpm32_readPHA(void);
void stpm32_enableLatch(void);

#endif