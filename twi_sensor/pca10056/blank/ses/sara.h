#ifndef __SARA_H
#define __SARA_H

#include <stdint.h>

typedef struct{
	char rxBuffer[64];
	char txBuffer[64];
	char *rxBuffer_ptr;
}t_sara;

static t_sara _sara;

void sara_init(void);
void sara_setup(void);
void sara_task(void);
void sara_write(char *buffer, char length);
void sara_read(uint8_t *buffer, int length);

#endif