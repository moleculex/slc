#ifndef __DALI_H
#define __DALI_H

#include <stdint.h>

#define BROADCAST_C 0b11111110
#define ON_C 0b00000101
#define _25 64
#define _50 127
#define _75 192
#define _100 254
#define OFF_C 0b00000000

#define VERSION 0b10010111

extern char isDali;

void dali_init(void);
void dali_tx(uint8_t cmd1, uint8_t cmd2);
void dali_task(void);

#endif
