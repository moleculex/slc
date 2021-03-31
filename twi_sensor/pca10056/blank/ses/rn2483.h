#ifndef __RN2483_H
#define __RN2483_H

#include <stdint.h>

#define	RN2483_INIT 0x01
#define	RN2483_TX 0x02

typedef struct{
	char rxBuffer[64];
	char txBuffer[64];
	char *rxBuffer_ptr;
	char deveui[16];
	char devaddr[8];
	char joined;
	long int session_timer;
}t_rn2483;

static t_rn2483 _rn2483;

typedef struct {
	char photocell;
	int pwm;
}t_light;

typedef struct {
	char latitude[10];
	char longitude[11];
	uint8_t buf[128];
	char status;
	char fix;
	char sent;
	char led;
}t_sys;

static t_light _light;
static t_sys _sys;

void rn2483_init(void);
void rn2483_task(void);
void rn2483_rx(void);
void rn2483_tx(char *buffer, char length);
void rn2483_read(uint8_t *buffer, int length);
void rn2483_write(char *buffer, char length);
void rn2483_reset(void);
void rn2483_rfInit(void);
void rn2483_macInit(void);
void rn2483_macResume(void);

void hexStr_to_hex(char *result, char *data, int byte_count);
void hex_to_hexStr(char *hex, char *ascii, int byte_count);
int32_t rand1( void );
void srand1( uint32_t seed );

#endif