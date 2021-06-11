#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#define PROTOCOL_FLASH_WRITE 0
#define PROTOCOL_RAM_WRITE 	2
#define PROTOCOL_STATUS 		6

typedef struct{
	union
	{
		char buffer[12];
		struct
		{
			unsigned char id[4];
			unsigned char start;
			unsigned char profile;
			unsigned char group;
			unsigned char command;
			unsigned char data[3];
			unsigned char end;
		}__attribute__((packed));
	};
}t_protocol;
static t_protocol _protocol;

typedef struct{
	union{
		char buffer[12];
		struct {
			char start;
			char profile;
			char group;
			char tilt;
			char seismic; //battery
			char voltage[2];
			char current[2];
			char phase_angle[2];
			//char bat[2];
			char end;
		}__attribute__((packed));
	};
}t_status;
extern t_status _status;

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

extern t_light _light;
extern t_sys _sys;

void protocol_decode(t_protocol *buf);
void protocol_encode(char *buf);

#endif
