#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#define FLASH_WRITE 0
#define RAM_WRITE 	2
#define STATUS 		6

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
		char buffer[14];
		struct {
			char start;
			char profile;
			char group;
			char tilt;
			char seismic; //battery
			char voltage[2];
			char current[2];
			char phase_angle[2];
			char bat[2];
			char end;
		}__attribute__((packed));
	};
}t_status;
static t_status _status;

void protocol_decode(t_protocol *buf);
void protocol_encode(char *buf);

#endif
