#ifndef __FLASH_H
#define __FLASH_H

#include <stdint.h>

typedef struct
{
    union
    {
        uint32_t buffer;
        struct {
            char profile;
            char group;
            char lux_100;
            unsigned char profile_set : 1;
            unsigned char group_set : 1;
            unsigned char lux_100_set : 1;
            unsigned char reserved : 5;
        }__attribute__((packed));

   };
}t_flash;

extern t_flash _flash;

void flash_init(void);
void flash_delete(void);
void flash_write(void);
void flash_read(void);


#endif