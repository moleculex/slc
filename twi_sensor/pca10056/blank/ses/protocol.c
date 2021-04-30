//#include "platform_config.h"
#include "protocol.h"
#include "pwm.h"
#include "flash.h"
#include "FreeRTOS.h"
#include "task.h"

t_flash _flash;

void protocol_decode(t_protocol *buf)
{
    if(buf->start == 0x7f && buf->end == 0xfe)
    {
        if(buf->command == PROTOCOL_FLASH_WRITE)
        {
            flash_read();
            vTaskDelay(100);
            _flash.profile = _protocol.data[1];
            _flash.group = _protocol.data[2];
            _flash.profile_set = 1;
            _flash.group_set = 1;

            flash_delete();
            vTaskDelay(100);
            flash_write();
            vTaskDelay(100);
        }

        if(buf->command == PROTOCOL_RAM_WRITE)
        {
            //_light.photocell = _protocol.data[0];
            _light.pwm = (_protocol.data[1] & 0x0F) * 100 + (_protocol.data[2] >> 4 & 0x0F) * 10 + (_protocol.data[2] & 0x0f);
            pwm_duty(_light.pwm);
        }

        if(buf->command == PROTOCOL_STATUS)
        {
            _sys.status = 1;
        }
    }
}
