//#include "platform_config.h"
#include "protocol.h"
//#include "pwm.h"
//#include "flash.h"

void protocol_decode(t_protocol *buf)
{
	if(buf->start == 0x7f && buf->end == 0xfe)
	{
		if(buf->command == FLASH_WRITE)
		{
			/*flash_read(FLASH_CONFIG, _flash.buffer, 2);
			flash_erase(FLASH_CONFIG);
			_flash.profile = _protocol.data[1];
			_flash.group = _protocol.data[2];
			flash_write(FLASH_CONFIG, _flash.buffer, 2);*/
		}

		if(buf->command == RAM_WRITE)
		{
			//_light.photocell = _protocol.data[0];
			//_light.pwm = (_protocol.data[1] & 0x0F) * 100 + (_protocol.data[2] >> 4 & 0x0F) * 10 + (_protocol.data[2] & 0x0f);
		}

		if(buf->command == STATUS)
		{
			//_sys.status = 1;
		}
	}
}
