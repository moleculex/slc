//#include "platform_config.h"
#include "protocol.h"
#include "pwm.h"
#include "fds.h"

static fds_record_t const m_dummy_record =
{
    .file_id           = 0x8010,
    .key               = 0x7010,
    .data.p_data       = &_flash,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(_flash) + 3) / sizeof(uint32_t),
};

void protocol_decode(t_protocol *buf)
{
	if(buf->start == 0x7f && buf->end == 0xfe)
	{
		if(buf->command == PROTOCOL_FLASH_WRITE)
		{
			//flash_read(FLASH_CONFIG, _flash.buffer, 2);
			//flash_erase(FLASH_CONFIG);
			_flash.profile = _protocol.data[1];
			_flash.group = _protocol.data[2];
			//flash_write(FLASH_CONFIG, _flash.buffer, 2);

                        fds_record_desc_t desc = {0};
                        fds_record_write(&desc, &m_dummy_record);
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
