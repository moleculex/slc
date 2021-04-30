#include <string.h>
#include "flash.h"
#include "fds.h"

static fds_record_t const m_record =
{
    .file_id           = 0x9012,
    .key               = 0x7010,
    .data.p_data       = &_flash,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(_flash) + 3) / sizeof(uint32_t),
};

void flash_init(void)
{
    fds_init();
}

void flash_delete(void)
{
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    fds_record_find(0x9012, 0x7010, &desc, &tok);
    fds_record_delete(&desc);
}

void flash_write(void)
{
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    fds_record_find(0x9012, 0x7010, &desc, &tok);
    fds_record_write(&desc, &m_record);
}

void flash_read(void)
{
    fds_record_desc_t desc = {0};
    fds_flash_record_t config = {0};
    fds_find_token_t  tok  = {0};

   _flash.profile = 0x00;
    _flash.group = 0x00;
    _flash.profile_set = 0;

    fds_record_find(0x9012, 0x7010, &desc, &tok);

    fds_record_open(&desc, &config);
    memcpy(&_flash, config.p_data, sizeof(t_flash));
    fds_record_close(&desc);
}