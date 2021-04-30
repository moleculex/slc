#include <string.h>
#include "gps.h"
#include "nrf_drv_twi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "protocol.h"
#include "boards.h"

#define TWI_INSTANCE_ID     0

static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static uint8_t m_sample[256];                        

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;

        default:
            break;
    }
}

void gps_rx(void)
{
    m_xfer_done = false;
    nrf_drv_twi_rx(&m_twi, 0x42, &_sys.buf, 128);
}

void gps_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 19,
       .sda                = 20,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    nrf_drv_twi_enable(&m_twi);

    nrf_gpio_cfg_output(GPS_RESET);

    nrf_gpio_pin_write(GPS_RESET, 1);
    vTaskDelay(1000);
    nrf_gpio_pin_write(GPS_RESET, 0);
    vTaskDelay(1000);
    nrf_gpio_pin_write(GPS_RESET, 1);
}

void gps_task(void)
{
    char *n;
    char latitude[12];
    char longitude[13];

    gps_init();

    for(;;)
    {
        gps_rx();

        if(_sys.buf[0] != 0xff && _sys.buf[0] != '\0' )
        {
            n = strstr((char *)_sys.buf, "$GNRMC,");//$GNGLL,2936.15343,S,03020.38750,E  $GNRMC,152138.00,A,2936.14719,S,03020.38501,E,0.376,,011219,,,A*7D

            if(n && (n[17] == 'A') && _sys.fix == 0x00)
            {
                vTaskDelay(200);

                memcpy(latitude, &n[19], 12);
                memcpy(longitude, &n[32], 13);

                memcpy(&_sys.longitude[0], &longitude[0], 5);
                memcpy(&_sys.longitude[5], &longitude[6], 5);
                if(longitude[12] == 'E')
                  _sys.longitude[10] = '1';
                if(longitude[12] == 'W')
                  _sys.longitude[10] = '0';

                memcpy(&_sys.latitude[0], &latitude[0], 4);
                memcpy(&_sys.latitude[4], &latitude[5], 5);
                if(latitude[11] == 'S')
                  _sys.latitude[9] = '0';
                if(latitude[11] == 'N')
                  _sys.latitude[9] = '1';

                _sys.fix = 0x01;

                /*nrf_gpio_pin_write(BUZZER, 1);
                vTaskDelay(200);
                nrf_gpio_pin_write(BUZZER, 0);
                vTaskDelay(200);
                nrf_gpio_pin_write(BUZZER, 1);
                vTaskDelay(200);
                nrf_gpio_pin_write(BUZZER, 0);*/			
            }	
        }

        vTaskDelay(1000);
    }
}