#include <stdio.h>
#include <string.h>
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_drv_clock.h"
#include "FreeRTOS.h"
#include "task.h"

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static uint16_t const              m_demo1_top  = 100;
static uint8_t                     m_demo1_phase;
static nrf_pwm_values_individual_t m_demo1_seq_values;
static nrf_pwm_sequence_t const    m_demo1_seq =
{
    .values.p_individual = &m_demo1_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_demo1_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

static void demo1_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
        //uint16_t * p_channels = (uint16_t *)&m_demo1_seq_values;
        //p_channels[0] = 5000;  
        (void)nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
    }
}

void pwm_init(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            BSP_PWM, // channel 0
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_125kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = m_demo1_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, demo1_handler));

    m_demo1_seq_values.channel_0 = 0;

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void pwm_duty(int percent)
{
    m_demo1_seq_values.channel_0 = percent;
}

void pwm_task(void)
{
    pwm_init();

    for (;;)
    {
       //pwm_duty(90);
       vTaskDelay(5000); 
       //pwm_duty(0);
       vTaskDelay(5000);
    }
}
