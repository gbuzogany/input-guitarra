/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "app_util.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_i2s.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_audio.h"
#include "app_error.h"
#include "boards.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**@file
 * @defgroup usbd_aduio_example main.c
 * @{
 * @ingroup usbd_audio_example
 * @brief USBD Audio class example
 *
 */

#define LED_USB_RESUME (BSP_BOARD_LED_0)
#define LED_USB_START  (BSP_BOARD_LED_1)
#define LED_AUDIO_RX   (BSP_BOARD_LED_2)
#define LED_AUDIO_TX   (BSP_BOARD_LED_3)

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief Audio class user event handler
 */
// static void hp_audio_user_ev_handler(app_usbd_class_inst_t const * p_inst,
//                                      app_usbd_audio_user_event_t   event);
static void mic_audio_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                      app_usbd_audio_user_event_t   event);

/* Channels and feature controls configuration */

/**
 * @brief   Input terminal channel configuration
 */
#define MIC_TERMINAL_CH_CONFIG()                                                                       \
        (APP_USBD_AUDIO_IN_TERM_CH_CONFIG_LEFT_FRONT)

/**
 * @brief   Feature controls
 *
 *      general
 *      channel 0
 *      channel 1
 */
#define MIC_FEATURE_CONTROLS()                                                                     \
        APP_USBD_U16_TO_RAW_DSC(APP_USBD_AUDIO_FEATURE_UNIT_CONTROL_MUTE),                         \
        APP_USBD_U16_TO_RAW_DSC(APP_USBD_AUDIO_FEATURE_UNIT_CONTROL_MUTE),                         \
        APP_USBD_U16_TO_RAW_DSC(APP_USBD_AUDIO_FEATURE_UNIT_CONTROL_MUTE)


/* Microphone descriptors */

/**
 * @brief   Audio class specific format descriptor
 */
APP_USBD_AUDIO_FORMAT_DESCRIPTOR(m_mic_form_desc, 
                                 APP_USBD_AUDIO_AS_FORMAT_I_DSC(    /* Format type 1 descriptor */
                                    1,                              /* Number of channels */
                                    2,                              /* Subframe size */
                                    16,                             /* Bit resolution */
                                    1,                              /* Frequency type */
                                    APP_USBD_U24_TO_RAW_DSC(47619)) /* Frequency */
                                );

/**
 * @brief   Audio class input terminal descriptor
 */
APP_USBD_AUDIO_INPUT_DESCRIPTOR(m_mic_inp_desc, 
                                APP_USBD_AUDIO_INPUT_TERMINAL_DSC(
                                    1,                                     /* Terminal ID */
                                    APP_USBD_AUDIO_TERMINAL_IN_MICROPHONE, /* Terminal type */
                                    1,                                     /* Number of channels */
                                    MIC_TERMINAL_CH_CONFIG())              /* Channels config */
                                );

/**
 * @brief   Audio class output terminal descriptor
 */
APP_USBD_AUDIO_OUTPUT_DESCRIPTOR(m_mic_out_desc, 
                                 APP_USBD_AUDIO_OUTPUT_TERMINAL_DSC(
                                    3,                                     /* Terminal ID */
                                    APP_USBD_AUDIO_TERMINAL_USB_STREAMING, /* Terminal type */
                                    2)                                     /* Source ID */
                                );

/**
 * @brief   Audio class feature unit descriptor
 */
APP_USBD_AUDIO_FEATURE_DESCRIPTOR(m_mic_fea_desc, 
                                  APP_USBD_AUDIO_FEATURE_UNIT_DSC(
                                    2,                      /* Unit ID */
                                    1,                      /* Source ID */
                                    MIC_FEATURE_CONTROLS()) /* List of controls */
                                 );

/* Interfaces lists */

/**
 * @brief Interfaces list passed to @ref APP_USBD_AUDIO_GLOBAL_DEF
 */
#define MIC_INTERFACES_CONFIG() APP_USBD_AUDIO_CONFIG_IN(2, 3)

/*lint -save -e26 -e64 -e123 -e505 -e651*/

/**
 * @brief Microphone Audio class instance
 */
APP_USBD_AUDIO_GLOBAL_DEF(m_app_audio_microphone,
                          MIC_INTERFACES_CONFIG(),
                          mic_audio_user_ev_handler,
                          &m_mic_form_desc,
                          &m_mic_inp_desc,
                          &m_mic_out_desc,
                          &m_mic_fea_desc,
                          0,
                          APP_USBD_AUDIO_AS_IFACE_FORMAT_PCM,
                          192,
                          APP_USBD_AUDIO_SUBCLASS_AUDIOSTREAMING,
                          3
);

/*lint -restore*/

/**
 * @brief Internal audio temporary buffer
 */

#define I2S_DATA_BLOCK_WORDS 32
static uint32_t m_buffer_rx[2][I2S_DATA_BLOCK_WORDS];
static uint32_t const * volatile mp_block_to_check = NULL;

#define AUDIO_BUF_SIZE 512
static uint16_t m_audio_buffer[AUDIO_BUF_SIZE];
static uint16_t read_pos = 0;
static uint16_t write_pos = 0;
static uint32_t buffer_length = 0;

#define BLOCKS_TO_TRANSFER  20

static uint8_t volatile m_blocks_transferred     = 0;
static uint8_t          m_zero_samples_to_ignore = 2;
static bool             m_error_encountered;
static bool             m_recording = false;
static uint32_t         m_tx_cnt    = 0;

static bool             buffer_full = false;
/**
 * @brief The size of last received block from the microphone
 */
// static size_t m_temp_buffer_size;

/**
 * @brief Actual microphone mute state
 */
static uint8_t  m_mute_mic;

/**
 * @brief Actual microphone sampling frequency
 */
static uint32_t m_freq_mic;

/**
 * @brief Audio class specific request handle (microphone)
 */
static void mic_audio_user_class_req(app_usbd_class_inst_t const * p_inst)
{
    app_usbd_audio_t const * p_audio = app_usbd_audio_class_get(p_inst);
    app_usbd_audio_req_t * p_req = app_usbd_audio_class_request_get(p_audio);

    UNUSED_VARIABLE(m_mute_mic);
    UNUSED_VARIABLE(m_freq_mic);

    switch (p_req->req_target)
    {
        case APP_USBD_AUDIO_CLASS_REQ_IN:

            if (p_req->req_type == APP_USBD_AUDIO_REQ_SET_CUR)
            {
                //Only mute control is defined
                p_req->payload[0] = m_mute_mic;
            }

            break;
        case APP_USBD_AUDIO_CLASS_REQ_OUT:

            if (p_req->req_type == APP_USBD_AUDIO_REQ_SET_CUR)
            {
                //Only mute control is defined
                m_mute_mic = p_req->payload[0];
            }

            break;
        case APP_USBD_AUDIO_EP_REQ_IN:
            break;
        case APP_USBD_AUDIO_EP_REQ_OUT:

            if (p_req->req_type == APP_USBD_AUDIO_REQ_SET_CUR)
            {
                //Only set frequency is supported
                m_freq_mic = uint24_decode(p_req->payload);
            }

            break;
        default:
            break;
    }
}

/**
 * @brief User event handler @ref app_usbd_audio_user_ev_handler_t (microphone)
 */
static void mic_audio_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                      app_usbd_audio_user_event_t   event)
{
    app_usbd_audio_t const * p_audio = app_usbd_audio_class_get(p_inst);
    UNUSED_VARIABLE(p_audio);

    switch (event)
    {
        case APP_USBD_AUDIO_USER_EVT_CLASS_REQ:
            mic_audio_user_class_req(p_inst);
            break;
        case APP_USBD_AUDIO_USER_EVT_TX_DONE:
        {
            bsp_board_led_invert(LED_AUDIO_TX);
            break;
        }
        default:
            break;
    }
}

/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
            if (APP_USBD_STATE_Configured != app_usbd_core_state_get())
            {
                break;
            }

            if (buffer_length >= 47) {
                uint32_t len = 47;
                if (buffer_length > 47*3) {
                    len = 48;
                }
                
                uint16_t buf[len];
                for (int i=0;i<len;i++) {
                    if (m_mute_mic) {
                        buf[i] = 0;
                        read_pos++;
                    }
                    else {
                        buf[i] = m_audio_buffer[read_pos++];
                    }
                    if (read_pos >= AUDIO_BUF_SIZE) {
                        read_pos -= AUDIO_BUF_SIZE;
                    }
                }
                // read_pos = write_pos;
                // buffer_length = 0;
                buffer_length -= len;

                /* Block from headphones copied into buffer, send it into microphone input */
                ret_code_t ret = app_usbd_audio_class_tx_start(&m_app_audio_microphone.base, buf, len * 2);
                if (NRF_SUCCESS == ret)
                {
                    bsp_board_led_invert(LED_AUDIO_TX);
                }
            }
            else {
                m_tx_cnt += 1;
            }
            break;
        case APP_USBD_EVT_DRV_SUSPEND:
            bsp_board_leds_off();
            m_recording = false;
            NRF_LOG_INFO("USB suspend");
            break;
        case APP_USBD_EVT_DRV_RESUME:
            NRF_LOG_INFO("USB resume");
            m_recording = true;
            // bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            NRF_LOG_INFO("USB started");
            // bsp_board_led_on(LED_USB_START);
            break;
        case APP_USBD_EVT_STOPPED:
            NRF_LOG_INFO("USB stopped");
            m_recording = false;
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}


static bool check_samples(uint32_t const * p_block)
{
    // [each data word contains one 24 biut]
    uint32_t i;

    for (i = 0; i < I2S_DATA_BLOCK_WORDS; ++i)
    {

        uint32_t const * p_word = &p_block[i];
        uint16_t sample_1 = ((uint16_t const *)p_word)[0];
        uint16_t sample_0 = ((uint16_t const *)p_word)[1];

        // Normally a couple of initial samples sent by the I2S peripheral
        // will have zero values, because it starts to output the clock
        // before the actual data is fetched by EasyDMA. As we are dealing
        // with streaming the initial zero samples can be simply ignored.
        if (m_zero_samples_to_ignore > 0 &&
            sample_1 == 0 &&
            sample_0 == 0)
        {
            --m_zero_samples_to_ignore;
        }
        else {
            if (buffer_length < AUDIO_BUF_SIZE - 2) {
                m_audio_buffer[write_pos+1]   = sample_0;
                m_audio_buffer[write_pos] = sample_1;
                write_pos+= 2;

                buffer_length += 2;
                if (write_pos >= AUDIO_BUF_SIZE) {
                    write_pos -= AUDIO_BUF_SIZE;
                }
            }
            else {
                buffer_full = true;
            }
        }
    }
    return true;
}

static void check_rx_data(uint32_t const * p_block)
{
    if (m_blocks_transferred < m_zero_samples_to_ignore) {
        ++m_blocks_transferred;
    }

    if (!m_error_encountered && m_recording)
    {
        m_error_encountered = !check_samples(p_block);
    }

    if (m_error_encountered)
    {
        NRF_LOG_INFO("error?");
        // bsp_board_led_off(LED_OK);
        // bsp_board_led_invert(LED_ERROR);
    }
    else
    {
        // NRF_LOG_INFO("rx");
        // bsp_board_led_off(LED_ERROR);
        bsp_board_led_invert(BSP_BOARD_LED_1);
    }
}

static void data_handler(nrf_drv_i2s_buffers_t const * p_released,
                         uint32_t                      status)
{
    // 'nrf_drv_i2s_next_buffers_set' is called directly from the handler
    // each time next buffers are requested, so data corruption is not
    // expected.
    ASSERT(p_released);

    // When the handler is called after the transfer has been stopped
    // (no next buffers are needed, only the used buffers are to be
    // released), there is nothing to do.
    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
        return;
    }

    // First call of this handler occurs right after the transfer is started.
    // No data has been transferred yet at this point, so there is nothing to
    // check. Only the buffers for the next part of the transfer should be
    // provided.
    if (p_released->p_rx_buffer == false)
    {
        nrf_drv_i2s_buffers_t const next_buffers = {
            .p_rx_buffer = m_buffer_rx[1],
            .p_tx_buffer = NULL,
        };
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));
    }
    else
    {
        // mp_block_to_check = p_released->p_rx_buffer;
        check_rx_data(p_released->p_rx_buffer);
        // The driver has just finished accessing the buffers pointed by
        // 'p_released'. They can be used for the next part of the transfer
        // that will be scheduled now.
        nrf_drv_i2s_buffers_t const next_buffers = {
            .p_rx_buffer = m_buffer_rx[0],
            .p_tx_buffer = NULL,
        };
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));
    }
}

void i2s_init() 
{
    // UNUSED_VARIABLE(data_handler);
    nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;
    config.sdin_pin  = I2S_CONFIG_SDIN_PIN;
    config.lrck_pin  = I2S_CONFIG_LRCK_PIN;
    config.sck_pin   = I2S_CONFIG_SCK_PIN;
    config.sdout_pin = NRFX_I2S_PIN_NOT_USED;
    config.mck_pin   = NRFX_I2S_PIN_NOT_USED;
    config.mck_setup = NRF_I2S_MCK_32MDIV21;
    config.ratio     = NRF_I2S_RATIO_32X;
    config.channels  = NRF_I2S_CHANNELS_LEFT;
    config.mode      = NRF_I2S_MODE_MASTER;
    config.sample_width = NRF_I2S_SWIDTH_16BIT;

    ret_code_t ret = nrf_drv_i2s_init(&config, data_handler);
    APP_ERROR_CHECK(ret);
}

static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 1;
    err_code = nrfx_rtc_init(&rtc, &config, NULL);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrfx_rtc_enable(&rtc);
}

uint32_t get_rtc_counter(void)
{
    return NRF_RTC2->COUNTER;
}

int main(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler,
        .enable_sof = true
    };

    // Initialize LEDs and buttons
    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);

    rtc_config();

    ret = NRF_LOG_INIT(get_rtc_counter);
    APP_ERROR_CHECK(ret);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    NRF_LOG_INFO("USBD audio example started.");

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_inst_mic =
    app_usbd_audio_class_inst_get(&m_app_audio_microphone);
    ret = app_usbd_class_append(class_inst_mic);
    APP_ERROR_CHECK(ret);

    bsp_board_leds_off();

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }

    mp_block_to_check = NULL;
    m_blocks_transferred = 0;

    UNUSED_VARIABLE(m_audio_buffer);
    UNUSED_VARIABLE(read_pos);
    UNUSED_VARIABLE(write_pos);

    i2s_init();

    nrf_drv_i2s_buffers_t const initial_buffers = {
        .p_tx_buffer = NULL,
        .p_rx_buffer = m_buffer_rx[0],
    };
    ret = nrf_drv_i2s_start(&initial_buffers, I2S_DATA_BLOCK_WORDS, 0);
    APP_ERROR_CHECK(ret);

    int i = 0;

    while (true)
    {
        
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        
        if (mp_block_to_check)
        {
            // check_rx_data(mp_block_to_check);
            // mp_block_to_check = NULL;
        }
    
        if (i % 5000 == 0) {
            if (m_tx_cnt > 0) {
                NRF_LOG_INFO("TX Failed: %d", m_tx_cnt);
                m_tx_cnt = 0;
            }
            if (buffer_full) {
                NRF_LOG_INFO("Buffer filled");
                buffer_full = false;
            }
            NRF_LOG_INFO("Buffer: %d", buffer_length);
            i = 0;
        }

        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        /* Sleep CPU only if there was no interrupt since last loop processing */
        __WFE();
        i++;
    }
}

/** @} */
