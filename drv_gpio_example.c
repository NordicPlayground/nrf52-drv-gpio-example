/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 * 
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 * 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "nrf_error.h"
#include "drv_gpio.h"

#include <string.h>

#ifndef NRF52
#error "ERROR: This example is only for NRF52."
#endif

#define M_BUTTON1_PIN   (13)
#define M_BUTTON2_PIN   (14)
#define M_BUTTON3_PIN   (15)
#define M_BUTTON4_PIN   (16)


#define M_LED1_PIN      (17)
#define M_LED2_PIN      (18)
#define M_LED3_PIN      (19)
#define M_LED4_PIN      (20)


#define M_BUTTONS_MSK   ((1UL << M_BUTTON1_PIN) | (1UL << M_BUTTON2_PIN) | (1UL << M_BUTTON3_PIN) | (1UL << M_BUTTON4_PIN))
#define M_LEDS_MSK      ((1UL << M_LED1_PIN)    | (1UL << M_LED2_PIN)    | (1UL << M_LED3_PIN)    | (1UL << M_LED4_PIN))


static void m_example_start_indicate(void)
{
    drv_gpio_outpin_cfg_t out_cfg = DRV_GPIO_OUTPIN_CFG_DEFAULT;

    drv_gpio_outpins_cfg(M_LEDS_MSK, out_cfg, NULL);
    
    for ( uint_fast8_t n = 0; n < 4; ++n )
    {
        for ( uint32_t i = 0; i < 0xFFFFF; ++i ) __NOP();
        drv_gpio_outport_modify(M_LEDS_MSK, 0);
        for ( uint32_t i = 0; i < 0xFFFFF; ++i ) __NOP();
        drv_gpio_outport_modify(0, M_LEDS_MSK);
    }
    
    drv_gpio_pins_disconnect(M_LEDS_MSK);
}


/*
    m_drv_gpio_pin_cfg_example shows:
        - Configurations of pins groups.
        - Getting the level of single pins.
        - Setting the level of single pins.
        - Disconnecting single pins.
 */
static void m_drv_gpio_pin_cfg_example(void)
{
    static const uint8_t MAPPING_SIZE                  = 4;
    static const uint8_t BUTTONS_PIN_MAP[MAPPING_SIZE] = {M_BUTTON1_PIN, M_BUTTON2_PIN, M_BUTTON3_PIN, M_BUTTON4_PIN};
    static const uint8_t LEDS_PIN_MAP[MAPPING_SIZE]    = {M_LED1_PIN,    M_LED2_PIN,    M_LED3_PIN,    M_LED4_PIN};
    
    m_example_start_indicate();
    
    drv_gpio_outpin_cfg_t out_cfg = DRV_GPIO_OUTPIN_CFG_DEFAULT;
    drv_gpio_inpin_cfg_t in_cfg   = DRV_GPIO_INPIN_CFG_DEFAULT;
 
    /* Tweak the default to use the internal pullup resistors of the nRF52 since
       there are no external pullup resistors on the nRF52 development board. */
    in_cfg.pull = DRV_GPIO_PULL_UP;

    drv_gpio_outpin_cfg(M_LED1_PIN, out_cfg, NULL);
    drv_gpio_outpin_cfg(M_LED2_PIN, out_cfg, NULL);
    drv_gpio_outpin_cfg(M_LED3_PIN, out_cfg, NULL);
    drv_gpio_outpin_cfg(M_LED4_PIN, out_cfg, NULL);
    
    drv_gpio_inpin_cfg(M_BUTTON1_PIN, in_cfg, NULL);
    drv_gpio_inpin_cfg(M_BUTTON2_PIN, in_cfg, NULL);
    drv_gpio_inpin_cfg(M_BUTTON3_PIN, in_cfg, NULL);
    drv_gpio_inpin_cfg(M_BUTTON4_PIN, in_cfg, NULL);
    
    do
    { 
        uint8_t level;
        
        for ( uint_fast8_t i = 0; i < MAPPING_SIZE; ++i )
        {
            if ( drv_gpio_inpin_get(BUTTONS_PIN_MAP[i], &level) == NRF_SUCCESS )
            {
                if ( level == DRV_GPIO_LEVEL_LOW )
                {
                    drv_gpio_outpin_level_set(LEDS_PIN_MAP[i], DRV_GPIO_LEVEL_HIGH);
                }
                else
                {
                    drv_gpio_outpin_level_set(LEDS_PIN_MAP[i], DRV_GPIO_LEVEL_LOW);
                }
            }
        }
    }
    while ((drv_gpio_inport_get() & M_BUTTONS_MSK) != 0);
    while ((drv_gpio_inport_get() & M_BUTTONS_MSK) != M_BUTTONS_MSK);
    
    for ( uint_fast8_t i = 0; i < MAPPING_SIZE; ++i )
    {
        drv_gpio_pin_disconnect(BUTTONS_PIN_MAP[i]);
        drv_gpio_pin_disconnect(LEDS_PIN_MAP[i]);
    }
}


/*
    m_drv_gpio_pins_cfg_example shows:
        - Configurations of pins groups.
        - Getting the level of a group of pins.
        - Setting the level of a group of pins.
        - Disconnecting a group of pins.
 */
static void m_drv_gpio_pins_cfg_example(void)
{
    uint32_t inport;
    
    drv_gpio_outpin_cfg_t out_cfg = DRV_GPIO_OUTPIN_CFG_DEFAULT;
    drv_gpio_inpin_cfg_t in_cfg   = DRV_GPIO_INPIN_CFG_DEFAULT;
 
    m_example_start_indicate();
    
    /* Tweak the default to use the internal pullup resistors of the nRF52 since
       there are no external pullup resistors on the nRF52 development board. */
    in_cfg.pull = DRV_GPIO_PULL_UP;

    drv_gpio_outpins_cfg(M_LEDS_MSK, out_cfg, NULL);
    drv_gpio_inpins_cfg(M_BUTTONS_MSK, in_cfg, NULL);
    
    do
    { 
        inport = drv_gpio_inport_get();
        
        drv_gpio_outport_set((inport >> M_BUTTON1_PIN) << M_LED1_PIN);
    }
    while ((inport & M_BUTTONS_MSK)                != 0);
    while ((drv_gpio_inport_get() & M_BUTTONS_MSK) != M_BUTTONS_MSK);
    
    drv_gpio_pins_disconnect(M_BUTTONS_MSK | M_LEDS_MSK);
}


/*
    m_drv_gpio_toggle_example shows:
        - Configurations of pins groups.
        - Getting the level of a group of pins.
        - Sensing a group of pins using interrupts and the low-power PORT-event.
        - Setting the level of single pins.
        - Disconnecting a group of pins.
 */
static void m_drv_gpio_toggle_example_sig_handler(uint8_t pin, uint8_t sensed_state)
{
    uint8_t level = ( sensed_state == DRV_GPIO_SENSE_HITOLO ) ? DRV_GPIO_LEVEL_HIGH : DRV_GPIO_LEVEL_LOW;
    
    switch ( pin )
    {
        case M_BUTTON1_PIN:
            drv_gpio_outpin_level_set(M_LED1_PIN, level);
            break;
        case M_BUTTON2_PIN:
            drv_gpio_outpin_level_set(M_LED2_PIN, level);
            break;
        case M_BUTTON3_PIN:
            drv_gpio_outpin_level_set(M_LED3_PIN, level);
            break;
        case M_BUTTON4_PIN:
            drv_gpio_outpin_level_set(M_LED4_PIN, level);
            break;
    }
}


static void m_drv_gpio_toggle_example(void)
{
    drv_gpio_outpin_cfg_t out_cfg = DRV_GPIO_OUTPIN_CFG_DEFAULT;
    drv_gpio_inpin_cfg_t in_cfg   = 
    {
        .sense    = DRV_GPIO_SENSE_ANY,
        .pull     = DRV_GPIO_PULL_UP,
        .gpiote   = DRV_GPIO_GPIOTE_DISABLE,
        .handler  = DRV_GPIO_HANDLER_ENABLE,
    };
    
    m_example_start_indicate();
 
    drv_gpio_outpins_cfg(M_LEDS_MSK, out_cfg, NULL);
    drv_gpio_inpins_cfg(M_BUTTONS_MSK, in_cfg, NULL);
    
    drv_gpio_sig_handler_set(m_drv_gpio_toggle_example_sig_handler);
    
    while ((drv_gpio_inport_get() & M_BUTTONS_MSK) != 0);
    while ((drv_gpio_inport_get() & M_BUTTONS_MSK) != M_BUTTONS_MSK);

    drv_gpio_pins_disconnect(M_BUTTONS_MSK | M_LEDS_MSK);
    
    drv_gpio_sig_handler_set(NULL);
}


/*
    m_drv_gpio_toggle_hw_example shows:
        - Configurations of pins groups.
        - Getting the level of a group of pins.
        - Toggling of sensing of inputs and toggling of outputs without firmware interraction.
        - Disconnecting a group of pins.
 */
static void m_drv_gpio_toggle_hw_example(void)
{
    uint32_t * tasks[4];
    uint32_t * events[4];
    drv_gpio_outpin_cfg_t out_cfg = 
    {
        .level    = DRV_GPIO_LEVEL_LOW,
        .drive    = DRV_GPIO_DRIVE_S0S1,
        .gpiote   = DRV_GPIO_GPIOTE_ENABLE,
        .task     = DRV_GPIO_TASK_TOGGLE,
    };
    drv_gpio_inpin_cfg_t in_cfg   = 
    {
        .sense    = DRV_GPIO_SENSE_ANY,
        .pull     = DRV_GPIO_PULL_UP,
        .gpiote   = DRV_GPIO_GPIOTE_ENABLE,
        .handler  = DRV_GPIO_HANDLER_DISABLE,
    };
    
    m_example_start_indicate();
 
    drv_gpio_outpins_cfg(M_LEDS_MSK, out_cfg, &(tasks[0]));
    drv_gpio_inpins_cfg(M_BUTTONS_MSK, in_cfg, &(events[0]));
    
    for ( uint_fast8_t i = 0; i < 4; ++i )
    {
        NRF_PPI->CH[i].EEP = (uint32_t)events[i];
        NRF_PPI->CH[i].TEP = (uint32_t)tasks[i];
        
        NRF_PPI->CHENSET = 1UL << i;
    }        
    
    while ((drv_gpio_inport_get() & M_BUTTONS_MSK) != 0);
    while ((drv_gpio_inport_get() & M_BUTTONS_MSK) != M_BUTTONS_MSK);

    for ( uint_fast8_t i = 0; i < 4; ++i )
    {
        NRF_PPI->CHENCLR = 1UL << i;
    }        

    drv_gpio_pins_disconnect(M_BUTTONS_MSK | M_LEDS_MSK);
}


/*
    m_drv_gpio_select_example shows:
        - Configurations of pins groups.
        - Sensing a group of pins using interrupts and GPIOTE IN-event.
        - Interrupt on falling edge for a group of pins.
        - Toggling output pins on interrupt.
        - Disconnecting a group of pins.
 */
static void m_drv_gpio_select_example_sig_handler(uint8_t pin, uint8_t sensed_state)
{
    switch ( pin )
    {
        case M_BUTTON1_PIN:
            drv_gpio_outport_toggle(1UL << M_LED1_PIN);
            break;
        case M_BUTTON2_PIN:
            drv_gpio_outport_toggle(1UL << M_LED2_PIN);
            break;
        case M_BUTTON3_PIN:
            drv_gpio_outport_toggle(1UL << M_LED3_PIN);
            break;
        case M_BUTTON4_PIN:
            drv_gpio_outport_toggle(1UL << M_LED4_PIN);
            break;
    }
}


static void m_drv_gpio_select_example(void)
{
    drv_gpio_outpin_cfg_t out_cfg = DRV_GPIO_OUTPIN_CFG_DEFAULT;
    drv_gpio_inpin_cfg_t in_cfg   = 
    {
        .sense    = DRV_GPIO_SENSE_HITOLO,
        .pull     = DRV_GPIO_PULL_UP,
        .gpiote   = DRV_GPIO_GPIOTE_ENABLE,
        .handler  = DRV_GPIO_HANDLER_ENABLE,
    };
    
    m_example_start_indicate();
 
    drv_gpio_outpins_cfg(M_LEDS_MSK, out_cfg, NULL);
    drv_gpio_inpins_cfg(M_BUTTONS_MSK, in_cfg, NULL);
    
    drv_gpio_sig_handler_set(m_drv_gpio_select_example_sig_handler);
    
    while ((drv_gpio_inport_get() & M_BUTTONS_MSK) != 0);
    while ((drv_gpio_inport_get() & M_BUTTONS_MSK) != M_BUTTONS_MSK);

    drv_gpio_pins_disconnect(M_BUTTONS_MSK | M_LEDS_MSK);
    
    drv_gpio_sig_handler_set(NULL);
}


int main(void)
{
    NVIC_EnableIRQ(GPIOTE_IRQn);
    
    m_drv_gpio_pin_cfg_example();
    
    m_drv_gpio_pins_cfg_example();
    
    m_drv_gpio_toggle_example();
    
    m_drv_gpio_toggle_hw_example();
    
    m_drv_gpio_select_example();

    for ( ;; )
    {
    };
}
