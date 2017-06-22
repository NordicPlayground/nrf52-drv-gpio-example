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


#ifndef DRV_GPIO_H__
#define DRV_GPIO_H__

#include "nrf.h"

#include <stdint.h>


/**@brief Definition for undefined signal handler. */
#define DRV_GPIO_NO_SIG_HANDLER         NULL


/**@brief Definition of a pointer that does not point to parameters. */
#define DRV_GPIO_NO_PARAM_PTR           NULL


/**@brief The total number of GPIO pins. */
#define DRV_GPIO_NR_OF_PINS             (32)


/**@brief The total number of GPIOTE instances. */
#define DRV_GPIO_NR_OF_GPIOTE_INSTANCES (8)


/**@brief Default input pin configuration is simple input without
          using the GPIOTE peripheral, sensing nor internal pull resistors. */
#define DRV_GPIO_INPIN_CFG_DEFAULT          \
{                                           \
    .sense    = DRV_GPIO_SENSE_NONE,        \
    .pull     = DRV_GPIO_PULL_NONE,         \
    .gpiote   = DRV_GPIO_GPIOTE_DISABLE,    \
    .handler  = DRV_GPIO_HANDLER_DISABLE,   \
}


/**@brief Default output pin configuration is initially logical
          low signal level with standard output drive without using the GPIOTE peripheral. */
#define DRV_GPIO_OUTPIN_CFG_DEFAULT         \
{                                           \
    .level    = DRV_GPIO_LEVEL_LOW,         \
    .drive    = DRV_GPIO_DRIVE_S0S1,        \
    .gpiote   = DRV_GPIO_GPIOTE_DISABLE,    \
}


#define DRV_GPIO_SENSE_Width        (2) /**< The width of the sense field. */
#define DRV_GPIO_SENSE_NONE         (0) /**< No sense. */
#define DRV_GPIO_SENSE_LOTOHI       (1) /**< Low to high sensing. */
#define DRV_GPIO_SENSE_HITOLO       (2) /**< High to low sensing. */
#define DRV_GPIO_SENSE_ANY          (3) /**< Either low to high or high to low sensing. */


#define DRV_GPIO_PULL_Width         (2) /**< The width of the pull field. */
#define DRV_GPIO_PULL_NONE          (0) /**< No pull. */
#define DRV_GPIO_PULL_UP            (3) /**< Pull-up. */
#define DRV_GPIO_PULL_DOWN          (1) /**< Pull-down */


#define DRV_GPIO_GPIOTE_Width       (1) /**< The width of the gpiote field. */
#define DRV_GPIO_GPIOTE_DISABLE     (0) /**< GPIOTE handling disabled. */
#define DRV_GPIO_GPIOTE_ENABLE      (1) /**< GPIOTE handling enabled. */


#define DRV_GPIO_HANDLER_Width      (1) /**< The width of the handler field. */
#define DRV_GPIO_HANDLER_DISABLE    (0) /**< Handler disabled. */
#define DRV_GPIO_HANDLER_ENABLE     (1) /**< Handler enabled. */


#define DRV_GPIO_LEVEL_Width        (1) /**< The width of the level field. */
#define DRV_GPIO_LEVEL_LOW          (0) /**< Logical low level. */
#define DRV_GPIO_LEVEL_HIGH         (1) /**< Logical high level. */


#define DRV_GPIO_DRIVE_Width        (3) /**< The width of the drive field. */
#define DRV_GPIO_DRIVE_S0S1         (0) /**< Standard '0', standard '1' */
#define DRV_GPIO_DRIVE_H0S1         (1) /**< High drive '0', standard '1' */
#define DRV_GPIO_DRIVE_S0H1         (2) /**< Standard '0', high drive '1' */
#define DRV_GPIO_DRIVE_H0H1         (3) /**< High drive '0', high drive '1' */
#define DRV_GPIO_DRIVE_D0S1         (4) /**< Disconnect '0' standard '1' (normally used for wired-or connections) */
#define DRV_GPIO_DRIVE_D0H1         (5) /**< Disconnect '0', high drive '1' (normally used for wired-or connections) */
#define DRV_GPIO_DRIVE_S0D1         (6) /**< Standard '0'. disconnect '1' (normally used for wired-and connections) */
#define DRV_GPIO_DRIVE_H0D1         (7) /**< High drive '0', disconnect '1' (normally used for wired-and connections) */


#define DRV_GPIO_TASK_Width         (2) /**< The width of the task field. */
#define DRV_GPIO_TASK_CLEAR         (0) /**< Task of clear-task type. */
#define DRV_GPIO_TASK_SET           (1) /**< Task of set-task type. */
#define DRV_GPIO_TASK_TOGGLE        (3) /**< Task of toggle-task type. */


/**@brief Input pin configuration. */
typedef struct
{
    uint8_t    sense    : DRV_GPIO_SENSE_Width;     /**< The sense type of the pin. */
    uint8_t    pull     : DRV_GPIO_PULL_Width;      /**< The pull resistor configuration of the pin. */
    uint8_t    gpiote   : DRV_GPIO_GPIOTE_Width;    /**< GPIOTE handling enable bit for the pin. */
    uint8_t    handler  : DRV_GPIO_HANDLER_Width;   /**< Interrupt handler enable bit for the pin. */
    uint8_t    rfu0     : sizeof(uint8_t) * 8 - 
    (
        DRV_GPIO_SENSE_Width  + 
        DRV_GPIO_PULL_Width   + 
        DRV_GPIO_GPIOTE_Width + 
        DRV_GPIO_HANDLER_Width
    );
} drv_gpio_inpin_cfg_t;


/**@brief Output pin configuration. */
typedef struct
{
    uint8_t    level    : DRV_GPIO_LEVEL_Width;     /**< The default level of the pin when initiated. */
    uint8_t    drive    : DRV_GPIO_DRIVE_Width;     /**< The drive configuration of the pin. */
    uint8_t    task     : DRV_GPIO_TASK_Width;      /**< The task type assosiated with the pin. */
    uint8_t    gpiote   : DRV_GPIO_GPIOTE_Width;    /**< GPIOTE handling enable bit for the pin. */
    uint8_t    rfu0     : sizeof(uint8_t) * 8 - 
    (
        DRV_GPIO_LEVEL_Width  + 
        DRV_GPIO_DRIVE_Width  + 
        DRV_GPIO_TASK_Width + 
        DRV_GPIO_GPIOTE_Width
    );
} drv_gpio_outpin_cfg_t;


/**@brief GPIO signal handler.
 *
 * @note The sensed edge is only unknown in case GPIOTE is sensing both edges.
 *
 * @param pin           The pin which trigged signal.
 * @param sense_edge    The edge that triggered the signal, or :DRV_GPIO_SENSE_ANY if unknown. */
typedef void (*drv_gpio_sig_handler_t)(uint8_t pin, uint8_t sensed_edge);


/**@brief Sets the signal handler function.
 *
 * @param   sig_handler The specified signal handler. */
void drv_gpio_sig_handler_set(drv_gpio_sig_handler_t sig_handler);


/**@brief Configures the input pin according to the specified pin configuration.
 *
 * @param   pin             The specified pin to configure.
 * @param   cfg             The specified pin configuration.
 * @param   p_event[out]    Points to storage (or NULL) for the address of the hardware event associated with the pin, or NULL if none. 
 *
 * @retval  NRF_ERROR_INVALID_PARAM If the specified pin does not exist.
 * @retval  NRF_ERROR_NOT_FOUND     If no available GPIO instance was found. 
 * @retval  NRF_SUCCESS             If successful. */
uint32_t drv_gpio_inpin_cfg(uint8_t pin, drv_gpio_inpin_cfg_t cfg, uint32_t ** p_event);


/**@brief Configures the input pin according to the specified pin configuration.
 *
 * @param   pin_msk             The mask specifying the pins to configure.
 * @param   cfg                 The specified pin configuration.
 * @param   p_event_arr[out]    Points to storage (or NULL) for the addresses of the hardware event associated with the pins, or NULL if none. 
 *
 * @retval  NRF_ERROR_INVALID_PARAM If the specified pin does not exist.
 * @retval  NRF_ERROR_NOT_FOUND     If no available GPIO instance was found. 
 * @retval  NRF_SUCCESS             If successful. */
uint32_t drv_gpio_inpins_cfg(uint32_t pin_msk, drv_gpio_inpin_cfg_t cfg, uint32_t ** p_event_arr);


/**@brief Configures the output pin according to the specified pin configuration.
 *  
 * @param   pin         The specified pin to configure.
 * @param   cfg         The specified pin configuration.
 * @param   p_task[out] Points to storage (or NULL) for the address of the hardware task associated with the pin, or NULL if none. 
 *
 * @retval  NRF_ERROR_INVALID_PARAM If the specified pin does not exist.
 * @retval  NRF_ERROR_NOT_FOUND     If no available GPIO instance was found. 
 * @retval  NRF_SUCCESS             If successful. */
uint32_t drv_gpio_outpin_cfg(uint8_t pin, drv_gpio_outpin_cfg_t cfg, uint32_t ** p_task);


/**@brief Configures the output pin according to the specified pin configuration.
 *
 * @param   pin_msk         The mask specifying the pins to configure.
 * @param   cfg             The specified pin configuration.
 * @param   p_task_arr[out] Points to storage (or NULL) for the addresses of the hardware task associated with the pins, or NULL if none. 
 *
 * @retval  NRF_ERROR_INVALID_PARAM If the specified pin does not exist.
 * @retval  NRF_ERROR_NOT_FOUND     If no available GPIO instance was found. 
 * @retval  NRF_SUCCESS             If successful. */
uint32_t drv_gpio_outpins_cfg(uint32_t pin_msk, drv_gpio_outpin_cfg_t cfg, uint32_t ** p_task_arr);


/**@brief Disconnects the specified pin.
 *  
 * @param   pin         The specified pin to disconnect.
 *
 * @retval  NRF_ERROR_INVALID_PARAM If the specified pin does not exist.
 * @retval  NRF_SUCCESS             If successful. */
uint32_t drv_gpio_pin_disconnect(uint8_t pin);


/**@brief Disconnects the specified pins.
 *
 * @param   pin_msk         The mask specifying the pins to disconnect.
 * @param   cfg             The specified pin configuration.
 * @param   p_task_arr[out] Points to storage (or NULL) for the addresses of the hardware task associated with the pins, or NULL if none. 
 *
 * @retval  NRF_ERROR_INVALID_PARAM If no pin specified.
 * @retval  NRF_SUCCESS             If successful. */
uint32_t drv_gpio_pins_disconnect(uint32_t pin_msk);


/**@brief Gets the logical level of the specified pin.
 *
 * @param   pin     The specified pin to configure.
 * @param   p_level Pointer to storage where the pin level is to be stored.
 *
 * @retval  NRF_ERROR_INVALID_PARAM If the specified pin does not exist or if the pointer is invalid.
 * @retval  NRF_SUCCESS             If successful. */
uint32_t drv_gpio_inpin_get(uint8_t pin, uint8_t *p_level);


/**@brief Gets the logical level of all pins.
 *
 * @return The logical levels of all pins. */
uint32_t drv_gpio_inport_get(void);


/**@brief Sets the specified logical level of the specified pin.
 *
 * @param   pin     The specified pin to configure.
 * @param   level   The specified level.
 *
 * @retval  NRF_ERROR_INVALID_PARAM If the specified pin or level does not exist.
 * @retval  NRF_SUCCESS             If successful. */
uint32_t drv_gpio_outpin_level_set(uint8_t pin, uint8_t level);


/**@brief Modifies the specified logical levels of the output port.
 *
 * @param   high_msk        The pins to be set to logical high value.
 * @param   low_msk         The pins to be cleared to logical low value.
 *
 * @retval  NRF_ERROR_INVALID_PARAM If the specified masks overlap.
 * @retval  NRF_SUCCESS             If successful. */
uint32_t drv_gpio_outport_modify(uint32_t high_msk, uint32_t low_msk);


/**@brief Toggles the logical levels of the specified output port pins.
 *
 * @param   toggle_msk      The pins to be alter their logical values.
 *
 * @return The logical levels of all pins. */
void drv_gpio_outport_toggle(uint32_t toggle_msk);


/**@brief Sets the specified logical level of the the output port.
 *
 * @param   outport The specified value of the outport. */
void drv_gpio_outport_set(uint32_t outport);


#endif // DRV_GPIO_H__
