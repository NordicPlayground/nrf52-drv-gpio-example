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


#ifdef DRV_GPIO_H__


#if DRV_GPIO_SENSE_NONE != GPIOTE_CONFIG_POLARITY_None
#error "ERROR: DRV_GPIO_SENSE_NONE != GPIOTE_CONFIG_POLARITY_None."
#endif
#if DRV_GPIO_SENSE_LOTOHI != GPIOTE_CONFIG_POLARITY_LoToHi
#error "ERROR: DRV_GPIO_SENSE_LOTOHI != GPIOTE_CONFIG_POLARITY_LoToHi."
#endif
#if DRV_GPIO_SENSE_HITOLO != GPIOTE_CONFIG_POLARITY_HiToLo
#error "ERROR: DRV_GPIO_SENSE_HITOLO != GPIOTE_CONFIG_POLARITY_HiToLo."
#endif
#if DRV_GPIO_SENSE_ANY != GPIOTE_CONFIG_POLARITY_Toggle
#error "ERROR: DRV_GPIO_SENSE_ANY != GPIOTE_CONFIG_POLARITY_Toggle."
#endif


#if DRV_GPIO_PULL_NONE != GPIO_PIN_CNF_PULL_Disabled
#error "ERROR: DRV_GPIO_PULL_NONE != GPIO_PIN_CNF_PULL_Disabled."
#endif
#if DRV_GPIO_PULL_UP != GPIO_PIN_CNF_PULL_Pullup
#error "ERROR: DRV_GPIO_PULL_UP != GPIO_PIN_CNF_PULL_Pullup."
#endif
#if DRV_GPIO_PULL_DOWN != GPIO_PIN_CNF_PULL_Pulldown
#error "ERROR: DRV_GPIO_PULL_DOWN != GPIO_PIN_CNF_PULL_Pulldown."
#endif


#if DRV_GPIO_LEVEL_LOW != GPIOTE_CONFIG_OUTINIT_Low
#error "ERROR: DRV_GPIO_LEVEL_LOW != GPIOTE_CONFIG_OUTINIT_Low."
#endif
#if DRV_GPIO_LEVEL_HIGH != GPIOTE_CONFIG_OUTINIT_High
#error "ERROR: DRV_GPIO_LEVEL_HIGH != GPIOTE_CONFIG_OUTINIT_High."
#endif


#if DRV_GPIO_DRIVE_S0S1 != GPIO_PIN_CNF_DRIVE_S0S1
#error "ERROR: DRV_GPIO_DRIVE_S0S1 != GPIO_PIN_CNF_DRIVE_S0S1."
#endif
#if DRV_GPIO_DRIVE_H0S1 != GPIO_PIN_CNF_DRIVE_H0S1
#error "ERROR: DRV_GPIO_DRIVE_H0S1 != GPIO_PIN_CNF_DRIVE_H0S1."
#endif
#if DRV_GPIO_DRIVE_S0H1 != GPIO_PIN_CNF_DRIVE_S0H1
#error "ERROR: DRV_GPIO_DRIVE_S0H1 != GPIO_PIN_CNF_DRIVE_S0H1."
#endif
#if DRV_GPIO_DRIVE_H0H1 != GPIO_PIN_CNF_DRIVE_H0H1
#error "ERROR: DRV_GPIO_DRIVE_H0H1 != GPIO_PIN_CNF_DRIVE_H0H1."
#endif
#if DRV_GPIO_DRIVE_D0S1 != GPIO_PIN_CNF_DRIVE_D0S1
#error "ERROR: DRV_GPIO_DRIVE_D0S1 != GPIO_PIN_CNF_DRIVE_D0S1."
#endif
#if DRV_GPIO_DRIVE_D0H1 != GPIO_PIN_CNF_DRIVE_D0H1
#error "ERROR: DRV_GPIO_DRIVE_D0H1 != GPIO_PIN_CNF_DRIVE_D0H1."
#endif
#if DRV_GPIO_DRIVE_S0D1 != GPIO_PIN_CNF_DRIVE_S0D1
#error "ERROR: DRV_GPIO_DRIVE_S0D1 != GPIO_PIN_CNF_DRIVE_S0D1."
#endif
#if DRV_GPIO_DRIVE_H0D1 != GPIO_PIN_CNF_DRIVE_H0D1
#error "ERROR: DRV_GPIO_DRIVE_H0D1 != GPIO_PIN_CNF_DRIVE_H0D1."
#endif

#else

#error "ERROR: No DRV_GPIO definitions to check because drv_gpio.h was not included prior to this check."

#endif // DRV_GPIO_H__
