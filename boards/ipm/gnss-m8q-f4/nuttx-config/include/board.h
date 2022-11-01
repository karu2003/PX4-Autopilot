/************************************************************************************
 * include/arch/board/board.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *   Author: Nathan Tsoi <nathan@vertile.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

#include "board_dma_map.h"

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The Adafruit Feather STM32F405 Express board features a single 12MHz crystal.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 12000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 6            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 138          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 12MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_USEHSE      1
#define STM32_BOARD_XTAL        12000000ul
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_LSE_FREQUENCY     32768

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(6)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(168)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8-11 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_APB2_TIM1_CLKIN
#define BOARD_TIM2_FREQUENCY    STM32_APB1_TIM2_CLKIN
#define BOARD_TIM3_FREQUENCY    STM32_APB1_TIM3_CLKIN
#define BOARD_TIM4_FREQUENCY    STM32_APB1_TIM4_CLKIN
#define BOARD_TIM5_FREQUENCY    STM32_APB1_TIM5_CLKIN
#define BOARD_TIM6_FREQUENCY    STM32_APB1_TIM6_CLKIN
#define BOARD_TIM7_FREQUENCY    STM32_APB1_TIM7_CLKIN
#define BOARD_TIM8_FREQUENCY    STM32_APB2_TIM8_CLKIN
#define BOARD_TIM9_FREQUENCY    STM32_APB2_TIM9_CLKIN
#define BOARD_TIM10_FREQUENCY   STM32_APB2_TIM10_CLKIN
#define BOARD_TIM11_FREQUENCY   STM32_APB2_TIM11_CLKIN
#define BOARD_TIM12_FREQUENCY   STM32_APB1_TIM12_CLKIN
#define BOARD_TIM13_FREQUENCY   STM32_APB1_TIM13_CLKIN
#define BOARD_TIM14_FREQUENCY   STM32_APB1_TIM14_CLKIN

#define SDIO_INIT_CLKDIV        (118 << SDIO_CLKCR_CLKDIV_SHIFT)

/* UART3 */
#define GPIO_USART3_RX GPIO_USART3_RX_1
#define GPIO_USART3_TX GPIO_USART3_TX_1

/* UART6 */
#define GPIO_USART6_RX GPIO_USART6_RX_1
#define GPIO_USART6_TX GPIO_USART6_TX_1

/* CAN */
#define GPIO_CAN1_TX    /* PB9 */ GPIO_CAN1_TX_2
#define GPIO_CAN1_RX    /* PB8 */ GPIO_CAN1_RX_2

/* I2C */
#define GPIO_I2C1_SCL   /* PB6  */ GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA   /* PB7  */ GPIO_I2C1_SDA_1

/* SPI */
#define ADJ_SLEW_RATE(p) (((p) & ~GPIO_SPEED_MASK) | (GPIO_SPEED_2MHz))

#define GPIO_SPI1_MISO /* PA6 */ GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI /* PB5 */ GPIO_SPI1_MOSI_2
#define GPIO_SPI1_SCK  /* PA5 */ ADJ_SLEW_RATE(GPIO_SPI1_SCK_1)
//#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1    /* PA5 */

/* SPI2:
 *  SD Card
 *  CS: PB12 -- configured in board_config.h
 *  CLK: PB13
 *  MISO: PB14
 *  MOSI: PB15
 */

// #define GPIO_SPI2_SCK	  GPIO_SPI2_SCK_2
// #define GPIO_SPI2_MISO	GPIO_SPI2_MISO_1
// #define GPIO_SPI2_MOSI	GPIO_SPI2_MOSI_1

/* SPI3: SD CARD
 *  CS: PC14 -- configured in board_config.h
 *  CLK: PB3
 *  MISO: PB4
 *  MOSI: PB5
 */

#endif  /* __ARCH_BOARD_BOARD_H */
