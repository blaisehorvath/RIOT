/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_saml21-xpro Atmel SAM L21 Xplained Pro
 * @ingroup     boards
 * @brief       Support for the Atmel SAM L21 Xplained Pro board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the Atmel SAM L21 Xplained Pro board.
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name LED pin definitions
 * @{
 */
#define LED_PORT            PORT->Group[1]
#define LED_PIN             (10)
/** @} */

/**
 * @name Macros for controlling the on-board LEDs.
 * @{
 */
#define LED_ON              (LED_PORT.OUTCLR.reg = 1<<LED_PIN)
#define LED_OFF             (LED_PORT.OUTSET.reg = 1<<LED_PIN)
#define LED_TOGGLE          (LED_PORT.OUTTGL.reg = 1<<LED_PIN)

/* for compatability to other boards */
#define LED_GREEN_ON        /* not available */
#define LED_GREEN_OFF       /* not available */
#define LED_GREEN_TOGGLE    /* not available */
#define LED_ORANGE_ON       LED_ON
#define LED_ORANGE_OFF      LED_OFF
#define LED_ORANGE_TOGGLE   LED_TOGGLE
#define LED_RED_ON          /* not available */
#define LED_RED_OFF         /* not available */
#define LED_RED_TOGGLE      /* not available */
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /** BOARD_H */
/** @} */
