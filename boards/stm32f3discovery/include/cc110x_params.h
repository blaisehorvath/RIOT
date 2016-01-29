/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup   board_msba2
 * @{
 *
 * @file
 * @brief     cc110x board specific configuration
 *
 * @author    Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifndef CC110X_PARAMS_H
#define CC110X_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name CC110X configuration
 */
 //SPI1 = SPI 0 :NSS = PA4,  SCK = PA5, MISO = PA6, MOSI = PA7
const cc110x_params_t cc110x_params[] = {
    {
        .spi  = 0,
        .cs   = GPIO_PIN(0,4), 
        .gdo0 = GPIO_PIN(0,0),
        .gdo1 = GPIO_PIN(0,6), 
        .gdo2 = GPIO_PIN(0,2)
    },
};
/** @} */

#ifdef __cplusplus
}
#endif
#endif /* CC110X_PARAMS_H */
/** @} */
