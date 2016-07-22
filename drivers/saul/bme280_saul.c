/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/*
 * @ingroup     drivers_saul
 * @{
 *
 * @file
 * @brief       SAUL wrapper for direct access to GPIO pins
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <string.h>

#include "saul.h"
#include "phydat.h"
#include "periph/gpio.h"

#include "bme280.h"

#define ENABLE_DEBUG (1)
#include "debug.h"


static int read(void *dev, phydat_t *res)
{
    /*BOSCH code*/

	s32 results = bme280_data_readout_template();


	/*RIOT code*/
	res->val[0] = 0;
    memset(&(res->val[1]), 0, 2 * sizeof(int16_t));
    res->unit = UNIT_BOOL;
    res->scale = 0;
    DEBUG("READ FROM BME280\n");
    return 1;
}

static int write(void *dev, phydat_t *state)
{
	state->val[0] = 1;
    return 1;
    DEBUG("WRITE TO BME280\n");
}

const saul_driver_t i2c_saul_driver = {
    .read = read,
    .write = write,
    .type = SAUL_ACT_SWITCH,
};
