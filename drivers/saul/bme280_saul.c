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
#include "stdio.h"
#include "phydat.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "xtimer.h"

#include "bme280.h"

#include "debug.h"
#define ENABLE_DEBUG (1)

struct bme280_t bme280;
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BME280_delay_msek(u32 msek);


static int read(void *dev, phydat_t *res)
{
	puts("dasfasc");
	printf("faszfasz");
    res->val[0] = 16;
    res->val[1] = 17;
    res->val[2] = 18;
    res->unit = UNIT_BOOL;
    res->scale = 0;
	s32 v_data_uncomp_temp_s32 = 0 ; v_data_uncomp_temp_s32++;
	s32 v_data_uncomp_pres_s32 = 0; v_data_uncomp_pres_s32++;
	s32 v_data_uncomp_hum_s32 = 0 ; v_data_uncomp_hum_s32++;
	bme280.bus_write = BME280_I2C_bus_write;
	bme280.bus_read = BME280_I2C_bus_read;
	bme280.dev_addr = BME280_I2C_ADDRESS2;
	bme280.delay_msec = BME280_delay_msek;
	printf("init beofre");
	bme280_init(&bme280);// ERROR HERE!!!
	printf("init after");
	bme280_read_uncomp_pressure_temperature_humidity(&v_data_uncomp_temp_s32,
				&v_data_uncomp_pres_s32, &v_data_uncomp_hum_s32);
	printf("temp:%ld\npres:%ld\nhum:%ld\n",v_data_uncomp_temp_s32,v_data_uncomp_pres_s32,v_data_uncomp_hum_s32);
	printf("endofread");
	return 1;
}

static int write(void *dev, phydat_t *state)
{
/*	state->val[0] = 1;    */
	return 0;
}

const saul_driver_t i2c_saul_driver = {
    .read = read,
    .write = write,
    .type = SAUL_CLASS_ANY,
};

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	printf("BUS_WRITE!\n");
	char res = 0;res++;
	res = i2c_write_regs((i2c_t) 0, dev_addr, reg_addr, (char*)reg_data, cnt);
	printf("BUS_WRITEEND!\n");
	return 0;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	printf("BUS_READ!\n");
	char res = 0;res++;
	res = i2c_read_regs((i2c_t) 0, dev_addr, reg_addr, (char*)reg_data, cnt);
	printf("BUS_READEND!\n");
	return 0;
}
void BME280_delay_msek(u32 msek)
{
	printf("BEFORE DELAY");
	xtimer_usleep(msek*1000);
	printf("AFTER DELAY");
	return;
}
