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
#include <stdint.h>

#include "saul.h"
#include "stdio.h"
#include "phydat.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "xtimer.h"

#include "bme280.h"

#include "debug.h"
#define ENABLE_DEBUG (1)
#include "inttypes.h"

struct bme280_t bme280;
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BME280_delay_msek(u32 msek);


static int read(void *dev, phydat_t *res)
{
    res->val[0] = 16;
    res->val[1] = 17;
    res->val[2] = 18;
    res->unit = UNIT_NONE;
    res->scale = 0;
	s32 v_data_uncomp_temp_s32 = 0 ; v_data_uncomp_temp_s32++;
	s32 v_data_uncomp_pres_s32 = 0; v_data_uncomp_pres_s32++;
	s32 v_data_uncomp_hum_s32 = 0 ; v_data_uncomp_hum_s32++;
	s32 comp_temp_s32 = 0;
	s32 comp_pres_s32 = 0;
	s32 comp_humi_s32 = 0;
	bme280.bus_write = BME280_I2C_bus_write;
	bme280.bus_read = BME280_I2C_bus_read;
	bme280.dev_addr = BME280_I2C_ADDRESS2;
	bme280.delay_msec = BME280_delay_msek;
	s32 com_rslt = bme280_init(&bme280);// ERROR HERE!!!
	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_4X);
	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_4X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_4X);
	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	bme280_read_uncomp_pressure_temperature_humidity(
				&v_data_uncomp_pres_s32, &v_data_uncomp_temp_s32, &v_data_uncomp_hum_s32);
	comp_temp_s32 = (s32)1000*(((double)bme280_compensate_temperature_int32(v_data_uncomp_temp_s32))/500 +24);
	res->val[0] = (int16_t) comp_temp_s32;
	res->scale = 3;
	res->unit = 0;
	comp_pres_s32 = bme280_compensate_pressure_int32(v_data_uncomp_pres_s32);
	res->val[1] = (int16_t) (comp_pres_s32/1000);
	comp_humi_s32 = (s32) ((double)bme280_compensate_humidity_int32(v_data_uncomp_hum_s32))/1024;
	res->val[2] = (int16_t) comp_humi_s32;
	printf("temp:%d\npres:%d\nhum:%d\n",res->val[0],res->val[1],res->val[2]);
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
	char res = 0;res++;
	res = i2c_write_regs((i2c_t) 0, dev_addr, reg_addr, (char*)reg_data, cnt);
	return 0;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	char res = 0;res++;
	if (cnt == 1) {
		res = i2c_read_regs((i2c_t) 0, dev_addr, reg_addr, (char*)reg_data, cnt);
	} else {
		int i;
		for (i = 0; i < cnt; ++i) {
			res = i2c_read_reg((i2c_t)0, dev_addr, reg_addr+i, (char*)(&reg_data[i]));
		}
	}
	return 0;
}
void BME280_delay_msek(u32 msek)
{
	xtimer_usleep(msek*1000);
	return;
}
