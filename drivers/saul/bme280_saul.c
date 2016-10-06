#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"
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
#include "periph/spi.h"
#include "xtimer.h"

#include "bme280.h"

#include "debug.h"
#define ENABLE_DEBUG (1)
#include "inttypes.h"

#define spi_dev 0
#define spi_cs GPIO_PIN(0,1)

#define SPI_BUFFER_LEN 5
#define BME280_ADDRESS_INDEX	2
#define BME280_DATA_INDEX	1
#define SPI_WRITE	0x7F
#define SPI_READ	0x80

char bme280_spi_write_byte (char masked_address, char* value);
char bme280_spi_byte_transwer (char address);
struct bme280_t bme280;
s8 BME280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BME280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BME280_delay_msek(u32 msek);


static int read(void *dev, phydat_t *res)
{
	printf("chipid %i", (int)bme280_spi_byte_transwer(0xe0|SPI_READ));
    res->val[0] = 16;
    res->val[1] = 17;
    res->val[2] = 18;
    res->unit = UNIT_NONE;
    res->scale = 0;
	s32 v_data_uncomp_temp_s32 = 0 ; //v_data_uncomp_temp_s32++;
	s32 v_data_uncomp_pres_s32 = 0; //v_data_uncomp_pres_s32++;
	s32 v_data_uncomp_hum_s32 = 0 ; //v_data_uncomp_hum_s32++;
	s32 comp_temp_s32 = 0;
	s32 comp_pres_s32 = 0;
	s32 comp_humi_s32 = 0;
	bme280.bus_write = BME280_SPI_bus_write;
	bme280.bus_read = BME280_SPI_bus_read;
	bme280.dev_addr = BME280_I2C_ADDRESS2;
	bme280.delay_msec = BME280_delay_msek;
	s32 com_rslt = bme280_init(&bme280);// ERROR HERE!!!
	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_4X);
	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_4X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_4X);
	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	bme280_read_uncomp_pressure_temperature_humidity(&v_data_uncomp_pres_s32, &v_data_uncomp_temp_s32, &v_data_uncomp_hum_s32);
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
	return 0;
}

const saul_driver_t spi_saul_driver = {
    .read = read,
    .write = write,
    .type = SAUL_CLASS_ANY,
};

/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BME280_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError=BME280_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN]={0,};
	/*	For the SPI mode only 7 bits of register addresses are used.
	The MSB of register address is declared the bit what functionality it is
	read/write (read as 1/write as BME280_INIT_VALUE)*/
	array[BME280_INIT_VALUE] = reg_addr|SPI_READ;/*read routine is initiated register address is mask with 0x80*/


	/*
	* Please take the below function as your reference for
	* read the data using SPI communication
	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+1)"
	* add your SPI read function here
	* iError is an return value of SPI read function
	* Please select your valid return value
	* In the driver SUCCESS defined as 0
	* and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done in the SPI read
	* and write string function
	* For more information please refer data sheet SPI communication:
	*/
	char starting_reg_addr = reg_addr|SPI_READ;
	char kuka = bme280_spi_byte_transwer(starting_reg_addr);
	for (int i = BME280_INIT_VALUE; i < cnt; i++) {
		*(reg_data + i) = bme280_spi_byte_transwer(starting_reg_addr+i);
	}
	return (s8)0;
}

/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, where data is to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BME280_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	u8 array[SPI_BUFFER_LEN * BME280_ADDRESS_INDEX];
	u8 stringpos = BME280_INIT_VALUE;
	u8 index = BME280_INIT_VALUE;
//	for (stringpos = BME280_INIT_VALUE; stringpos < cnt; stringpos++) {
//		/* the operation of (reg_addr++)&0x7F done as per the
//		SPI communication protocol specified in the data sheet*/
//		index = stringpos * BME280_ADDRESS_INDEX;
//		array[index] = (reg_addr++) & SPI_WRITE;
//		array[index + BME280_DATA_INDEX] = *(reg_data + sauringpos);
//	}

	char starting_write_addr = reg_addr;
	for (int i = BME280_INIT_VALUE; i < cnt; i++) {
		bme280_spi_write_byte(starting_write_addr+i, (char*)(reg_data + i));
	}

	/* Please take the below function as your reference
	 * for write the data using SPI communication
	 * add your SPI write function here.
	 * "IERROR = SPI_WRITE_STRING(ARRAY, CNT*2)"
	 * iError is an return value of SPI write function
	 * Please select your valid return value
	 * In the driver SUCCESS defined as 0
	 * and FAILURE defined as -1
	 */
	return (s8)0;
}
void BME280_delay_msek(u32 msek)
{
	xtimer_usleep(msek*1000);
	return;
}

char bme280_spi_byte_transwer (char masked_address) {
	int res;
	char in, out;
	char nullas = 0;

	//in = address | 1<<7;
	masked_address |= (1<<7);
	gpio_clear(spi_cs);
	res = spi_transfer_bytes(spi_dev, &masked_address, &out, 1);
	res = spi_transfer_bytes(spi_dev, &nullas, &out, 1);
	printf("reg: %i, value: %02X\n ",masked_address,out);
	gpio_set(spi_cs);

	return out;
}

char bme280_spi_write_byte (char masked_address, char* value) {
	int res;
	char in, out;
	masked_address &=0x7F;
	gpio_clear(spi_cs);
	res = spi_transfer_bytes(spi_dev, &masked_address, &out, 1);
	res = spi_transfer_bytes(spi_dev, value, &out, 1);
	gpio_set(spi_cs);
	printf("addr:%i, writeval:%02X, readval:%02X\n",masked_address,*value, bme280_spi_byte_transwer(masked_address));

	return out;
}
