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

#include "debug.h"
#define ENABLE_DEBUG (1)
#include "inttypes.h"

#define spi_cs GPIO_PIN(1,12)
static	int spi_dev = SPI_1;

#define BME280_REGISTER_DIG_T1               0x88
#define BME280_REGISTER_DIG_T2               0x8A
#define BME280_REGISTER_DIG_T3               0x8C

#define BME280_REGISTER_DIG_P1               0x8E
#define BME280_REGISTER_DIG_P2               0x90
#define BME280_REGISTER_DIG_P3               0x92
#define BME280_REGISTER_DIG_P4               0x94
#define BME280_REGISTER_DIG_P5               0x96
#define BME280_REGISTER_DIG_P6               0x98
#define BME280_REGISTER_DIG_P7               0x9A
#define BME280_REGISTER_DIG_P8               0x9C
#define BME280_REGISTER_DIG_P9               0x9E

#define BME280_REGISTER_DIG_H1               0xA1
#define BME280_REGISTER_DIG_H2               0xE1
#define BME280_REGISTER_DIG_H3               0xE3
#define BME280_REGISTER_DIG_H4               0xE4
#define BME280_REGISTER_DIG_H5               0xE5
#define BME280_REGISTER_DIG_H6               0xE7

#define BME280_REGISTER_CHIPID              0xD0
#define BME280_REGISTER_VERSION             0xD1
#define BME280_REGISTER_SOFTRESET           0xE0

#define BME280_REGISTER_CAL26               0xE1  // R calibration stored in 0xE1-0xF0

#define BME280_REGISTER_CONTROLHUMID        0xF2
#define BME280_REGISTER_CONTROL             0xF4
#define BME280_REGISTER_CONFIG              0xF5
#define BME280_REGISTER_PRESSUREDATA        0xF7
#define BME280_REGISTER_TEMPDATA            0xFA
#define BME280_REGISTER_HUMIDDATA           0xFD

typedef struct
{
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;

	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
} bme280_calib_data;

void readCoefficients(void);
uint8_t spixfer(uint8_t x);

void      write8(uint8_t reg, uint8_t value);
uint8_t   read8(uint8_t reg);
uint8_t read84real(uint8_t reg);
uint16_t  read16(uint8_t reg);
uint32_t  read24(uint8_t reg);
uint16_t   readS16(uint8_t reg);
uint16_t  read16_LE(uint8_t reg); // little endian
int16_t   readS16_LE(uint8_t reg); // little endian

uint8_t   _i2caddr;
int32_t   _sensorID;
int32_t t_fine;

int8_t _cs, _mosi, _miso, _sck;
int8_t  begin(uint8_t addr);
float readTemperature(void);
float readPressure(void);
float readHumidity(void);
float readAltitude(float seaLevel);
float seaLevelForAltitude(float altitude, float atmospheric);
bme280_calib_data _bme280_calib;


static int write(void *dev, phydat_t *state)
{
	return 0;
}

static int read(void *dev, phydat_t *res)
{
	if (read8(BME280_REGISTER_CHIPID) != 0x60)return false;

	readCoefficients();

	//Set before CONTROL_meas (DS 5.4.3)
	write8(BME280_REGISTER_CONTROLHUMID, 0x05); //16x oversampling

	write8(BME280_REGISTER_CONTROL, 0xB7); // 16x ovesampling, normal mode

	/* Filling in the values to the response struct */
	res->val[0] = (int)(readTemperature() * 100); // the temperature in centiCelsius
	res->val[1] = (int)(readPressure()/10); // dkPa
	res->val[2] = (int)(readHumidity()*100); // 1/10000
	res->unit = UNIT_UNDEF;
	res->scale = 1;


	printf("\nTEMP:%i, PRESS:%i, HUM:%i\n",(int)readTemperature(),(int)readPressure(),(int)readHumidity());
	return 1;
}
const saul_driver_t spi_saul_driver = {
    .read = read,
    .write = write,
    .type = SAUL_CLASS_ANY,
};

uint8_t read8(uint8_t reg)
{
	reg |=	0x80;
	uint8_t value,nullas=0;
	gpio_clear(spi_cs);
	spi_transfer_bytes(spi_dev, (char*)&reg, (char*)&value, 1);
	spi_transfer_bytes(spi_dev, (char*)&nullas, (char*)&value, 1);
	gpio_set(spi_cs);
	DEBUG("read8 reg");
	DEBUG("%i",reg);
	DEBUG(", value:");
	DEBUG("%02X\n",value);
	return value;
}

uint8_t read84real(uint8_t reg)
{
	reg |=	0x80;
	uint8_t value,nullas=0;
	gpio_clear(spi_cs);
	spi_transfer_bytes(spi_dev, (char*)&reg, (char*)&value, 1);
	spi_transfer_bytes(spi_dev, (char*)&nullas, (char*)&value, 1);
	gpio_set(spi_cs);
	return value;
}

void readCoefficients(void)
{
    _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
    _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
    _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

    _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
    _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
    _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
    _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
    _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
    _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
    _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
    _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
    _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

    _bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
    _bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
    _bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
    _bme280_calib.dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF);
    _bme280_calib.dig_H5 = (read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
    _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

uint16_t read16_LE(uint8_t reg) {

	uint16_t temp = read16(reg);
	uint16_t retval;
	DEBUG("read16_LE reg:");
	DEBUG("%i",reg);
	DEBUG(", value:");
	retval = (temp >> 8) | (temp << 8);
	DEBUG("%04X\n",retval);
	return retval;
}
uint16_t read16(uint8_t reg)
{
	uint16_t value;
	uint8_t* temp;
	temp = (uint8_t*)&value;
	*temp=read84real(reg+1);
	temp++;
	*temp=read84real(reg);
	DEBUG("read16 reg:");
	DEBUG("%i",reg);
	DEBUG(", value:");
	DEBUG("%04X\n",value);
	return value;
}

int16_t readS16_LE(uint8_t reg)
{
	/*uint16_t temp = read16_LE(reg);
	DEBUG("readS16_LE reg:");
	DEBUG("%i",reg);
	DEBUG(", value:");
	DEBUG("%04X\n",temp);*/
	return (int16_t)read16_LE(reg);
}

void write8(uint8_t reg, uint8_t value)
{
	uint8_t nullas;
	reg &=0x7F;
	gpio_clear(spi_cs);
	spi_transfer_bytes(spi_dev, (char*)&reg, (char*)&nullas, 1);
	spi_transfer_bytes(spi_dev, (char*)&value, (char*)&nullas, 1);
	gpio_set(spi_cs);
    DEBUG("write8 reg:");
    DEBUG("%i",reg);
	DEBUG(", value:");
	DEBUG("%i",value);
	DEBUG(", read:");
	DEBUG("%02X\n",read8(reg));
  }

float readTemperature(void){
	  int32_t var1, var2;
	  float T;
	  int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
	  adc_T >>= 4;

	  var1  = ((((adc_T>>3) - ((int32_t)_bme280_calib.dig_T1 <<1))) *
		   ((int32_t)_bme280_calib.dig_T2)) >> 11;

	  var2  = (((((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1)) *
		     ((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
		   ((int32_t)_bme280_calib.dig_T3)) >> 14;

	  t_fine = var1 + var2;

	  T  = (t_fine * 5 + 128) >> 8;
	  return T/100;
}

uint32_t read24(uint8_t reg)
{
	uint32_t value;
	//temp = (uint8_t*)&value;
	value=read84real(reg);
	value <<=8;
	value|=read84real(reg+1);
	value <<=8;
	value|=read84real(reg+2);

	DEBUG("read24 reg:");
	DEBUG("%i",reg);
	DEBUG(", value:");
	DEBUG("%li\n",value);
	return value;
}

float readPressure(void) {
  int64_t var1, var2, p;

  readTemperature(); // must be done first to get t_fine

  int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bme280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bme280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bme280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bme280_calib.dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bme280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7)<<4);
  return (float)p/256;
}

float readHumidity(void) {

  readTemperature(); // must be done first to get t_fine

  int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);

  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));

  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
		  (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
	       (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
		    (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
		  ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
			     ((int32_t)_bme280_calib.dig_H1)) >> 4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  float h = (v_x1_u32r>>12);
  return  h / 1024.0;
}
