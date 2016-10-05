#ifdef MODULE_SAUL_BME280

#include <stdint.h>
#include <stdio.h>

#include "periph_conf.h"
#include "periph/spi.h"

#include "saul_reg.h"
#include "saul/periph.h"

#define BUFSIZE        (128U)

/**
 * @brief: Hard coded device variables
 */

#define addr 0x77 //uint 119
#define reg 0xd0 //uint 208
#define length 1 /* read 1 byte*/

static	int spi_dev = SPI_0;
#define	 spi_mode  SPI_CONF_FIRST_RISING
#define	 spi_speed  SPI_SPEED_1MHZ
#define port  0
#define pin  1
#define spi_cs  GPIO_PIN(port,pin)

#define ENABLE_DEBUG (0)
#include "debug.h"

extern saul_driver_t spi_saul_driver;
//static i2c_t i2c_dev = 0;
//static i2c_t i2c_speed = 1;
//static i2c_t bme_280_dev_id = 0; /* i2c device id, equivalent to the value of the i2c_dev variable */
//static char data[BUFSIZE];

static saul_reg_t bme280 = {
	.next = 0,
	.dev = &spi_dev,
	.driver = &spi_saul_driver,
	.name = "BME280"
};

int checkChipID(char* out);

void auto_init_bme280(void) {
	int res;
	/* Initializing the device */
	spi_acquire(spi_dev);
	res = spi_init_master(spi_dev, spi_mode, spi_speed);
	spi_release(spi_dev);
	if (res < 0) {
	printf("spi_init_master: error initializing SPI_%i device (code %i)\n",
			spi_dev, res);
	}
	res = gpio_init(spi_cs, GPIO_DIR_OUT, GPIO_PULLUP);
	if (res < 0) {
	printf("gpio_init: error initializing GPIO_%ld as CS line (code %i)\n",
			(long)spi_cs, res);
	}
	gpio_set(spi_cs);
	printf("SPI_%i successfully initialized as master, cs: GPIO_%ld, mode: %i, speed: %i\n", spi_dev, (long)spi_cs, spi_mode, spi_speed);

	/*Reading the chip id*/
	char in, out;
	char nullas = 0;

	int id = 0xd0;
	in = id | 1<<7;

	gpio_clear(spi_cs);
	res = spi_transfer_bytes(spi_dev, &in, &out, 1);
	res = spi_transfer_bytes(spi_dev, &nullas, &out, 1);
	gpio_set(spi_cs);

	res = checkChipID(&out);

	// if the chipID is correct
	if (res == 1) {
		printf("BME280 detected!\n");

		/* adding the device to the SAUL*/
		saul_reg_add(&(bme280));
	}
	else {
		printf("No BME280 found on SPI \n");
	}
}

/**
 * @brief: This function checks if the device's chip id the one thats in the datasheet
 */
int checkChipID (char* out) {
	if ((int)*out == 0x60) {
		return 1;
	}
	else {
		return 0;
	}
}


#endif
