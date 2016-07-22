#ifdef MODULE_SAUL_BME280

#include <stdint.h>
#include <stdio.h>

#include "periph_conf.h"
#include "periph/i2c.h"

#include "saul_reg.h"
#include "saul/periph.h"

#define BUFSIZE        (128U)

/**
 * @brief: Hard coded device variables
 */
#define i2c_dev 0
#define i2c_speed 1
#define addr 0x77 //uint 119
#define reg 0xd0 //uint 208
#define length 1 /* read 1 byte*/

#define ENABLE_DEBUG (1)
#include "debug.h"

extern saul_driver_t i2c_saul_driver;

static i2c_t bme_280_dev_id = 0; /* i2c device id, equivalent to the value of the i2c_dev variable */

static saul_reg_t bme280 = {
	.next = 0,
	.dev = &bme_280_dev_id,
	.driver = &i2c_saul_driver,
	.name = "BME280"
};

void auto_init_bme280(void){

	/* Initializing the device */
    DEBUG("auto init bme280 SAUL\n");

    int res = i2c_init_master(i2c_dev, i2c_speed);

    if (res == -1) {
        puts("Error: Init: Given device not available");
    }
    else if (res == -2) {
        puts("Error: Init: Unsupported speed value");
    }
    else {

    	/* Checking the device id if it's successfully initialized */

        printf("I2C_%i successfully initialized as master!\n", i2c_dev);
        char data[BUFSIZE];
        res = i2c_read_regs(i2c_dev, (uint8_t)addr, (uint8_t)reg, data, length);

        int n = sizeof(data) / sizeof(int);
        unsigned int device_id = (unsigned int)data[0];

        if (n == 32 && device_id == (unsigned int)96) {
    		DEBUG("The ID of the BME280 device is: 0x%02x \n", device_id);

    		/* adding the device to the SAUL*/

    		saul_reg_add(&(bme280));

        } else {
        	printf("ERROR: BME280 device cannot be accessed!\n");
        }
    }
}

#endif
