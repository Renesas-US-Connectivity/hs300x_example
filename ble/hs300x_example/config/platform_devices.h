/*
 * platform_devices.h
 *
 *  Created on: Sep 8, 2022
 *      Author: a5137667
 */

#ifndef CONFIG_PLATFORM_DEVICES_H_
#define CONFIG_PLATFORM_DEVICES_H_

#include <ad_i2c.h>

typedef const ad_i2c_controller_conf_t* i2c_device;

/* I2X Interface */
#define I2C_GPIO_LEVEL HW_GPIO_POWER_V33

/* I2C */
#define I2C_PORT    HW_GPIO_PORT_0
#define I2C_MASTER_SCL_PIN  HW_GPIO_PIN_30
#define I2C_MASTER_SDA_PIN  HW_GPIO_PIN_31

#define I2C_SLAVE_ADDRESS    (0x44)

#define I2C_CTRLR_INSTANCE       (HW_I2C1)

#define HS300x_POWER_GPIO_PORT HW_GPIO_PORT_0
#define HS300x_POWER_GPIO_PIN  HW_GPIO_PIN_18

extern i2c_device hs300x_i2c_config;
extern gpio_config *hs300x_power_gpio;

#endif /* CONFIG_PLATFORM_DEVICES_H_ */
