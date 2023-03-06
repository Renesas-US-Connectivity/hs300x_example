/*
 * hs3001_task.h
 *
 *  Created on: Aug 15, 2022
 *      Author: a5137667
 */

#ifndef HS3001_TASK_H_
#define HS3001_TASK_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <osal.h>
#include "hs300x.h"

void hs300x_task(void *pvParameters);
uint32_t hs300x_task_get_sensor_id();
uint32_t hs300x_task_get_sample_rate();
void hs300x_task_reconfig_gpio_on_wake();
void hs300x_task_set_sample_rate(uint32_t rate);
void hs300x_task_setup_hardware();

#endif /* HS3001_TASK_H_ */
