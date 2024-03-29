/*
 * sensor_service.h
 *
 *  Created on: Mar 2, 2023
 *      Author: a5137667
 */

#ifndef SENSOR_SERVICE_H_
#define SENSOR_SERVICE_H_

#include <stdint.h>
#include <ble_service.h>
#include "hs300x.h"

/* User-defined callback functions prototypes */
typedef void (* sensor_svc_get_sample_rate_cb_t) (ble_service_t *svc, uint16_t conn_idx);
typedef void (* sensor_svc_get_sensor_id_cb_t) (ble_service_t *svc, uint16_t conn_idx);
typedef void (* sensor_svc_set_sample_rate_cb_t) (ble_service_t *svc, uint16_t conn_idx, const uint32_t value);

/* User-defined callback function structure */
typedef struct {

        // Read request handler for the sensor sample rate
		sensor_svc_get_sample_rate_cb_t get_sample_rate_cb;

		// Read request handler for the sensor id
		sensor_svc_get_sensor_id_cb_t get_sensor_id_cb;

        // Write request handler for sensor sample rate
        sensor_svc_set_sample_rate_cb_t set_sample_rate_cb;

} sensor_service_cb_t;

ble_service_t *sensor_service_init(const sensor_service_cb_t *cb);
void sensor_service_get_sample_rate_cfm(ble_service_t *svc, uint16_t conn_idx, att_error_t status, const uint32_t *value);
void sensor_service_get_sensor_id_cfm(ble_service_t *svc, uint16_t conn_idx, att_error_t status, const uint32_t *value);
void sensor_service_notify_measurement(ble_service_t *svc, uint16_t conn_idx, const hs300x_data_t *value);
void sensor_service_notify_measurement_to_all_connected(ble_service_t *svc, const hs300x_data_t *value);
void sensor_service_set_sample_rate_cfm(ble_service_t *svc, uint16_t conn_idx, att_error_t status);

#endif /* SENSOR_SERVICE_H_ */
