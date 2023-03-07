/*
 * sensor_service.c
 *
 *  Created on: Mar 2, 2023
 *      Author: a5137667
 */


#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "osal.h"
#include "ble_att.h"
#include "ble_bufops.h"
#include "ble_common.h"
#include "ble_gatt.h"
#include "ble_gatts.h"
#include "ble_storage.h"
#include "ble_uuid.h"
#include "sensor_service.h"

/* Service related variables */
typedef struct {
	ble_service_t svc;

	// User-defined callback functions
	const sensor_service_cb_t *cb;

	// Attribute handles of BLE service
	uint16_t sensor_id_value_h;
	uint16_t sensor_id_user_desc_h;

	uint16_t sample_rate_value_h;
	uint16_t sample_rate_user_desc_h;

	uint16_t measurement_value_h;
	uint16_t measurement_value_user_desc_h;
	uint16_t measurement_value_ccc_h;

} sensor_service_t;

static void cleanup(ble_service_t *svc);
static void handle_measurement_value_ccc_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt);
static att_error_t handle_measurement_value_ccc_write(sensor_service_t *sample_service_handle,
													  uint16_t conn_idx,
													  uint16_t offset,
													  uint16_t length,
													  const uint8_t *value);

static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt);
static att_error_t handle_sample_rate_write(sensor_service_t *sensor_service_handle,
												  uint16_t conn_idx,
												  uint16_t offset,
												  uint16_t length,
												  const uint8_t *value);

static void handle_sample_rate_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt);
static void handle_sensor_id_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt);
static void handle_write_req(ble_service_t *svc, const ble_evt_gatts_write_req_t *evt);

static const char sensor_id_char_user_description[]  = "Sensor ID";
static const char sample_rate_char_user_description[]  = "Sample Rate";
static const char measurement_value_char_user_description[]  = "Measurement Value";

/* Function to be called after a cleanup event */
static void cleanup(ble_service_t *svc)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	ble_storage_remove_all(sensor_service_handle->measurement_value_ccc_h);

	OS_FREE(sensor_service_handle);
}

/* This function is called upon read requests to characteristic attribue value */
static void handle_measurement_value_ccc_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt)
{
	uint16_t ccc = 0x0000;

	/* Extract the CCC value from the ble storage */
	ble_storage_get_u16(evt->conn_idx, sensor_service_handle->measurement_value_ccc_h, &ccc);

	// We're little-endian - OK to write directly from uint16_t
	ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(ccc), &ccc);
}


/* This function is called upon write requests to CCC attribute value */
static att_error_t handle_measurement_value_ccc_write(sensor_service_t *sample_service_handle,
													  uint16_t conn_idx,
													  uint16_t offset,
													  uint16_t length,
													  const uint8_t *value)
{
	att_error_t error = ATT_ERROR_OK;

	if(offset)
	{
		error = ATT_ERROR_ATTRIBUTE_NOT_LONG;
	}
	else if(length != sizeof(uint16_t))
	{
		error = ATT_ERROR_INVALID_VALUE_LENGTH;
	}
	else
	{
		uint16_t ccc = get_u16(value);

		/* Store the envoy CCC value to the ble storage */
		ble_storage_put_u32(conn_idx, sample_service_handle->measurement_value_ccc_h, ccc, true);

		ble_gatts_write_cfm(conn_idx, sample_service_handle->measurement_value_ccc_h, error);

	}

	return error;
}

/* Handler for read requests, that is BLE_EVT_GATTS_READ_REQ */
static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	/*
	 * Identify for which attribute handle the read request has been sent to
	 * and call the appropriate function.
	 */

	if(evt->handle == sensor_service_handle->sensor_id_value_h)
	{
		handle_sensor_id_read(sensor_service_handle, evt);
	}
	else if(evt->handle == sensor_service_handle->sample_rate_value_h)
	{
		handle_sample_rate_read(sensor_service_handle, evt);
	}
	else if(evt->handle == sensor_service_handle->measurement_value_ccc_h )
	{
		handle_measurement_value_ccc_read(sensor_service_handle, evt);
	}
	/* Otherwise read operations are not permitted */
	else
	{
		ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_READ_NOT_PERMITTED, 0, NULL);
	}

}

/* This function is called upon read requests to characteristic attribue value */
static void handle_sample_rate_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt)
{
	/*
	 * Check whether the application has defined a callback function
	 * for handling the event.
	 */
	if(!sensor_service_handle->cb || !sensor_service_handle->cb->get_sample_rate_cb)
	{
		ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_READ_NOT_PERMITTED, 0, NULL);
	}
	else
	{
		/*
		 * The application should provide the requested data to the peer device.
		 */
		sensor_service_handle->cb->get_sample_rate_cb(&sensor_service_handle->svc, evt->conn_idx);
	}
}

/* This function is called upon write requests to characteristic attribute value */
static att_error_t handle_sample_rate_write(sensor_service_t *sensor_service_handle,
												  uint16_t conn_idx,
												  uint16_t offset,
												  uint16_t length,
												  const uint8_t *value)
{
	att_error_t error = ATT_ERROR_OK;

	if(offset)
	{
		error = ATT_ERROR_ATTRIBUTE_NOT_LONG;
	}

	/* Check if the length of the envoy data exceed the maximum permitted */
	else if(length != sizeof(uint32_t))
	{
		error = ATT_ERROR_INVALID_VALUE_LENGTH;
	}

	/*
	 * Check whether the application has defined a callback function
	 * for handling the event.
	 */
	else if(!sensor_service_handle->cb || !sensor_service_handle->cb->set_sample_rate_cb) {
		error = ATT_ERROR_WRITE_NOT_PERMITTED;
	}
	else
	{
		uint32_t data = get_u32(value);

		printf("Rxd: [0]: %02X, [1]: %02X, [2]: %02X, [3]: %02X, data hex: %08X, data: %d\r\n", value[0], value[1], value[2], value[3], data, data);

		/*
		 * The application should get the data written by the peer device.
		 */
		sensor_service_handle->cb->set_sample_rate_cb(&sensor_service_handle->svc, conn_idx, data);
	}

	return error;

}

/* This function is called upon read requests to characteristic attribue value */
static void handle_sensor_id_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt)
{
	/*
	 * Check whether the application has defined a callback function
	 * for handling the event.
	 */
	if(!sensor_service_handle->cb || !sensor_service_handle->cb->get_sensor_id_cb)
	{
		ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_READ_NOT_PERMITTED, 0, NULL);
	}
	else
	{
		/*
		 * The application should provide the requested data to the peer device.
		 */
		sensor_service_handle->cb->get_sensor_id_cb(&sensor_service_handle->svc, evt->conn_idx);
	}
}

/* Handler for write requests, that is BLE_EVT_GATTS_WRITE_REQ */
static void handle_write_req(ble_service_t *svc, const ble_evt_gatts_write_req_t *evt)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;
	att_error_t status = ATT_ERROR_WRITE_NOT_PERMITTED;

	/*
	 * Identify for which attribute handle the write request has been sent to
	 * and call the appropriate function.
	 */

	if(evt->handle == sensor_service_handle->sample_rate_value_h)
	{
		status = handle_sample_rate_write(sensor_service_handle, evt->conn_idx, evt->offset, evt->length, evt->value);
	}
	else if(evt->handle == sensor_service_handle->measurement_value_ccc_h)
	{
		//status = ATT_ERROR_OK;
		//ble_gatts_write_cfm(evt->conn_idx, evt->handle, status);
		status = handle_measurement_value_ccc_write(sensor_service_handle, evt->conn_idx, evt->offset, evt->length, evt->value);
	}

	if (status != ATT_ERROR_OK)
	{
		ble_gatts_write_cfm(evt->conn_idx, evt->handle, status);
	}
}

/* Initialization function for My Custom Service (mcs).*/
ble_service_t *sensor_service_init(const sensor_service_cb_t *cb)
{
	sensor_service_t *sensor_service_handle;

	uint16_t num_attr;
	att_uuid_t uuid;

	/* Allocate memory for the sevice hanle */
	sensor_service_handle = (sensor_service_t *) OS_MALLOC(sizeof(sensor_service_t));
	memset(sensor_service_handle, 0, sizeof(sensor_service_t));

	/* Declare handlers for specific BLE events */
	sensor_service_handle->svc.read_req  = handle_read_req;
	sensor_service_handle->svc.write_req = handle_write_req;
	sensor_service_handle->svc.cleanup   = cleanup;
	sensor_service_handle->cb = cb;

	/*
	 * 0 --> Number of Included Services
	 * 3 --> Number of Characteristic Declarations
	 * 4 --> Number of Descriptors
	 */
	num_attr = ble_gatts_get_num_attr(0, 3, 4);

	/* Service declaration */
	ble_uuid_from_string("00000000-1111-2222-2222-333333333333", &uuid);
	ble_gatts_add_service(&uuid, GATT_SERVICE_PRIMARY, num_attr);


	/* Characteristic declaration for Sensor ID*/
	ble_uuid_from_string("11111111-2222-3333-4444-555555555555", &uuid);
	ble_gatts_add_characteristic(&uuid,
								 GATT_PROP_READ,
								 ATT_PERM_READ,
								 4,
								 GATTS_FLAG_CHAR_READ_REQ,
								 NULL,
								 &sensor_service_handle->sensor_id_value_h);

	/* Define descriptor of type Characteristic User Description (CUD) for Sensor ID */
	ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
	ble_gatts_add_descriptor(&uuid,
							 ATT_PERM_READ,
							 sizeof(sensor_id_char_user_description)-1, // -1 to account for NULL char
							 0,
							 &sensor_service_handle->sensor_id_user_desc_h);


	/* Characteristic declaration for Sample Rate*/
	ble_uuid_from_string("44444444-5555-6666-7777-888888888888", &uuid);
	ble_gatts_add_characteristic(&uuid,
								 GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY,
								 ATT_PERM_RW,
								 4,
								 GATTS_FLAG_CHAR_READ_REQ,
								 NULL,
								 &sensor_service_handle->sample_rate_value_h);

	/* Define descriptor of type Client Characteristic Configuration (CCC) */
	/*ble_uuid_create16(UUID_GATT_CLIENT_CHAR_CONFIGURATION, &uuid);
	ble_gatts_add_descriptor(&uuid,
							 ATT_PERM_RW,
							 2,
							 0,
							 &sensor_service_handle->sample_rate_ccc_h);*/

	/* Define descriptor of type Characteristic User Description (CUD) for Sensor ID */
	ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
	ble_gatts_add_descriptor(&uuid,
							 ATT_PERM_READ,
							 sizeof(sample_rate_char_user_description)-1,  // -1 to account for NULL char
							 0,
							 &sensor_service_handle->sample_rate_user_desc_h);


	/* Characteristic declaration for Measurement Value*/
	ble_uuid_from_string("99999999-AAAA-BBBB-CCCC-DDDDDDDDDDDD", &uuid);
	ble_error_t error = ble_gatts_add_characteristic(&uuid,
								 GATT_PROP_NOTIFY,
								 ATT_PERM_NONE,
								 sizeof(hs300x_data_t),
								 0,
								 NULL,
								 &sensor_service_handle->measurement_value_h);

	/* Define descriptor of type Characteristic User Description (CUD) for Measurement Value*/
	ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
	ble_gatts_add_descriptor(&uuid,
							 ATT_PERM_READ,
							 sizeof(measurement_value_char_user_description)-1, // -1 to account for NULL char
							 0,
							 &sensor_service_handle->measurement_value_user_desc_h);

	ble_uuid_create16(UUID_GATT_CLIENT_CHAR_CONFIGURATION, &uuid);
	ble_gatts_add_descriptor(&uuid,
							 ATT_PERM_RW,
							 2,
							 0,
							 &sensor_service_handle->measurement_value_ccc_h);



	/*
	 * Register all the attribute handles so that they can be updated
	 * by the BLE manager automatically.
	 */
	ble_gatts_register_service(&sensor_service_handle->svc.start_h,
							   &sensor_service_handle->sensor_id_value_h,
							   &sensor_service_handle->sensor_id_user_desc_h,
							   &sensor_service_handle->sample_rate_value_h,
							   //&sensor_service_handle->sample_rate_ccc_h,
							   &sensor_service_handle->sample_rate_user_desc_h,
							   &sensor_service_handle->measurement_value_h,
							   &sensor_service_handle->measurement_value_user_desc_h,
							   &sensor_service_handle->measurement_value_ccc_h,
							   0);

	/* Calculate the last attribute handle of the BLE service */
	sensor_service_handle->svc.end_h = sensor_service_handle->svc.start_h + num_attr;

	/* Set default attribute values */
	ble_gatts_set_value(sensor_service_handle->sensor_id_user_desc_h,
						sizeof(sensor_id_char_user_description)-1,
						sensor_id_char_user_description);

	ble_gatts_set_value(sensor_service_handle->sample_rate_user_desc_h,
						sizeof(sample_rate_char_user_description)-1,
						sample_rate_char_user_description);

	ble_gatts_set_value(sensor_service_handle->measurement_value_user_desc_h,
						sizeof(measurement_value_char_user_description)-1,
						measurement_value_char_user_description);

	/* Register the BLE service in BLE framework */
	ble_service_add(&sensor_service_handle->svc);

	/* Return the service handle */
	return &sensor_service_handle->svc;

}

/*
 * This function should be called by the application as a response to read requests
 */
void sensor_service_get_sample_rate_cfm(ble_service_t *svc, uint16_t conn_idx, att_error_t status, const uint32_t *value)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	/* This function should be used as a response for every read request */
	ble_gatts_read_cfm(conn_idx, sensor_service_handle->sample_rate_value_h, status, sizeof(uint32_t), (uint8_t*)value);
}


/*
 * This function should be called by the application as a response to read requests
 */
void sensor_service_get_sensor_id_cfm(ble_service_t *svc, uint16_t conn_idx, att_error_t status, const uint32_t *value)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	/* This function should be used as a response for every read request */
	ble_gatts_read_cfm(conn_idx, sensor_service_handle->sensor_id_value_h, status, sizeof(uint32_t), (uint8_t*)value);
}


/* Notify the peer device that characteristic attribute value has been updated */
void sensor_service_notify_measurement(ble_service_t *svc, uint16_t conn_idx, const hs300x_data_t *value)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	uint16_t ccc = 0x0000;

	ble_storage_get_u16(conn_idx, sensor_service_handle->measurement_value_ccc_h, &ccc);

	/*
	 * Check if the notifications are enabled from the peer device,
	 * otherwise don't send anything.
	 */
	if (ccc & GATT_CCC_NOTIFICATIONS)
	{
		ble_gatts_send_event(conn_idx, sensor_service_handle->measurement_value_h, GATT_EVENT_NOTIFICATION, sizeof(hs300x_data_t), (uint8_t *)value);
	}
}

/*
 * Notify all the connected peer devices that characteristic attribute value
 * has been updated
 */
void sensor_service_notify_measurement_to_all_connected(ble_service_t *svc, const hs300x_data_t *value)
{
	uint8_t num_conn;
	uint16_t *conn_idx_array;

	ble_gap_get_connected(&num_conn, &conn_idx_array);

	while ((num_conn--) > 0)
	{
		sensor_service_notify_measurement(svc, conn_idx_array[num_conn], value);
	}

	if (conn_idx_array)
	{
		OS_FREE(conn_idx_array);
	}
}

/*
 * This function should be called by the application as a response to write requests
 */
void sensor_service_set_sample_rate_cfm(ble_service_t *svc, uint16_t conn_idx, att_error_t status)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	/* This function should be used as a response for every write request */
	ble_gatts_write_cfm(conn_idx, sensor_service_handle->sample_rate_value_h, status);
}
