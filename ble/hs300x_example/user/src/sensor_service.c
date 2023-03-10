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

/* Custom sensor service  structure*/
typedef struct {
	ble_service_t svc;

	// User-defined callback functions
	const sensor_service_cb_t *cb;

	// Attribute handles of BLE service
	uint16_t sensor_id_value_h;	                // Sensor ID Value
	uint16_t sensor_id_user_desc_h;			// Sensor ID User Description

	uint16_t sample_rate_value_h;			// Sample Rate Value
	uint16_t sample_rate_user_desc_h;		// Sample Rate User Description

	uint16_t measurement_value_h;			// Measurement Value
	uint16_t measurement_value_user_desc_h;         // Measurement Value User Description
	uint16_t measurement_value_ccc_h;		// Measurement Value Client Characteristic Configuration Descriptor. Used for notifications

} sensor_service_t;


/* Private function prototypes */
static void cleanup(ble_service_t *svc);
static void handle_measurement_ccc_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt);
static att_error_t handle_measurement_ccc_write(sensor_service_t *sample_service_handle, const ble_evt_gatts_write_req_t *evt);
static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt);
static void handle_sample_rate_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt);
static att_error_t handle_sample_rate_write(sensor_service_t *sensor_service_handle, const ble_evt_gatts_write_req_t *evt);
static void handle_sensor_id_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt);
static void handle_write_req(ble_service_t *svc, const ble_evt_gatts_write_req_t *evt);

/* Service Constants */
static const char sensor_id_char_user_description[]  = "Sensor ID";
static const char sample_rate_char_user_description[]  = "Sample Rate";
static const char measurement_value_char_user_description[]  = "Measurement Value";

/* Service Defines */
#define SENSOR_ID_CHAR_SIZE 			sizeof(uint32_t)
#define SAMPLE_RATE_CHAR_SIZE 			sizeof(uint32_t)
#define MEASUREMENT_VALUE_CHAR_SIZE 	sizeof(hs300x_data_t)

/**
 * \brief Service cleanup function.
 *
 * \param[in] svc          pointer BLE service
 *
 * \return void
 */
static void cleanup(ble_service_t *svc)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	ble_storage_remove_all(sensor_service_handle->measurement_value_ccc_h);

	OS_FREE(sensor_service_handle);
}

/**
 * \brief This function is called when their is a read request for the Measurement Value Characteristic CCC
 *
 * \param[in] sensor_service_handle         pointer sensor service handle
 * \param[in] evt          		    pointer to the read request
 *
 * \return void
 */
static void handle_measurement_ccc_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt)
{
	uint16_t ccc = 0x0000;

	// Extract the CCC value from the ble storage
	ble_storage_get_u16(evt->conn_idx, sensor_service_handle->measurement_value_ccc_h, &ccc);

	// Send a read confirmation with the value from storage
	ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_OK, sizeof(ccc), &ccc);
}

/**
 * \brief This function is called when their is a write request for the Measurement Value Characteristic CCC
 *
 * \param[in] sensor_service_handle         pointer sensor service handle
 * \param[in] evt          		    pointer to the write request
 *
 * \return att_error_t indicating the status of the request.
 */
static att_error_t handle_measurement_ccc_write(sensor_service_t *sample_service_handle, const ble_evt_gatts_write_req_t *evt)
{
	att_error_t error = ATT_ERROR_OK;

	// Verify the write request is valid
	if(evt->offset)
	{
		error = ATT_ERROR_ATTRIBUTE_NOT_LONG;
	}
	else if(evt->length != sizeof(uint16_t)) // All CCCs are 2 bytes
	{
		error = ATT_ERROR_INVALID_VALUE_LENGTH;
	}
	else
	{
		uint16_t ccc = get_u16(evt->value);

		// Store the CCC value to ble storage
		ble_storage_put_u32(evt->conn_idx, sample_service_handle->measurement_value_ccc_h, ccc, true);

		// Respond to the write requst
		ble_gatts_write_cfm(evt->conn_idx, sample_service_handle->measurement_value_ccc_h, error);
	}

	return error;
}

/**
 * \brief This function is called when their is a read request for an attribute in our custom sensor service
 *
 * \param[in] sensor_service_handle         pointer service handle
 * \param[in] evt          		    pointer to the read request
 *
 * \return void
 */
static void handle_read_req(ble_service_t *svc, const ble_evt_gatts_read_req_t *evt)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	/*
	 * Identify which attribute handle the read request has been sent to
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
		handle_measurement_ccc_read(sensor_service_handle, evt);
	}
	// Otherwise read operations are not permitted
	else
	{
		ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_READ_NOT_PERMITTED, 0, NULL);
	}

}

/**
 * \brief This function is called when their is a read request for the Sample Rate
 *
 * \param[in] sensor_service_handle         pointer service handle
 * \param[in] evt          		    pointer to the read request
 *
 * \return void
 */
static void handle_sample_rate_read(sensor_service_t *sensor_service_handle, const ble_evt_gatts_read_req_t *evt)
{
	/*
	 * Check whether the application has defined a callback function
	 * for handling the event.
	 */
	if(!sensor_service_handle->cb || !sensor_service_handle->cb->get_sample_rate_cb)
	{
		// if no callback exists to handle the event, inform the client the operation is not permitted
		ble_gatts_read_cfm(evt->conn_idx, evt->handle, ATT_ERROR_READ_NOT_PERMITTED, 0, NULL);
	}
	else
	{
		// The application will provide the requested data to the peer device.
		sensor_service_handle->cb->get_sample_rate_cb(&sensor_service_handle->svc, evt->conn_idx);
	}
}

/**
 * \brief This function is called when their is a write request for the Sample Rate
 *
 * \param[in] sensor_service_handle         pointer service handle
 * \param[in] evt          		    pointer to the write request
 *
 * \return att_error_t indicating the status of the request.
 */
static att_error_t handle_sample_rate_write(sensor_service_t *sensor_service_handle, const ble_evt_gatts_write_req_t *evt)
{
	att_error_t error = ATT_ERROR_OK;

	// Verify the write request is valid
	if(evt->offset)
	{
		error = ATT_ERROR_ATTRIBUTE_NOT_LONG;
	}
	else if(evt->length != SAMPLE_RATE_CHAR_SIZE)
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
		uint32_t data = get_u32(evt->value);

		/*
		 * The application should get the data written by the peer device.
		 */
		sensor_service_handle->cb->set_sample_rate_cb(&sensor_service_handle->svc, evt->conn_idx, data);
	}

	return error;
}

/**
 * \brief This function is called when their is a read request for the Sensor ID
 *
 * \param[in] sensor_service_handle         pointer service handle
 * \param[in] evt          		    pointer to the read request
 *
 * \return void
 */
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

/**
 * \brief This function is called when their is a write request for an attribute in our custom sensor service
 *
 * \param[in] sensor_service_handle         pointer service handle
 * \param[in] evt          		    pointer to the write request
 *
 * \return void
 */
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
		status = handle_sample_rate_write(sensor_service_handle, evt);
	}
	else if(evt->handle == sensor_service_handle->measurement_value_ccc_h)
	{
		status = handle_measurement_ccc_write(sensor_service_handle, evt);
	}

	/* If the status is anything other than ATT_ERROR_OK, inform the client the write is rejected
	 * If the status is ATT_ERROR_OK, the application (or one of the above write handlers) will take care of
	 * providing the data to the client */
	if (status != ATT_ERROR_OK)
	{
		ble_gatts_write_cfm(evt->conn_idx, evt->handle, status);
	}
}

/**
 * \brief This function is called when their is a write request for an attribute in our custom sensor service
 *
 * \param[in] cb         	pointer to application callback structure to handle service events
 *
 * \return pointer to the handle created for this service
 */
ble_service_t *sensor_service_init(const sensor_service_cb_t *cb)
{
	sensor_service_t *sensor_service_handle;

	uint16_t num_attr;
	att_uuid_t uuid;

	// Allocate memory for the service handle
	sensor_service_handle = (sensor_service_t *) OS_MALLOC(sizeof(sensor_service_t));
	memset(sensor_service_handle, 0, sizeof(sensor_service_t));

	// Declare handlers for specific BLE events
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

	// Service declaration
	ble_uuid_from_string("00000000-1111-2222-2222-333333333333", &uuid);
	ble_gatts_add_service(&uuid, GATT_SERVICE_PRIMARY, num_attr);

	// Characteristic declaration for Sensor ID
	ble_uuid_from_string("11111111-2222-3333-4444-555555555555", &uuid);
	ble_gatts_add_characteristic(&uuid,
								 GATT_PROP_READ,
								 ATT_PERM_READ,
								 SENSOR_ID_CHAR_SIZE,
								 GATTS_FLAG_CHAR_READ_REQ,
								 NULL,
								 &sensor_service_handle->sensor_id_value_h);

	// Define descriptor of type Characteristic User Description for Sensor ID
	ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
	ble_gatts_add_descriptor(&uuid,
							 ATT_PERM_READ,
							 sizeof(sensor_id_char_user_description)-1, // -1 to account for NULL char
							 0,
							 &sensor_service_handle->sensor_id_user_desc_h);

	// Characteristic declaration for Sample Rate
	ble_uuid_from_string("44444444-5555-6666-7777-888888888888", &uuid);
	ble_gatts_add_characteristic(&uuid,
								 GATT_PROP_READ | GATT_PROP_WRITE,
								 ATT_PERM_RW,
								 SAMPLE_RATE_CHAR_SIZE,
								 GATTS_FLAG_CHAR_READ_REQ,
								 NULL,
								 &sensor_service_handle->sample_rate_value_h);

	// Define descriptor of type Characteristic User Description for Sensor ID
	ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
	ble_gatts_add_descriptor(&uuid,
							 ATT_PERM_READ,
							 sizeof(sample_rate_char_user_description)-1,  // -1 to account for NULL char
							 0,
							 &sensor_service_handle->sample_rate_user_desc_h);

	// Characteristic declaration for Measurement Value
	ble_uuid_from_string("99999999-AAAA-BBBB-CCCC-DDDDDDDDDDDD", &uuid);
	ble_gatts_add_characteristic(&uuid,
								 GATT_PROP_NOTIFY,
								 ATT_PERM_NONE,
								 MEASUREMENT_VALUE_CHAR_SIZE,
								 0,
								 NULL,
								 &sensor_service_handle->measurement_value_h);

	// Define descriptor of type Characteristic User Description for Measurement Value
	ble_uuid_create16(UUID_GATT_CHAR_USER_DESCRIPTION, &uuid);
	ble_gatts_add_descriptor(&uuid,
							 ATT_PERM_READ,
							 sizeof(measurement_value_char_user_description)-1, // -1 to account for NULL char
							 0,
							 &sensor_service_handle->measurement_value_user_desc_h);

	// Define descriptor of type Cleint Characteristic Configuration Descriptor for Measurement Value
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
							   &sensor_service_handle->sample_rate_user_desc_h,
							   &sensor_service_handle->measurement_value_h,
							   &sensor_service_handle->measurement_value_user_desc_h,
							   &sensor_service_handle->measurement_value_ccc_h,
							   0);

	// Calculate the last attribute handle of the BLE service
	sensor_service_handle->svc.end_h = sensor_service_handle->svc.start_h + num_attr;

	// Set default values for User Descriptions
	ble_gatts_set_value(sensor_service_handle->sensor_id_user_desc_h,
						sizeof(sensor_id_char_user_description)-1,
						sensor_id_char_user_description);

	ble_gatts_set_value(sensor_service_handle->sample_rate_user_desc_h,
						sizeof(sample_rate_char_user_description)-1,
						sample_rate_char_user_description);

	ble_gatts_set_value(sensor_service_handle->measurement_value_user_desc_h,
						sizeof(measurement_value_char_user_description)-1,
						measurement_value_char_user_description);

	// Register the BLE service in BLE framework
	ble_service_add(&sensor_service_handle->svc);

	// Return the service handle
	return &sensor_service_handle->svc;

}

/**
 * \brief This function should be called by the application in response to Sample Rate read requests
 *
 * \param[in] svc           pointer to service handle
 * \param[in] conn_idx      connection index of the client to send confirmation to
 * \param[in] status        status of the request
 * \param[in] value         sample rate value to respond with
 *
 * \return void
 */
void sensor_service_get_sample_rate_cfm(ble_service_t *svc, uint16_t conn_idx, att_error_t status, const uint32_t *value)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	ble_gatts_read_cfm(conn_idx, sensor_service_handle->sample_rate_value_h, status, SENSOR_ID_CHAR_SIZE, (uint8_t*)value); // TODO SAMPLE_RATE_SIZE
}

/**
 * \brief This function should be called by the application in response to Sensor ID read requests
 *
 * \param[in] svc           pointer to service handle
 * \param[in] conn_idx      connection index of the client to send confirmation to
 * \param[in] status        status of the request
 * \param[in] value         sensor ID value to respond with
 *
 * \return void
 */
void sensor_service_get_sensor_id_cfm(ble_service_t *svc, uint16_t conn_idx, att_error_t status, const uint32_t *value)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	ble_gatts_read_cfm(conn_idx, sensor_service_handle->sensor_id_value_h, status, SENSOR_ID_CHAR_SIZE, (uint8_t*)value);
}


/**
 * \brief This function should be called by the application to notify a client of a new Measurement Value
 *
 * \param[in] svc           pointer to service handle
 * \param[in] conn_idx      connection index of the client to send notification to
 * \param[in] value         measurement value to notify client with
 *
 * \return void
 */
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
		ble_gatts_send_event(conn_idx, sensor_service_handle->measurement_value_h, GATT_EVENT_NOTIFICATION, MEASUREMENT_VALUE_CHAR_SIZE, (uint8_t *)value);
	}
}

/**
 * \brief This function can be called by the application to notify all connected clients of a new Measurement Value
 *
 * \param[in] svc           pointer to service handle
 * \param[in] value         measurement value to notify client with
 *
 * \return void
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

/**
 * \brief This function should be called by the application in response to Sample Rate write requests
 *
 * \param[in] svc           pointer to service handle
 * \param[in] conn_idx      connection index of the client to send notification to
 * \param[in] status        status of the request
 *
 * \return void
 */
void sensor_service_set_sample_rate_cfm(ble_service_t *svc, uint16_t conn_idx, att_error_t status)
{
	sensor_service_t *sensor_service_handle = (sensor_service_t *) svc;

	/* This function should be used as a response for every write request */
	ble_gatts_write_cfm(conn_idx, sensor_service_handle->sample_rate_value_h, status);
}
