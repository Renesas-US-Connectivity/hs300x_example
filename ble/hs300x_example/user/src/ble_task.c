/*
 * ble_task.c
 *
 *  Created on: Mar 6, 2023
 *      Author: a5137667
 */

#include <stdbool.h>
#include <string.h>
#include "osal.h"
#include "ble_att.h"
#include "ble_common.h"
#include "ble_gap.h"
#include "ble_gatts.h"

#include "ble_task.h"
#include "sensor_service.h"
#include "hs300x_task.h"

/* Private function prototypes */
static void get_sample_rate(ble_service_t *svc, uint16_t conn_idx);
static void get_sensor_id(ble_service_t *svc, uint16_t conn_idx);
static void handle_evt_gap_adv_completed(ble_evt_gap_adv_completed_t *evt);
static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt);
static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt);
static void handle_evt_gap_pair_req(ble_evt_gap_pair_req_t *evt);
static void set_sample_rate(ble_service_t *svc, uint16_t conn_idx, const uint32_t new_rate);

/* Private variables */
// Step 6.1 - Update the device name to something unique
char device_name[] = "Patrick_HS3001Demo";

static const sensor_service_cb_t sensor_service_callbacks =
{
	.get_sensor_id_cb = get_sensor_id,
	.get_sample_rate_cb = get_sample_rate,
	.set_sample_rate_cb = set_sample_rate,
};

static const gap_adv_ad_struct_t adv_data[] = {

	GAP_ADV_AD_STRUCT(GAP_DATA_TYPE_LOCAL_NAME, sizeof(device_name), device_name)
};

/**
 * \brief BLE task. This task handles BLE communication for the application
 *
 * \param[in] pvParameters      Used to pass in a queue holding measurements from the sensor
 *
 * \return void
 */
void ble_task(void *pvParameters)
{
	OS_QUEUE sample_q = (OS_QUEUE)pvParameters;

	hs300x_task_event_queue_register(OS_GET_CURRENT_TASK());

	/*************************************************************************************************\
	 * Initialize BLE
	 */
	/* Start BLE device as peripheral */
	// Step 6.2 add the appropriate API to start BLE as a peripheral


	/* Register task to BLE framework to receive BLE event notifications */
	// Step 6.4 add the appropriate API to register the application to receive BLE event notifications


	/* Set device name */
	// Step 6.6 add the appropriate API to set the GAP device name.
	// Note you should use the device_name variable above


	/* Set a random address*/
	own_address_t random_addr = {PRIVATE_RANDOM_RESOLVABLE_ADDRESS};
	ble_gap_address_set(&random_addr, 3600);

	/*************************************************************************************************\
	 * Initialize BLE services
	 */
	/* Add custom sensor service */
	ble_service_t* sensor_service_handle = sensor_service_init(&sensor_service_callbacks);

	/*************************************************************************************************\
	 * Start advertising
	 *
	 * Set advertising data and scan response, then start advertising.
	 *
	 * By default, advertising interval is set to "fast connect" and a timer is started to
	 * switch to "reduced power" interval afterwards.
	 */
	ble_gap_adv_ad_struct_set(ARRAY_LENGTH(adv_data), adv_data, 0 , NULL);
	// Step 6.8 add the appropriate API to start the advertising in undirected mode



	for (;;)
	{
		uint32_t notif;

		/*
		 * Wait on any of the notification bits, then clear them all
		 */
		OS_BASE_TYPE ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
		/* Blocks forever waiting for the task notification. Therefore, the return value must
		 * always be OS_OK
		 */
		OS_ASSERT(ret == OS_OK);

		/* Notified from BLE manager */
		if (notif & BLE_APP_NOTIFY_MASK)
		{
			ble_evt_hdr_t *hdr;
			hdr = ble_get_event(false);

			if (hdr)
			{
				/*
				 * First, the application needs to check if the event is handled by the
				 * ble_service framework. If it is not handled, the application may handle
				 * it by defining a case for it in the `switch ()` statement below. If the
				 * event is not handled by the application either, it is handled by the
				 * default event handler.
				 */
				if (!ble_service_handle_event(hdr)) {
					switch (hdr->evt_code)
					{
						case BLE_EVT_GAP_CONNECTED:
							handle_evt_gap_connected((ble_evt_gap_connected_t *) hdr);
							break;
						case BLE_EVT_GAP_DISCONNECTED:
							handle_evt_gap_disconnected((ble_evt_gap_disconnected_t *) hdr);
							break;
						case BLE_EVT_GAP_ADV_COMPLETED:
							handle_evt_gap_adv_completed((ble_evt_gap_adv_completed_t *) hdr);
							break;
						case BLE_EVT_GAP_PAIR_REQ:
							handle_evt_gap_pair_req((ble_evt_gap_pair_req_t *) hdr);
							break;
						default:
							ble_handle_event_default(hdr);
							break;
					}
				}

				/* Free event buffer (it's not needed anymore) */
				OS_FREE(hdr);
			}


			/*
			 * If there are more events waiting in queue, application should process
			 * them now.
			 */
			if (ble_has_event()) {
				OS_TASK_NOTIFY(OS_GET_CURRENT_TASK(), BLE_APP_NOTIFY_MASK, OS_NOTIFY_SET_BITS);
			}
		}

                /* Notified HS3001 Task */
                if (notif & HS3001_MEASUREMENT_NOTIFY_MASK)
                {
                        // Process all items on the measurement queue
                        OS_BASE_TYPE q_status = OS_QUEUE_OK;
                        while (q_status != OS_QUEUE_EMPTY)
                        {
                                // Get a measurement from the queue
                                hs300x_data_t sample = {0};
                                q_status = OS_QUEUE_GET(sample_q, &sample, OS_QUEUE_NO_WAIT);

                                // if a measurement is available, notify all connected clients
                                if(q_status == OS_QUEUE_OK)
                                {
                                       /* Step 7.6
                                          Add the appropriate API from sensor_service.h to notify all connected clients
                                          that a new sample measurement is available
                                       */

                                }
                        }
                }
	}
}

/**
 * \brief Callback to handle Sample Rate read requests
 *
 * \param[in] svc      		service handle
 * \param[in] conn_idx      	connection index associated with the client making the request
 *
 * \return void
 */
static void get_sample_rate(ble_service_t *svc, uint16_t conn_idx)
{
	uint32_t sample_rate_ms = hs300x_task_get_sample_rate();
	sensor_service_get_sample_rate_cfm(svc, conn_idx, ATT_ERROR_OK, &sample_rate_ms);
}

/**
 * \brief Callback to handle Sensor ID read requests
 *
 * \param[in] svc      		service handle
 * \param[in] conn_idx      	connection index associated with the client making the request
 *
 * \return void
 */
static void get_sensor_id(ble_service_t *svc, uint16_t conn_idx)
{
        /* Step 7.2 - When a client attempts to read the Sensor ID characteristic,
           this callback will invoked.
           Add the appropriate API from hs300x_task.h to get the sensor ID
           Then add the appropriate API from sensor_service.h to confirm the value
           with the BLE client
        */


}

/**
 * \brief Handler for advertising completed event
 *
 * \param[in] evt      		pointer to the advertising completed event
 *
 * \return void
 */
static void handle_evt_gap_adv_completed(ble_evt_gap_adv_completed_t *evt)
{
	/*
	 * If advertising is completed, just restart it. It's either because a new client connected
	 * or it was cancelled in order to change the interval values.
	 */
	ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
}

/**
 * \brief Handler for connected event
 *
 * \param[in] evt      		pointer to the connected event
 *
 * \return void
 */
static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt)
{
	// Manage behavior upon connection
}

/**
 * \brief Handler for disconnected event
 *
 * \param[in] evt      		pointer to the disconnected event
 *
 * \return void
 */
static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{

	// Manage behavior upon disconnection

	// Restart advertising
	ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
}

/**
 * \brief Handler for pairing request event
 *
 * \param[in] evt      		pointer to the pairing request event
 *
 * \return void
 */
static void handle_evt_gap_pair_req(ble_evt_gap_pair_req_t *evt)
{
	ble_gap_pair_reply(evt->conn_idx, true, evt->bond);
}

/**
 * \brief Callback to handle Sample Rate write requests
 *
 * \param[in] evt      		pointer to the advertising completed event
 *
 * \return void
 */
static void set_sample_rate(ble_service_t *svc, uint16_t conn_idx, const uint32_t new_rate)
{
       /* Step 7.4 - When a client attempts to write the Sample Rate characteristic,
          this callback will invoked.
          Add the appropriate API from hs300x_task.h to set the Sample Rate
          Then add the appropriate API from sensor_service.h to confirm with the client
          the write has been processed
       */

}
