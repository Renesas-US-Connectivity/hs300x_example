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
#include "ble_l2cap.h"
#include "sdk_list.h"
#include "bas.h"
#include "ias.h"
#include "lls.h"
#include "tps.h"

#include "ble_task.h"
#include "sensor_service.h"
#include "hs300x_task.h"

/*
 * The maximum length of name in adv_packet
 */
uint8_t device_name[] = "YourName_HS3001Demo";

static void get_sample_rate(ble_service_t *svc, uint16_t conn_idx);
static void get_sensor_id(ble_service_t *svc, uint16_t conn_idx);
static void set_sample_rate(ble_service_t *svc, uint16_t conn_idx, const uint32_t new_rate);

static const sensor_service_cb_t sensor_service_callbacks =
{
	.get_sensor_id_cb = get_sensor_id,
	.get_sample_rate_cb = get_sample_rate,
	.set_sample_rate_cb = set_sample_rate,
};

static void get_sample_rate(ble_service_t *svc, uint16_t conn_idx)
{
	printf("get_sample_Rate\r\n");
	uint32_t sample_rate_ms = hs300x_task_get_sample_rate();
	sensor_service_get_sample_rate_cfm(svc, conn_idx, ATT_ERROR_OK, &sample_rate_ms);
}

static void get_sensor_id(ble_service_t *svc, uint16_t conn_idx)
{
	printf("get_sensor_id\r\n");
	uint32_t id = hs300x_task_get_sensor_id();
	sensor_service_get_sensor_id_cfm(svc, conn_idx, ATT_ERROR_OK, &id);
}

static void set_sample_rate(ble_service_t *svc, uint16_t conn_idx, const uint32_t new_rate)
{
	printf("set_sample_rate\r\n");

	hs300x_task_set_sample_rate(new_rate);
	sensor_service_set_sample_rate_cfm(svc, conn_idx, ATT_ERROR_OK);
}

/*
 * PXP advertising and scan response data
 *
 * While not required, the PXP specification states that a PX reporter device using the peripheral
 * role can advertise support for LLS. Device name is set in scan response to make it easily
 * recognizable.
 */
static const gap_adv_ad_struct_t adv_data[] = {

	GAP_ADV_AD_STRUCT(GAP_DATA_TYPE_LOCAL_NAME, sizeof(device_name), device_name)
};

static void handle_evt_gap_connected(ble_evt_gap_connected_t *evt)
{
	/**
	 * Manage behavior upon connection
	 */
	printf("Connected to central\r\n");

}

static void handle_evt_gap_disconnected(ble_evt_gap_disconnected_t *evt)
{
	 /**
	 * Manage behavior upon disconnection
	 */
	printf("Disconnected from central\r\n");

	// Restart advertising
	ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
}

static void handle_evt_gap_pair_req(ble_evt_gap_pair_req_t *evt)
{
        ble_gap_pair_reply(evt->conn_idx, true, evt->bond);
}

static void handle_evt_gap_adv_completed(ble_evt_gap_adv_completed_t *evt)
{
	/*
	 * If advertising is completed, just restart it. It's either because a new client connected
	 * or it was cancelled in order to change the interval values.
	 */
	ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);
}

void ble_task(void *params)
{
	uint16_t name_len;
	uint8_t name_buf[BLE_ADV_DATA_LEN_MAX + 1];        /* 1 byte for '\0' character */

	/*************************************************************************************************\
	 * Initialize BLE
	 */
	/* Start BLE device as peripheral */
	ble_peripheral_start();

	/* Register task to BLE framework to receive BLE event notifications */
	ble_register_app();

	/* Get device name from NVPARAM if valid or use default otherwise */
	name_len = sizeof(device_name);
	strcpy(name_buf, device_name);

	/* Set device name */
	ble_gap_device_name_set(name_buf, ATT_PERM_READ);

	/*************************************************************************************************\
	 * Initialize BLE services
	 */
	/* Add custom sensor service */
	sensor_service_init(&sensor_service_callbacks);

	/*************************************************************************************************\
	 * Start advertising
	 *
	 * Set advertising data and scan response, then start advertising.
	 *
	 * By default, advertising interval is set to "fast connect" and a timer is started to
	 * switch to "reduced power" interval afterwards.
	 */
	ble_gap_adv_ad_struct_set(ARRAY_LENGTH(adv_data), adv_data, 0 , NULL);
	ble_gap_adv_start(GAP_CONN_MODE_UNDIRECTED);

	for (;;)
	{
		//OS_BASE_TYPE ret __UNUSED;
		uint32_t notif;

		/*
		 * Wait on any of the notification bits, then clear them all
		 */
		OS_BASE_TYPE ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
		/* Blocks forever waiting for the task notification. Therefore, the return value must
		 * always be OS_OK
		 */
		OS_ASSERT(ret == OS_OK);

		/* Notified from BLE manager? */
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
	}
}

