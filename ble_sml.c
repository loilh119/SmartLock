#include "sdk_common.h"
#include "ble_sml.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"
#include "R301.h"
#include "nrf_delay.h"
#include "app_uart.h"
#include "nrf_uart.h"
#include "ble_link_ctx_manager.h"

#define BLE_NUS_MAX_RX_CHAR_LEN        BLE_NUS_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_NUS_MAX_TX_CHAR_LEN        BLE_NUS_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */

static void on_connect(ble_sml_t * p_sml, ble_evt_t const * p_ble_evt)
{				
		p_sml->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		
		ble_sml_evt_t evt;
		
		evt.evt_type = BLE_SML_EVT_CONNECTED;

    p_sml->evt_handler(p_sml, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_hrs       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_sml_t * p_sml, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
	
    p_sml->conn_handle = BLE_CONN_HANDLE_INVALID;
	
		ble_sml_evt_t evt;

    evt.evt_type = BLE_SML_EVT_DISCONNECTED;

    p_sml->evt_handler(p_sml, &evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_hrs       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_sml_t * p_sml, ble_evt_t const * p_ble_evt)
{
    ble_sml_evt_t                 evt;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    memset(&evt, 0, sizeof(ble_sml_evt_t));
    evt.p_sml       = p_sml;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;

		if ((p_evt_write->handle == p_sml->lock_status_handle.cccd_handle)
        && (p_evt_write->len == 2))
    {
        // CCCD written, call application event handler
        if (p_sml->evt_handler != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
								NRF_LOG_INFO("notification");
                evt.evt_type = BLE_SML_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
								NRF_LOG_INFO("not notification");
                evt.evt_type = BLE_SML_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_sml->evt_handler(p_sml, &evt);
        }
		}
		
		else if ((p_evt_write->handle == p_sml->lock_control_handle.value_handle) &&
             (p_sml->evt_handler != NULL))
    {
			nrf_gpio_pin_toggle(LED_3);
			NRF_LOG_INFO("LENGTH: %d",p_evt_write->len);
			
			if(p_evt_write->data[0] == 'u')
			{
					evt.evt_type               = BLE_SML_EVT_LOCK_OPEN;
					evt.params.sml_data.p_data = p_evt_write->data;
					evt.params.sml_data.length = p_evt_write->len;
					p_sml->evt_handler(p_sml, &evt);
			}				
			else
			{
					evt.evt_type               = BLE_SML_EVT;
					evt.params.sml_data.p_data = p_evt_write->data;
					evt.params.sml_data.length = p_evt_write->len;
					p_sml->evt_handler(p_sml, &evt);
			}
    }
		else if ((p_evt_write->handle == p_sml->finger_print_handle.value_handle) &&
             (p_sml->evt_handler != NULL))
    {
			nrf_gpio_pin_toggle(LED_3);
			switch(p_evt_write->data[0])
			{
				case 'f':
					evt.evt_type               = BLE_SML_EVT_FINGER;
					evt.params.sml_data.p_data = p_evt_write->data;
					evt.params.sml_data.length = p_evt_write->len;
					p_sml->evt_handler(p_sml, &evt);
					break;
				case 'e':
					evt.evt_type               = BLE_SML_EVT_DELETE_FINGER;
					evt.params.sml_data.p_data = p_evt_write->data;
					evt.params.sml_data.length = p_evt_write->len;
					p_sml->evt_handler(p_sml, &evt);
					break;
				case 'c':
					evt.evt_type               = BLE_SML_EVT_CANCEL;
					evt.params.sml_data.p_data = p_evt_write->data;
					evt.params.sml_data.length = p_evt_write->len;
					p_sml->evt_handler(p_sml, &evt);
					break;
				default:
					evt.evt_type               = BLE_SML_EVT;
					evt.params.sml_data.p_data = p_evt_write->data;
					evt.params.sml_data.length = p_evt_write->len;
					p_sml->evt_handler(p_sml, &evt);
					break;
			}
		}
}


void ble_sml_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_sml_t * p_sml = (ble_sml_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sml, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sml, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sml, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t custom_value_char_add(ble_sml_t * p_sml, const ble_sml_init_t * p_sml_init)
{
		//lock control characteristic
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
		ble_gatts_attr_md_t cccd_md;
	
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL; 
    char_md.p_sccd_md         = NULL;
		
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_sml_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_sml_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
		
    ble_uuid.type = p_sml->uuid_type;
    ble_uuid.uuid = VALUE_CHAR_LOCK_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;

		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&attr_md.write_perm);

    err_code = sd_ble_gatts_characteristic_add(p_sml->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_sml->lock_control_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		//Lock status characteristic
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 0;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_sml_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_sml_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_sml->uuid_type;
    ble_uuid.uuid = VALUE_CHAR_STATUS_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 4;

		memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
		
    err_code = sd_ble_gatts_characteristic_add(p_sml->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_sml->lock_status_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		// Finger characteristic
		memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL; 
    char_md.p_sccd_md         = NULL;
		
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_sml_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_sml_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
		
    ble_uuid.type = p_sml->uuid_type;
    ble_uuid.uuid = VALUE_CHAR_FINGER_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;
		
		BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&attr_md.write_perm);

    err_code = sd_ble_gatts_characteristic_add(p_sml->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_sml->finger_print_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
    return NRF_SUCCESS;
}

uint32_t ble_sml_init(ble_sml_t * p_sml, const ble_sml_init_t * p_sml_init)
{
    if (p_sml == NULL || p_sml_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
		p_sml->evt_handler							 = p_sml_init->evt_handler;
    p_sml->conn_handle               = BLE_CONN_HANDLE_INVALID;
		
		p_sml->lock_status               = LOCK_OPEN;
    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_sml->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_sml->uuid_type;
    ble_uuid.uuid = SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_sml->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    return custom_value_char_add(p_sml, p_sml_init);;
}


uint32_t ble_sml_custom_value_update(ble_sml_t * p_sml, uint8_t * custom_value, uint8_t length)
{
    NRF_LOG_INFO("In ble_sml_custom_value_update. \r\n"); 
    if (p_sml == NULL)
    {
				NRF_LOG_ERROR("NRF_ERROR_NULL");
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

//    gatts_value.len     = sizeof(uint32_t);
//    gatts_value.offset  = 0;
//    gatts_value.p_value = (uint8_t *)custom_value;
		
		gatts_value.len     = length;
    gatts_value.offset  = 0;
    gatts_value.p_value = custom_value;
    // Update database.
    err_code = sd_ble_gatts_value_set(p_sml->conn_handle,
                                      p_sml->lock_status_handle.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
				NRF_LOG_ERROR("%d",err_code);
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_sml->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_sml->lock_status_handle.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_sml->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
		NRF_LOG_ERROR("%d",err_code);
    return err_code;
}
