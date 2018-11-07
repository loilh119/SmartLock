#include "sdk_common.h"
#include "ble_sml.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

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
		uint8_t a[4] = "Open";
		uint8_t b[4] = "Lock";
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_sml->lock_control_handle.value_handle) && (*p_evt_write->data == 0x11))
		{
			nrf_gpio_pin_set(28);
					if(nrf_gpio_pin_read(26))
					{
						ble_sml_custom_value_update(p_sml, a);	
					}
					else
					{
						ble_sml_custom_value_update(p_sml, b);	
					}
		}
		else
		{
				nrf_gpio_pin_clear(28);
					if(nrf_gpio_pin_read(26))
					{
						ble_sml_custom_value_update(p_sml, a);	
					}
					else
					{
						ble_sml_custom_value_update(p_sml, b);	
					}
		}
			
		if ((p_evt_write->handle == p_sml->lock_status_handle.cccd_handle)
        && (p_evt_write->len == 2)
       )
    {
        // CCCD written, call application event handler
        if (p_sml->evt_handler != NULL)
        {
            ble_sml_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_SML_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_SML_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_sml->evt_handler(p_sml, &evt);
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

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 0;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 0; 
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
    ble_uuid.uuid = CUSTOM_VALUE_CHAR_UUID_1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);

    err_code = sd_ble_gatts_characteristic_add(p_sml->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_sml->lock_control_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		//Lock status characteristic
		ble_gatts_char_md_t char_md_2;
    ble_gatts_attr_md_t cccd_md_2;
    ble_gatts_attr_t    attr_char_value_2;
    ble_gatts_attr_md_t attr_md_2;

    memset(&char_md_2, 0, sizeof(char_md_2));

    char_md_2.char_props.read   = 1;
    char_md_2.char_props.write  = 0;
    char_md_2.char_props.notify = 1; 
    char_md_2.p_char_user_desc  = NULL;
    char_md_2.p_char_pf         = NULL;
    char_md_2.p_user_desc_md    = NULL;
    char_md_2.p_cccd_md         = &cccd_md_2; 
    char_md_2.p_sccd_md         = NULL;
		
    memset(&attr_md_2, 0, sizeof(attr_md_2));

    attr_md_2.read_perm  = p_sml_init->custom_value_char_attr_md.read_perm;
    attr_md_2.write_perm = p_sml_init->custom_value_char_attr_md.write_perm;
    attr_md_2.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md_2.rd_auth    = 0;
    attr_md_2.wr_auth    = 0;
    attr_md_2.vlen       = 0;

    ble_uuid.type = p_sml->uuid_type;
    ble_uuid.uuid = CUSTOM_VALUE_CHAR_UUID_2;

    memset(&attr_char_value_2, 0, sizeof(attr_char_value_2));

    attr_char_value_2.p_uuid    = &ble_uuid;
    attr_char_value_2.p_attr_md = &attr_md;
    attr_char_value_2.init_len  = sizeof(uint8_t);
    attr_char_value_2.init_offs = 0;
    attr_char_value_2.max_len   = sizeof(uint8_t);

		memset(&cccd_md_2, 0, sizeof(cccd_md_2));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md_2.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md_2.write_perm);
    
    cccd_md_2.vloc       = BLE_GATTS_VLOC_STACK;
		
    err_code = sd_ble_gatts_characteristic_add(p_sml->service_handle, &char_md_2,
                                               &attr_char_value_2,
                                               &p_sml->lock_status_handle);
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

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_sml->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_sml->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_sml->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    return custom_value_char_add(p_sml, p_sml_init);;
}

uint32_t ble_sml_custom_value_update(ble_sml_t * p_sml, uint8_t * custom_value)
{
    NRF_LOG_INFO("In ble_sml_custom_value_update. \r\n"); 
    if (p_sml == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = custom_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_sml->conn_handle,
                                      p_sml->lock_status_handle.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
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


    return err_code;
}
