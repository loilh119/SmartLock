#include "sdk_common.h"
#include <stdlib.h>

#include "ble.h"
#include "ble_alarm_c.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"

#define NRF_LOG_MODULE_NAME ble_alarm_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/**@brief     Function for initializing the Nordic UART client module.
 *
 * @details   This function registers with the Database Discovery module
 *            for the NUS. Doing so will make the Database Discovery
 *            module look for the presence of a NUS instance at the peer when a
 *            discovery is started.
 *
 * @param[in] p_ble_nus_c      Pointer to the NUS client structure.
 * @param[in] p_ble_nus_c_init Pointer to the NUS initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS If the module was initialized successfully. Otherwise, an error
 *                        code is returned. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_alarm_c_init(ble_alarm_c_t * p_ble_alarm_c, ble_alarm_c_init_t * p_ble_alarm_c_init)
{
    uint32_t      err_code;
    ble_uuid_t    uart_uuid;
    ble_uuid128_t nus_base_uuid = CUSTOM_SERVICE_CENTRAL_UUID_BASE;
		
		
    VERIFY_PARAM_NOT_NULL(p_ble_alarm_c);
    VERIFY_PARAM_NOT_NULL(p_ble_alarm_c_init);
		
	
    err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_ble_alarm_c->uuid_type);
		
		VERIFY_SUCCESS(err_code);
		

    uart_uuid.type = p_ble_alarm_c->uuid_type;
    uart_uuid.uuid = CUSTOM_SERVICE_CENTRAL_UUID;

    p_ble_alarm_c->conn_handle           = BLE_CONN_HANDLE_INVALID;
    p_ble_alarm_c->evt_handler           = p_ble_alarm_c_init->evt_handler;
    p_ble_alarm_c->handles.nus_tx_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_alarm_c->handles.nus_rx_handle = BLE_GATT_HANDLE_INVALID;
		
    return ble_db_discovery_evt_register(&uart_uuid);
}

/**@brief Function for handling events from the database discovery module.
 *
 * @details This function will handle an event from the database discovery module, and determine
 *          if it relates to the discovery of NUS at the peer. If so, it will
 *          call the application's event handler indicating that NUS has been
 *          discovered at the peer. It also populates the event with the service related
 *          information before providing it to the application.
 *
 * @param[in] p_ble_nus_c Pointer to the NUS client structure.
 * @param[in] p_evt       Pointer to the event received from the database discovery module.
 */
void ble_alarm_c_on_db_disc_evt(ble_alarm_c_t * p_ble_alarm_c, ble_db_discovery_evt_t * p_evt)
{
    ble_alarm_c_evt_t alarm_c_evt;
    memset(&alarm_c_evt,0,sizeof(ble_alarm_c_evt_t));

    ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

    // Check if the NUS was discovered.
    if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
        &&  (p_evt->params.discovered_db.srv_uuid.uuid == CUSTOM_SERVICE_CENTRAL_UUID)
        &&  (p_evt->params.discovered_db.srv_uuid.type == p_ble_alarm_c->uuid_type))
    {
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            switch (p_chars[i].characteristic.uuid.uuid)
            {
								case ALARM_TX_VALUE_CHAR_UUID:
										alarm_c_evt.handles.nus_tx_handle = p_chars[i].characteristic.handle_value;
										alarm_c_evt.handles.nus_tx_cccd_handle = p_chars[i].cccd_handle;
                    break;
								
								case ALARM_RX_VALUE_CHAR_UUID:
										alarm_c_evt.handles.nus_rx_handle = p_chars[i].characteristic.handle_value;
										break;
								
                default:
                    break;
            }
        }
        if (p_ble_alarm_c->evt_handler != NULL)
        {
            alarm_c_evt.conn_handle = p_evt->conn_handle;
            alarm_c_evt.evt_type    = BLE_ALARM_C_EVT_DISCOVERY_COMPLETE;
            p_ble_alarm_c->evt_handler(p_ble_alarm_c, &alarm_c_evt);
        }
    }
}

/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will uses the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the NUS TX characteristic from the peer. If
 *            it is, this function will decode the data and send it to the
 *            application.
 *
 * @param[in] p_ble_nus_c Pointer to the NUS Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
//static void on_notify(ble_alarm_c_t * p_ble_alarm_c, ble_evt_t const * p_ble_evt)
//{
//    // HVX can only occur from client sending.
//    if (   (p_ble_alarm_c->handles.nus_tx_handle != BLE_GATT_HANDLE_INVALID)
//        && (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_alarm_c->handles.nus_tx_handle)
//        && (p_ble_alarm_c->evt_handler != NULL))
//    {
//        ble_alarm_c_evt_t ble_alarm_c_evt;

//        ble_alarm_c_evt.evt_type = BLE_ALARM_C_EVT_NUS_TX_EVT;
//        ble_alarm_c_evt.p_data   = (uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
//        ble_alarm_c_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;

//        p_ble_alarm_c->evt_handler(p_ble_alarm_c, &ble_alarm_c_evt);
//        NRF_LOG_INFO("Client sending data.");
//    }
//}

/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function handles the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the NUS module, it is used to update
 *            internal variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the NUS client structure.
 */
void ble_alarm_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_alarm_c_t * p_ble_alarm_c = (ble_alarm_c_t *)p_context;

    if ((p_ble_alarm_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    if ( (p_ble_alarm_c->conn_handle != BLE_CONN_HANDLE_INVALID)
       &&(p_ble_alarm_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
       )
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
//						on_notify(p_ble_alarm_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_alarm_c->conn_handle
                    && p_ble_alarm_c->evt_handler != NULL)
            {
                ble_alarm_c_evt_t nus_c_evt;

                nus_c_evt.evt_type = BLE_ALARM_C_EVT_DISCONNECTED;

                p_ble_alarm_c->conn_handle = BLE_CONN_HANDLE_INVALID;
                p_ble_alarm_c->evt_handler(p_ble_alarm_c, &nus_c_evt);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t cccd_handle, bool enable)
{
    uint8_t buf[BLE_CCCD_VALUE_LEN];

    buf[0] = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    buf[1] = 0;

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = cccd_handle,
        .offset   = 0,
        .len      = sizeof(buf),
        .p_value  = buf
    };

    return sd_ble_gattc_write(conn_handle, &write_params);
}
/**@brief   Function for requesting the peer to start sending notification of TX characteristic.
 *
 * @details This function enables notifications of the NUS TX characteristic at the peer
 *          by writing to the CCCD of the NUS TX characteristic.
 *
 * @param   p_ble_nus_c Pointer to the NUS client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code is returned. This function propagates the error
 *                      code returned by the SoftDevice API @ref sd_ble_gattc_write.
 */
uint32_t ble_alarm_c_tx_notif_enable(ble_alarm_c_t * p_ble_alarm_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_alarm_c);

    if ( (p_ble_alarm_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_alarm_c->handles.nus_tx_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_alarm_c->conn_handle,p_ble_alarm_c->handles.nus_tx_cccd_handle, true);
}

/**@brief Function for sending a string to the server.
 *
 * @details This function writes the RX characteristic of the server.
 *
 * @param[in] p_ble_nus_c Pointer to the NUS client structure.
 * @param[in] p_string    String to be sent.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_alarm_c_string_send(ble_alarm_c_t * p_ble_alarm_c, uint8_t * p_string, uint16_t length)
{
    VERIFY_PARAM_NOT_NULL(p_ble_alarm_c);

    if (length > BLE_NUS_MAX_DATA_LEN)
    {
        NRF_LOG_INFO("Content too long.");
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_ble_alarm_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_INFO("Connection handle invalid.");
        return NRF_ERROR_INVALID_STATE;
    }

    ble_gattc_write_params_t const write_params =
    {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = p_ble_alarm_c->handles.nus_rx_handle,
        .offset   = 0,
        .len      = length,
        .p_value  = p_string
    };

    return sd_ble_gattc_write(p_ble_alarm_c->conn_handle, &write_params);
}

/**@brief Function for assigning handles to a this instance of nus_c.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate this link to this instance of the module. This makes it
 *          possible to handle several link and associate each link to a particular
 *          instance of this module. The connection handle and attribute handles will be
 *          provided from the discovery event @ref BLE_NUS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_nus_c    Pointer to the NUS client structure instance to associate with these
 *                           handles.
 * @param[in] conn_handle    Connection handle to associated with the given NUS Instance.
 * @param[in] p_peer_handles Attribute handles on the NUS server that you want this NUS client to
 *                           interact with.
 *
 * @retval    NRF_SUCCESS    If the operation was successful.
 * @retval    NRF_ERROR_NULL If a p_nus was a NULL pointer.
 */
uint32_t ble_alarm_c_handles_assign(ble_alarm_c_t               * p_ble_alarm_c,
                                  uint16_t                      conn_handle,
                                  ble_alarm_c_handles_t const * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_alarm_c);

    p_ble_alarm_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
				p_ble_alarm_c->handles.nus_tx_handle			= p_peer_handles->nus_tx_handle;
        p_ble_alarm_c->handles.nus_tx_cccd_handle = p_peer_handles->nus_tx_cccd_handle;
        p_ble_alarm_c->handles.nus_rx_handle      = p_peer_handles->nus_rx_handle;
    }
    return NRF_SUCCESS;
}
