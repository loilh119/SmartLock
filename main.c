#include "sdk_config.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "app_util.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_conn_state.h"
#include "fds.h"
#include "nrf_crypto.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "ble_sml.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_alarm_c.h"
#include "SSD1306.h"
#include "nrf_drv_gpiote.h"
#include "app_uart.h"
#include "nrf_uart.h"
#include "R301.h"
#include "nrf_drv_clock.h"

#define LESC_DEBUG_MODE                 0                                               /**< Set to 1 to use the LESC debug keys. The debug mode allows you to use a sniffer to inspect traffic. */
#define LESC_MITM_NC                    1                                               /**< Use MITM (Numeric Comparison). */

/** @brief The maximum number of peripheral and central links combined. */
#define NRF_BLE_LINK_COUNT              (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + NRF_SDH_BLE_CENTRAL_LINK_COUNT)

#define APP_BLE_CONN_CFG_TAG            1                                               /**< Tag that identifies the SoftDevice BLE configuration. */

#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1
#define PERIPHERAL_ADVERTISING_LED      BSP_BOARD_LED_2
#define PERIPHERAL_CONNECTED_LED        BSP_BOARD_LED_3

#define SCAN_DURATION                   0x0000                                          /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning continues until it is explicitly disabled. */
#define APP_ADV_DURATION                18000                                           /**< The advertising duration (180 seconds) in units of 10 milliseconds. */


#define SEC_PARAMS_BOND                 1                                               /**< Perform bonding. */
#if LESC_MITM_NC
#define SEC_PARAMS_MITM                 1                                               /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES      BLE_GAP_IO_CAPS_DISPLAY_YESNO                   /**< Display Yes/No to force Numeric Comparison. */
#else
#define SEC_PARAMS_MITM                 0                                               /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES      BLE_GAP_IO_CAPS_NONE                            /**< No I/O caps. */
#endif
#define SEC_PARAMS_LESC                 1                                               /**< LE Secure Connections pairing required. */
#define SEC_PARAMS_KEYPRESS             0                                               /**< Keypress notifications not required. */
#define SEC_PARAMS_OOB                  0                                               /**< Out Of Band data not available. */
#define SEC_PARAMS_MIN_KEY_SIZE         7                                               /**< Minimum encryption key size in octets. */
#define SEC_PARAMS_MAX_KEY_SIZE         16                                              /**< Maximum encryption key size in octets. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                           /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                          /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                               /**< Number of attempts before giving up the connection parameter negotiation. */
#define UART_TIMER					            APP_TIMER_TICKS(200)
#define LOCK_TIMER					            APP_TIMER_TICKS(4000)
#define DELAY_TIMER					            APP_TIMER_TICKS(1000)
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
/**@brief   Priority of the application BLE event handler.
 * @note    There is no need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO           3


typedef struct
{
    bool           is_connected;
    ble_gap_addr_t address;
} conn_peer_t;

static int status_scan_finger = 0;
static int status_confirm_finger = 0;
static volatile bool scan_int_act = true;
static volatile bool scan_finger_act = true;
static volatile bool delete_finger_act = true;
static volatile bool phone_lock = false;

static int check = 0;
static int enable_int = 1;

NRF_BLE_GATT_DEF(m_gatt);                                                   /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                      /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                         /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                            /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                                   /**< Scanning Module instance. */
BLE_SML_DEF(m_sml);
BLE_ALARM_C_DEF(m_alarm_c);
APP_TIMER_DEF(scan_finger);
APP_TIMER_DEF(scan_int);
APP_TIMER_DEF(delete_finger);
APP_TIMER_DEF(lock);
APP_TIMER_DEF(delay_timer);
static uint16_t           m_conn_handle_hrs_c                = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for the HRS central application. */
static volatile uint16_t  m_conn_handle_num_comp_central     = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for the central that needs a numeric comparison button press. */
static volatile uint16_t  m_conn_handle_num_comp_peripheral  = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for the peripheral that needs a numeric comparison button press. */

static conn_peer_t        m_connected_peers[NRF_BLE_LINK_COUNT];                         /**< Array of connected peers. */

static char * roles_str[] =
{
    "INVALID_ROLE",
    "PERIPHERAL",
    "CENTRAL",
};

/**@brief Names that the central application scans for, and that are advertised by the peripherals.
 *  If these are set to empty strings, the UUIDs defined below are used.
 */
static const char m_target_periph_name[] = "Alarm";


/**@brief UUIDs that the central application scans for if the name above is set to an empty string,
 * and that are to be advertised by the peripherals.
 */
static ble_uuid_t m_adv_uuids[] = {{CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}};
                                   
/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code that contains information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

void uart_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }else if (p_event->evt_type == APP_UART_DATA_READY)
		{
			handler_uart();
		}
}

void init_uart()
{
	  uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_handle,
                       APP_IRQ_PRIORITY_HIGH,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    ble_uuid_t          target_uuid = 
    {
        .uuid = CUSTOM_SERVICE_CENTRAL_UUID,
        .type = BLE_UUID_TYPE_VENDOR_BEGIN
    };
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    if (strlen(m_target_periph_name) != 0)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, 
                                           SCAN_NAME_FILTER, 
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_ble_scan_filter_set(&m_scan, 
                                       SCAN_UUID_FILTER, 
                                       &target_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, 
                                           NRF_BLE_SCAN_NAME_FILTER | NRF_BLE_SCAN_UUID_FILTER, 
                                           false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Scanning");
}


/**@brief Function for initializing the advertising and the scanning.
 */
static void adv_scan_start(void)
{
    ret_code_t err_code;

    scan_start();

    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);

    // Start advertising.
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Advertising");
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            adv_scan_start();
            break;

        default:
            break;
    }
}


/**@brief Function for changing filter settings after establishing the connection.
 */
static void filter_settings_change(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_all_filter_remove(&m_scan);
    APP_ERROR_CHECK(err_code);

    if (strlen(m_target_periph_name) != 0)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, 
                                           SCAN_NAME_FILTER, 
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_alarm_chars_received(uint8_t * p_data, uint16_t data_len)
{
    NRF_LOG_INFO("Receiving data.");
		nrf_gpio_pin_toggle(LED_4);		
		
}
 /**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void alarm_c_evt_handler(ble_alarm_c_t * p_alarm_c, ble_alarm_c_evt_t * p_alarm_c_evt)
{
		ret_code_t err_code;
		switch(p_alarm_c_evt->evt_type)
		{
			case BLE_ALARM_C_EVT_DISCOVERY_COMPLETE:
				NRF_LOG_INFO("Discovery complete.");
				
				filter_settings_change();
			
				err_code = ble_alarm_c_handles_assign(p_alarm_c, p_alarm_c_evt->conn_handle, &p_alarm_c_evt->handles);
				APP_ERROR_CHECK(err_code);
				
//				err_code = ble_alarm_c_tx_notif_enable(p_alarm_c);
//				APP_ERROR_CHECK(err_code);
				NRF_LOG_INFO("Connected to device");
				break;
			case BLE_ALARM_C_EVT_NUS_TX_EVT:
				ble_alarm_chars_received(p_alarm_c_evt->p_data, p_alarm_c_evt->data_len);
				break;
			case BLE_ALARM_C_EVT_DISCONNECTED:
				NRF_LOG_INFO("Disconnected.");
				scan_start();
				break;
			default:
				break;
		}
}
/**@brief Function for checking whether a link already exists with a newly connected peer.
 *
 * @details This function checks whether the newly connected device is already connected.
 *
 * @param[in]   p_connected_evt Bluetooth connected event.
 * @return                      True if the peer's address is found in the list of connected peers,
 *                              false otherwise.
 */
static bool is_already_connected(ble_gap_addr_t const * p_connected_adr)
{
    for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++)
    {
        if (m_connected_peers[i].is_connected)
        {
            if (m_connected_peers[i].address.addr_type == p_connected_adr->addr_type)
            {
                if (memcmp(m_connected_peers[i].address.addr,
                           p_connected_adr->addr,
                           sizeof(m_connected_peers[i].address.addr)) == 0)
                {
                    return true;
                }
            }
        }
    }
    return false;
}


/** @brief Function for handling a numeric comparison match request. */
static void on_match_request(uint16_t conn_handle, uint8_t role)
{
    // Mark the appropriate conn_handle as pending. The rest is handled on button press.
    NRF_LOG_INFO("Press Button 1 to confirm, Button 2 to reject");
    if (role == BLE_GAP_ROLE_CENTRAL)
    {
        m_conn_handle_num_comp_central = conn_handle;
    }
    else if (role == BLE_GAP_ROLE_PERIPH)
    {
        m_conn_handle_num_comp_peripheral = conn_handle;
    }
}





/**@brief Function for assigning new connection handle to the available instance of QWR module.
 *
 * @param[in] conn_handle New connection handle.
 */
static void multi_qwr_conn_handle_assign(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }
}


/**@brief Function for handling BLE Stack events that are common to both the central and peripheral roles.
 * @param[in] conn_handle Connection Handle.
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(uint16_t conn_handle, ble_evt_t const * p_ble_evt)
{
    uint8_t     passkey[BLE_GAP_PASSKEY_LEN + 1];
    uint16_t    role = ble_conn_state_role(conn_handle);

    pm_handler_secure_on_connection(p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_connected_peers[conn_handle].is_connected = true;
            m_connected_peers[conn_handle].address = p_ble_evt->evt.gap_evt.params.connected.peer_addr;
            multi_qwr_conn_handle_assign(conn_handle);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            memset(&m_connected_peers[conn_handle], 0x00, sizeof(m_connected_peers[0]));
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_INFO("%s: BLE_GAP_EVT_SEC_PARAMS_REQUEST", nrf_log_push(roles_str[role]));
            break;

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
            memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, BLE_GAP_PASSKEY_LEN);
            passkey[BLE_GAP_PASSKEY_LEN] = 0x00;
            NRF_LOG_INFO("%s: BLE_GAP_EVT_PASSKEY_DISPLAY: passkey=%s match_req=%d",
                         nrf_log_push(roles_str[role]),
                         nrf_log_push((char*)passkey),
                         p_ble_evt->evt.gap_evt.params.passkey_display.match_request);
												 
//						ssd1306_clear_screen(0x00);
//						ssd1306_display_string(32, 14, "PASS KEY", 16, 1);						 
//						ssd1306_display_string(37, 31, (char*)passkey, 16, 1);
//						ssd1306_refresh_gram();
												 
            if (p_ble_evt->evt.gap_evt.params.passkey_display.match_request)
            {
                on_match_request(conn_handle, role);
            }
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_KEY_REQUEST", nrf_log_push(roles_str[role]));
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("%s: BLE_GAP_EVT_LESC_DHKEY_REQUEST", nrf_log_push(roles_str[role]));
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
					 				 
             NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          nrf_log_push(roles_str[role]),
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            ret_code_t err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE Stack events that are related to central application.
 *
 * @details This function keeps the connection handles of central application up-to-date. It
 * parses scanning reports, initiates a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report the central application's activity.
 *
 * @note        Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *              must be dispatched to the target application before invoking this function.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    ret_code_t            err_code;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral is connected (HR or RSC), initiate DB
        //  discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("CENTRAL: Connected, handle: %d.", p_gap_evt->conn_handle);
            // If no Heart Rate Sensor is currently connected, try to find them on this peripheral.
            if (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID)
            {
                NRF_LOG_INFO("CENTRAL: Searching for Client on conn_handle 0x%x", p_gap_evt->conn_handle);

                err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);
            }
            // Update status of LEDs.
            bsp_board_led_off(CENTRAL_SCANNING_LED);
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the status of LEDs, and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("CENTRAL: Disconnected, handle: %d, reason: 0x%x",
                         p_gap_evt->conn_handle,
                       p_gap_evt->params.disconnected.reason);

            // Update the status of LEDs.
            bsp_board_led_off(CENTRAL_CONNECTED_LED);
            bsp_board_led_on(CENTRAL_SCANNING_LED);

            if (p_gap_evt->conn_handle == m_conn_handle_hrs_c)
            {
                ble_uuid_t target_uuid = {.uuid = BLE_UUID_HEART_RATE_SERVICE, .type = BLE_UUID_TYPE_BLE};
                m_conn_handle_hrs_c    = BLE_CONN_HANDLE_INVALID;

                err_code = nrf_ble_scan_filter_set(&m_scan, 
                                                   SCAN_UUID_FILTER, 
                                                   &target_uuid);
                APP_ERROR_CHECK(err_code);
            }
            
            scan_start();
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_TIMEOUT:
        {
            // Timeout for scanning is not specified, so only connection attemps can time out.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("CENTRAL: Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("CENTRAL: GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("CENTRAL: GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE Stack events that involves peripheral applications. Manages the
 * LEDs used to report the status of the peripheral applications.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("PERIPHERAL: Connected, handle %d.", p_ble_evt->evt.gap_evt.conn_handle);
            bsp_board_led_off(PERIPHERAL_ADVERTISING_LED);
            bsp_board_led_on(PERIPHERAL_CONNECTED_LED);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("PERIPHERAL: Disconnected, handle %d, reason 0x%x.",
                         p_ble_evt->evt.gap_evt.conn_handle,
                         p_ble_evt->evt.gap_evt.params.disconnected.reason);
            // LED indication will be changed when advertising starts.
        break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("PERIPHERAL: GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("PERIPHERAL: GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            bsp_board_led_on(PERIPHERAL_ADVERTISING_LED);
            bsp_board_led_off(PERIPHERAL_CONNECTED_LED);
            break;

        case BLE_ADV_EVT_IDLE:
        {
            ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role        = ble_conn_state_role(conn_handle);

    if (    (p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
        &&  (is_already_connected(&p_ble_evt->evt.gap_evt.params.connected.peer_addr)))
    {
        NRF_LOG_INFO("%s: Already connected to this device as %s (handle: %d), disconnecting.",
                     (role == BLE_GAP_ROLE_PERIPH) ? "PERIPHERAL" : "CENTRAL",
                     (role == BLE_GAP_ROLE_PERIPH) ? "CENTRAL"    : "PERIPHERAL",
                     conn_handle);

        (void)sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

        // Do not process the event further.
        return;
    }

    on_ble_evt(conn_handle, p_ble_evt);

    if (role == BLE_GAP_ROLE_PERIPH)
    {
        // Manages peripheral LEDs.
        on_ble_peripheral_evt(p_ble_evt);
    }
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {
        on_ble_central_evt(p_ble_evt);
    }
}

static void alarm_c_init(void)
{
		ret_code_t					err_code;
		ble_alarm_c_init_t 	init;
		
		init.evt_handler = alarm_c_evt_handler;
	
		err_code = ble_alarm_c_init(&m_alarm_c, &init);
		APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

     // Configure the BLE stack by using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the Peer Manager. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_params;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_params, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_params.bond           = SEC_PARAMS_BOND;
    sec_params.mitm           = SEC_PARAMS_MITM;
    sec_params.lesc           = SEC_PARAMS_LESC;
    sec_params.keypress       = SEC_PARAMS_KEYPRESS;
    sec_params.io_caps        = SEC_PARAMS_IO_CAPABILITIES;
    sec_params.oob            = SEC_PARAMS_OOB;
    sec_params.min_key_size   = SEC_PARAMS_MIN_KEY_SIZE;
    sec_params.max_key_size   = SEC_PARAMS_MAX_KEY_SIZE;
    sec_params.kdist_own.enc  = 1;
    sec_params.kdist_own.id   = 1;
    sec_params.kdist_peer.enc = 1;
    sec_params.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_params);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Delete all data stored for all peers. */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for accepting or rejecting a numeric comparison. */
static void num_comp_reply(uint16_t conn_handle, bool accept)
{
    uint8_t    key_type;
    ret_code_t err_code;

    if (accept)
    {
        NRF_LOG_INFO("Numeric Match. Conn handle: %d", conn_handle);
        key_type = BLE_GAP_AUTH_KEY_TYPE_PASSKEY;
    }
    else
    {
        NRF_LOG_INFO("Numeric REJECT. Conn handle: %d", conn_handle);
        key_type = BLE_GAP_AUTH_KEY_TYPE_NONE;
    }

    err_code = sd_ble_gap_auth_key_reply(conn_handle,
                                         key_type,
                                         NULL);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for handling button presses for numeric comparison match requests. */
static void on_num_comp_button_press(bool accept)
{
    // Check whether any links have pending match requests, and if so, send a reply.
    if (m_conn_handle_num_comp_central != BLE_CONN_HANDLE_INVALID)
    {
        num_comp_reply(m_conn_handle_num_comp_central, accept);
        m_conn_handle_num_comp_central = BLE_CONN_HANDLE_INVALID;
				
//				if(accept == true)
//				{
//						ssd1306_clear_screen(0x00);
//						ssd1306_display_string(32, 20, "ACCEPT", 16, 1);						 
//						ssd1306_refresh_gram();
//						ssd1306_clear_screen(0x00);
//				}
//				else if(accept == false)
//				{
//						ssd1306_clear_screen(0x00);
//						ssd1306_display_string(32, 20, "REJECT", 16, 1);						 
//						ssd1306_refresh_gram();
//						ssd1306_clear_screen(0x00);
//				}
    }
    else if (m_conn_handle_num_comp_peripheral != BLE_CONN_HANDLE_INVALID)
    {
//			NRF_LOG_INFO("debug 1");
        num_comp_reply(m_conn_handle_num_comp_peripheral, accept);
//			NRF_LOG_INFO("debug 2");
        m_conn_handle_num_comp_peripheral = BLE_CONN_HANDLE_INVALID;
//			NRF_LOG_INFO("debug 3");
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
		NRF_LOG_INFO("num event %d", event);
    switch (event)
    {
        case BSP_EVENT_KEY_0:
          on_num_comp_button_press(true);
          break;

      case BSP_EVENT_KEY_1:
					on_num_comp_button_press(false);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  True if the clear bonding button is pressed to
 *                            wake the application up. False otherwise.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;
	
//		nrf_gpio_cfg_output(LED_3);
//		nrf_gpio_cfg_output(LED_2);
//		nrf_gpio_cfg_output(LED_4);
	
		nrf_gpio_cfg_input(25, NRF_GPIO_PIN_PULLDOWN);
	
		nrf_gpio_cfg_output(28);
		nrf_gpio_pin_clear(28);
	
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the GAP.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device, including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params = m_scan.conn_params;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function is passed to each service that may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Queued Write instances.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;  // Ignore events.
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
		ble_alarm_c_on_db_disc_evt(&m_alarm_c, p_evt);
}


/**@brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the advertising functionality. */
static void advertising_init(void)
{		
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

		init.srdata.name_type								 = BLE_ADVDATA_FULL_NAME;
		init.srdata.include_appearance			 = true;
		
		init.advdata.name_type							 = BLE_ADVDATA_FULL_NAME;
		init.advdata.include_appearance			 = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
		
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}



static void scan_finger_handler(void * p_context)
{
		ret_code_t err_code;
    UNUSED_PARAMETER(p_context);		
		
		if(scan_finger_act == true)
		{
			if(check == 0)
			{
				switch(status_scan_finger)
				{
					case 0:	
						Collect_Finger();
						check = 1;
						break;
					
					case 1:
						Generate_Character(1);
						check = 1;
						break;
					
					case 2:
						Collect_Finger();
						check = 1;
						break;
					
					case 3:
						Generate_Character(2);
						check = 1;
						break;
					
					case 4:
						Generate_Template();
						check = 1;
						break;
					
					case 5:
						Store_Template(1, 1);
						check = 2;
						break;
					
					default:
						break;
				}
			}
			else if(check == 1)
			{
				switch(check_sum(4,12))
				{
					case 0x00:
						check = 0;
						status_scan_finger ++;
						break;
					
					case -1:
						break;
					
					default:
						check = 0;
						break;
				}
			}
			else if(check == 2)
			{
				switch(check_sum(4,12))
				{
					case 0x00:
						check = 0;
						
						status_scan_finger = 0;
					
						scan_int_act = true;
						delete_finger_act = true;
										
						err_code = app_timer_stop(scan_finger);
						APP_ERROR_CHECK(err_code);
						break;
					
					case -1:
						break;
					
					default:
						check = 0;
						break;
				}
			}
		}
}

static void delete_finger_handler(void * p_context)
{
		ret_code_t err_code;
    UNUSED_PARAMETER(p_context);
		
		if(delete_finger_act == true)
		{
			if(check == 0)
			{
				Library_Empty();
				check = 1;
			}
			else if(check == 1)
			{
				switch(check_sum(4,12))
				{
					case 0x00:
						
						scan_int_act = true;
						scan_finger_act = true;
					
						check = 0;
					
						err_code = app_timer_stop(delete_finger);
						APP_ERROR_CHECK(err_code);
						break;
					
					case -1:
						break;
					
					default:
						check = 0;
						break;
				}
			}
		}
}

static void delay_handler(void * p_context)
{
		ret_code_t err_code;
		
    UNUSED_PARAMETER(p_context);
	
		enable_int = 1;
		
		err_code = app_timer_stop(delay_timer);
		APP_ERROR_CHECK(err_code);	
}

static void lock_handler(void * p_context)
{
		ret_code_t err_code;
    UNUSED_PARAMETER(p_context);	
		if(phone_lock == true)
		{
			char *a = "ULCK";
			char *c = "ERRO";
			if(nrf_gpio_pin_read(25) == 0)
			{
					ble_sml_custom_value_update(&m_sml, a);	
			}
			else
			{
					ble_sml_custom_value_update(&m_sml, c);	
			}
		}
		
		nrf_gpio_pin_clear(28);
		
		err_code = app_timer_stop(lock);
		APP_ERROR_CHECK(err_code);	
		
		err_code = app_timer_start(delay_timer, DELAY_TIMER, NULL);
		APP_ERROR_CHECK(err_code);
}

static void scan_int_handler(void * p_context)
{
		ret_code_t err_code;  
		UNUSED_PARAMETER(p_context);	
	
		if(scan_int_act == true)
		{
			if(check == 0)
			{
				switch(status_confirm_finger)
				{
					case 0:	
						Collect_Finger();
						check = 1;
						break;
					case 1:
						Generate_Character(1);
						check = 1;
						break;
					case 2:
						Search_Finger(1, 15);
						check = 2;
						break;		
					default:
						break;
				}
			}
			else if(check == 1)
			{
				switch(check_sum(4,12))
				{
					case 0x00:
						check = 0;
						status_confirm_finger ++;
						break;
					
					case -1:
						break;
					
					default:
						check = 0;
						break;
				}
			}
			else if(check == 2)
			{
				switch(check_sum(8,16))
				{
					case 0x00:
						check = 0;
						
						status_confirm_finger = 0;
					
						scan_int_act = true;
						scan_finger_act = true;
						delete_finger_act = true;
						
						nrf_gpio_pin_set(28);
						m_sml.lock_status = LOCK_OPEN;
					
						phone_lock = false;
					
						err_code = app_timer_stop(scan_int);
						APP_ERROR_CHECK(err_code);

						err_code = app_timer_start(lock, LOCK_TIMER, NULL);
						APP_ERROR_CHECK(err_code);
						
						break;
					
					case 0x09:
						check = 0;
						
						status_confirm_finger = 0;
					
						scan_int_act = true;
						scan_finger_act = true;
						delete_finger_act = true;
						
						enable_int = 1;
					
						m_sml.lock_status = LOCK_CLOSE;
					
						err_code = app_timer_stop(scan_int);
						APP_ERROR_CHECK(err_code);
						break;
					
					case -1:
						break;
					
					default:
						check = 0;
						break;
				}
			}
		}
}

static void lfclk_request(void)
{
    uint32_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&scan_finger, APP_TIMER_MODE_REPEATED, scan_finger_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&delete_finger, APP_TIMER_MODE_REPEATED, delete_finger_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&scan_int, APP_TIMER_MODE_REPEATED, scan_int_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&lock, APP_TIMER_MODE_REPEATED, lock_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&delay_timer, APP_TIMER_MODE_REPEATED, delay_handler);
    APP_ERROR_CHECK(err_code);
	
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log or key operations, or both, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;
    
    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);
    
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void on_sml_evt(ble_sml_t     * p_sml_service,
                       ble_sml_evt_t * p_evt)
{
		ret_code_t err_code;
    switch(p_evt->evt_type)
    {
        case BLE_SML_EVT_CONNECTED:
//						ssd1306_clear_screen(0x00);
//						ssd1306_display_string(25, 20, "CONNECTING", 16, 1);						 
//						ssd1306_refresh_gram();
//						ssd1306_clear_screen(0x00);
            break;

        case BLE_SML_EVT_DISCONNECTED:
//						ssd1306_clear_screen(0x00);
//						ssd1306_display_string(25, 20, "DISCONNECT", 16, 1);	
//						ssd1306_refresh_gram();
//						ssd1306_clear_screen(0x00);
            break;
				
				case BLE_SML_EVT_NOTIFICATION_ENABLED:
            break;

        case BLE_SML_EVT_NOTIFICATION_DISABLED:
            break;
				
				case BLE_SML_EVT_LOCK_OPEN:
					
					phone_lock = true;
				
					nrf_gpio_pin_set(28);					

					err_code = app_timer_start(lock, LOCK_TIMER, NULL);
					APP_ERROR_CHECK(err_code);

					break;
				
				case BLE_SML_EVT_FINGER:
					
					scan_int_act = false;
					delete_finger_act = false;
				
					check = 0;
				
					status_scan_finger = 0;
				
					err_code = app_timer_start(scan_finger, UART_TIMER, NULL);
					APP_ERROR_CHECK(err_code);
					break;
				case BLE_SML_EVT_DELETE_FINGER:	
					
					scan_int_act = false;
					scan_finger_act = false;
					
					check = 0;
				
					err_code = app_timer_start(delete_finger, UART_TIMER, NULL);
					APP_ERROR_CHECK(err_code);
					break;
				
				case BLE_SML_EVT_CANCEL:
					
					scan_int_act = true;
					scan_finger_act = true;
					delete_finger_act = true;
				
					err_code = app_timer_stop_all();
					APP_ERROR_CHECK(err_code);
					break;
        default:
              // No implementation needed.
              break;
    }
}

static void services_init(void)
{
    /* YOUR_JOB: Add code to initialize the services used by the application.*/
    ret_code_t                         err_code;
    ble_sml_init_t                     sml_init;
		
		sml_init.evt_handler                = on_sml_evt;
    // Initialize CUS Service init structure to zero.
    memset(&sml_init, 0, sizeof(sml_init));
		
		sml_init.evt_handler                = on_sml_evt;
	
//		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sml_init.custom_value_char_attr_md.read_perm);
//		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sml_init.custom_value_char_attr_md.write_perm);
		
		BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&sml_init.custom_value_char_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&sml_init.custom_value_char_attr_md.write_perm);
	
    err_code = ble_sml_init(&m_sml, &sml_init);
    APP_ERROR_CHECK(err_code);	
}

void finger_scan(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	ret_code_t err_code;
	nrf_gpio_pin_toggle(LED_2);

	if(nrf_gpio_pin_read(4) == 0)
	{
		if(scan_int_act == true)
		{
			if(enable_int == 1)
			{
				nrf_gpio_pin_toggle(LED_4);
				
				status_confirm_finger = 0;
				
				enable_int = 0;
				
				check = 0;
				
				scan_finger_act = false;
				delete_finger_act = false;
				
				err_code = app_timer_start(scan_int, APP_TIMER_TICKS(300), NULL);
				APP_ERROR_CHECK(err_code);
			}
		}
	}
}

void sensor(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint8_t a[1] = {0x55};
	if(nrf_gpio_pin_read(29) == 1 && m_sml.lock_status == LOCK_CLOSE)
	{
		ble_alarm_c_string_send(&m_alarm_c, a, 1);
	}
}
/**

 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;
	
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(LED_4, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    err_code = nrf_drv_gpiote_in_init(4, &in_config, finger_scan);
    APP_ERROR_CHECK(err_code);
		
    nrf_drv_gpiote_in_event_enable(4, true);
		
		nrf_drv_gpiote_in_config_t in_config1 = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
		
		err_code = nrf_drv_gpiote_in_init(29, &in_config1, sensor);
    APP_ERROR_CHECK(err_code);
		
    nrf_drv_gpiote_in_event_enable(29, true);
}


int main(void)
{
    bool erase_bonds;
    // Initialize.
//		twi_init();
//		ssd1306_init(); 
//		ssd1306_clear_screen(0x00);
//		ssd1306_display_string(25, 20, "SMART LOCK", 16, 1);
//		ssd1306_refresh_gram();
	
		lfclk_request();
    timer_init();
	
		init_uart();
    log_init();
		gpio_init();	
		
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
		
		scan_init();
		gap_params_init();
    gatt_init();
    conn_params_init();
    db_discovery_init();
    qwr_init();
		services_init();
		
    peer_manager_init();
    advertising_init();
		
		alarm_c_init();
    // Start execution.
    NRF_LOG_INFO("LE Secure Connections example started.");

    if (erase_bonds == true)
    {
        delete_bonds();
        // Scanning and advertising is started by PM_EVT_PEERS_DELETE_SUCEEDED.
    }
    else
    {
        adv_scan_start();
    }

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
