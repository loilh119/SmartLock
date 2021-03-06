#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "ble_link_ctx_manager.h"

#define SERVICE_UUID_BASE         			{0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, \
																				0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}
#define SERVICE_UUID               			0x1405
#define VALUE_CHAR_LOCK_UUID          	0x1401
#define VALUE_CHAR_STATUS_UUID          0x1402
#define VALUE_CHAR_FINGER_UUID          0x1403																		
/**@brief   Macro for defining a ble_hrs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_SML_DEF(_name)                                                                          \
static ble_sml_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_sml_on_ble_evt, &_name)

typedef enum
{
	LOCK_OPEN,
	LOCK_CLOSE
}lock_sta;

// Forward declaration of the ble_sml_t type.
typedef struct ble_sml_s ble_sml_t;

typedef enum
{
	BLE_SML_EVT_COMM_STARTED,
	BLE_SML_EVT,
	BLE_SML_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
  BLE_SML_EVT_NOTIFICATION_DISABLED,
	BLE_SML_EVT_CONNECTED,
	BLE_SML_EVT_DISCONNECTED,
	BLE_SML_EVT_LOCK_OPEN,
	BLE_SML_EVT_FINGER,
	BLE_SML_EVT_DELETE_FINGER,
	BLE_SML_EVT_CANCEL
} ble_sml_evt_type_t;

/**@brief   Nordic SML Service @ref BLE_EVT_SML_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_SML_EVT occurs.
 */
typedef struct
{
    uint8_t const * p_data; /**< A pointer to the buffer with received data. */
    uint16_t        length; /**< Length of received data. */
} ble_evt_sml_data_t;

typedef struct
{
    ble_sml_evt_type_t 				 evt_type;    /**< Type of event. */
    ble_sml_t                * p_sml;       /**< A pointer to the instance. */
    uint16_t                   conn_handle; /**< Connection handle. */
    union
    {
        ble_evt_sml_data_t		 sml_data; /**< @ref BLE_NUS_EVT_RX_DATA event data. */
    } params;
} ble_sml_evt_t;
							
typedef void (*ble_sml_evt_handler_t) (ble_sml_t * p_sml, ble_sml_evt_t * p_evt);

/**@brief Heart Rate Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
		ble_sml_evt_handler_t					evt_handler;
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_sml_init_t;

/**@brief Smart Lock Service structure. This contains various status information for the service. */
struct ble_sml_s
{
		ble_sml_evt_handler_t					evt_handler;
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      lock_control_handle;          	/**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      lock_status_handle;           	/**< Handles related to the Custom Value characteristic. */
		ble_gatts_char_handles_t      finger_print_handle;
		uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
		lock_sta											lock_status;
	
		blcm_link_ctx_storage_t * const p_link_ctx_storage; /**< Pointer to link context storage with handles of all current connections and its context. */
};

/**@brief Function for initializing the Heart Rate Service.
 *
 * @param[out]  p_hrs       Heart Rate Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_hrs_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_sml_init(ble_sml_t * p_sml, ble_sml_init_t const * p_sml_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Heart Rate Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Heart Rate Service structure.
 */
void ble_sml_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_sml_custom_value_update(ble_sml_t * p_sml, uint8_t * custom_value, uint8_t length);


