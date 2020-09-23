/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_nus_c.h"
#include "ble_nus.h"

#include "nordic_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf_strerror.h"
#include "bsp_btn_ble.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

#include "m_ucp_flash.h"


#include "nrf_power.h"

#define UART_TX_BUF_SIZE            512                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE            512                           /**< UART RX buffer size. */

#define MIN_CONN_INTERVAL    MSEC_TO_UNITS(80, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL    MSEC_TO_UNITS(150, UNIT_1_25_MS)
#define SUP_TIMEOUT		 MSEC_TO_UNITS(4000, UNIT_10_MS)

#define APP_ADV_INTERVAL                 MSEC_TO_UNITS(40, UNIT_0_625_MS)                                        /**< The advertising interval (in units of 0.625 ms). This value corresponds to 25 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS       0   /**< The advertising timeout in units of seconds. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000)                     /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */
		

#define APP_FEATURE_NOT_SUPPORTED   BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                              /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */


#define SCAN_INTERVAL               MSEC_TO_UNITS(300, UNIT_0_625_MS)//320//0x00F0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 MSEC_TO_UNITS(40, UNIT_0_625_MS)//0x0014                             /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE                 1                                  /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE              0                                  /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT                0x0000                             /**< Timout when scanning. 0x0000 disables timeout. */
												
#define SCHED_MAX_EVENT_DATA_SIZE       256//AT_CMD_LENGTH//MAX(APP_TIMER_SCHED_EVT_SIZE, \
                                        //    BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE               5   
////////////Added for bonding///////////
#define APP_BLE_CONN_CFG_TAG                                                                       \
    1 /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. \                                                                                                 \
         Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO                                                                      \
    3 /**< Application's BLE observer priority. You shoulnd't need to modify this value. */

#define NUS_SERVICE_UUID_TYPE                                                                      \
    BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */


/**@brief   Macro for defining multiple ble_lbs_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 */
#define BLE_NUS_C_ARRAY_DEF(_name, _cnt)                                                           \
static ble_nus_c_t _name[_cnt];                                                                    \
NRF_SDH_BLE_OBSERVERS(                                                                             \
        _name##_obs, BLE_NUS_C_BLE_OBSERVER_PRIO, ble_nus_c_on_ble_evt, &_name, _cnt)

NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */     /**< BLE NUS service client instance. */
BLE_NUS_C_ARRAY_DEF(m_ble_nus_array_c, CENTRAL_LINK_COUNT); /**< client instances. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc,NRF_SDH_BLE_CENTRAL_LINK_COUNT); /**< Database discovery module instances. */

BLE_ADVERTISING_DEF(m_advertising);
static ble_nus_t m_nus;


volatile uint16_t     m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH;

static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {0x0001, NUS_SERVICE_UUID_TYPE}
};

ble_gap_conn_params_t m_connection_param;
ble_gap_scan_params_t m_scan_param;

void conn_timeout_timer_stop(void);
void conn_timeout_timer_start(void);
void ble_param_init(void);

/**@brief NUS uuid. */
static ble_uuid_t const m_nus_uuid = {.uuid = 0x0001, .type = NUS_SERVICE_UUID_TYPE};

/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}
/**@brief Function for initializing the timer. */
static void app_timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_DEBUG("call to ble_nus_on_db_disc_evt for instance %d and link 0x%x!",
        p_evt->conn_handle, p_evt->conn_handle);

    ble_nus_c_on_db_disc_evt(&m_ble_nus_array_c[p_evt->conn_handle], p_evt);
}

// ble_gap_addr_t m_peer_gap_addr = {0};
/**@brief Function for initiating scanning.
 */
uint32_t scan_start(void)
{
    ret_code_t err_code;
    (void)sd_ble_gap_scan_stop();
	m_scan_param.active = SCAN_ACTIVE;
	m_scan_param.interval = SCAN_INTERVAL;
	m_scan_param.window   = SCAN_WINDOW;
	m_scan_param.timeout  = SCAN_TIMEOUT;

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    if (err_code == NRF_ERROR_INVALID_PARAM)
    {
      
    }
      NRF_LOG_INFO("scan_start() err_code = %d\r\n",err_code);
    return NRF_SUCCESS;
}
///**@brief   Function for handling app_uart events.
// *
// * @details This function will receive a single character from the app_uart module and append it
// to
// *          a string. The string will be be sent over BLE when the last character received was a
// *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
// */
void uart_event_handle(app_uart_evt_t * p_event)
{
	static uint8_t receive_data;
    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&receive_data));
            NRF_LOG_INFO("Reveice UART data %x",receive_data);
            break;
        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            break;
        default:
            break;
    }
}

/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;
    uint8_t * data;
    uint16_t len;
	uint16_t length=0;
	NRF_LOG_INFO("central evt id =%d conn_handle=>%d",p_ble_nus_evt->evt_type,p_ble_nus_evt->conn_handle);
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:

            NRF_LOG_INFO("NUS service discovered on conn_handle 0x%x", p_ble_nus_evt->conn_handle);
             err_code = ble_nus_c_handles_assign(
             &m_ble_nus_array_c[p_ble_nus_evt->conn_handle], p_ble_nus_evt->conn_handle,  &p_ble_nus_evt->handles);
             err_code = ble_nus_c_tx_notif_enable(&m_ble_nus_array_c[p_ble_nus_evt->conn_handle]);
            if (err_code != NRF_ERROR_INVALID_STATE) // assert the link was disconnected
                APP_ERROR_CHECK(err_code);
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            {
				NRF_LOG_INFO("receive from handle=%d",p_ble_nus_evt->conn_handle);
                NRF_LOG_HEXDUMP_INFO(p_ble_nus_evt->p_data,p_ble_nus_evt->data_len);
            }
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
			 NRF_LOG_INFO("Disconnected.");
             scan_start();
            break;
        case BLE_NUS_C_EVT_DISCOVERY_NOT_FOUND:
			NRF_LOG_INFO("Service not found.");
            err_code = sd_ble_gap_disconnect(p_ble_nus_evt->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if(err_code != NRF_ERROR_INVALID_STATE)
              APP_ERROR_CHECK(err_code);
            
			scan_start();
            break;
        case BLE_NUS_C_EVT_WRITE_RSP:
			NRF_LOG_INFO("cccs resp");
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;


    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
             
            // Initiate connection.
//			uint32_t err_code;  
//            err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
//                &m_scan_param, &m_connection_param, APP_BLE_CONN_CFG_TAG);
//            if (err_code != NRF_SUCCESS)
//            {
//                NRF_LOG_ERROR("Connection Request Failed, reason %d\r\n", err_code);
//            }
            break;
        }
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.", p_gap_evt->conn_handle);
            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

            err_code = ble_nus_c_handles_assign(&m_ble_nus_array_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
			APP_ERROR_CHECK(err_code);
            break;
        }
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("peripheral 0x%x disconnected (reason: 0x%x)",p_gap_evt->conn_handle, p_gap_evt->params.disconnected.reason);
            scan_start();
        }
        break;
        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(
                p_gap_evt->conn_handle, &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys = {
                .rx_phys = BLE_GAP_PHY_AUTO, .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        }
        break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(
                p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(
                p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
            NRF_LOG_DEBUG("WRITE CMD TX COMPLETE.");
            break;
        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {

        }
        break;
        default:
            break;
    }
}

/**@brief   Function for handling BLE events from peripheral applications.
 * @details Updates the status LEDs used to report the activity of the peripheral applications.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    uint32_t    periph_link_cnt = ble_conn_state_n_peripherals(); // Number of peripheral links.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("slave Connected");
            if (periph_link_cnt != NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
            {
                err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("slave disconnected reason=%d",p_ble_evt->evt.gap_evt.params.disconnected.reason);
            if (periph_link_cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1))
            {
                // Advertising is not running when all connections are taken, and must therefore be
                // started.
                err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
                APP_ERROR_CHECK(err_code);
            }
            break;
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            NRF_LOG_DEBUG("HVN TX COMPLETE.");
            break;
#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys = {
                .rx_phys = BLE_GAP_PHY_AUTO, .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        }
        break;
#endif      
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(
                p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(
                p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            NRF_LOG_DEBUG("GATT Server attr missing.");
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(
                        p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        }
        break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST
        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            NRF_LOG_DEBUG("As Peripheral,CONN_PARAM_UPDATED:%4d, %4d, %4d, %4d\r",
                p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params
                    .max_conn_interval,
                p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params
                    .min_conn_interval,
                p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params
                    .conn_sup_timeout,
                p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params.slave_latency);
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for checking if a bluetooth stack event is an advertising timeout.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static bool ble_evt_is_advertising_timeout(ble_evt_t const * p_ble_evt)
{
    return ((p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT) &&
            (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING));
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role = ble_conn_state_role(conn_handle);
    // Based on the role this device plays in the connection, dispatch to the right handler.
    if (role == BLE_GAP_ROLE_PERIPH || ble_evt_is_advertising_timeout(p_ble_evt))
    {
        ble_nus_on_ble_evt(p_ble_evt, &m_nus);
        on_ble_peripheral_evt(p_ble_evt);
    }
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {
        on_ble_central_evt(p_ble_evt);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count =NRF_SDH_BLE_CENTRAL_LINK_COUNT + NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; // PERIPHERAL_LINK_COUNT + CENTRAL_LINK_COUNT;
    ble_cfg.conn_cfg.params.gap_conn_cfg.event_length =NRF_SDH_BLE_GAP_EVENT_LENGTH;
    ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_CONN_CFG_GAP.",
            nrf_strerror_get(err_code));
    }

    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = 4;
    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 7;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &ble_cfg, ram_start);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_CONN_CFG_GATTC.",
            nrf_strerror_get(err_code));
    }

    // Configure the connection roles.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count = NRF_SDH_BLE_PERIPHERAL_LINK_COUNT;
#ifndef S112
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_SDH_BLE_CENTRAL_LINK_COUNT;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = NRF_SDH_BLE_CENTRAL_LINK_COUNT ? BLE_GAP_ROLE_COUNT_CENTRAL_SEC_DEFAULT : 0;
#endif
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_GAP_CFG_ROLE_COUNT.",
            nrf_strerror_get(err_code));
    }

// Configure the maximum ATT MTU.
#if (BLE_GATT_MAX_MTU_SIZE != 23)
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_CONN_CFG_GATT.",
            nrf_strerror_get(err_code));
    }
#endif
	memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = NRF_SDH_BLE_VS_UUID_COUNT;

    err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, &ram_start);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_COMMON_CFG_VS_UUID.",
                      nrf_strerror_get(err_code));
    }

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t comm_params = {.rx_pin_no = RX_PIN_NUMBER,
        .tx_pin_no = TX_PIN_NUMBER,
        .rts_pin_no = RTS_PIN_NUMBER,
        .cts_pin_no = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity = true,
        .baud_rate = UARTE_BAUDRATE_BAUDRATE_Baud38400};

    APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_event_handle,
        APP_IRQ_PRIORITY_LOW, err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the NUS Client. */
static void nus_c_init(void)
{
    ret_code_t err_code;
    ble_nus_c_init_t nus_c_init_obj;

    nus_c_init_obj.evt_handler = ble_nus_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_nus_c_init(&m_ble_nus_array_c[i], &nus_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}

void disconnect_all(void)
{
    uint32_t err_code;
    sdk_mapped_flags_key_list_t conn_handles = ble_conn_state_periph_handles();

    for (uint8_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if (m_ble_nus_array_c[i].conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            err_code = sd_ble_gap_disconnect(
                m_ble_nus_array_c[i].conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_SUCCESS && err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
        else
        {
        }
    }

    for (uint8_t i = 0; i < conn_handles.len; i++)
    {
        err_code = sd_ble_gap_disconnect(
            conn_handles.flag_keys[i], BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_SUCCESS && err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**@brief Function for finding a matched instance from m_ble_nus_c */
uint32_t find_nus_c_instance(uint16_t conn_index, ble_nus_c_t ** pp_ble_nus_c)
{
    uint32_t err_code = NRF_ERROR_NOT_FOUND;
    //    uint8_t i;

    if (conn_index >= NRF_SDH_BLE_CENTRAL_LINK_COUNT)
        return NRF_ERROR_NOT_FOUND;

    *pp_ble_nus_c = &m_ble_nus_array_c[conn_index];
    err_code = NRF_SUCCESS;

    return err_code;
}

/**@brief Function for finding a matched instance from m_ble_gatt */
uint32_t get_gatt_instance(nrf_ble_gatt_t ** pp_ble_gatt)
{
    uint32_t err_code;

    *pp_ble_gatt = &m_gatt;
    err_code = NRF_SUCCESS;

    return err_code;
}

/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the Power manager. */
static void power_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the Database Discovery Module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void) { APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE); }

static void conn_evt_len_ext_set(bool status)
{
    ret_code_t err_code;
    ble_opt_t opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = status ? 1 : 0;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}

void data_len_ext_set(bool status)
{
    //    m_test_params.data_len_ext_enabled = status;
    uint8_t data_length = status ? (NRF_SDH_BLE_GATT_MAX_MTU_SIZE + 4) : (23 + 4);
    (void)nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, data_length);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            break;

        case BLE_ADV_EVT_IDLE:
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
#define DEVICE_NAME  "Lihai_box"
static void gap_params_init(void)
{
      uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
		NRF_LOG_INFO("BLE_CONN_PARAMS_EVT_FAILED");
		err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
	NRF_LOG_INFO("conn_params_error_handler");
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    m_connection_param.min_conn_interval = MIN_CONN_INTERVAL;
    m_connection_param.max_conn_interval = MAX_CONN_INTERVAL;
    m_connection_param.slave_latency     = 0;
    m_connection_param.conn_sup_timeout  = SUP_TIMEOUT;


    cp_init.p_conn_params                  = &m_connection_param;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;
	int8_t tx_power=0;
	memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval =  MSEC_TO_UNITS(40, UNIT_0_625_MS);
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(uint16_t conn_handle, ble_nus_evt_t * p_evt)
{
	uint16_t length = 0;
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_INFO("receive collector data");
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
		NRF_LOG_INFO("cccd enable");
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
    {
		NRF_LOG_INFO("cccd disable");
    }

}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ble_nus_init_t nus_init;
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

uint32_t get_nus_instance(ble_nus_t ** pp_ble_nus)
{
    uint32_t err_code;

    *pp_ble_nus = &m_nus;
    err_code = NRF_SUCCESS;

    return err_code;  
}

uint32_t adv_start()
{
    return ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);  
}

int main(void)
{
    ret_code_t err_code;
    app_timers_init();
    log_init();
    power_init();
    scheduler_init();
	ble_stack_init();

	user_param_flash_register();

	db_discovery_init();
	gatt_init();
	nus_c_init();
	uart_init();
	gap_params_init();
	conn_params_init();
	services_init();
	advertising_init();
    NRF_LOG_INFO("BLE UART central example started.");
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    scan_start();
    for (;;)
    {
        param_write_to_flash();
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}
