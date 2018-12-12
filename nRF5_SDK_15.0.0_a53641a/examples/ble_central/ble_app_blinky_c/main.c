/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the LED Button service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf.h"
#include "nrf_drv_timer.h"

#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1                     /**< Connected LED will be on when the device is connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                     /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define SCAN_INTERVAL                   0x0010                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION                   0x05                                /**< Timout when scanning. 0x0000 disables timeout. */
//scan for 50 ms then timeout

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                        /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                 /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL_PING           32                                  /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION_PING           1                                   /**< The advertising time-out (in units of 10 milliseconds). When set to 0, we will never time out. */
//Ping once then time out

#define APP_ADV_INTERVAL_REP            32                                 /**< The advertising interval (in units of 0.625 ms; this value corresponds to 20 ms). */
#define APP_ADV_DURATION_REP            100                                  /**< The advertising time-out (in units of 10 milliseconds). When set to 0, we will never time out. */
//Report every 20 ms 5 times

#define ADV_IDENTIFIER                  0x54524158                          //TRAX ascii to hex
#define MAX_DEVICES_SUPPORTED           4

BLE_LBS_C_DEF(m_ble_lbs_c);                                     /**< Main structure used by the LBS client module. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                /**< DB discovery module instance. */

typedef struct{
  uint8_t addr[6];
  int8_t rssi;
}beacon_entry_t;

static beacon_entry_t devices_found[MAX_DEVICES_SUPPORTED];

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata_ping[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static ble_gap_adv_params_t m_adv_params_ping;

static ble_gap_adv_data_t m_adv_data_ping =
{
    .adv_data =
    {
        .p_data = m_enc_advdata_ping,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = (0)

    }
};

//static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata_rep[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static ble_gap_adv_params_t m_adv_params_rep;

static ble_gap_adv_data_t m_adv_data_rep =
{
    .adv_data =
    {
        .p_data = m_enc_advdata_rep,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = (0)

    }
};

/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,

    .timeout           = SCAN_DURATION,
    .scan_phys         = BLE_GAP_PHY_1MBPS,
    .filter_policy     = BLE_GAP_SCAN_FP_ACCEPT_ALL,
};

static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_MAX]; /**< buffer where advertising reports will be stored by the SoftDevice. */

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_MAX
};

static void update_device_list(ble_gap_addr_t address, int8_t rssi)
{
  //NRF_LOG_HEXDUMP_INFO(address.addr, sizeof(address.addr));
  uint8_t empty_address[6] = {0};
  for(int i=0; i<MAX_DEVICES_SUPPORTED; i++)
  {
    if (memcmp(address.addr, devices_found[i].addr, 6) == 0)
    {
      if (devices_found[i].rssi < rssi)
      {
        devices_found[i].rssi = rssi;
      }
      return;
    }
    if (memcmp(devices_found[i].addr, empty_address, 6) == 0)
    {
      memcpy(devices_found[i].addr, address.addr, 6);
      devices_found[i].rssi = rssi;
      return;
    }
  }
  //NRF_LOG_DEBUG("Can't add new device. Buffer Full");
  return;
}

static void clear_device_strengths()
{
  for(int i=0; i<MAX_DEVICES_SUPPORTED; i++)
  {
    devices_found[i].rssi = 0x80;
  }
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void ping_advertising_init(void)
{
    ret_code_t    err_code;
    uint8_t adata[31];
    memset(adata, 0x00, sizeof(adata));
    adata[0] = 5;
    adata[1] = 9;
    adata[2] = 0x54; //T
    adata[3] = 0x52; //R
    adata[4] = 0x41; //A
    adata[5] = 0x58; //X
    
    m_adv_data_ping.adv_data.p_data = adata;
    m_adv_data_ping.adv_data.len = 6;
  
    memset(&m_adv_params_ping, 0, sizeof(m_adv_params_ping));
    m_adv_params_ping.primary_phy     = BLE_GAP_PHY_1MBPS;
    m_adv_params_ping.duration        = APP_ADV_DURATION_PING;
    m_adv_params_ping.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params_ping.p_peer_addr     = NULL;
    m_adv_params_ping.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params_ping.interval        = APP_ADV_INTERVAL_PING;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data_ping, &m_adv_params_ping);
    APP_ERROR_CHECK(err_code);
}

static void report_advertising_init(void)
{
    ret_code_t    err_code;
    uint8_t adata[31];
    memset(adata, 0x00, sizeof(adata));
    adata[0] = 30;
    adata[1] = 0x23;
    adata[2] = 0x52; //R
    memcpy((uint8_t *)&adata[3], (uint8_t *)&devices_found[0], 7);
    memcpy((uint8_t *)&adata[10], (uint8_t *)&devices_found[1], 7);
    memcpy((uint8_t *)&adata[17], (uint8_t *)&devices_found[2], 7);
    memcpy((uint8_t *)&adata[24], (uint8_t *)&devices_found[3], 7);
    //NRF_LOG_HEXDUMP_INFO(adata, sizeof(adata));
    
    memcpy(m_enc_advdata_rep, adata, sizeof(adata));
    m_adv_data_rep.adv_data.p_data = m_enc_advdata_rep;
    m_adv_data_rep.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

    memset(&m_adv_params_rep, 0, sizeof(m_adv_params_rep));
    m_adv_params_rep.primary_phy     = BLE_GAP_PHY_1MBPS;
    m_adv_params_rep.duration        = APP_ADV_DURATION_REP;
    m_adv_params_rep.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params_rep.p_peer_addr     = NULL;
    m_adv_params_rep.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params_rep.interval        = APP_ADV_INTERVAL_REP;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data_rep, &m_adv_params_rep);
    APP_ERROR_CHECK(err_code);
    
    clear_device_strengths();
}

/**@brief Function for starting advertising.
Called every ping transmission.
Called once per report.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    APP_ERROR_CHECK(err_code);
}

static void fsm_scan_timed_out(void)
{
  //When scan times out, increment counter and check whether to send ping or report
  static uint8_t ping_count = 0;
  //NRF_LOG_DEBUG("In fsm_scan_timed_out. ping_count = %d", ping_count);
  if(ping_count >= 5)
  {
    //send report
    //NRF_LOG_HEXDUMP_INFO(devices_found, sizeof(devices_found));
    report_advertising_init();
    advertising_start();
    ping_count = 0;
  }
  else if(ping_count == 0)
  {
    //start pinging, increment count
    ping_advertising_init();
    advertising_start();
    ping_count++;
  }
  else 
  {
    //continue pinging, increment count
    advertising_start();
    ping_count++;
  }
}

static void fsm_adv_timed_out(void)
{
  //when ping advertising times out, start scan
  scan_start();
}

/**@brief Function to handle asserts in the SoftDevice.
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


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Handles events coming from the LED Button central module.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
}

/**@brief Function for handling the advertising report BLE event.
 *
  Logs ping packets from Doggy Trax devices and continues scanning.

 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t err_code;

    ble_gap_addr_t addr = p_adv_report->peer_addr;
    char address_string[20];
    if((p_adv_report->data.p_data[2] != 0x52 && p_adv_report->data.p_data[2] != 0x54) ||
      (p_adv_report->data.len != 6 && p_adv_report->data.len != 31))
    {
        //ignore packet
        err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
        APP_ERROR_CHECK(err_code);
        return;
    }
    //process packet
    sprintf(address_string, "%02X:%02X:%02X:%02X:%02X:%02X",
                 addr.addr[5], addr.addr[4], addr.addr[3], addr.addr[2],
                 addr.addr[1], addr.addr[0]);
    //NRF_LOG_INFO("Packet Received, address: %s, signal strength: %d Packet Data:", 
    //address_string, p_adv_report->rssi);
//    if(p_adv_report->data.p_data[0] == 0x52 && p_adv_report->data.len == 31)
//    {
//        NRF_LOG_HEXDUMP_INFO(p_adv_report->data.p_data, p_adv_report->data.len);
//    }
//    NRF_LOG_FLUSH();
    
    update_device_list(p_adv_report->peer_addr, p_adv_report->rssi);

    err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    //ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            on_adv_report(&p_gap_evt->params.adv_report);
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                //NRF_LOG_DEBUG("Scan timed out.");
                fsm_scan_timed_out();
            }
        } break;
        
        case BLE_GAP_EVT_ADV_SET_TERMINATED:
        {
          //NRF_LOG_DEBUG("Advertising Timed Out. Advertisements: %d", 
            //p_gap_evt->params.adv_set_terminated.num_completed_adv_events);
          fsm_adv_timed_out();
        } break;
        
        default:
            // No implementation needed.
            break;
    }
}


/**@brief LED Button client initialization.
 */
static void lbs_c_init(void)
{
    ret_code_t       err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

    err_code = ble_lbs_c_init(&m_ble_lbs_c, &lbs_c_init_obj);
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

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, 4);
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, 4);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:
            err_code = ble_lbs_led_status_send(&m_ble_lbs_c, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            if (err_code == NRF_SUCCESS)
            {
                //NRF_LOG_INFO("LBS write LED state %d", button_action);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
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
    ble_lbs_on_db_disc_evt(&m_ble_lbs_c, p_evt);
}


/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the log.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Power manager. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    //NRF_LOG_FINAL_FLUSH();
    nrf_pwr_mgmt_run();
}


int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    leds_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    lbs_c_init();
    for(int i=0; i<MAX_DEVICES_SUPPORTED; i++)
    {
        memset(devices_found[i].addr, 0, 6);
        devices_found[i].rssi = 0x80; 
    }
  
    ping_advertising_init();

    // Start execution.
    NRF_LOG_INFO("Doggy Trax Beacon Started.");
    
    //kick off ping/scan and report loop
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
