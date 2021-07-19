/*
 * tsdz_bt.c
 *
 *  Created on: 10 set 2019
 *      Author: Max
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "tsdz_bt.h"
#include "tsdz_data.h"
#include "tsdz_uart.h"
#include "tsdz_commands.h"
#include "tsdz_data.h"

static const char *TAG = "tsdz_bt";

#define PROFILE_NUM                 3
#define TSDZ_PROFILE                0
#define CYCLING_POWER_PROFILE       1
#define CYCLING_CSCP_PROFILE		2
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "TSDZ_ESP32"

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
 *  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
 */

#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)


static uint8_t adv_config_done       = 0;
//static uint16_t conn_id              = 0xffff;

uint16_t tsdz_handle_table[IDX_TSDZ_DB_NUM];
uint16_t cycling_handle_table[IDX_CYCLING_DB_NUM];
uint16_t cscp_handle_table[IDX_CSCP_DB_NUM];

// if set the client is connected and able to receive notification from Command Characteristic
volatile uint8_t btCommandReady = 0;


typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
    esp_gatt_if_t            gatt_if;
    uint16_t                handle;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

static uint8_t service_uuid128[48] = {
        /* LSB <--------------------------------------------------------------------------------> MSB */
        //first uuid, 16bit, [12],[13] is the value
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
		//second uuid, 32bit, [12], [13], [14], [15] is the value
		// UUIDs are needed to discover sensor from Garmin Watch
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, // add second uuid for Power Sensor
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x16, 0x18, 0x00, 0x00, // add third uuid for CSCP Sensor
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
        .set_scan_rsp        = false,
        .include_name        = true,
        .include_txpower     = true,
        .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
        .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
        .appearance          = 1152, // 1152 = Cyclinc Generic Category (use decimal notation, not hex value!)
        .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
        .p_manufacturer_data = NULL, //test_manufacturer,
        .service_data_len    = 0,
        .p_service_data      = NULL,
        .service_uuid_len    = sizeof(service_uuid128),
        .p_service_uuid      = service_uuid128,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
        .set_scan_rsp        = true,
        .include_name        = true,
        .include_txpower     = true,
        .min_interval        = 0x0006,
        .max_interval        = 0x0010,
		.appearance			 = 1152, // 1152 = Cyclinc Generic Category (use decimal notation, not hex value!)
        .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
        .p_manufacturer_data = NULL, //&test_manufacturer[0],
        .service_data_len    = 0,
        .p_service_data      = NULL,
        .service_uuid_len    = sizeof(service_uuid128),
        .p_service_uuid      = service_uuid128,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
        .adv_int_min         = 0x20,
        .adv_int_max         = 0x40,
        .adv_type            = ADV_TYPE_IND,
        .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
        .channel_map         = ADV_CHNL_ALL,
        .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};


enum {
    FEATURE_PEDAL_POWER_BALANCE                 = 0x00000001,
    FEATURE_ACCUMULATED_TORQUE                  = 0x00000002,
    FEATURE_WHEEL_REVOLUTION_DATA               = 0x00000004,
    FEATURE_CRANK_REVOLUTION_DATA               = 0x00000008,
    FEATURE_EXTREME_MAGNITUDES                  = 0x00000010,
    FEATURE_EXTREME_ANGLES                      = 0x00000020,
    FEATURE_TOP_AND_BOTTOM_DEAD_SPOT_ANGLES     = 0x00000040,
    FEATURE_ACCUMULATED_ENERGY                  = 0x00000080,
    FEATURE_OFFSET_COMPENSATION_INDICATOR       = 0x00000100,
    FEATURE_OFFSET_COMPENSATION                 = 0x00000200,
    FEATURE_MEASUREMENT_CONTENT_MASKING         = 0x00000400,
    FEATURE_MULTIPLE_SENSOR_LOCATIONS           = 0x00000800,
    FEATURE_CRANK_LENGTH_ADJUSTMENT             = 0x00001000,
    FEATURE_CHAIN_LENGTH_ADJUSTMENT             = 0x00002000,
    FEATURE_CHAIN_WEIGHT_ADJUSTMENT             = 0x00004000,
    FEATURE_SPAN_LENGTH_ADJUSTMENT              = 0x00008000,
    FEATURE_SENSOR_MEASUREMENT_TORQUE_BASED     = 0x00010000,
    FEATURE_INSTANTANEOUS_MEASUREMENT_DIRECTION = 0x00020000,
    FEATURE_FACTORY_CALIBRATION_DATE            = 0x00040000,
};

enum {
    FLAG_PEDAL_POWER_BALANCE_PRESENT           = 0x0001,
    FLAG_PEDAL_POWER_BALANCE_REFERENCE_LEFT    = 0x0002,
    FLAG_ACCUMULATED_TORQUE_PRESENT            = 0x0004,
    FLAG_ACCUMULATED_TORQUE_SOURCE_CRANK_BASED = 0x0008,
    FLAG_WHEEL_REVOLUTION_DATA_PRESENT         = 0x0010,
    FLAG_CRANK_REVOLUTION_DATA_PRESENT         = 0x0020,
    FLAG_EXTREME_FORCE_MAGNITUDES_PRESENT      = 0x0040,
    FLAG_EXTREME_TORQUE_MAGNITUDES_PRESENT     = 0x0080,
    FLAG_EXTREME_ANGLES_PRESENT                = 0x0100,
    FLAG_TOP_DEAD_SPOT_ANGLE_PRESENT           = 0x0200,
    FLAG_BOTTOM_DEAD_SPOT_ANGLE_PRESENT        = 0x0400,
    FLAG_ACCUMULATED_ENERGY_PRESENT            = 0x0800,
    FLAG_OFFSET_COMPENSATION_INDICATOR         = 0x1000,
};



static void gatts_tsdz_profile_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_cycling_profile_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_cscp_profile_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gatts_profile_tab[PROFILE_NUM] = {
    [TSDZ_PROFILE] = {
        .gatts_cb = gatts_tsdz_profile_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .conn_id = 0xffff
    },
    [CYCLING_POWER_PROFILE] = {
        .gatts_cb = gatts_cycling_profile_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .conn_id = 0xffff
    },
	[CYCLING_CSCP_PROFILE] = {
		.gatts_cb = gatts_cscp_profile_handler,
		.gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		.conn_id = 0xffff
	}
};

/* TSDZ Service */
static const uint16_t GATTS_SERVICE_UUID_TSDZ      = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TSDZ_STATUS  = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TSDZ_DEBUG   = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_TSDZ_CFG     = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_TSDZ_COMMAND = 0xFF04;

#define MAX_MEASURE_BYTES 19
static const uint8_t sensor_loction_attr[1] = {0x0B};
static uint8_t cyclingMsg[MAX_MEASURE_BYTES];
/* Cycling Power Service */
static const uint16_t GATTS_SERVICE_UUID_CYCLING_POWER          = 0x1818;
static const uint16_t GATTS_CHAR_UUID_SENSOR_LOCATION           = 0x2A5D;
static const uint16_t GATTS_CHAR_UUID_CYCLING_POWER_FEATURE     = 0x2A65;
static const uint16_t GATTS_CHAR_UUID_CYCLING_POWER_MEASUREMENT = 0x2A63;
// Wheel revs, cCank revs, and Accumulated Energy
static const uint8_t power_feature_attr[4] = {0x8C,0x00,0x00,0x00};

#define MAX_CSCP_MEASURE_BYTES 11
static uint8_t cscpMsg[MAX_CSCP_MEASURE_BYTES];
/* Cycling Speed and Cadence Service */
static const uint16_t GATTS_SERVICE_UUID_CYCLING_CSCP          = 0x1816;
static const uint16_t GATTS_CHAR_UUID_CYCLING_CSCP_FEATURE     = 0x2A5C;
static const uint16_t GATTS_CHAR_UUID_CYCLING_CSCP_MEASUREMENT = 0x2A5B;
static const uint8_t cscp_feature_attr[2] = {0x03,0x00}; // 0x03 = cadence and speed supported


static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write_notify        = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//static const uint8_t char_prop_read_notify         = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_notify              = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write          = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;

static const uint8_t tsdz_attr_ccc[2] = {0x00, 0x00};


static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
   char *key_str = NULL;
   switch(key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

   }

   return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
   char *auth_str = NULL;
   switch(auth_req) {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
        auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
   }

   return auth_str;
}

static void remove_bonded_devices_except(esp_bd_addr_t keep)
{
    int dev_num = esp_ble_get_bond_device_num();
    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        if (memcmp(keep, dev_list[i].bd_addr, sizeof(esp_bd_addr_t))) {
            esp_ble_remove_bond_device(dev_list[i].bd_addr);
        }
    }
    free(dev_list);
}

/* TSDZ Service Full Database Description  */
static const esp_gatts_attr_db_t tsdz_gatt_db[IDX_TSDZ_DB_NUM] = {
        // Service Declaration
        [IDX_TSDZ_SVC] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &primary_service_uuid, ESP_GATT_PERM_READ,
                sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TSDZ),
                (uint8_t *) &GATTS_SERVICE_UUID_TSDZ } },
        // Characteristic Declaration
        [IDX_CHAR_STATUS] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                (uint8_t *) &char_prop_notify } },

        // Characteristic Value
        [IDX_CHAR_VAL_STATUS] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &GATTS_CHAR_UUID_TSDZ_STATUS, ESP_GATT_PERM_READ,
                sizeof(tsdz_status), sizeof(tsdz_status),
                (uint8_t *) (&tsdz_status) } },

        // Client Characteristic Configuration Descriptor
        [IDX_CHAR_CFG_STATUS] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ
                        | ESP_GATT_PERM_WRITE_ENCRYPTED, sizeof(uint16_t),
                sizeof(tsdz_attr_ccc), (uint8_t *) tsdz_attr_ccc } },

        // Characteristic Declaration
        [IDX_CHAR_DEBUG] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                (uint8_t *) &char_prop_notify } },

        // Characteristic Value
        [IDX_CHAR_VAL_DEBUG] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &GATTS_CHAR_UUID_TSDZ_DEBUG, ESP_GATT_PERM_READ,
                sizeof(tsdz_debug), sizeof(tsdz_debug),
                (uint8_t *) (&tsdz_debug) } },

        // Client Characteristic Configuration Descriptor
        [IDX_CHAR_CFG_DEBUG] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ
                        | ESP_GATT_PERM_WRITE_ENCRYPTED, sizeof(uint16_t),
                sizeof(tsdz_attr_ccc), (uint8_t *) tsdz_attr_ccc } },

        // Characteristic Declaration
        [IDX_CHAR_CONFIG] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                (uint8_t *) &char_prop_read_write } },

        // Characteristic Value
        [IDX_CHAR_VAL_CONFIG] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &GATTS_CHAR_UUID_TSDZ_CFG,
                ESP_GATT_PERM_WRITE_ENCRYPTED|ESP_GATT_PERM_READ_ENCRYPTED, sizeof(tsdz_cfg),
                sizeof(tsdz_cfg), (uint8_t *) (&tsdz_cfg) } },
        // Characteristic Declaration
        [IDX_CHAR_COMMAND] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                (uint8_t *) &char_prop_write_notify } },
        // Characteristic Value
        [IDX_CHAR_VAL_COMMAND] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &GATTS_CHAR_UUID_TSDZ_COMMAND,
                ESP_GATT_PERM_WRITE_ENCRYPTED|ESP_GATT_PERM_READ_ENCRYPTED, 128, 2, (uint8_t *) "A" } },
        // Client Characteristic Configuration Descriptor
        [IDX_CHAR_CFG_COMMAND] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ
                        | ESP_GATT_PERM_WRITE_ENCRYPTED, sizeof(uint16_t),
                sizeof(tsdz_attr_ccc), (uint8_t *) tsdz_attr_ccc } },
};

/* Cyclcing Power Service Full Database Description  */
static const esp_gatts_attr_db_t cycling_gatt_db[IDX_CYCLING_DB_NUM] = {
// Service Declaration
        [IDX_CYCLING_SVC] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &primary_service_uuid, ESP_GATT_PERM_READ,
                sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_CYCLING_POWER),
                (uint8_t *) &GATTS_SERVICE_UUID_CYCLING_POWER } },
        // Characteristic Declaration
        [IDX_CHAR_CYCLING_POWER_MEASUREMENT] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                (uint8_t *) &char_prop_notify } },
        // Characteristic Value
        [IDX_CHAR_VAL_CYCLING_POWER_MEASUREMENT] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &GATTS_CHAR_UUID_CYCLING_POWER_MEASUREMENT, ESP_GATT_PERM_READ,
                MAX_MEASURE_BYTES, MAX_MEASURE_BYTES,
                (uint8_t *) (&cyclingMsg)} },
        // Client Characteristic Configuration Descriptor
        [IDX_CHAR_CFG_CYCLING_POWER_MEASUREMENT] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ
                        | ESP_GATT_PERM_WRITE, sizeof(uint16_t),
                sizeof(tsdz_attr_ccc), (uint8_t *) tsdz_attr_ccc } },
        // Characteristic Declaration
        [IDX_CHAR_CYCLING_POWER_FEATURE] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                (uint8_t *) &char_prop_read } },
        // Characteristic Value
        [IDX_CHAR_VAL_CYCLING_POWER_FEATURE] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &GATTS_CHAR_UUID_CYCLING_POWER_FEATURE, ESP_GATT_PERM_READ,
                sizeof(power_feature_attr), sizeof(power_feature_attr),
                (uint8_t *) (&power_feature_attr) } },
        // Characteristic Declaration
        [IDX_CHAR_SENSOR_LOCATION] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                (uint8_t *) &char_prop_read } },
        // Characteristic Value
        [IDX_CHAR_VAL_SENSOR_LOCATION] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &GATTS_CHAR_UUID_SENSOR_LOCATION, ESP_GATT_PERM_READ,
                sizeof(sensor_loction_attr), sizeof(sensor_loction_attr),
                (uint8_t *) (&sensor_loction_attr) } }
};

/* Cycling CSCP Service Full Database Description  */
static const esp_gatts_attr_db_t cscp_gatt_db[IDX_CSCP_DB_NUM] = {
		// Service Declaration
        [IDX_CSCP_SVC] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &primary_service_uuid, ESP_GATT_PERM_READ,
                sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_CYCLING_CSCP),
                (uint8_t *) &GATTS_SERVICE_UUID_CYCLING_CSCP } },
        // Characteristic Declaration
        [IDX_CHAR_CYCLING_CSCP_MEASUREMENT] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                (uint8_t *) &char_prop_notify } },
        // Characteristic Value
        [IDX_CHAR_VAL_CYCLING_CSCP_MEASUREMENT] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &GATTS_CHAR_UUID_CYCLING_CSCP_MEASUREMENT, ESP_GATT_PERM_READ,
				MAX_CSCP_MEASURE_BYTES, MAX_CSCP_MEASURE_BYTES,
				(uint8_t *) (&cscpMsg)} },
        // Client Characteristic Configuration Descriptor
        [IDX_CHAR_CFG_CYCLING_CSCP_MEASUREMENT] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_client_config_uuid, ESP_GATT_PERM_READ
                        | ESP_GATT_PERM_WRITE, sizeof(uint16_t),
                sizeof(tsdz_attr_ccc), (uint8_t *) tsdz_attr_ccc } },
        // Characteristic Declaration
        [IDX_CHAR_CYCLING_CSCP_FEATURE] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
                (uint8_t *) &char_prop_read } },
        // Characteristic Value
        [IDX_CHAR_VAL_CYCLING_CSCP_FEATURE] =
        { { ESP_GATT_AUTO_RSP }, { ESP_UUID_LEN_16,
                (uint8_t *) &GATTS_CHAR_UUID_CYCLING_CSCP_FEATURE, ESP_GATT_PERM_READ,
                sizeof(cscp_feature_attr), sizeof(cscp_feature_attr),
                (uint8_t*) (&cscp_feature_attr) } }
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* advertising start complete event to indicate advertising start successfully or failed */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "advertising start failed");
        }else{
            ESP_LOGI(TAG, "advertising start successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising stop failed");
        }
        else {
            ESP_LOGI(TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                param->update_conn_params.status,
                param->update_conn_params.min_int,
                param->update_conn_params.max_int,
                param->update_conn_params.conn_int,
                param->update_conn_params.latency,
                param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
        ESP_LOGI(TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        /* Call the following function to input the passkey which is displayed on the remote device */
        //esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT: {
        ESP_LOGI(TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        break;
    }
    case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
        ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
        ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer device.
        ESP_LOGI(TAG, "The passkey Notify number:%06d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        ESP_LOGI(TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGI(TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        } else {
            ESP_LOGI(TAG, "auth mode = %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
        }
        remove_bonded_devices_except(bd_addr);
        break;
    }
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
        ESP_LOGD(TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
        ESP_LOGI(TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
        esp_log_buffer_hex(TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "------------------------------------");
        break;
    }
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
            break;
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
        }else{
            adv_config_done |= ADV_CONFIG_FLAG;
        }

        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
        }else{
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        }

        break;
    default:
        break;
    }
}

esp_gatt_status_t handle_write_value(uint16_t handle, uint8_t *value, uint16_t len) {
    if (tsdz_handle_table[IDX_CHAR_VAL_CONFIG] == handle) {
        if ((len != sizeof(struct _tsdz_cfg)) || tsdz_update_cfg((struct_tsdz_cfg*)value)) {
            ESP_LOGE(TAG, "CFG update FAILED");
            esp_err_t set_attr_ret = esp_ble_gatts_set_attr_value(tsdz_handle_table[IDX_CHAR_VAL_CONFIG], sizeof(tsdz_cfg), (uint8_t*)(&tsdz_cfg));
            if (set_attr_ret){
                ESP_LOGE(TAG, "set attr failed, error code = %x", set_attr_ret);
            }
            return ESP_GATT_INVALID_PDU;
        } else
            return ESP_GATT_OK;
    } else if (tsdz_handle_table[IDX_CHAR_VAL_COMMAND] == handle) {
        if (exec_command(value, len)) {
            ESP_LOGE(TAG, "Command exec FAILED");
            return ESP_GATT_INVALID_PDU;
        } else
            return ESP_GATT_OK;
    } else if (tsdz_handle_table[IDX_CHAR_CFG_STATUS] == handle) {
        if (value[0] == 0x01){
            ESP_LOGI(TAG, "IDX_CHAR_CFG_STATUS notify enable");
        } else if (value[0] == 0x00){
            ESP_LOGI(TAG, "IDX_CHAR_CFG_STATUS notify disable ");
        } else{
            ESP_LOGE(TAG, "IDX_CHAR_CFG_STATUS unknown descr value");
        }
        return ESP_GATT_OK;
    } else if (tsdz_handle_table[IDX_CHAR_CFG_DEBUG] == handle) {
        if (value[0] == 0x01){
            ESP_LOGI(TAG, "IDX_CHAR_CFG_DEBUG notify enable");
        } else if (value[0] == 0x00){
            ESP_LOGI(TAG, "IDX_CHAR_CFG_DEBUG notify disable ");
        } else{
            ESP_LOGE(TAG, "IDX_CHAR_CFG_DEBUG unknown descr value");
        }
        return ESP_GATT_OK;
    } else if (tsdz_handle_table[IDX_CHAR_CFG_COMMAND] == handle) {
        if (value[0] == 0x01){
            ESP_LOGI(TAG, "IDX_CHAR_CFG_COMMAND notify enable");
            btCommandReady = 1;
        } else if (value[0] == 0x00){
            ESP_LOGI(TAG, "IDX_CHAR_CFG_COMMAND notify disable ");
            btCommandReady = 0;
        } else{
            ESP_LOGE(TAG, "IDX_CHAR_CFG_COMMAND unknown descr value");
        }
        return ESP_GATT_OK;
    } else {
        ESP_LOGI(TAG, "Invalid Handle: %d", handle);
        return ESP_GATT_INVALID_HANDLE;
    }

}

void write_event_env(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    esp_gatt_status_t status;
    //esp_log_buffer_hex(TAG, param->write.value, param->write.len);
    status = handle_write_value(param->write.handle, param->write.value, param->write.len);
    /* send response when param->write.need_rsp is true*/
    if (param->write.need_rsp){
        ESP_LOGI(TAG, "send response %d ", status);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
    }
}

void prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    //ESP_LOGI(TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    prepare_write_env->handle = param->write.handle;
    prepare_write_env->gatt_if = gatts_if;
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
                ESP_LOGE(TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
            param->write.value,
            param->write.len);
    prepare_write_env->prepare_len += param->write.len;
}

void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        //esp_log_buffer_hex(TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
        handle_write_value(prepare_write_env->handle, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    } else{
        ESP_LOGI(TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }

    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_tsdz_profile_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:{
    	ESP_LOGI(TAG, "T ESP_GATTS_REG_EVT");
        esp_err_t ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        if (ret){
            ESP_LOGE(TAG, "set device name failed, error code = %x", ret);
        }
        ret = esp_ble_gap_config_local_privacy(true);
        if (ret) {
            ESP_LOGE(TAG, "esp_ble_gap_config_local_privacy failed, error code = %x", ret);
        }
        ret = esp_ble_gatts_create_attr_tab(tsdz_gatt_db, gatts_if, IDX_TSDZ_DB_NUM, TSDZ_PROFILE);
        if (ret){
            ESP_LOGE(TAG, "create attr table failed, error code = %x", ret);
        }
    }
    break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "T ESP_GATTS_READ_EVT");
        break;
    case ESP_GATTS_WRITE_EVT:
        //ESP_LOGI(TAG, "T ESP_GATTS_WRITE_EVT");
        if (!param->write.is_prep){
            //ESP_LOGI(TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
            write_event_env(gatts_if, param);
        }else{
            /* handle prepare write */
            prepare_write_event_env(gatts_if, &prepare_write_env, param);
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(TAG, "T ESP_GATTS_EXEC_WRITE_EVT");
        exec_write_event_env(&prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "T ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        //ESP_LOGI(TAG, "T ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "T SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "T ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);

        // unlock the bike
        bike_locked = 0;

        /* start security connect with peer device when receive the connect event sent by the master */
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);

        gatts_profile_tab[TSDZ_PROFILE].conn_id = param->connect.conn_id;
        //esp_log_buffer_hex(TAG, param->connect.remote_bda, 6);
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x30;    // max_int = 0x30*1.25ms = 60ms
        conn_params.min_int = 0x20;    // min_int = 0x20*1.25ms = 40ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
    	gatts_profile_tab[TSDZ_PROFILE].conn_id = 0xffff;
        ESP_LOGI(TAG, "T ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        tsdz_data_disconnect_actions();
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
        if (param->add_attr_tab.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "T create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != IDX_TSDZ_DB_NUM){
            ESP_LOGE(TAG, "T create attribute table abnormally, num_handle (%d) \
                        doesn't equal to IDX_TSDZ_NUM(%d)", param->add_attr_tab.num_handle, IDX_TSDZ_DB_NUM);
        }
        else {
            ESP_LOGI(TAG, "T create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
            memcpy(tsdz_handle_table, param->add_attr_tab.handles, sizeof(tsdz_handle_table));
            esp_ble_gatts_start_service(tsdz_handle_table[IDX_TSDZ_SVC]);
        }
        break;
    }
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    default:
        break;
    }
}

static void gatts_cycling_profile_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:{
    	ESP_LOGI(TAG, "C ESP_GATTS_REG_EVT");
        esp_err_t ret = esp_ble_gatts_create_attr_tab(cycling_gatt_db, gatts_if, IDX_CYCLING_DB_NUM, CYCLING_POWER_PROFILE);
        if (ret){
            ESP_LOGE(TAG, "create attr table failed, error code = %x", ret);
        }
    }
    break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_READ_EVT");
        break;
    case ESP_GATTS_WRITE_EVT:
        //ESP_LOGI(TAG, "C ESP_GATTS_WRITE_EVT");
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_EXEC_WRITE_EVT");
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        //ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "C SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);

        // unlock the bike
        bike_locked = 0;

        gatts_profile_tab[CYCLING_POWER_PROFILE].conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
    	gatts_profile_tab[CYCLING_POWER_PROFILE].conn_id = 0xffff;
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
        if (param->add_attr_tab.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "C create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != IDX_CYCLING_DB_NUM){
            ESP_LOGE(TAG, "C create attribute table abnormally, num_handle (%d) \
                        doesn't equal to IDX_TSDZ_NUM(%d)", param->add_attr_tab.num_handle, IDX_CYCLING_DB_NUM);
        } else {
            ESP_LOGI(TAG, "C create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
            memcpy(cycling_handle_table, param->add_attr_tab.handles, sizeof(cycling_handle_table));
            esp_ble_gatts_start_service(cycling_handle_table[IDX_CYCLING_SVC]);
        }
        break;
    }
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    default:
        break;
    }
}

static void gatts_cscp_profile_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:{
    	ESP_LOGI(TAG, "C ESP_GATTS_REG_EVT");
        esp_err_t ret = esp_ble_gatts_create_attr_tab(cscp_gatt_db, gatts_if, IDX_CSCP_DB_NUM, CYCLING_CSCP_PROFILE);
        if (ret){
            ESP_LOGE(TAG, "create attr table failed, error code = %x", ret);
        }
    }
    break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_READ_EVT");
        break;
    case ESP_GATTS_WRITE_EVT:
        //ESP_LOGI(TAG, "C ESP_GATTS_WRITE_EVT");
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_EXEC_WRITE_EVT");
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        //ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "C SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        gatts_profile_tab[CYCLING_CSCP_PROFILE].conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "C ESP_GATTS_DISCONNECT_EVT");
    	gatts_profile_tab[CYCLING_CSCP_PROFILE].conn_id = 0xffff;
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
        if (param->add_attr_tab.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "C create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != IDX_CSCP_DB_NUM){
            ESP_LOGE(TAG, "C create attribute table abnormally, num_handle (%d) \
                        doesn't equal to IDX_CSCP_DB_NUM(%d)", param->add_attr_tab.num_handle, IDX_CSCP_DB_NUM);
        } else {
            ESP_LOGI(TAG, "C create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
            memcpy(cscp_handle_table, param->add_attr_tab.handles, sizeof(cscp_handle_table));
            esp_ble_gatts_start_service(cscp_handle_table[IDX_CSCP_SVC]);
        }
        break;
    }
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			gatts_profile_tab[param->reg.app_id].gatts_if = gatts_if;
		} else {
			ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d\n",
					param->reg.app_id,
					param->reg.status);
			return;
		}
	}

	do {
		int idx;
		for (idx = 0; idx < PROFILE_NUM; idx++) {
			if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
					gatts_if == gatts_profile_tab[idx].gatts_if) {
				if (gatts_profile_tab[idx].gatts_cb) {
					gatts_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}
	} while (0);
}

void tsdz_bt_init(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(TSDZ_PROFILE);
    if (ret){
        ESP_LOGE(TAG, "T gatts app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(CYCLING_POWER_PROFILE);
    if (ret){
        ESP_LOGE(TAG, "C gatts app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(CYCLING_CSCP_PROFILE);
    if (ret){
        ESP_LOGE(TAG, "C gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }


    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
    // esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;            //set the IO capability to input/output
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE; // Set NO-IO to prevent pairing code requirement

    uint8_t key_size = 16;      //the key size should be 7~16 bytes

    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    //set static passkey
    uint32_t key = bt_passkey;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &key, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    /* If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the master;
    If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

void tsdz_bt_stop(void)
{
    esp_err_t ret;
    ret = esp_ble_gatts_app_unregister(gatts_profile_tab[TSDZ_PROFILE].gatts_if);
    if (ret){
        ESP_LOGE(TAG, "esp_ble_gatts_app_unregister error, error code = %x", ret);
    }
    ret = esp_ble_gatts_app_unregister(gatts_profile_tab[CYCLING_POWER_PROFILE].gatts_if);
    if (ret){
        ESP_LOGE(TAG, "esp_ble_gatts_app_unregister error, error code = %x", ret);
    }
    ret = esp_ble_gatts_app_unregister(gatts_profile_tab[CYCLING_CSCP_PROFILE].gatts_if);
    if (ret){
        ESP_LOGE(TAG, "esp_ble_gatts_app_unregister error, error code = %x", ret);
    }
    ret = esp_bluedroid_disable();
    if (ret){
        ESP_LOGE(TAG, "esp_bluedroid_disable error, error code = %x", ret);
    }
    ret = esp_bluedroid_deinit();
    if (ret){
        ESP_LOGE(TAG, "esp_bluedroid_deinit error, error code = %x", ret);
    }
    ret = esp_bt_controller_disable();
    if (ret){
        ESP_LOGE(TAG, "esp_bt_controller_disable error, error code = %x", ret);
    }
    ret = esp_bt_controller_deinit();
    if (ret){
        ESP_LOGE(TAG, "esp_bt_controller_deinit error, error code = %x", ret);
    }
}

void tsdz_bt_update(void) {
	esp_err_t ret;
    if (gatts_profile_tab[TSDZ_PROFILE].conn_id != 0xffff) {
        // Client Charactersitic Configuration bit 0 set -> Notification enabled
        uint16_t l;
        const uint8_t* value;
        ret = esp_ble_gatts_get_attr_value(tsdz_handle_table[IDX_CHAR_CFG_STATUS], &l, &value);
        if (ret==ESP_OK  && l==2 && (value[0]&0x01))
            ret = esp_ble_gatts_send_indicate(gatts_profile_tab[TSDZ_PROFILE].gatts_if, gatts_profile_tab[TSDZ_PROFILE].conn_id, tsdz_handle_table[IDX_CHAR_VAL_STATUS],
                    sizeof(tsdz_status), (uint8_t *)(&tsdz_status), false);
        if (ret){
            ESP_LOGE(TAG, "tsdz_bt_update, tsdz_status notification failed, error code = %x", ret);
        }
        ret = esp_ble_gatts_get_attr_value(tsdz_handle_table[IDX_CHAR_CFG_DEBUG], &l, &value);
        if (ret==ESP_OK  && l==2 && (value[0]&0x01))
            ret |= esp_ble_gatts_send_indicate(gatts_profile_tab[TSDZ_PROFILE].gatts_if, gatts_profile_tab[TSDZ_PROFILE].conn_id, tsdz_handle_table[IDX_CHAR_VAL_DEBUG],
                sizeof(tsdz_debug), (uint8_t *)(&tsdz_debug), false);
        if (ret){
            ESP_LOGE(TAG, "tsdz_bt_update, tsdz_debug notification failed, error code = %x", ret);
        }
    }
}


void cycling_bt_update(void) {
    static const uint16_t flags = (FLAG_WHEEL_REVOLUTION_DATA_PRESENT +
    		FLAG_CRANK_REVOLUTION_DATA_PRESENT +
			FLAG_ACCUMULATED_ENERGY_PRESENT);
    static uint32_t dCumulativeWheelRev = 0;
    static uint16_t dLastWheelEventTime = 0;
    static uint16_t dCumulativeCrankRev = 0;
    static uint16_t dLastCrankEventTime = 0;

    esp_err_t ret;
    if (gatts_profile_tab[CYCLING_POWER_PROFILE].conn_id != 0xffff) {
        // Client Charactersitic Configuration bit 0 set -> Notification enabled
        uint16_t l;
        const uint8_t* value;
        ret = esp_ble_gatts_get_attr_value(cycling_handle_table[IDX_CHAR_CFG_CYCLING_POWER_MEASUREMENT], &l, &value);
        if (ret==ESP_OK && l==2 && (value[0]&0x01)) {
        	uint32_t tmp;
            unsigned i = 0;

        	// flags
            cyclingMsg[i++] = (uint8_t)(flags & 0xFF);
            cyclingMsg[i++] = (uint8_t)((flags >> 8) & 0xFF);

            // intantaneus Power (W)
            tmp = tsdz_status.ui16_battery_voltage_x1000 * tsdz_status.ui8_battery_current_x10 / 10000;
            ESP_LOGI(TAG, "power:%d", tmp&0xffff);
            cyclingMsg[i++] = (uint8_t)(tmp & 0xff);
            cyclingMsg[i++] = (uint8_t)((tmp >> 8) & 0xff);

            // Wheel revolutions
            ESP_LOGI(TAG, "wheel_revolutions:%u", wheel_revolutions);
            cyclingMsg[i++] = (uint8_t)(wheel_revolutions & 0xff);
            cyclingMsg[i++] = (uint8_t)((wheel_revolutions>>8) & 0xff);
            cyclingMsg[i++] = (uint8_t)((wheel_revolutions>>16) & 0xff);
            cyclingMsg[i++] = (uint8_t)((wheel_revolutions>>24) & 0xff);

            // last Wheel revolution time (sec/2048)
			if ( (wheel_revolutions > dCumulativeWheelRev) && (tsdz_status.ui16_wheel_speed_x10 > 0) ) {
				tmp = (wheel_revolutions - dCumulativeWheelRev) * (uint32_t)tsdz_cfg.ui16_wheel_perimeter * 2048L
						 / ((uint32_t)tsdz_status.ui16_wheel_speed_x10 * 1000L / 36L);
				tmp += dLastWheelEventTime;
				dLastWheelEventTime = tmp & 0xffff;
			}
            dCumulativeWheelRev = wheel_revolutions;
            ESP_LOGI(TAG, "wheel time:%u", dLastWheelEventTime);
            cyclingMsg[i++] = (uint8_t)(dLastWheelEventTime & 0xff);
            cyclingMsg[i++] = (uint8_t)((dLastWheelEventTime>>8) & 0xff);

            // Crank revolutions
            ESP_LOGI(TAG, "crank_revolutions:%u", crank_revolutions);
            cyclingMsg[i++] = (uint8_t)(crank_revolutions & 0xff);
            cyclingMsg[i++] = (uint8_t)((crank_revolutions>>8) & 0xff);

            // Crank revolution time (sec/1024) (60*1024/rmp)
			if ( (crank_revolutions > dCumulativeCrankRev) && (tsdz_status.ui8_pedal_cadence_RPM > 0) ) {
				// N. revs * 60/rpm * 1024
				tmp = (crank_revolutions - dCumulativeCrankRev) * 61440 / tsdz_status.ui8_pedal_cadence_RPM;
				tmp += dLastCrankEventTime;
				dLastCrankEventTime = tmp & 0xffff;
			}
            dCumulativeCrankRev = crank_revolutions;
            ESP_LOGI(TAG, "crank time:%d", dLastCrankEventTime);
            cyclingMsg[i++] = (dLastCrankEventTime & 0xff);
            cyclingMsg[i++] = ((dLastCrankEventTime>>8) & 0xff);

            // Accumulated Energy (KJ = Wh * 3600 / 1000)
            tmp = (uint32_t)tsdz_status.ui16_battery_wh * 36L / 10L;
            cyclingMsg[i++] = (tmp & 0xff);
            cyclingMsg[i++] = ((tmp>>8) & 0xff);

            ret = esp_ble_gatts_send_indicate(gatts_profile_tab[CYCLING_POWER_PROFILE].gatts_if, gatts_profile_tab[CYCLING_POWER_PROFILE].conn_id, cycling_handle_table[IDX_CHAR_VAL_CYCLING_POWER_MEASUREMENT],
                    i, cyclingMsg, false);
            if (ret)
                ESP_LOGE(TAG, "cycling_bt_update, cycling_power notifiation failed, error code = %x", ret);
        }
    }
}

void cscp_bt_update(void) {

	static uint32_t last_wheel_revs = 0;
	static uint16_t last_wheel_time = 0;
	static uint16_t last_crank_revs = 0;
	static uint16_t last_crank_time = 0;
    esp_err_t ret;
    if (gatts_profile_tab[CYCLING_CSCP_PROFILE].conn_id != 0xffff) {
        // Client Charactersitic Configuration bit 0 set -> Notification enabled
        uint16_t l;
        unsigned i = 0;

        const uint8_t* value;
        ret = esp_ble_gatts_get_attr_value(cscp_handle_table[IDX_CHAR_CFG_CYCLING_CSCP_MEASUREMENT], &l, &value);
        if (ret==ESP_OK && l==2 && (value[0]&0x01)) {
            uint32_t tmp;

            // flags
			cscpMsg[i++] = (uint8_t)(0x03);

            // Wheel revolutions
            ESP_LOGI(TAG, "wheel_revolutions:%u", wheel_revolutions);
            cscpMsg[i++] = (wheel_revolutions & 0xff);
            cscpMsg[i++] = ((wheel_revolutions>>8) & 0xff);
            cscpMsg[i++] = ((wheel_revolutions>>16) & 0xff);
            cscpMsg[i++] = ((wheel_revolutions>>24) & 0xff);

            // last Wheel revolution time (sec/1024)
			if ( (wheel_revolutions > last_wheel_revs) && (tsdz_status.ui16_wheel_speed_x10 > 0) ) {
				tmp = (wheel_revolutions - last_wheel_revs) * (uint32_t)tsdz_cfg.ui16_wheel_perimeter * 1024L
						 / ((uint32_t)tsdz_status.ui16_wheel_speed_x10 * 1000L / 36L);
				tmp += last_wheel_time;
				last_wheel_time = tmp & 0xffff;
			}
            last_wheel_revs = wheel_revolutions;
            ESP_LOGI(TAG, "wheel time:%u", last_wheel_time);
            cscpMsg[i++] = (last_wheel_time & 0xff);
            cscpMsg[i++] = ((last_wheel_time>>8) & 0xff);

            // Crank revolutions
            ESP_LOGI(TAG, "crank_revolutions:%u", crank_revolutions);
            cscpMsg[i++] = (crank_revolutions & 0xff);
            cscpMsg[i++] = ((crank_revolutions>>8) & 0xff);

            // Crank revolution time (sec/1024) (60*1024/rmp)
			if ( (crank_revolutions > last_crank_revs) && (tsdz_status.ui8_pedal_cadence_RPM > 0) ) {
				// N. revs * 60/rpm * 1024
				tmp = (crank_revolutions - last_crank_revs) * 61440 / tsdz_status.ui8_pedal_cadence_RPM;
				tmp += last_crank_time;
				last_crank_time = tmp & 0xffff;
			}

            last_crank_revs = crank_revolutions;
            ESP_LOGI(TAG, "crank time:%d", last_crank_time);
            cscpMsg[i++] = (last_crank_time & 0xff);
            cscpMsg[i++] = ((last_crank_time>>8) & 0xff);

            ret = esp_ble_gatts_send_indicate(gatts_profile_tab[CYCLING_CSCP_PROFILE].gatts_if, gatts_profile_tab[CYCLING_CSCP_PROFILE].conn_id, cscp_handle_table[IDX_CHAR_VAL_CYCLING_CSCP_MEASUREMENT],
                    sizeof(cscpMsg), cscpMsg, false);
            if (ret)
                ESP_LOGE(TAG, "cycling_bt_update, cycling_cscp notifiation failed, error code = %x", ret);
            //else
            	//ESP_LOGI(TAG, "cycling_bt_update done");
        }
    }
}
void tsdz_bt_notify_command(uint8_t* data, uint8_t length) {
    // Client Charactersitic Configuration bit 0 set -> Notification enabled

    if (gatts_profile_tab[TSDZ_PROFILE].conn_id != 0xffff) {
        uint16_t l;
        const uint8_t* value;
        esp_err_t ret = esp_ble_gatts_get_attr_value(tsdz_handle_table[IDX_CHAR_CFG_COMMAND], &l, &value);
        if (ret == ESP_OK  && l==2 && (value[0]&0x01))
            ret = esp_ble_gatts_send_indicate(gatts_profile_tab[TSDZ_PROFILE].gatts_if, gatts_profile_tab[TSDZ_PROFILE].conn_id, tsdz_handle_table[IDX_CHAR_VAL_COMMAND],
                length, data, false);
        if (ret){
            ESP_LOGE(TAG, "tsdz_bt_notify_command, notification failed, error code = %x", ret);
        }
    }
}
