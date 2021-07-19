/*
 * tsdz_bt.h
 *
 *  Created on: 10 set 2019
 *      Author: Max
 */

#ifndef MAIN_TSDZ_BT_H_
#define MAIN_TSDZ_BT_H_

enum
{
    IDX_CSCP_SVC,

    IDX_CHAR_CYCLING_CSCP_MEASUREMENT,
    IDX_CHAR_VAL_CYCLING_CSCP_MEASUREMENT,
    IDX_CHAR_CFG_CYCLING_CSCP_MEASUREMENT,

    IDX_CHAR_CYCLING_CSCP_FEATURE,
    IDX_CHAR_VAL_CYCLING_CSCP_FEATURE,

    IDX_CSCP_DB_NUM
};

enum
{
    IDX_CYCLING_SVC,

    IDX_CHAR_CYCLING_POWER_MEASUREMENT,
    IDX_CHAR_VAL_CYCLING_POWER_MEASUREMENT,
    IDX_CHAR_CFG_CYCLING_POWER_MEASUREMENT,

    IDX_CHAR_CYCLING_POWER_FEATURE,
    IDX_CHAR_VAL_CYCLING_POWER_FEATURE,

    IDX_CHAR_SENSOR_LOCATION,
    IDX_CHAR_VAL_SENSOR_LOCATION,

    IDX_CYCLING_DB_NUM
};

enum
{
    IDX_TSDZ_SVC,

    IDX_CHAR_STATUS,
    IDX_CHAR_VAL_STATUS,
    IDX_CHAR_CFG_STATUS,

    IDX_CHAR_DEBUG,
    IDX_CHAR_VAL_DEBUG,
    IDX_CHAR_CFG_DEBUG,

    IDX_CHAR_CONFIG,
    IDX_CHAR_VAL_CONFIG,

    IDX_CHAR_COMMAND,
    IDX_CHAR_VAL_COMMAND,
    IDX_CHAR_CFG_COMMAND,

    IDX_TSDZ_DB_NUM
};

volatile uint8_t btCommandReady;

void tsdz_bt_init(void);
void tsdz_bt_update(void);
void cycling_bt_update(void);
void cscp_bt_update(void);
void tsdz_bt_notify_command(uint8_t* value, uint8_t length);
void tsdz_bt_stop(void);

#endif /* MAIN_TSDZ_BT_H_ */
