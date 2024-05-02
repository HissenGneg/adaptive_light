#ifndef __CONFIG_H
#define __CONFIG_H

#define CAN_ID_MASTER           0
#define CAN_ID_USER_INTERFACE   1
#define CAN_ID_LED_1            2
#define CAN_ID_LED_2            4
#define CAN_ID_LED_3            8
#define CAN_ID_LED_4            16

#define CAN_MSG_TYPE_POLL           0x01
#define CAN_MSG_TYPE_BROADCAST      0x02
#define CAN_MSG_TYPE_SET_LAMP       0x03
#define CAN_MSG_TYPE_POLL_ANSWER    0x01

#define CAN_POLL_FIELD_MSGTYPE  0
#define CAN_POLL_FIELD_RX_ID    1
#define CAN_POLL_LENGTH         1

#define CAN_BROADCAST_FIELD_MSGTYPE         0
#define CAN_BROADCAST_FIELD_BATT_VOLTAGE    1
#define CAN_BROADCAST_FIELD_IS_CHARGING     2
#define CAN_BROADCAST_LENGTH                3

#define CAN_SETLAMP_FIELD_MSGTYPE   0
#define CAN_SETLAMP_FIELD_LAMP_ID   1
#define CAN_SETLAMP_FIELD_WHITE     2
#define CAN_SETLAMP_FIELD_RED       3
#define CAN_SETLAMP_FIELD_AMBER     4
#define CAN_SETLAMP_LENGTH          5

#define CAN_POLL_ANSWER_LED_FIELD_MSGTYPE       0
#define CAN_POLL_ANSWER_LED_FIELD_BRIGHTNESS_HI 1
#define CAN_POLL_ANSWER_LED_FIELD_BRIGHTNESS_LO 2
#define CAN_POLL_ANSWER_LED_FIELD_ACCEL_X       3
#define CAN_POLL_ANSWER_LED_FIELD_ACCEL_Y       4
#define CAN_POLL_ANSWER_LED_FIELD_ACCEL_Z       5
#define CAN_POLL_ANSWER_LED_FIELD_TEMPERATURE   6
#define CAN_POLL_ANSWER_LED_LENGTH              7

#define CAN_POLL_ANSWER_UI_FIELD_MSGTYPE            0
#define CAN_POLL_ANSWER_UI_FIELD_TURNSIGNAL         1
#define CAN_POLL_ANSWER_UI_FIELD_BRIGHTNESSADJUST   2
#define CAN_POLL_ANSWER_UI_FIELD_LIGHT_POS_1        3
#define CAN_POLL_ANSWER_UI_FIELD_LIGHT_POS_2        4
#define CAN_POLL_ANSWER_UI_LENGTH                   5

#define LED_POSITION_FRONT_MASK     1<<0
#define LED_POSITION_BACK_MASK      1<<1
#define LED_POSITION_LEFT_MASK      1<<2
#define LED_POSITION_RIGHT_MASK     1<<3


#define GP_USB_PG   3
#define GP_STAT1    10
#define GP_STAT2    11

#define GP_MISO     16
#define GP_CSn      17
#define GP_SCK      18
#define GP_MOSI     19
#define SPI_PORT    spi0

#define GP_VBUS_ADC 26
#define GP_BATT_ADC 27

#define GP_POWER_OUT_EN 23

#endif