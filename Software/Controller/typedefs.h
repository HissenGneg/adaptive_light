#include "pico/stdlib.h"
#include <stdio.h>

#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H


typedef struct

{
    bool is_front;
    bool is_back;
    bool is_left;
    bool is_right;
    bool can_indicate_turn;
} led_module_position_data_t;

typedef struct
{
    uint16_t light_level;
    float accel_x;
    float accel_y;
    float accel_z;
    float temperature;
    time_t last_rx;
    time_t last_tx;
    bool healthy;
    led_module_position_data_t position;
} led_module_data_t;

typedef struct
{
    uint8_t brightness_adjustment;
    bool indicate_left;
    bool indicate_right;
} system_config_data_t;

typedef enum
{
    TURN_NONE = 0b00,
    TURN_LEFT = 0b01,
    TURN_RIGHT = 0b10,
    PARK_ANYWHERE_YOU_WANT = 0b11
} turn_enum;

#endif