#include "typedefs.h"

#ifndef __COMMON_H
#define __COMMON_H

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

extern led_module_data_t LEDS[4];
extern system_config_data_t current_config;
extern turn_enum current_turn;

size_t can_id_to_index(uint8_t can_id);
size_t index_to_can_id(uint8_t index);
#endif