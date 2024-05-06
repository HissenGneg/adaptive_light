#ifndef __LIGHT_STRENGTH_CONTROL_H
#define __LIGHT_STRENGTH_CONTROL_H

#include "boardcfg.h"
#include "lib/CAN/CAN.h"
#include "typedefs.h"
#include "common.h"


#define N_LIGHT_READINGS 64
void add_light_reading(uint16_t reading);
void update_lights();
float get_light_reading_average();
#endif