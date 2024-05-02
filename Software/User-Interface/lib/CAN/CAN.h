#ifndef __CAN_H
#define __CAN_H
#include "pico/stdlib.h"
#include "hardware/spi.h"

typedef struct
{
	uint32_t id;
    bool extended_id;
    bool remote_frame;
    uint8_t length;
	uint8_t data[8];
} can_msg_t;

void can_init(uint MOSI, uint MISO, uint SCK, uint CS, spi_inst_t* spi_instance);
bool can_send_message(can_msg_t *msg);
bool can_get_message(can_msg_t *msg);

#endif
