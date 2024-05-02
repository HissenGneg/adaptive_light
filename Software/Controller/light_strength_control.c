
#include "boardcfg.h"
#include "lib/CAN/CAN.h"
#include "typedefs.h"
#include "common.h"
#include "light_strength_control.h"
#include "stdio.h"

uint16_t light_readings[N_LIGHT_READINGS];
uint64_t light_readings_index = 0;
uint64_t turn_signal_timer_cnt = 0;


void add_light_reading(uint16_t reading)
{
    light_readings[light_readings_index % N_LIGHT_READINGS] = reading;
    light_readings_index += 1;
}

float get_light_reading_average()
{
    if(unlikely(light_readings_index==0))
        return 0;
    if(unlikely(light_readings_index==1))
        return light_readings[0];

    float total = 0;
    size_t average_count= (light_readings_index < N_LIGHT_READINGS) ? N_LIGHT_READINGS : light_readings_index;
    for (size_t i = 0; i < average_count; i += 1)
    {
        total += light_readings[i];
    }
    return total/average_count;
}

void update_lights()
{
    turn_signal_timer_cnt += 1;
    bool turn_signal_state = (turn_signal_timer_cnt % 10) < 5;

    uint8_t adjustment = current_config.brightness_adjustment;
    float average = get_light_reading_average();

    uint8_t white_brightness = 0;
    uint8_t amber_brightness = 100;
    uint8_t red_brightness = 0;

     //if(average<2048)
     //    white_brightness=128;
     //else if (average < 4096)
     //    white_brightness = 64;
     //else if(average<8192)
     //    white_brightness=32;
     //else
         white_brightness = 0;

    can_msg_t msg;
    msg.id = CAN_ID_MASTER;
    msg.extended_id=false;
    msg.remote_frame=false;
    msg.length = CAN_SETLAMP_LENGTH;

    msg.data[CAN_SETLAMP_FIELD_MSGTYPE] = CAN_MSG_TYPE_SET_LAMP;
    sleep_ms(10);
    for (size_t i = 0; i < 4; i++)
    {
        msg.data[CAN_SETLAMP_FIELD_LAMP_ID] = index_to_can_id(i);
        led_module_position_data_t *position_data = &LEDS[i].position;
        msg.data[CAN_SETLAMP_FIELD_WHITE] = LEDS[i].position.is_front ? white_brightness : 0;

        uint8_t amber = 0;
        if (position_data->can_indicate_turn)
        {
            bool canUpdate= ((position_data->is_left&&(current_turn&TURN_LEFT))||(position_data->is_right&&(current_turn&TURN_RIGHT)));
            amber = canUpdate ? (turn_signal_state ? amber_brightness : 0) : 0;
        }
        msg.data[CAN_SETLAMP_FIELD_RED] = 0; // R
        msg.data[CAN_SETLAMP_FIELD_AMBER] = amber;
        while (!can_send_message(&msg))
            ;
        sleep_ms(10);
    }
}