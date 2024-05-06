
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
    size_t average_count= (light_readings_index < N_LIGHT_READINGS) ? light_readings_index:N_LIGHT_READINGS;
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

    float adjustment = current_config.brightness_adjustment-128;
    float average = get_light_reading_average();
    float white=100-0.2*average+adjustment/2;
    printf("Adjust: %hhu, white=%f\n", current_config.brightness_adjustment,white);
    if(white<0)
        white=0;
    if(white>128)
        white=128;

    float amber=28-0.1*average;
    if(amber>128)
        amber=128;
    if(amber<32)
        amber=32;
    
    uint8_t white_brightness = white;
    uint8_t amber_brightness = amber;
    uint8_t red_brightness = white/2;

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
        if (LEDS[i].position.can_indicate_turn)
        {
            bool canUpdate= ((position_data->is_left&&(current_turn&TURN_LEFT))||(position_data->is_right&&(current_turn&TURN_RIGHT)));
            amber = canUpdate ? (turn_signal_state ? amber_brightness : 0) : 0;
        }
        msg.data[CAN_SETLAMP_FIELD_RED] = (current_turn!=TURN_NONE)? 0 : LEDS[i].position.is_front?0:red_brightness; // R
        msg.data[CAN_SETLAMP_FIELD_AMBER] = amber;
        
        while (!can_send_message(&msg))
            ;
        sleep_ms(10);
    }
}