#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "boardcfg.h"
#include "lib/CAN/CAN.h"
#include "typedefs.h"
#include "common.h"
#include "light_strength_control.h"

#define HEALTHY_TIMEOUT_US  500e3 //500ms timeout

led_module_data_t LEDS[4];
system_config_data_t current_config;
turn_enum current_turn;

bool should_tx = false;

void msg_handle_lamp(can_msg_t *msg)
{
    if (msg->data[CAN_POLL_ANSWER_UI_FIELD_MSGTYPE] != CAN_MSG_TYPE_POLL_ANSWER)
        return;
    if(msg->length!=CAN_POLL_ANSWER_LED_LENGTH)
        return;
    
    led_module_data_t* curr_led_module=&LEDS[can_id_to_index(msg->id)];

    curr_led_module->light_level = (((uint16_t)(msg->data[CAN_POLL_ANSWER_LED_FIELD_BRIGHTNESS_HI])) << 8) | (msg->data[CAN_POLL_ANSWER_LED_FIELD_BRIGHTNESS_LO]);
    curr_led_module->accel_x = ((float)(msg->data[CAN_POLL_ANSWER_LED_FIELD_ACCEL_X])) / 10;
    curr_led_module->accel_y = ((float)(msg->data[CAN_POLL_ANSWER_LED_FIELD_ACCEL_Y])) / 10;
    curr_led_module->accel_z = ((float)(msg->data[CAN_POLL_ANSWER_LED_FIELD_ACCEL_Z])) / 10;
    curr_led_module->temperature = ((float)(msg->data[CAN_POLL_ANSWER_LED_FIELD_TEMPERATURE])) / 2;

    curr_led_module->last_rx = time_us_64();
    curr_led_module->healthy = true;
    
    add_light_reading(curr_led_module->light_level);
}

void set_led_position_data(led_module_position_data_t *led, uint8_t data)
{
    led->is_front = (data & LED_POSITION_FRONT_MASK) != 0;
    led->is_back = (data & LED_POSITION_BACK_MASK) != 0;
    led->is_left = (data & LED_POSITION_LEFT_MASK) != 0;
    led->is_right = (data & LED_POSITION_RIGHT_MASK) != 0;
    led->can_indicate_turn = (led->is_left != led->is_right);
}

void msg_handle_user_interface(can_msg_t *msg)
{
    if (msg->data[CAN_POLL_ANSWER_UI_FIELD_MSGTYPE] != CAN_MSG_TYPE_POLL_ANSWER)
        return;

    current_turn = msg->data[CAN_POLL_ANSWER_UI_FIELD_TURNSIGNAL];
    current_config.brightness_adjustment = msg->data[CAN_POLL_ANSWER_UI_FIELD_BRIGHTNESSADJUST];
    printf("Turn: %hhx\n", current_turn);
    //set_led_position_data(&LEDS[0].position, (msg->data[CAN_POLL_ANSWER_UI_FIELD_LIGHT_POS_1]) >> 4);
    //set_led_position_data(&LEDS[1].position, (msg->data[CAN_POLL_ANSWER_UI_FIELD_LIGHT_POS_1]) & 0x0F);
    //set_led_position_data(&LEDS[2].position, (msg->data[CAN_POLL_ANSWER_UI_FIELD_LIGHT_POS_2]) >> 4);
    //set_led_position_data(&LEDS[3].position, (msg->data[CAN_POLL_ANSWER_UI_FIELD_LIGHT_POS_2]) & 0x0F);
}


uint64_t can_task_index = 0;
can_msg_t txmsg;

void can_tx()
{
    memset(&txmsg, 0, sizeof(can_msg_t));

    txmsg.id = CAN_ID_MASTER;
    txmsg.extended_id=false;
    txmsg.remote_frame=false;
    can_task_index += 1;
    bool transmit=true;

    switch (can_task_index % 10)
    {
    case 0: // broadcast
        txmsg.data[CAN_BROADCAST_FIELD_MSGTYPE] = CAN_MSG_TYPE_BROADCAST;
        adc_select_input(1); 
        txmsg.data[CAN_BROADCAST_FIELD_BATT_VOLTAGE] = (10*(float)adc_read()*0.0090585);    // battery level
        txmsg.data[CAN_BROADCAST_FIELD_IS_CHARGING] = 0;                                    //Charging not working
        txmsg.length = CAN_BROADCAST_LENGTH;
        break;
    case 1: // poll LED module 1
        txmsg.data[CAN_POLL_FIELD_MSGTYPE] = CAN_MSG_TYPE_POLL;
        txmsg.data[CAN_POLL_FIELD_RX_ID] = CAN_ID_LED_1;
        LEDS[0].last_tx = time_us_64();
        txmsg.length = 2;
        break;
    case 2: // poll LED module 2
        txmsg.data[CAN_POLL_FIELD_MSGTYPE] = CAN_MSG_TYPE_POLL;
        txmsg.data[CAN_POLL_FIELD_RX_ID] = CAN_ID_LED_2;
        LEDS[1].last_tx = time_us_64();
        txmsg.length = 2;
        break;
    case 3: // poll LED module 3
        txmsg.data[CAN_POLL_FIELD_MSGTYPE] = CAN_MSG_TYPE_POLL;
        txmsg.data[CAN_POLL_FIELD_RX_ID] = CAN_ID_LED_3;
        LEDS[2].last_tx = time_us_64();
        txmsg.length = 2;
        break;
    case 4: //  poll LED module 4
        txmsg.data[CAN_POLL_FIELD_MSGTYPE] = CAN_MSG_TYPE_POLL;
        txmsg.data[CAN_POLL_FIELD_RX_ID] = CAN_ID_LED_4;
        LEDS[3].last_tx = time_us_64();
        txmsg.length = 2;
        break;
    case 5: // poll user interface
        txmsg.data[CAN_POLL_FIELD_MSGTYPE] = CAN_MSG_TYPE_POLL;
        txmsg.data[CAN_POLL_FIELD_RX_ID] = CAN_ID_USER_INTERFACE;
        txmsg.length = 2;
        break;
    case 9:
        update_lights();
        transmit=false;
        break;

    default:
        transmit=false;
        break;
    }

    if(transmit) while(!can_send_message(&txmsg));
    
    for (size_t i = 0; i < 4; i++)
    {
        led_module_data_t* led = &LEDS[i];
        if(led->last_rx<(time_us_64()-HEALTHY_TIMEOUT_US))
        {
            led->healthy=false;
        }
    }
}



void init()
{
    stdio_init_all();
    sleep_ms(2000);
    printf("start\n");
    gpio_init(GP_POWER_OUT_EN);
    gpio_set_dir(GP_POWER_OUT_EN, GPIO_OUT);
    gpio_put(GP_POWER_OUT_EN, 1);

    adc_init();

    can_init(GP_MOSI, GP_MISO, GP_SCK, GP_CSn, SPI_PORT);
}

bool can_tx_timer_task(struct repeating_timer *t)
{
    should_tx = true;
    return true;
}

void core1_task()
{
    struct repeating_timer tx_timer;
    add_repeating_timer_ms(100, can_tx_timer_task, NULL, &tx_timer);

    can_msg_t msg;
    while (true)
    {
        if (should_tx)
        {
            can_tx();
            should_tx = false;
        }
        sleep_ms(5); //allow for answer to be processed
        bool message_ok = can_get_message(&msg);
        if (message_ok)
        {
            switch (msg.id)
            {
            case CAN_ID_LED_1:
            case CAN_ID_LED_2:
            case CAN_ID_LED_3:
            case CAN_ID_LED_4:
                msg_handle_lamp(&msg);
                break;
            case CAN_ID_USER_INTERFACE:
                msg_handle_user_interface(&msg);
            default:
                break;
            }
        }
        sleep_ms(1);
    }
}

int main()
{
    init();


    LEDS[0].position.can_indicate_turn = true;
    LEDS[0].position.is_back = false;
    LEDS[0].position.is_front = true;
    LEDS[0].position.is_left = true;
    LEDS[0].position.is_right = false;

    LEDS[1].position.can_indicate_turn = true;
    LEDS[1].position.is_back = false;
    LEDS[1].position.is_front = true;
    LEDS[1].position.is_left = false;
    LEDS[1].position.is_right = true;

    LEDS[2].position.can_indicate_turn = true;
    LEDS[2].position.is_back = true;
    LEDS[2].position.is_front = false;
    LEDS[2].position.is_left = true;
    LEDS[2].position.is_right = false;

    LEDS[3].position.can_indicate_turn = true;
    LEDS[3].position.is_back = true;
    LEDS[3].position.is_front = false;
    LEDS[3].position.is_left = false;
    LEDS[3].position.is_right = true;

    multicore_launch_core1(core1_task);


    while (true)
    {
        //current_turn=TURN_LEFT;
        //sleep_ms(2000);
        //current_turn=TURN_RIGHT;
        //sleep_ms(2000);
        //current_turn=PARK_ANYWHERE_YOU_WANT;
        sleep_ms(2000);
    }
}
