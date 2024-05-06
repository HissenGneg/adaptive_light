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
#include "lib/pico-ssd1306/ssd1306.h"

uint16_t get_joystick_x()
{
    adc_select_input(1);
    return adc_read();
}
uint16_t get_joystick_y()
{
    adc_select_input(0);
    return adc_read();
}

#define DEBOUNCE_TIME_US 50000

ssd1306_t disp;

float battery_voltage=0;
uint8_t brightness_adjust=128;

void init()
{
    stdio_init_all();

    adc_init();

    i2c_init(i2c0, 400000);
    gpio_set_function(GP_SCL, GPIO_FUNC_I2C);
    gpio_set_function(GP_SDA, GPIO_FUNC_I2C);

    ssd1306_init(&disp, 128, 32, 0x3C, i2c0);
    ssd1306_clear(&disp);

    can_init(GP_MOSI, GP_MISO, GP_SCK, GP_CSn,SPI_PORT);

    gpio_init(GP_BTN_LEFT);
    gpio_init(GP_BTN_RIGHT);
    gpio_init(GP_VR_BTN);

    gpio_set_dir(GP_BTN_LEFT, GPIO_IN);
    gpio_set_dir(GP_BTN_RIGHT, GPIO_IN);
    gpio_set_dir(GP_VR_BTN, GPIO_IN);
    
    gpio_set_pulls(GP_BTN_LEFT, true, false);
    gpio_set_pulls(GP_BTN_RIGHT, true, false);
    gpio_set_pulls(GP_VR_BTN, true, false);
}
typedef enum
{
    TURN_NONE = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    PARK_ANYWHERE_YOU_WANT = 4
} turn_enum;

turn_enum current_turn=TURN_NONE;

void reply_poll()
{
    can_msg_t msg;
    msg.id=CAN_ID_USER_INTERFACE;
    msg.remote_frame=false;
    msg.extended_id=false;
    msg.data[0]=0x01;
    msg.data[1]=current_turn;
    //set_led_position_data(&LEDS[0].position, (msg->data[3]) >> 4);
    //set_led_position_data(&LEDS[1].position, (msg->data[3]) & 0x0F);
    //set_led_position_data(&LEDS[2].position, (msg->data[4]) >> 4);
    //set_led_position_data(&LEDS[3].position, (msg->data[4]) & 0x0F);

    msg.data[2]=brightness_adjust;
    msg.data[3]=0;
    msg.data[4]=0;
    msg.length=5;
    can_send_message(&msg);
}

void process_broadcast(can_msg_t* msg)
{
    battery_voltage=((float)(msg->data[1]))*(0.1f);
}

uint32_t left_counter=0;
uint32_t right_counter=0;

int64_t turn_reset_callback(alarm_id_t id, void *user_data)
{
    current_turn=TURN_NONE;
    return 0;
}

void process_turn(bool falling)
{
    if(falling)
    {
        if(left_counter>50&&left_counter<1000)
        {
            current_turn=TURN_LEFT;
            add_alarm_in_ms(4000, turn_reset_callback, NULL, false);
        }
        if(right_counter>50&&right_counter<1000)
        {
            current_turn=TURN_RIGHT;
            add_alarm_in_ms(4000, turn_reset_callback, NULL, false);
        }
        if(left_counter>1000||right_counter>1000)
            current_turn=TURN_NONE;
    }
    else
    {
        if(left_counter>50)
            current_turn=TURN_LEFT;
        if(right_counter>50)
            current_turn=TURN_RIGHT;
    }
}

void core1_task()
{
    can_msg_t msg;
    while (true)
    {
        bool message_ok = can_get_message(&msg);
        if (message_ok && msg.id==CAN_ID_MASTER)
        {
            
            switch(msg.data[0])
            {
            case 0x01: //POLL
            if(msg.data[1]==CAN_ID_USER_INTERFACE)
            {
                reply_poll(&msg);
            }
                break;
            case 0x02:
                process_broadcast(&msg);
            default:
                break;
            }
        }
        sleep_ms(1);
        if(!gpio_get(GP_BTN_LEFT))
            left_counter+=1;
        else
        {
            process_turn(true);
            left_counter=0;
        }
        if(!gpio_get(GP_BTN_RIGHT))
            right_counter+=1;
        else
        {
            process_turn(true);
            right_counter=0;
        }
        process_turn(false);

        if(get_joystick_y()<1024)
        {
            if(brightness_adjust>10)
                brightness_adjust-=1;
            sleep_ms(20);
        }
        else if(get_joystick_y()>3072)
        {
            if(brightness_adjust<245)
                brightness_adjust+=1;
            sleep_ms(20);
        }
    }
}

int main()
{
    init();

    multicore_launch_core1(core1_task);

    char disp_msg[32];
    
    
    while (true)
    {
        float percent_left=-2.319*battery_voltage*battery_voltage*battery_voltage + 158.9*battery_voltage*battery_voltage- 3593*battery_voltage + 2.684e+04;
        if(percent_left>100)
            percent_left=100;
        if(percent_left<0)
            percent_left=0;
        if(battery_voltage>=25)
            percent_left=100;
        if(battery_voltage<20)
            percent_left=0;
        ssd1306_draw_square(&disp,14,8,percent_left,16);
        ssd1306_draw_empty_square(&disp, 14,8,100,16);

        ssd1306_draw_square(&disp,14,25,(float)brightness_adjust*(100.0f/256.0f),6);
        ssd1306_draw_empty_square(&disp, 14,25,100,6);
        
        if(brightness_adjust<128)
            ssd1306_draw_line(&disp,64,25,64,31);
        if(brightness_adjust>128)
            ssd1306_clear_line(&disp,64,26,64,30);

        ssd1306_draw_string(&disp, 8, 0, 1, "Battery left:");
        ssd1306_show(&disp);
        sleep_ms(100);
        ssd1306_clear(&disp);
        
    }
}
