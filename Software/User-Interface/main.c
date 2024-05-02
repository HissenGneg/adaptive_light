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
bool btn_left_clicked=false;
bool btn_right_clicked=false;
bool btn_vr_clicked=false;
time_t last_btn_left=0;
time_t last_btn_right=0;
time_t last_vr_btn=0;

ssd1306_t disp;

float battery_voltage=0;

void gpio_irq_callback(uint gpio, uint32_t events)
{
    switch (gpio)
    {
    case GP_BTN_LEFT:
        if(time_us_64()-last_btn_left>DEBOUNCE_TIME_US)
        {
            last_btn_left=time_us_64();
            btn_left_clicked=true;
        }
        break;
    case GP_BTN_RIGHT:
        if(time_us_64()-last_btn_right>DEBOUNCE_TIME_US)
        {
            last_btn_right=time_us_64();
            btn_right_clicked=true;
        }
        break;
    case GP_VR_BTN:
        if(time_us_64()-last_vr_btn>DEBOUNCE_TIME_US)
        {
            last_vr_btn=time_us_64();
            btn_vr_clicked=true;
        }
        break;
    
    default:
        break;
    }
}

void init()
{
    stdio_init_all();

    adc_init();

    i2c_init(i2c0, 400000);
    gpio_set_function(GP_SCL, GPIO_FUNC_I2C);
    gpio_set_function(GP_SDA, GPIO_FUNC_I2C);

    ssd1306_init(&disp, 128, 32, 0x3C, i2c0);
    ssd1306_invert(&disp,true);
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

    gpio_set_irq_enabled_with_callback(GP_BTN_LEFT, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
    gpio_set_irq_enabled_with_callback(GP_BTN_RIGHT, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
    gpio_set_irq_enabled_with_callback(GP_VR_BTN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
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
//[turn signal][global brightness][light positions]
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

    msg.data[2]=128;
    msg.data[3]=0;
    msg.data[4]=0;
    msg.length=5;
    can_send_message(&msg);
}

void process_broadcast(can_msg_t* msg)
{
    //[Battery voltage][is charging]
    //adc_read() >> 4
    //0->0V
    //255->3.3V
    //33V
    //printf("SKRRT %hhu\n", msg->data[1]);
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
            add_alarm_in_ms(3000, turn_reset_callback, NULL, false);
        }
        if(right_counter>50&&right_counter<1000)
        {
            current_turn=TURN_RIGHT;
            add_alarm_in_ms(3000, turn_reset_callback, NULL, false);
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
                printf("AAAAAAAA\n");
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
    }
}

int main()
{
    init();

    multicore_launch_core1(core1_task);

    char disp_msg[32];
    
    
    while (true)
    {
        sprintf(disp_msg, "Batt: %.1f",battery_voltage);
        //sprintf(disp_msg,"BTN: %s",gpio_get(GP_BTN_LEFT)?gpio_get(GP_BTN_RIGHT)?"X":"R":"L");
        ssd1306_draw_string(&disp, 8, 8, 2, disp_msg);
        ssd1306_show(&disp);
        sleep_ms(100);
        ssd1306_clear(&disp);
        
    }
}
