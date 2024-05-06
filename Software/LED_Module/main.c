#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#include "pico/multicore.h"
#include "hardware/pwm.h"

#include <string.h>

#include "boardcfg.h"
#include "main_LED_module.h"
#include "lib/CAN/CAN.h"

#include "sensors.h"

uint slice_1;
uint slice_2;

uint8_t light_hi = 0;
uint8_t light_lo = 0;
uint8_t accel_x, accel_y, accel_z = 0;
float battery_voltage;
bool is_charging;

void setup_pwm()
{
    gpio_set_function(GP_PWM_WHITE, GPIO_FUNC_PWM); // Set pin functions to make the PWM slice output go out to pins
    gpio_set_function(GP_PWM_RED, GPIO_FUNC_PWM);
    gpio_set_function(GP_PWM_AMBER, GPIO_FUNC_PWM);

    slice_1 = pwm_gpio_to_slice_num(GP_PWM_WHITE); // Get first slice
    slice_2 = pwm_gpio_to_slice_num(GP_PWM_AMBER); // Get second slice (2 pins per slice)

    pwm_set_chan_level(slice_1, 0, 0); // Make sure outputs are off by default
    pwm_set_chan_level(slice_1, 1, 0);
    pwm_set_chan_level(slice_2, 0, 0);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 1024);
    pwm_config_set_clkdiv_int(&config, 64); // around 30kHz, no audible coil whine!
    pwm_init(slice_1, &config, true);
    pwm_init(slice_2, &config, true);

    pwm_set_enabled(slice_1, true);
    pwm_set_enabled(slice_2, true);
}
void set_white_brightness(uint8_t brightness)
{
    uint16_t brightness_scaled = (uint16_t)brightness * 4;
    if (brightness_scaled > PWM_MAX)
        return;
    pwm_set_chan_level(slice_1, PWM_CHAN_A, brightness_scaled);
}
void set_red_brightness(uint8_t brightness)
{
    uint16_t brightness_scaled = (uint16_t)brightness * 4;
    if (brightness_scaled > PWM_MAX)
        return;
    pwm_set_chan_level(slice_1, PWM_CHAN_B, brightness_scaled);
}
void set_amber_brightness(uint8_t brightness)
{
    uint16_t brightness_scaled = (uint16_t)brightness * 4;
    if (brightness_scaled > PWM_MAX)
        return;
    pwm_set_chan_level(slice_2, PWM_CHAN_A, brightness_scaled);
}
float get_temperature()
{
    return (adc_read() - 4096 * (0.4f / 3.3f)) * (4096 / 3300) / (19.5f);
}
void reply_poll()
{
    can_msg_t msg_tx;
    msg_tx.id = CAN_ID;
    msg_tx.data[0] = 0x01;
    msg_tx.data[1] = light_hi;
    msg_tx.data[2] = light_lo;
    msg_tx.data[3] = accel_x;
    msg_tx.data[4] = accel_y;
    msg_tx.data[5] = accel_z;
    msg_tx.data[6] = (uint8_t)(get_temperature() * 2);
    msg_tx.length = 7;
    msg_tx.remote_frame = false;
    msg_tx.extended_id = false;
    can_send_message(&msg_tx);
}

void process_message(can_msg_t *msg)
{
    switch (msg->data[0])
    {
    case 0x01:
        if (msg->data[1] == CAN_ID)
            reply_poll();
        break;
    case 0x02: // broadcast
        battery_voltage = msg->data[1] * 8;
        is_charging = msg->data[2] > 0;
        break;
    case 0x03:

        if (msg->data[1] == CAN_ID)
        {
            set_white_brightness(msg->data[2]);
            set_red_brightness(msg->data[3]);
            set_amber_brightness(msg->data[4]);
        }
        break;
    default:
        break;
    }
}

void init()
{
    gpio_init(GP_STATUS1);
    gpio_init(GP_STATUS2);
    gpio_init(GP_STATUS3);
    gpio_set_dir(GP_STATUS1, GPIO_OUT);
    gpio_set_dir(GP_STATUS2, GPIO_OUT);
    gpio_set_dir(GP_STATUS3, GPIO_OUT);
    gpio_set_drive_strength(GP_STATUS1, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(GP_STATUS2, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(GP_STATUS3, GPIO_DRIVE_STRENGTH_12MA);

    adc_init();
    adc_select_input(0);

    stdio_init_all();
    setup_pwm();
    can_init(GP_MOSI, GP_MISO, GP_SCK, GP_CSn, SPI_PORT);
    sleep_ms(1500);
    i2c_sensors_init();
    gpio_put(GP_STATUS3, 0);
}

int main()
{
    init();
    can_msg_t msg_rx;

    while (true)
    {
        uint16_t light_level = ltr_get_light_level();
        light_hi = (uint8_t)(light_level >> 8);
        light_lo = (uint8_t)(light_level & 0xFF);
        accel_x = (uint8_t)(mc34_get_x() * 0.0239429f); //Convert to m/s^2
        accel_y = (uint8_t)(mc34_get_y() * 0.0239429f);
        accel_z = (uint8_t)(mc34_get_z() * 0.0239429f);

        if (get_temperature() > 60.0f)
        {
            set_amber_brightness(0);
            set_red_brightness(0);
            set_white_brightness(0);
            while (get_temperature() > 30.0f)
            {
                sleep_ms(100);
                printf("OVERHEAT!!! cooling down, temp: %f\n", adc_read(), get_temperature());
            }
        }
        if (can_get_message(&msg_rx))
        {

            gpio_put(GP_STATUS1, 1);

            if (msg_rx.id == CAN_ID_MASTER)
            {
                process_message(&msg_rx);
            }
            gpio_put(GP_STATUS1, 0);
        }
    }
}

/*
PWM 32/255:  25V 45mA  1.125W in -> 750mW ut, 67%
PWM 64/255:  25V 103mA 2.575W in -> 2.1W ut , 81%
PWM 128/255: 25V 227mA 5.675W in -> 5.03W ut, 88%
*/