#include "hardware/gpio.h"
#include "boardcfg.h"
#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/i2c.h"
#include "sensors.h"



void ltr_init()
{
    uint8_t val[]={0x80, 1};
    i2c_write_blocking(I2C_PORT, 0x29, val, 2, true);
    sleep_ms(50);
    val[0]=0x85;
    val[1]=0x12;
    i2c_write_blocking(I2C_PORT, 0x29, val, 2, true);


}

bool ltr_has_new_data()
{
    uint8_t data;
    uint8_t addr=0xA0;

    i2c_write_blocking(I2C_PORT, 0x29, &addr,1,true);
    i2c_read_blocking(I2C_PORT, 0x29, &data, 1, false);
    printf("LTR Part ID: %hhu\n", data);

    addr=0x8C;
    
    i2c_write_blocking(I2C_PORT, 0x29, &addr,1, true);
    i2c_read_blocking(I2C_PORT, 0x29, &data, 1, false);

    
    return (((data&(1<<7))==0) && ((data&(1<<1))!=0));
}

uint16_t ltr_get_light_level()
{
    uint16_t measurement;
    uint8_t addr0=0x88;
    uint8_t addr1=0x89;
    uint8_t addr2=0x8A;
    uint8_t addr3=0x8B;

    uint8_t ch1_1, ch1_2, ch0_1, ch0_2=0;

    i2c_write_blocking(I2C_PORT,0x29, &addr0,1,true);
    i2c_read_blocking(I2C_PORT, 0x29, &ch1_1,1,false);
    
    i2c_write_blocking(I2C_PORT,0x29, &addr1,1,true);
    i2c_read_blocking(I2C_PORT, 0x29, &ch1_2,1,false);

    i2c_write_blocking(I2C_PORT,0x29, &addr2,1,true);
    i2c_read_blocking(I2C_PORT, 0x29, &ch0_1,1,false);

    i2c_write_blocking(I2C_PORT,0x29, &addr3,1,true);
    i2c_read_blocking(I2C_PORT, 0x29, &ch0_2,1,false);
    
    measurement=(((uint16_t)ch0_2)<<8)|ch0_1;
    return measurement;
}

void mc34_reg_write(uint8_t addr, uint8_t data)
{
    uint8_t val[]={addr, data};
    i2c_write_blocking(I2C_PORT, 0x4C, val, 2, true);
}
uint8_t mc34_reg_read(uint8_t addr)
{
    uint8_t val;
    i2c_write_blocking(I2C_PORT, 0x4C, &addr, 1, true);
    i2c_read_blocking(I2C_PORT, 0x4C, &val,1,false);
    return val;
}
void mc34_set_mode(uint8_t mode)
{
    uint8_t tmp = mc34_reg_read(0x07);
    tmp &= 0b11110000;
    tmp |= mode;
    mc34_reg_write(0x07, tmp);
}

void mc34_set_range(uint8_t range)
{
    mc34_set_mode(0b011);
    uint8_t data=mc34_reg_read(0x20);
    data &= 0b00000111;
    data |= (range << 4) & 0x70;
    mc34_reg_write(0x20, data);
}

void mc34_init()
{
    mc34_reg_write(0x07, 0b011);
    sleep_ms(10);
    mc34_reg_write(0x1C, 0x40);
    sleep_ms(50);
    mc34_reg_write(0x06, 0x00);
    mc34_reg_write(0x2B, 0x00);
    sleep_ms(10);
    mc34_reg_write(0x15, 0x00);
    sleep_ms(50);
    mc34_set_mode(0b011);
    sleep_ms(50);
    mc34_set_range(0b010); // 8g
    mc34_reg_write(0x08,0x13);
    mc34_set_mode(0b001);
}
int16_t mc34_get_accel(uint8_t lsbaddr)
{
    return (int16_t)(((int16_t)mc34_reg_read(lsbaddr+1))<<8)|mc34_reg_read(lsbaddr);
}

int16_t mc34_get_x()
{
    return mc34_get_accel(0x0D);
}
int16_t mc34_get_y()
{
    return mc34_get_accel(0x0F);
}
int16_t mc34_get_z()
{
    return mc34_get_accel(0x11);
}
void i2c_sensors_init()
{
    i2c_init(I2C_PORT, 100e3);
    gpio_set_function(GP_SCL, GPIO_FUNC_I2C);
    gpio_set_function(GP_SDA, GPIO_FUNC_I2C);

    ltr_init();
    mc34_init();
}
