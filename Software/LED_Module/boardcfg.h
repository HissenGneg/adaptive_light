/* 
 * CAN message:
 *  Controller -> module
 *  Byte 0:
 *  0x01: Poll
 *  0x02: Broadcast
 *  0x03: Set lamp
 * ========
 *  Poll: 0x01
 *      [module ID]
 *  Broadcast: 0x02
 *      [Battery voltage][is charging]
 *  Set lamp 0x03:
 *      [lamp ID][White][Red][Amber]
 *  
 * Module -> controller
 *  0x01: Poll answer
 *  0x02: Nothing
 *  ========
 *  Poll answer:
 *      LED:
 *          [brightness hi][brightness lo][accel X*10][accel Y*10][accel Z*10]
 *      User interface:
 *          [turn signal][global brightness][light positions]
 *                                          -> 0bAABBCCDD AA->lamp 1, BB->lamp2 ...
 *                                               -> 00=front right, 01=front left, 10=back right, 11=back left
 * 0x03: nothing
 * 
*/

#define CAN_ID_MASTER 0
#define CAN_ID_USER_INTERFACE 1
#define CAN_ID_LED_1 2
#define CAN_ID_LED_2 4
#define CAN_ID_LED_3 8
#define CAN_ID_LED_4 16

#define MODULE_NUM 1

#define CAN_ID CAN_ID_LED_1
#define GP_SDA 4
#define GP_SCL 5

#define GP_PWM_WHITE 10
#define GP_PWM_RED 11
#define GP_PWM_AMBER 12

#define GP_STATUS1 15
#define GP_STATUS2 14
#define GP_STATUS3 13

#define GP_MISO 16
#define GP_CSn 17
#define GP_SCK 18
#define GP_MOSI 19

#define GP_TEMP_ADC 26

#define PWM_MAX 1024
#define PWM_CLKDIV 6

#define SPI_PORT spi0
#define I2C_PORT i2c0
