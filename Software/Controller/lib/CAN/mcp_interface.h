#ifndef __MCP_INTERFACE
#define __MCP_INTERFACE
#include "CAN.h"

#define MCP_CMD_RESET           0b11000000
#define MCP_CMD_READ            0b00000011
#define MCP_CMD_READ_RXBUF(n)   ((n==1)?0b10010010:0b10010000) // CANINTF will be cleared after setting CS high!

#define MCP_CMD_WRITE           0b00000010

#define MCP_CMD_LOADTXBUF_000   0b01000000
#define MCP_CMD_LOADTXBUF_001   0b01000001
#define MCP_CMD_LOADTXBUF_010   0b01000010
#define MCP_CMD_LOADTXBUF_011   0b01000011
#define MCP_CMD_LOADTXBUF_100   0b01000100
#define MCP_CMD_LOADTXBUF_101   0b01000101
#define MCP_CMD_LOADTXBUF_110   0b01000110
#define MCP_CMD_LOADTXBUF_111   0b01000111

#define MCP_CMD_RTS_TXB(n)      (0b10000000|(1<<n))

#define MCP_CMD_READ_STATUS     0b10100000
#define MCP_FLAG_CMDRS_TX2IF    1<<7            //TX buf 2 is full
#define MCP_FLAG_CMDRS_TX2REQ   1<<6            //TX buf 2 is pending transmission
#define MCP_FLAG_CMDRS_TX1IF    1<<5            //TX buf 1 is full
#define MCP_FLAG_CMDRS_TX1REQ   1<<4            //TX buf 1 is pending transmission
#define MCP_FLAG_CMDRS_TX0IF    1<<3            //TX buf 0 is full
#define MCP_FLAG_CMDRS_TX0REQ   1<<2            //TX buf 0 is pending transmission
#define MCP_FLAG_CMDRS_RX1IF    1<<1            //RX buf 1 is full
#define MCP_FLAG_CMDRS_RX0IF    1<<0            //RX buf 0 is full

#define MCP_CMD_RX_STATUS       0b10110000
#define MCP_CMD_BIT_MODIFY      0b00000101


#define MCP_ADDR_CNF3               0x28
#define MCP_ADDR_CNF2               0x29
#define MCP_ADDR_CNF1               0x2A

#define MCP_ADDR_TEC                0x1C
#define MCP_ADDR_REC                0x1D

#define MCP_ADDR_EFLG               0x2D
#define MCP_FLAG_EFLG_RX1OVR        1<<7
#define MCP_FLAG_EFLG_RX0OVR        1<<6
#define MCP_FLAG_EFLG_TXBO          1<<5
#define MCP_FLAG_EFLG_TXEP          1<<4
#define MCP_FLAG_EFLG_RXEP          1<<3
#define MCP_FLAG_EFLG_TXWAR         1<<2
#define MCP_FLAG_EFLG_RXWAR         1<<1
#define MCP_FLAG_EFLG_EWARN         1<<0


#define MCP_ADDR_CANINTE            0x2B
#define MCP_FLAG_CANINTE_MERRE      1<<7            //Message Error Interrupt Enable bit
#define MCP_FLAG_CANINTE_WAKIE      1<<6            //Wake-up Interrupt Enable bit
#define MCP_FLAG_CANINTE_ERRIE      1<<5            //Error Interrupt Enable bit
#define MCP_FLAG_CANINTE_TX2IE      1<<4            //Transmit Buffer 2 Empty Interrupt Enable bit
#define MCP_FLAG_CANINTE_TX1IE      1<<3            //Transmit Buffer 1 Empty Interrupt Enable bit
#define MCP_FLAG_CANINTE_TX0IE      1<<2            //Transmit Buffer 0 Empty Interrupt Enable bit
#define MCP_FLAG_CANINTE_RX1IE      1<<1            //Receive Buffer 1 Full Interrupt Enable bit
#define MCP_FLAG_CANINTE_RX0IE      1<<0            //Receive Buffer 0 Full Interrupt Enable bit

#define MCP_ADDR_CANINTF            0x2C
#define MCP_FLAG_CANINTF_MERRF      1<<7            //Message Error Interrupt Enable Flag bit
#define MCP_FLAG_CANINTF_WAKIF      1<<6            //Wake-up Interrupt Enable Flag bit
#define MCP_FLAG_CANINTF_ERRIF      1<<5            //Error Interrupt Enable Flag bit
#define MCP_FLAG_CANINTF_TX2IF      1<<4            //Transmit Buffer 2 Empty Interrupt Enable Flag bit
#define MCP_FLAG_CANINTF_TX1IF      1<<3            //Transmit Buffer 1 Empty Interrupt Enable Flag bit
#define MCP_FLAG_CANINTF_TX0IF      1<<2            //Transmit Buffer 0 Empty Interrupt Enable Flag bit
#define MCP_FLAG_CANINTF_RX1IF      1<<1            //Receive Buffer 1 Full Interrupt Enable Flag bit
#define MCP_FLAG_CANINTF_RX0IF      1<<0            //Receive Buffer 0 Full Interrupt Enable Flag bit

#define MCP_ADDR_CANCTRL            0x0F            //CAN control flags
#define MCP_FLAG_CANCTRL_OP_NORML   0b00000000      //normal operation mode
#define MCP_FLAG_CANCTRL_OP_SLEEP   0b00100000      //sleep mode
#define MCP_FLAG_CANCTRL_OP_LPBCK   0b01000000      //loopback mode
#define MCP_FLAG_CANCTRL_OP_LISTN   0b01100000      //listen only mode
#define MCP_FLAG_CANCTRL_OP_CONFG   0b10000000      //configuration mode
#define MCP_MASK_CANCTRL_OP         0b11100000      //mode mask
#define MCP_FLAG_CANCTRL_ABORT      1<<4            //abort all pending transmissions. Illegal to use in 15 US states.
#define MCP_FLAG_CANCTRL_OSM        1<<3            //One-shot mode
#define MCP_FLAG_CANCTRL_CLKOUT     1<<2            //CLKOUT enabled flag
#define MCP_MASK_CANCTRL_CLKPRE     0b11            //CLKOUT prescaler bits

#define MCP_ADDR_CANSTAT            0x0E            //CAN status flags

#define MCP_LEN_TXBn                3               //TX buffer count
#define MCP_ADDR_TXBnCTRL(n)        (0x30+0x10*n)   //Get TX buffer n address
#define MCP_FLAG_TXBnCTRL_ABTF      1<<6            //Message Aborted Flag bit, 1 = Message was aborted, 0 = Message completed transmission successfully
#define MCP_FLAG_TXBnCTRL_MLOA      1<<5            //Message Lost Arbitration bit, 1 = Message lost arbitration while being sent, 0 = Message did not lose arbitration while being sent
#define MCP_FLAG_TXBnCTRL_TXERR     1<<4            //Transmission Error Detected bit, 1 = A bus error occurred while the message was being transmitted, 0 = No bus error occurred while the message was being transmitted
#define MCP_FLAG_TXBnCTRL_TXREQ     1<<3            //Message Transmit Request bit (MCU sets this bit to request message be transmitted, autoclears), 1 = Buffer is currently pending transmission, 0 = Buffer is not currently pending transmission
#define MCP_MASK_TXBnCTRL_TXP       0b11

#define MCP_ADDR_TXRTSCTRL          0x0D
#define MCP_FLAG_TXRTSCTRL_B2RTS    1<<5            //TX2RTSn Pin State bit
#define MCP_FLAG_TXRTSCTRL_B1RTS    1<<4            //TX1RTSn Pin State bit
#define MCP_FLAG_TXRTSCTRL_B0RTS    1<<3            //TX0RTSn Pin State bit
#define MCP_FLAG_TXRTSCTRL_B2RTSM   1<<2            //TX2RTSn Pin mode bit, 1 = Pin is used to request message transmission of TXB2 buffer (on falling edge), 0 = Digital input
#define MCP_FLAG_TXRTSCTRL_B1RTSM   1<<1            //TX1RTSn Pin mode bit, 1 = Pin is used to request message transmission of TXB2 buffer (on falling edge), 0 = Digital input
#define MCP_FLAG_TXRTSCTRL_B0RTSM   1<<0            //TX0RTSn Pin mode bit, 1 = Pin is used to request message transmission of TXB2 buffer (on falling edge), 0 = Digital input

#define MCP_ADDR_TXBnSIDH(n)        (0x31+0x10*n)   //TX buffer Standard ID High
#define MCP_ADDR_TXBnSIDL(n)        (0x32+0x10*n)   //TX buffer Standard ID Low, last 2 bits -> EID17,EID16
#define MCP_ADDR_TXBnEID8(n)        (0x33+0x10*n)   //TX buffer Extended ID EID15-EID8
#define MCP_ADDR_TXBnEID0(n)        (0x34+0x10*n)   //TX buffer Extended ID EID7-EID0
#define MCP_FLAG_TXBnSIDL_EXIDE     (1<<3)          //Extended Identifier Enable bit

#define MCP_ADDR_TXBnDLC(n)         (0x35+0x10*n)   //Data length code register
#define MCP_FLAG_TXBnDLC_RTR        (1<<6)          //Remote Transmission Request
#define MCP_MASK_TXBnDLC_DLC        0b111           //Message length mask

#define MCP_LEN_TXBnD               8
#define MCP_ADDR_TXBnD_BASE(n)      (0x36+0x10*n)   //Transmit buffer n data register base

#define MCP_ADDR_RXBnCTRL(n)        (0x60+0x10*n)   //Receive buffer 1 control register
#define MCP_FLAG_RXBn_RXRTR         1<<3            //Remote frame
#define MCP_MASK_RXBn_RXM           0b01100000      //RXM receive mask/filter mode
#define MCP_FLAG_RXBn_RXM_OFF       0b11<<5         //Turn receive mask/filter off
#define MCP_FLAG_RXBn_RXM_ON        0b00<<5         //Turn receive mask/filter on

#define MCP_FLAG_RXB0_BUKT_EN       1<<2            //Rollover enable bit
#define MCP_FLAG_RXB0_FILT1         1               //Message hit FILT1 instead of FILT0

#define MCP_FLAG_RXB1_FILT0         0b000           //Message hit FILT0 (BUKT needs to be set in RXB0CTRL)
#define MCP_FLAG_RXB1_FILT1         0b001           //Message hit FILT0 (BUKT needs to be set in RXB0CTRL)
#define MCP_FLAG_RXB1_FILT2         0b010           //Message hit FILT2
#define MCP_FLAG_RXB1_FILT3         0b011           //Message hit FILT3
#define MCP_FLAG_RXB1_FILT4         0b100           //Message hit FILT4
#define MCP_FLAG_RXB1_FILT5         0b101           //Message hit FILT5

#define MCP_ADDR_BFPCTRL            0x0C
#define MCP_FLAG_BFPCTRL_B1BFS      1<<5            //RX1BFn Pin State bit
#define MCP_FLAG_BFPCTRL_B0BFS      1<<4            //RX0BFn Pin State bit
#define MCP_FLAG_BFPCTRL_B1BFE_EN   1<<3            //RX1BFn Pin enable
#define MCP_FLAG_BFPCTRL_B0BFE_EN   1<<2            //RX0BFn Pin enable
#define MCP_FLAG_BFPCTRL_B1BFM_INT  1<<1            //RX1BFn Pin mode = interrupt
#define MCP_FLAG_BFPCTRL_B0BFM_INT  1<<0            //RX0BFn Pin mode = interrupt

#define MCP_LEN_RXBn                2
#define MCP_ADDR_RXBnSIDH(n)        (0x61+0x10*n)   //Receive buffer n standard ID register High
#define MCP_ADDR_RXBnSIDL(n)        (0x62+0x10*n)   //Receive buffer n standard ID register Low, last 2 bits -> EID17,EID16
#define MCP_ADDR_RXBnEID8(n)        (0x63+0x10*n)   //Receive buffer n extended ID register EID15-EID8
#define MCP_ADDR_RXBnEID0(n)        (0x64+0x10*n)   //Receive buffer n extended ID register EID7-EID0
#define MCP_FLAG_RXBnSIDL_SRR       1<<4            //Standard Frame Remote Transmit Request bit
#define MCP_FLAG_RXBnSIDL_IDE       1<<3            //Extended Identifier Flag bit

#define MCP_ADDR_RXBnDLC(n)         (0x65+0x10*n)   //Receive buffer n data length code register
#define MCP_FLAG_RXBnDLC_RTR        1<<6            //Extended Frame Remote Transmission Request bit
#define MCP_MASK_RXBnDLC_DLC        0b111           //Message length mask

#define MCP_LEN_RXBnD               8
#define MCP_ADDR_RXBnD_BASE(n)      (0x66+0x10*n)   //Receive buffer n data register base

#define MCP_LEN_RXF                 6               //rx filter count
#define MCP_ADDR_RXFSIDH(n)         (0x00+0x04*n)   //rx filter n standard ID High
#define MCP_ADDR_RXFSIDL(n)         (0x01+0x04*n)   //rx filter n standard ID High, last 2 bits -> EID17,EID16
#define MCP_ADDR_RXFEID8(n)         (0x02+0x04*n)   //rx filter n extended ID EID15-EID8
#define MCP_ADDR_RXFEID0(n)         (0x03+0x04*n)   //rx filter n extended ID EID7-EID0
#define MCP_FLAG_RXFSIDL_EXIDE      1<<3

#define MCP_LEN_RXM                 2               //rx mask count
#define MCP_ADDR_RXMnSIDH(n)        (0x20+0x04*n)   //rx mask n standard ID High
#define MCP_ADDR_RXMnSIDL(n)        (0x21+0x04*n)   //rx mask n standard ID High, last 2 bits -> EID17,EID16
#define MCP_ADDR_RXMnEID8(n)        (0x22+0x04*n)   //rx mask n extended ID EID15-EID8
#define MCP_ADDR_RXMnEID0(n)        (0x23+0x04*n)   //rx mask n extended ID EID7-EID0

#define MCP_FLAG_RXSTAT_RXB0FULL    1<<6            //for rx_status command, indicated RXB0 buffer is full
#define MCP_FLAG_RXSTAT_RXB1FULL    1<<7            //for rx_status command, indicated RXB1 buffer is full

#define MCP_CNF1_20MHz_1MBd         0x00
#define MCP_CNF2_20MHz_1MBd         0xD9
#define MCP_CNF3_20MHz_1MBd         0x82

#define MCP_CNF1_20MHz_500kBd       0x00
#define MCP_CNF2_20MHz_500kBd       0xFA
#define MCP_CNF3_20MHz_500kBd       0x87

#define MCP_CNF1_20MHz_250kBd       0x41
#define MCP_CNF2_20MHz_250kBd       0xFB
#define MCP_CNF3_20MHz_250kBd       0x86

#define MCP_CNF1_20MHz_200kBd       0x01
#define MCP_CNF2_20MHz_200kBd       0xFF
#define MCP_CNF3_20MHz_200kBd       0x87

#define MCP_CNF1_20MHz_125kBd       0x03
#define MCP_CNF2_20MHz_125kBd       0xFA
#define MCP_CNF3_20MHz_125kBd       0x87

#define MCP_CNF1_20MHz_100kBd       0x04
#define MCP_CNF2_20MHz_100kBd       0xFA
#define MCP_CNF3_20MHz_100kBd       0x87

#define MCP_CNF1_20MHz_80kBd        0x04
#define MCP_CNF2_20MHz_80kBd        0xFF
#define MCP_CNF3_20MHz_80kBd        0x87

#define MCP_CNF1_20MHz_50kBd        0x09
#define MCP_CNF2_20MHz_50kBd        0xFA
#define MCP_CNF3_20MHz_50kBd        0x87

#define MCP_CNF1_20MHz_40kBd        0x09
#define MCP_CNF2_20MHz_40kBd        0xFF
#define MCP_CNF3_20MHz_40kBd        0x87


#define MCP_CNF1_16MHz_1MBd         0x00
#define MCP_CNF2_16MHz_1MBd         0xD0
#define MCP_CNF3_16MHz_1MBd         0x82

#define MCP_CNF1_16MHz_500kBd       0x00
#define MCP_CNF2_16MHz_500kBd       0xF0
#define MCP_CNF3_16MHz_500kBd       0x86

#define MCP_CNF1_16MHz_250kBd       0x41
#define MCP_CNF2_16MHz_250kBd       0xF1
#define MCP_CNF3_16MHz_250kBd       0x85

#define MCP_CNF1_16MHz_200kBd       0x01
#define MCP_CNF2_16MHz_200kBd       0xFA
#define MCP_CNF3_16MHz_200kBd       0x87

#define MCP_CNF1_16MHz_125kBd       0x03
#define MCP_CNF2_16MHz_125kBd       0xF0
#define MCP_CNF3_16MHz_125kBd       0x86

#define MCP_CNF1_16MHz_100kBd       0x03
#define MCP_CNF2_16MHz_100kBd       0xFA
#define MCP_CNF3_16MHz_100kBd       0x87

#define MCP_CNF1_16MHz_80kBd        0x03
#define MCP_CNF2_16MHz_80kBd        0xFF
#define MCP_CNF3_16MHz_80kBd        0x87

#define MCP_CNF1_16MHz_50kBd        0x07
#define MCP_CNF2_16MHz_50kBd        0xFA
#define MCP_CNF3_16MHz_50kBd        0x87

#define MCP_CNF1_16MHz_40kBd        0x07
#define MCP_CNF2_16MHz_40kBd        0xFF
#define MCP_CNF3_16MHz_40kBd        0x87

#define MCP_CNF1_16MHz_20kBd        0x0F
#define MCP_CNF2_16MHz_20kBd        0xFF
#define MCP_CNF3_16MHz_20kBd        0x87

#define MCP_CNF1_16MHz_10kBd        0x1F
#define MCP_CNF2_16MHz_10kBd        0xFF
#define MCP_CNF3_16MHz_10kBd        0x87

#define MCP_CNF1_16MHz_5kBd         0x3F
#define MCP_CNF2_16MHz_5kBd         0xFF
#define MCP_CNF3_16MHz_5kBd         0x87


#define MCP_CNF1_8MHz_1MBd          0x00
#define MCP_CNF2_8MHz_1MBd          0x80
#define MCP_CNF3_8MHz_1MBd          0x80

#define MCP_CNF1_8MHz_500kBd        0x00
#define MCP_CNF2_8MHz_500kBd        0x90
#define MCP_CNF3_8MHz_500kBd        0x82

#define MCP_CNF1_8MHz_250kBd        0x00
#define MCP_CNF2_8MHz_250kBd        0xB1
#define MCP_CNF3_8MHz_250kBd        0x85

#define MCP_CNF1_8MHz_200kBd        0x00
#define MCP_CNF2_8MHz_200kBd        0xB4
#define MCP_CNF3_8MHz_200kBd        0x86

#define MCP_CNF1_8MHz_125kBd        0x01
#define MCP_CNF2_8MHz_125kBd        0xB1
#define MCP_CNF3_8MHz_125kBd        0x85

#define MCP_CNF1_8MHz_100kBd        0x01
#define MCP_CNF2_8MHz_100kBd        0xB4
#define MCP_CNF3_8MHz_100kBd        0x86

#define MCP_CNF1_8MHz_80kBd         0x01
#define MCP_CNF2_8MHz_80kBd         0xBF
#define MCP_CNF3_8MHz_80kBd         0x87

#define MCP_CNF1_8MHz_50kBd         0x03
#define MCP_CNF2_8MHz_50kBd         0xB4
#define MCP_CNF3_8MHz_50kBd         0x86

#define MCP_CNF1_8MHz_40kBd         0x03
#define MCP_CNF2_8MHz_40kBd         0xBF
#define MCP_CNF3_8MHz_40kBd         0x87

#define MCP_CNF1_8MHz_20kBd         0x07
#define MCP_CNF2_8MHz_20kBd         0xBF
#define MCP_CNF3_8MHz_20kBd         0x87

#define MCP_CNF1_8MHz_10kBd         0x0F
#define MCP_CNF2_8MHz_10kBd         0xBF
#define MCP_CNF3_8MHz_10kBd         0x87

#define MCP_CNF1_8MHz_5kBd          0x1F
#define MCP_CNF2_8MHz_5kBd          0xBF
#define MCP_CNF3_8MHz_5kBd          0x87


void mcp_spi_init(uint MOSI, uint MISO, uint SCK, uint CS, spi_inst_t *spi_instance);
void mcp_reset();
void cs_select();
void cs_deselect();
void mcp_write(uint8_t data);
uint8_t mcp_read();
void mcp_reg_write(uint8_t addr, uint8_t val);
uint8_t mcp_reg_read(uint8_t addr);
void mcp_set_bitmask(uint8_t addr, uint8_t mask, uint8_t data);
uint8_t mcp_read_status();
uint8_t mcp_read_rx_status();

bool mcp_send_message(can_msg_t *msg);
bool mcp_get_message(can_msg_t *msg);
void mcp_send_rts(uint8_t buf_id);
typedef enum
{
    CAN_PRIO_HI = 0b11,
    CAN_PRIO_MIDHI = 0b10,
    CAN_PRIO_MIDLO = 0b01,
    CAN_PRIO_LO = 0b00,
} tx_priority;

typedef enum
{
    OPMODE_NORMAL = MCP_FLAG_CANCTRL_OP_NORML,
    OPMODE_SLEEP = MCP_FLAG_CANCTRL_OP_SLEEP,
    OPMODE_LOOPBACK = MCP_FLAG_CANCTRL_OP_LPBCK,
    OPMODE_LISTEN = MCP_FLAG_CANCTRL_OP_LISTN,
    OPMODE_CONFIG = MCP_FLAG_CANCTRL_OP_CONFG,

} mcp_opmode;

void mcp_set_opmode(mcp_opmode mode);

#endif