#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "CAN.h"
#include "stdio.h"
#include "mcp_interface.h"

void can_init(uint MOSI, uint MISO, uint SCK, uint CS, spi_inst_t *spi_instance)
{
    mcp_spi_init(MOSI, MISO, SCK, CS, spi_instance);

    mcp_reset();

    sleep_ms(50);
    cs_select();
    mcp_write(MCP_CMD_WRITE);      // WRITE
    mcp_write(MCP_ADDR_CNF3);      // Address of CNF3
    mcp_write(MCP_CNF3_8MHz_1MBd); // CNF3, 0x28
    mcp_write(MCP_CNF2_8MHz_1MBd); // CNF2, 0x29
    mcp_write(MCP_CNF1_8MHz_1MBd); // CNF1, 0x2A
    mcp_write(0);                  // CANINTE, 0x2B, do not use interrupts
    cs_deselect();

    if (mcp_reg_read(0x28) != MCP_CNF3_8MHz_1MBd) // Communication error
    {
        for (;;)
        {
            printf("CAN bus init error!\n");
            sleep_ms(100);
        }
    }

    mcp_reg_write(MCP_ADDR_BFPCTRL, 0); // set pin states
    mcp_reg_write(MCP_ADDR_TXRTSCTRL, 0);

    mcp_reg_write(MCP_ADDR_RXBnCTRL(0), MCP_FLAG_RXBn_RXM_OFF); // filters off
    mcp_reg_write(MCP_ADDR_RXBnCTRL(1), MCP_FLAG_RXBn_RXM_OFF);

    mcp_reg_write(MCP_ADDR_CANCTRL,0);
    //mcp_set_opmode(OPMODE_NORMAL);
}

bool can_get_message(can_msg_t *msg)
{
    uint8_t status = mcp_read_rx_status();
    uint8_t rxbuf;

    if (status & MCP_FLAG_RXSTAT_RXB0FULL)
        rxbuf = 0;
    else if (status & MCP_FLAG_RXSTAT_RXB1FULL)
        rxbuf = 1;
    else
        return false;

    cs_select();
    mcp_write(MCP_CMD_READ);
    mcp_write(MCP_ADDR_RXBnSIDH(rxbuf));

    uint32_t rxid0 = mcp_read(); //[SSSS SSSS]
    uint32_t rxid1 = mcp_read(); //[SSS(SRR) (IDE)_EE]
    uint32_t rxid2 = mcp_read(); //[EEEE EEEE]
    uint32_t rxid3 = mcp_read(); //[EEEE EEEE]
    cs_deselect();

    if (rxid1 & MCP_FLAG_RXBnSIDL_SRR)
        msg->remote_frame = true;
    else
        msg->remote_frame = false;
    if (rxid1 & MCP_FLAG_RXBnSIDL_IDE)
        msg->extended_id = true;
    else
        msg->extended_id = false;

    msg->id = (((rxid0 << 3)) |
               ((rxid1 >> 5) & 0b111) |
               (msg->extended_id ? (((rxid1 & 0b11) << 11)) : 0) |
               (msg->extended_id ? (((rxid2) << 13)) : 0) |
               (msg->extended_id ? (((rxid3) << 21)) : 0));

    msg->length = mcp_reg_read(MCP_ADDR_RXBnDLC(rxbuf)) & MCP_MASK_RXBnDLC_DLC;
    if (!msg->remote_frame)
        for (size_t i = 0; i < msg->length; i++)
        {
            msg->data[i] = mcp_reg_read(MCP_ADDR_RXBnD_BASE(rxbuf) + i);
        }

    sleep_us(10);
    printf("ID=%lu\n",msg->id);
    printf("ID part 1=%lu\n",(((rxid0 << 3))));
    printf("ID part 2=%lu\n",((rxid1 >> 5) & 0b111) );
    printf("ID part 3=%lu\n",(msg->extended_id ? (((rxid1 & 0b11) << 11)) : 0) );
    printf("ID part 4=%lu\n",(msg->extended_id ? (((rxid2) << 13)) : 0) );
    printf("ID part 5=%lu\n",(msg->extended_id ? (((rxid3) << 21)) : 0));
    //printf("rxbuf=%hhx\n",rxbuf);
    //printf("len=%hhx\n",msg->length);
    mcp_set_bitmask(MCP_ADDR_CANINTF,1<<rxbuf, 0); // clear rx buffer full bit
    return true;
}

bool can_send_message(can_msg_t *msg)
{
    uint8_t status = mcp_read_status();
    uint8_t txbuf;
    if ((status & MCP_FLAG_CMDRS_TX0REQ) == 0)
        txbuf = 0;
    else if ((status & MCP_FLAG_CMDRS_TX1REQ) == 0)
        txbuf = 1;
    else if ((status & MCP_FLAG_CMDRS_TX2REQ) == 0)
        txbuf = 2;
    else
        return false;

#define id0 0xFF & (msg->id >> 3)
#define id1_1 ((msg->id << 5) & 0b11000000)
#define id1_2 (msg->extended_id ? (MCP_FLAG_TXBnSIDL_EXIDE | ((msg->id >> 11) & 0b11)) : 0)
#define id1 0xFF & (id1_1 | id1_2)
#define id2 0xFF & (msg->id >> 13)
#define id3 0xFF & (msg->id >> 21)

    cs_select();
    mcp_write(MCP_CMD_WRITE);
    mcp_write(MCP_ADDR_TXBnSIDH(txbuf)); // start from address of SID H
    mcp_write(id0);                      // MCP_ADDR_TXBnSIDH
    mcp_write(id1);                      // MCP_ADDR_TXBnSIDL
    mcp_write(id2);                      // MCP_ADDR_TXBnEID8
    mcp_write(id3);                      // MCP_ADDR_TXBnEID0
    cs_deselect();

    mcp_reg_write(MCP_ADDR_TXBnDLC(txbuf), (msg->length & MCP_MASK_TXBnDLC_DLC) | (msg->remote_frame ? MCP_FLAG_TXBnDLC_RTR : 0));
    if (msg->length > 8)
        msg->length = 8; // git gud

    for (size_t i = 0; i < msg->length; i++)
    {
        mcp_reg_write(MCP_ADDR_TXBnD_BASE(txbuf) + i, msg->data[i]);
    }
    sleep_us(1000);

    //mcp_reg_write(MCP_ADDR_TXBnCTRL(txbuf), MCP_FLAG_TXBnCTRL_TXREQ);
    mcp_send_rts(txbuf);
    return true;
}