#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "mcp_interface.h"

uint gpio_CS;
spi_inst_t *spi_port;


void mcp_spi_init(uint MOSI, uint MISO, uint SCK, uint CS, spi_inst_t *spi_instance)
{
    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SCK, GPIO_FUNC_SPI);

    gpio_init(CS);
    gpio_set_dir(CS, GPIO_OUT);

    spi_port = spi_instance;
    gpio_CS = CS;

    spi_init(spi_port, 1e6);
}
bool mcp_was_message_aborted(uint8_t buf_id)
{
    return (mcp_reg_read(MCP_ADDR_TXBnCTRL(buf_id) | MCP_FLAG_TXBnCTRL_ABTF)) != 0;
}
bool mcp_was_message_arbitration_lost(uint8_t buf_id)
{
    return (mcp_reg_read(MCP_ADDR_TXBnCTRL(buf_id) | MCP_FLAG_TXBnCTRL_MLOA)) != 0;
}
bool mcp_has_transmission_error(uint8_t buf_id)
{
    return (mcp_reg_read(MCP_ADDR_TXBnCTRL(buf_id) | MCP_FLAG_TXBnCTRL_TXERR)) != 0;
}
bool mcp_is_message_pending_transmission(uint8_t buf_id)
{
    return (mcp_reg_read(MCP_ADDR_TXBnCTRL(buf_id) | MCP_FLAG_TXBnCTRL_TXREQ)) != 0;
}
void mcp_set_transfer_priority(uint8_t buf_id, tx_priority priority)
{
    if (priority > CAN_PRIO_HI)
        return;
    mcp_set_bitmask(MCP_ADDR_TXBnCTRL(buf_id), MCP_MASK_TXBnCTRL_TXP, priority);
}
bool mcp_get_TX2RTS()
{
    return (mcp_reg_read(MCP_ADDR_TXRTSCTRL) & MCP_FLAG_TXRTSCTRL_B2RTS) != 0;
}
bool mcp_get_TX1RTS()
{
    return (mcp_reg_read(MCP_ADDR_TXRTSCTRL) & MCP_FLAG_TXRTSCTRL_B1RTS) != 0;
}
bool mcp_get_TX0RTS()
{
    return (mcp_reg_read(MCP_ADDR_TXRTSCTRL) & MCP_FLAG_TXRTSCTRL_B0RTS) != 0;
}
// mcp_set_TX2RTS_mode(bool interrupt)
// interrupt=true -> pin used as an interrupt
// interrupt=false -> pin used as a digital input
void mcp_set_TX2RTS_mode(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_TXRTSCTRL, MCP_FLAG_TXRTSCTRL_B2RTSM, interrupt ? MCP_FLAG_TXRTSCTRL_B2RTSM : 0);
}
// mcp_set_TX1RTS_mode(bool interrupt)
// interrupt=true -> pin used as an interrupt
// interrupt=false -> pin used as a digital input
void mcp_set_TX1RTS_mode(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_TXRTSCTRL, MCP_FLAG_TXRTSCTRL_B2RTSM, interrupt ? MCP_FLAG_TXRTSCTRL_B1RTSM : 0);
}
// mcp_set_TX0RTS_mode(bool interrupt)
// interrupt=true -> pin used as an interrupt
// interrupt=false -> pin used as a digital input
void mcp_set_TX0RTS_mode(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_TXRTSCTRL, MCP_FLAG_TXRTSCTRL_B2RTSM, interrupt ? MCP_FLAG_TXRTSCTRL_B0RTSM : 0);
}

void mcp_set_rxbn_filter(uint8_t buf_id, bool enabled)
{
    mcp_set_bitmask(MCP_ADDR_RXBnCTRL(buf_id), MCP_MASK_RXBn_RXM, enabled ? MCP_FLAG_RXBn_RXM_ON : MCP_FLAG_RXBn_RXM_OFF);
}
void mcp_set_rxb0_rollover(bool enabled)
{
    mcp_set_bitmask(MCP_ADDR_RXBnCTRL(0), MCP_FLAG_RXB0_BUKT_EN, enabled ? MCP_FLAG_RXB0_BUKT_EN : 0);
}
void mcp_reset()
{
    cs_select();
    mcp_write(MCP_CMD_RESET);
    cs_deselect();
}

// mcp_set_RX1BF_pin_state(bool enable)
// enable=true -> pin set to output high
// enable=false -> pin set to output low
void mcp_set_RX1BF_pin_state(bool enable)
{
    mcp_set_bitmask(MCP_ADDR_BFPCTRL, MCP_FLAG_BFPCTRL_B1BFS, enable ? MCP_FLAG_BFPCTRL_B1BFS : 0);
}
// mcp_set_RX1BF_pin_state(bool enable)
// enable=true -> pin set to output high
// enable=false -> pin set to output low
void mcp_set_RX0BF_pin_state(bool enable)
{
    mcp_set_bitmask(MCP_ADDR_BFPCTRL, MCP_FLAG_BFPCTRL_B0BFS, enable ? MCP_FLAG_BFPCTRL_B0BFS : 0);
}
// mcp_set_RX1BF_pin_enable(bool enable)
// enable=true -> pin set to it's mode
// enable=false -> pin set to Hi-Z
void mcp_set_RX1BF_pin_enable(bool enable)
{
    mcp_set_bitmask(MCP_ADDR_BFPCTRL, MCP_FLAG_BFPCTRL_B1BFE_EN, enable ? MCP_FLAG_BFPCTRL_B1BFE_EN : 0);
}
// mcp_set_RX0BF_pin_enable(bool enable)
// enable=true -> pin set to it's mode
// enable=false -> pin set to Hi-Z
void mcp_set_RX0BF_pin_enable(bool enable)
{
    mcp_set_bitmask(MCP_ADDR_BFPCTRL, MCP_FLAG_BFPCTRL_B0BFE_EN, enable ? MCP_FLAG_BFPCTRL_B0BFE_EN : 0);
}
// mcp_set_RX0BF_pin_mode(bool interrupt)
// interrupt=true -> pin set to interrupt mode
// interrupt=false -> pin set to digital output mode
void mcp_set_RX0BF_pin_mode(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_BFPCTRL, MCP_FLAG_BFPCTRL_B0BFM_INT, interrupt ? MCP_FLAG_BFPCTRL_B0BFM_INT : 0);
}
// mcp_set_RX0BF_pin_mode(bool enable)
// interrupt=true -> pin set to interrupt mode
// interrupt=false -> pin set to digital output mode
void mcp_set_RX1BF_pin_mode(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_BFPCTRL, MCP_FLAG_BFPCTRL_B1BFM_INT, interrupt ? MCP_FLAG_BFPCTRL_B1BFM_INT : 0);
}

void mcp_set_rx_filter(uint8_t filter_id, uint32_t id, bool extended_id)
{
    if (filter_id > MCP_LEN_RXF)
        return;

    cs_select();
    mcp_write(MCP_CMD_WRITE);
    mcp_write(MCP_ADDR_RXFSIDH(filter_id));
    uint8_t id0 = 0xFF & (id >> 3);
    uint8_t id1_1 = ((id << 5) & 0b11000000);
    uint8_t id1_2 = (extended_id ? (MCP_FLAG_RXFSIDL_EXIDE | ((id >> 11) & 0b11)) : 0);
    uint8_t id1 = 0xFF & (id1_1 | id1_2);
    uint8_t id2 = 0xFF & (id >> 13);
    uint8_t id3 = 0xFF & (id >> 21);

    cs_deselect();
}
void mcp_set_rx_mask(uint8_t mask_id, uint32_t id, bool extended_id)
{
    if (mask_id > MCP_LEN_RXM)
        return;

    cs_select();
    mcp_write(MCP_CMD_WRITE);
    mcp_write(MCP_ADDR_RXMnSIDH(mask_id));
    uint8_t id0 = 0xFF & (id >> 3);
    uint8_t id1_1 = ((id << 5) & 0b11000000);
    uint8_t id1_2 = (extended_id ? (MCP_FLAG_RXFSIDL_EXIDE | ((id >> 11) & 0b11)) : 0);
    uint8_t id1 = 0xFF & (id1_1 | id1_2);
    uint8_t id2 = 0xFF & (id >> 13);
    uint8_t id3 = 0xFF & (id >> 21);
    cs_deselect();
}

uint8_t mcp_get_transmit_error_counter()
{
    return mcp_reg_read(MCP_ADDR_TEC);
}
uint8_t mcp_get_receive_error_counter()
{
    return mcp_reg_read(MCP_ADDR_REC);
}
void mcp_reset_rx_buffer_overflow_flag(uint8_t buf_id)
{
    if(buf_id>MCP_LEN_RXBn)
        return;
    if (buf_id == 0)
        mcp_set_bitmask(MCP_ADDR_EFLG, MCP_FLAG_EFLG_RX0OVR, 0);
    if (buf_id == 1)
        mcp_set_bitmask(MCP_ADDR_EFLG, MCP_FLAG_EFLG_RX0OVR, 0);
}
bool mcp_get_rx_buffer_overflow_flag(uint8_t buf_id)
{
    if(buf_id>MCP_LEN_RXBn)
        return false;
    if (buf_id == 0)
        return (mcp_reg_read(MCP_ADDR_EFLG) & MCP_FLAG_EFLG_RX0OVR) != 0;
    if (buf_id == 1)
        return (mcp_reg_read(MCP_ADDR_EFLG) & MCP_FLAG_EFLG_RX1OVR) != 0;
    return false;
}
bool mcp_get_bus_off_error()
{
    return (mcp_reg_read(MCP_ADDR_EFLG) & MCP_FLAG_EFLG_TXBO) != 0;
}
bool mcp_get_tx_error_passive()
{
    return (mcp_reg_read(MCP_ADDR_EFLG) & MCP_FLAG_EFLG_TXEP) != 0;
}
bool mcp_get_rx_error_passive()
{
    return (mcp_reg_read(MCP_ADDR_EFLG) & MCP_FLAG_EFLG_RXEP) != 0;
}
bool mcp_get_tx_error_warning()
{
    return (mcp_reg_read(MCP_ADDR_EFLG) & MCP_FLAG_EFLG_TXWAR) != 0;
}
bool mcp_get_rx_error_warning()
{
    return (mcp_reg_read(MCP_ADDR_EFLG) & MCP_FLAG_EFLG_RXWAR) != 0;
}
bool mcp_get_warning()
{
    return (mcp_reg_read(MCP_ADDR_EFLG) & MCP_FLAG_EFLG_EWARN) != 0;
}

bool mcp_get_message_error_interrupt()
{
    return (mcp_reg_read(MCP_ADDR_CANINTE) & MCP_FLAG_CANINTE_MERRE) != 0;
}
bool mcp_get_wakeup_interrupt()
{
    return (mcp_reg_read(MCP_ADDR_CANINTE) & MCP_FLAG_CANINTE_WAKIE) != 0;
}
bool mcp_get_error_interrupt()
{
    return (mcp_reg_read(MCP_ADDR_CANINTE) & MCP_FLAG_CANINTE_ERRIE) != 0;
}
bool mcp_get_tx_buffer_empty_interrupt(uint8_t buf_id)
{
    if (buf_id > MCP_LEN_TXBn)
        return false;
    if (buf_id == 0)
        return (mcp_reg_read(MCP_ADDR_CANINTE) & MCP_FLAG_CANINTE_TX0IE) != 0;
    if (buf_id == 1)
        return (mcp_reg_read(MCP_ADDR_CANINTE) & MCP_FLAG_CANINTE_TX1IE) != 0;
    if (buf_id == 2)
        return (mcp_reg_read(MCP_ADDR_CANINTE) & MCP_FLAG_CANINTE_TX2IE) != 0;
    return false;
}
bool mcp_get_rx_buffer_full_interrupt(uint8_t buf_id)
{
    if(buf_id>MCP_LEN_RXBn)
        return false;
    if(buf_id==0)
        return (mcp_reg_read(MCP_ADDR_CANINTE) & MCP_FLAG_CANINTE_RX0IE) != 0;
    if(buf_id==1)
        return (mcp_reg_read(MCP_ADDR_CANINTE) & MCP_FLAG_CANINTE_RX1IE) != 0;
    return false;
}

void mcp_set_message_error_interrupt(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_CANINTE, MCP_FLAG_CANINTE_MERRE, interrupt?MCP_FLAG_CANINTE_MERRE:0);
}
void mcp_set_wakeup_interrupt(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_CANINTE, MCP_FLAG_CANINTE_WAKIE, interrupt?MCP_FLAG_CANINTE_WAKIE:0);
}
void mcp_set_error_interrupt(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_CANINTE, MCP_FLAG_CANINTE_ERRIE, interrupt?MCP_FLAG_CANINTE_ERRIE:0);
}
void mcp_set_tx_buffer_empty_interrupt(uint8_t buf_id,bool interrupt)
{
    if (buf_id > MCP_LEN_TXBn)
        return;
    if (buf_id == 0)
        mcp_set_bitmask(MCP_ADDR_CANINTE, MCP_FLAG_CANINTE_TX0IE, interrupt?MCP_FLAG_CANINTE_TX0IE:0);
    if (buf_id == 1)
        mcp_set_bitmask(MCP_ADDR_CANINTE, MCP_FLAG_CANINTE_TX1IE, interrupt?MCP_FLAG_CANINTE_TX1IE:0);
    if (buf_id == 2)
        mcp_set_bitmask(MCP_ADDR_CANINTE, MCP_FLAG_CANINTE_TX2IE, interrupt?MCP_FLAG_CANINTE_TX2IE:0);
}
void mcp_set_rx_buffer_full_interrupt(uint8_t buf_id,bool interrupt)
{
    if(buf_id>MCP_LEN_RXBn)
        return;
    if(buf_id==0)
        mcp_set_bitmask(MCP_ADDR_CANINTE, MCP_FLAG_CANINTE_RX0IE, interrupt?MCP_FLAG_CANINTE_RX0IE:0);
    if(buf_id==1)
        mcp_set_bitmask(MCP_ADDR_CANINTE, MCP_FLAG_CANINTE_RX1IE, interrupt?MCP_FLAG_CANINTE_RX1IE:0);
}




bool mcp_get_message_error_flag()
{
    return (mcp_reg_read(MCP_ADDR_CANINTF) & MCP_FLAG_CANINTF_MERRF) != 0;
}
bool mcp_get_wakeup_flag()
{
    return (mcp_reg_read(MCP_ADDR_CANINTF) & MCP_FLAG_CANINTE_WAKIE) != 0;
}
bool mcp_get_error_flag()
{
    return (mcp_reg_read(MCP_ADDR_CANINTF) & MCP_FLAG_CANINTE_ERRIE) != 0;
}
bool mcp_get_tx_buffer_empty_flag(uint8_t buf_id)
{
    if (buf_id > MCP_LEN_TXBn)
        return false;
    if (buf_id == 0)
        return (mcp_reg_read(MCP_ADDR_CANINTF) & MCP_FLAG_CANINTF_TX0IF) != 0;
    if (buf_id == 1)
        return (mcp_reg_read(MCP_ADDR_CANINTF) & MCP_FLAG_CANINTF_TX1IF) != 0;
    if (buf_id == 2)
        return (mcp_reg_read(MCP_ADDR_CANINTF) & MCP_FLAG_CANINTF_TX2IF) != 0;
    return false;
}
bool mcp_get_rx_buffer_full_flag(uint8_t buf_id)
{
    if(buf_id>MCP_LEN_RXBn)
        return false;
    if(buf_id==0)
        return (mcp_reg_read(MCP_ADDR_CANINTF) & MCP_FLAG_CANINTF_RX0IF) != 0;
    if(buf_id==1)
        return (mcp_reg_read(MCP_ADDR_CANINTF) & MCP_FLAG_CANINTF_RX1IF) != 0;
    return false;
}

void mcp_set_message_error_flag(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_CANINTF, MCP_FLAG_CANINTF_MERRF, interrupt?MCP_FLAG_CANINTF_MERRF:0);
}
void mcp_set_wakeup_flag(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_CANINTF, MCP_FLAG_CANINTF_WAKIF, interrupt?MCP_FLAG_CANINTF_WAKIF:0);
}
void mcp_set_error_flag(bool interrupt)
{
    mcp_set_bitmask(MCP_ADDR_CANINTF, MCP_FLAG_CANINTF_ERRIF, interrupt?MCP_FLAG_CANINTF_ERRIF:0);
}
void mcp_set_tx_buffer_empty_flag(uint8_t buf_id,bool interrupt)
{
    if (buf_id > MCP_LEN_TXBn)
        return;
    if (buf_id == 0)
        mcp_set_bitmask(MCP_ADDR_CANINTF, MCP_FLAG_CANINTF_TX0IF, interrupt?MCP_FLAG_CANINTF_TX0IF:0);
    if (buf_id == 1)
        mcp_set_bitmask(MCP_ADDR_CANINTF, MCP_FLAG_CANINTF_TX1IF, interrupt?MCP_FLAG_CANINTF_TX1IF:0);
    if (buf_id == 2)
        mcp_set_bitmask(MCP_ADDR_CANINTF, MCP_FLAG_CANINTF_TX2IF, interrupt?MCP_FLAG_CANINTF_TX2IF:0);
}
void mcp_set_rx_buffer_full_flag(uint8_t buf_id,bool interrupt)
{
    if(buf_id>MCP_LEN_RXBn)
        return;
    if(buf_id==0)
        mcp_set_bitmask(MCP_ADDR_CANINTF, MCP_FLAG_CANINTF_RX0IF, interrupt?MCP_FLAG_CANINTF_RX0IF:0);
    if(buf_id==1)
        mcp_set_bitmask(MCP_ADDR_CANINTF, MCP_FLAG_CANINTF_RX1IF, interrupt?MCP_FLAG_CANINTF_RX1IF:0);
}

void mcp_set_opmode(mcp_opmode mode)
{
    mcp_set_bitmask(MCP_ADDR_CANCTRL, MCP_MASK_CANCTRL_OP, mode);
}
//WARNING: illegal to use in 15 US states.
void mcp_abort_pending_tx()
{
    mcp_set_bitmask(MCP_ADDR_CANCTRL, MCP_FLAG_CANCTRL_ABORT, MCP_FLAG_CANCTRL_ABORT);
}
void mcp_unabort_pending_tx()
{
    mcp_set_bitmask(MCP_ADDR_CANCTRL, MCP_FLAG_CANCTRL_ABORT, 0);
}
void mcp_set_oneshot_mode()
{
    mcp_set_bitmask(MCP_ADDR_CANCTRL, MCP_FLAG_CANCTRL_OSM, MCP_FLAG_CANCTRL_OSM);
}
void mcp_unset_oneshot_mode()
{
    mcp_set_bitmask(MCP_ADDR_CANCTRL, MCP_FLAG_CANCTRL_OSM, 0);
}
void mcp_enable_clkout()
{
    mcp_set_bitmask(MCP_ADDR_CANCTRL, MCP_FLAG_CANCTRL_CLKOUT, MCP_FLAG_CANCTRL_CLKOUT);
}
void mcp_disable_clkout()
{
    mcp_set_bitmask(MCP_ADDR_CANCTRL, MCP_FLAG_CANCTRL_CLKOUT, 0);
}


void cs_select()
{
    gpio_put(gpio_CS, 0);
}
void cs_deselect()
{
    gpio_put(gpio_CS, 1);
}

void mcp_write(uint8_t data)
{
    spi_write_blocking(spi_port, &data, 1);
}

void mcp_write_with_cs(uint8_t data)
{
    cs_select();
    spi_write_blocking(spi_port, &data, 1);
    cs_deselect();
}

uint8_t mcp_read()
{
    uint8_t data;
    spi_read_blocking(spi_port, 0xFF, &data, 1);
    return data;
}

uint8_t mcp_reg_read(uint8_t addr)
{
    cs_select();
    mcp_write(MCP_CMD_READ); // register read
    mcp_write(addr);         // select address of register
    uint8_t val = mcp_read();
    cs_deselect();
    return val;
}

void mcp_reg_write(uint8_t addr, uint8_t val)
{
    cs_select();
    uint8_t buf[] = {MCP_CMD_WRITE, addr, val};
    spi_write_blocking(spi_port, buf, 3);
    cs_deselect();
}

uint8_t mcp_rxbuf_read(uint8_t buffer)
{
    cs_select();
    mcp_write(MCP_CMD_READ_RXBUF(buffer)); // register read
    uint8_t val = mcp_read();
    cs_deselect();
    return val;
}

void mcp_send_rts(uint8_t buf_id)
{
    if (buf_id < MCP_LEN_TXBn)
        mcp_write_with_cs(MCP_CMD_RTS_TXB(buf_id));
}

void mcp_set_bitmask(uint8_t addr, uint8_t mask, uint8_t data)
{
    cs_select();
    mcp_write(MCP_CMD_BIT_MODIFY);
    mcp_write(addr);
    mcp_write(mask);
    mcp_write(data);
    cs_deselect();
}
uint8_t mcp_read_status()
{
    cs_select();
    mcp_write(MCP_CMD_READ_STATUS);
    uint8_t val = mcp_read();
    mcp_read(); // repeat
    cs_deselect();

    return val;
}
uint8_t mcp_read_rx_status()
{
    cs_select();
    mcp_write(MCP_CMD_RX_STATUS);
    uint8_t val = mcp_read();
    mcp_read(); // repeat
    cs_deselect();

    return val;
}