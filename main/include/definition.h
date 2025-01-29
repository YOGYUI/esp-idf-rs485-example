#ifndef _DEFINITION_HPP_
#define _DEFINITION_HPP_
#pragma once

#include <string>
#include <vector>

#define GPIO_PIN_UART1_TXD          23
#define GPIO_PIN_UART1_RXD          22
#define GPIO_PIN_UART1_RTS          18

#define RS485_RX_FIFO_SIZE          256
#define ENABLE_TASK_RS485_INSTANCE  false

struct rs485_config_t
{
    uint32_t    baudrate;
    uint8_t     databits;
    char        parity;
    float       stopbits;

    rs485_config_t() {
        baudrate    = 9600;
        databits    = 8;
        parity      = 'N';
        stopbits    = 1;
    }
};

struct rs485_send_data_t {
    std::vector<uint8_t> packet;
    size_t length;
    bool verbose;
};

static std::string prettify_packet(uint8_t *packet, size_t length) {
    std::string result = "";
    char temp[3]{};
    for (size_t i = 0; i < length; i++) {
        snprintf(temp, 3, "%02X", packet[i]);
        result += temp;
        if (i < length - 1) 
            result += " ";
    }
    return result;
}

#endif