#ifndef _PERIPHERAL_RS485_HPP_
#define _PERIPHERAL_RS485_HPP_
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdint.h>
#include <cstddef>
#include <functional>
#include <vector>
#include "definition.h"

#define TASK_RS485_STACK_DEPTH 4096

#ifdef __cplusplus
extern "C" {
#endif

typedef std::function<void(uint8_t*, size_t, uint8_t)> fn_rs485_callback;

class CRS485
{
public:
    CRS485(int uart_port_num = 0, int gpio_txd = 0, int gpio_rxd = 0, int gpio_rts = 0, uint8_t index = 0);
    virtual ~CRS485();

public:
    int get_uart_port_num();
    void set_config(rs485_config_t *config);
    void set_config(uint32_t baudrate, uint8_t databits, char parity, float stopbits);
    rs485_config_t* get_config();
    bool send_packet(uint8_t *packet, size_t length, uint32_t timeout_ms = 1, bool verbose = true);
    void set_enable_log_rx(bool enable);
    bool is_enable_log_rx();
    void set_enable_log_tx(bool enable);
    bool is_enable_log_tx();
    void set_enable_echo(bool enable);
    bool is_enable_echo();
    bool is_line_busy();
    void set_line_busy(bool busy);
    void register_callback_tx(fn_rs485_callback callback);
    void unregister_callback_tx();
    void register_callback_rx(fn_rs485_callback callback);
    void unregister_callback_rx();
    void register_callback_rx_raw(fn_rs485_callback callback);
    void unregister_callback_rx_raw();
    void set_factory_config();

#if !ENABLE_TASK_RS485_INSTANCE
    void recv_callback(uint8_t *data, size_t length);
#endif
    QueueHandle_t get_queue_tx();

protected:
    int m_uart_port_num;
    QueueHandle_t m_queue_tx;
    bool m_enable_log_rx;
    bool m_enable_log_tx;
    bool m_enable_echo;
    bool m_line_busy;
    uint8_t m_index;    // multiple uart port discriminator
    rs485_config_t m_config;

#if ENABLE_TASK_RS485_INSTANCE
    bool m_keepalive;
#if TASK_USE_STATIC_MEM
    StaticTask_t m_task_handle;
    StackType_t m_stack_task[TASK_RS485_STACK_DEPTH];
#else
    TaskHandle_t m_task_handle;
#endif
#endif

    fn_rs485_callback m_callback_ptr_rx;
    fn_rs485_callback m_callback_ptr_tx;
    fn_rs485_callback m_callback_ptr_rx_raw;

    bool apply_config(bool save_memory = false);
    bool refresh_config();

private:
#if ENABLE_TASK_RS485_INSTANCE
    static void task_function(void *param);
#endif
};

#ifdef __cplusplus
}
#endif
#endif