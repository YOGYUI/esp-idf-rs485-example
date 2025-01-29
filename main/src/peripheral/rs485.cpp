#include "rs485.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "definition.h"
#include "logger.hpp"
#include "memory.hpp"
#include <cstring>
#include "memory.h"

CRS485::CRS485(int uart_port_num/*=0*/, int gpio_txd/*=0*/, int gpio_rxd/*=0*/, int gpio_rts/*=0*/, uint8_t index/*=0*/)
{
    m_uart_port_num = uart_port_num;
    m_enable_log_rx = false;
    m_enable_log_tx = true;
    m_enable_echo = false;
    m_line_busy = false;
    m_index = index;
    m_callback_ptr_rx = nullptr;
    m_callback_ptr_tx = nullptr;
    m_callback_ptr_rx_raw = nullptr;
    m_config = rs485_config_t();

    ESP_ERROR_CHECK(uart_set_pin((uart_port_t)m_uart_port_num, gpio_txd, gpio_rxd, gpio_rts, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_hw_flow_ctrl((uart_port_t)m_uart_port_num, UART_HW_FLOWCTRL_DISABLE, 122));
    ESP_ERROR_CHECK(uart_driver_install((uart_port_t)m_uart_port_num, RS485_RX_FIFO_SIZE, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_set_mode((uart_port_t)m_uart_port_num, UART_MODE_RS485_HALF_DUPLEX));

    // default parameters for general home-network
    if (GetMemory()->load_rs485_config(m_uart_port_num, &m_config)) {
        apply_config(false);
    } else {
        set_factory_config();
    }

#if ENABLE_TASK_RS485_INSTANCE
    m_queue_tx = xQueueCreate(10, sizeof(rs485_send_data_t));
    m_keepalive = true;
    char task_name[32]{};
    snprintf(task_name, sizeof(task_name), "TASK_RS485(%d)", m_uart_port_num);
#if TASK_USE_STATIC_MEM
    xTaskCreateStatic(task_function, task_name, TASK_RS485_STACK_DEPTH, this, TASK_PRIORITY_RS485_TXRX, m_stack_task, &m_task_handle);
#else
    xTaskCreate(task_function, task_name, TASK_RS485_STACK_DEPTH, this, TASK_PRIORITY_RS485_TXRX, &m_task_handle);
#endif
#endif
    GetLogger(eLogType::Info)->Log("'RS485 Port%d' Initialized", m_uart_port_num);
}

CRS485::~CRS485()
{
#if ENABLE_TASK_RS485_INSTANCE
    m_keepalive = false;
#endif
    uart_driver_delete((uart_port_t)m_uart_port_num);
    unregister_callback_tx();
    unregister_callback_rx();
    unregister_callback_rx_raw();
}

int CRS485::get_uart_port_num()
{
    return m_uart_port_num;
}

void CRS485::set_config(rs485_config_t *config)
{
    memcpy(&m_config, config, sizeof(rs485_config_t));
    apply_config(true);
}

void CRS485::set_config(uint32_t baudrate, uint8_t databits, char parity, float stopbits)
{
    m_config.baudrate = baudrate;
    m_config.databits = databits;
    m_config.parity = parity;
    m_config.stopbits = stopbits;
    apply_config(true);
}

rs485_config_t* CRS485::get_config()
{
    refresh_config();
    return &m_config;
}

bool CRS485::apply_config(bool save_memory/*=false*/)
{
    esp_err_t ret;
    // baudrate
    ret = uart_set_baudrate((uart_port_t)m_uart_port_num, m_config.baudrate);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Error)->Log("Failed to set baudrate (ret: %d)", ret);
        return false;
    }
    GetLogger(eLogType::Info)->Log("RS485(%d) set baud rate: %d", m_uart_port_num, m_config.baudrate);

    // data bit
    uart_word_length_t word = UART_DATA_8_BITS;
    switch (m_config.databits) {
    case 5:
        word = UART_DATA_5_BITS;
        break;
    case 6:
        word = UART_DATA_6_BITS;
        break;
    case 7:
        word = UART_DATA_7_BITS;
        break;
    default:
        word = UART_DATA_8_BITS;
        break;
    }
    ret = uart_set_word_length((uart_port_t)m_uart_port_num, word);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Error)->Log("Failed to set databit (ret: %d)", ret);
        return false;
    }
    GetLogger(eLogType::Info)->Log("RS485(%d) set databit: %d", m_uart_port_num, m_config.databits);

    // parity
    uart_parity_t parity_value = UART_PARITY_DISABLE;
    switch (m_config.parity) {
    case 'E':
        parity_value = UART_PARITY_EVEN;
        break;
    case 'O':
        parity_value = UART_PARITY_ODD;
        break;
    default:
        parity_value = UART_PARITY_DISABLE;
        break;
    }
    ret = uart_set_parity((uart_port_t)m_uart_port_num, parity_value);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Error)->Log("Failed to set parity (ret: %d)", ret);
        return false;
    }
    GetLogger(eLogType::Info)->Log("RS485(%d) set parity: %c", m_uart_port_num, m_config.parity);

    // stop bit
    uart_stop_bits_t stopbit = UART_STOP_BITS_1;
    if (m_config.stopbits == 1.5) {
        stopbit = UART_STOP_BITS_1_5;
    } else if (m_config.stopbits == 2) {
        stopbit = UART_STOP_BITS_2;
    }
    ret = uart_set_stop_bits((uart_port_t)m_uart_port_num, stopbit);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Error)->Log("Failed to set stopbits (ret: %d)", ret);
        return false;
    }

    GetLogger(eLogType::Info)->Log("RS485(%d) set stopbits: %g", m_uart_port_num, m_config.stopbits);
    if (save_memory)
        GetMemory()->save_rs485_config(m_uart_port_num, &m_config);

    
    return true;
}

bool CRS485::refresh_config()
{
    esp_err_t ret;
    // baudrate
    ret = uart_get_baudrate((uart_port_t)m_uart_port_num, &m_config.baudrate);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Error)->Log("Failed to get baudrate (ret: %d)", ret);
        return false;
    }
    
    // data bit
    uart_word_length_t databits = UART_DATA_8_BITS;
    ret = uart_get_word_length((uart_port_t)m_uart_port_num, &databits);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Error)->Log("Failed to get databit (ret: %d)", ret);
        return false;
    }
    switch (databits) {
    case UART_DATA_5_BITS:
        m_config.databits = 5;
        break;
    case UART_DATA_6_BITS:
        m_config.databits = 6;
        break;
    case UART_DATA_7_BITS:
        m_config.databits = 7;
        break;
    default:
        m_config.databits = 8;
        break;
    }

    // parity
    uart_parity_t parity = UART_PARITY_DISABLE;
    ret = uart_get_parity((uart_port_t)m_uart_port_num, &parity);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Error)->Log("Failed to get parity (ret: %d)", ret);
        return false;
    }
    switch (parity) {
    case UART_PARITY_EVEN:
        m_config.parity = 'E';
        break;
    case UART_PARITY_ODD:
        m_config.parity = 'O';
        break;
    default:
        m_config.parity = 'N';
        break;
    }

    // stop bit
    uart_stop_bits_t stopbits = UART_STOP_BITS_1;
    ret = uart_get_stop_bits((uart_port_t)m_uart_port_num, &stopbits);
    if (ret != ESP_OK) {
        GetLogger(eLogType::Error)->Log("Failed to get stopbit (ret: %d)", ret);
        return false;
    }
    switch (stopbits) {
    case UART_STOP_BITS_1_5:
        m_config.stopbits = 1.5;
        break;
    case UART_STOP_BITS_2:
        m_config.stopbits = 2;
        break;
    default:
        m_config.stopbits = 1;
        break;
    }

    return true;
}

bool CRS485::send_packet(uint8_t *packet, size_t length, uint32_t timeout_ms/*=1*/, bool verbose/*=true*/)
{
    if (!length) {
        GetLogger(eLogType::Warning)->Log("Nothing to send");
        return false;
    }

#if ENABLE_TASK_RS485_INSTANCE
    uint32_t result;
    rs485_send_data_t *data_to_send = new rs485_send_data_t();
    if (!data_to_send) {
        GetLogger(eLogType::Error)->Log("Failed to allocate packet structure");
        return false;
    }

    data_to_send->packet.resize(length);
    data_to_send->length = length;
    data_to_send->verbose = verbose;
    memcpy(&data_to_send->packet[0], packet, length);
    result = xQueueSend(m_queue_tx, (void *)data_to_send, timeout_ms / portTICK_PERIOD_MS);
    if (result != pdTRUE) {
        GetLogger(eLogType::Error)->Log("Failed to add queue (return code: %u)", result);
        xQueueReset(m_queue_tx);
        data_to_send->packet.clear();
        delete data_to_send;
        return false;
    }
#else
    m_line_busy = true;
    uart_write_bytes((uart_port_t)m_uart_port_num, (void *)packet, length);
    if (m_enable_log_tx && verbose) {
        std::string temp = prettify_packet(packet, length);
        GetLogger(eLogType::Info)->Log("RS485 Port%d Send: %s", m_uart_port_num, temp.c_str());
    }
    if (m_callback_ptr_tx != nullptr) {
        m_callback_ptr_tx(packet, length, m_index);
    }
    m_line_busy = false;
#endif
    return true;
}

void CRS485::set_enable_log_rx(bool enable) 
{
    m_enable_log_rx = enable;
}

bool CRS485::is_enable_log_rx() 
{
    return m_enable_log_rx;
}

void CRS485::set_enable_log_tx(bool enable) 
{
    m_enable_log_tx = enable;
}

bool CRS485::is_enable_log_tx() 
{
    return m_enable_log_tx;
}

void CRS485::set_enable_echo(bool enable) 
{
    m_enable_echo = enable;
}

bool CRS485::is_enable_echo() 
{
    return m_enable_echo;
}

bool CRS485::is_line_busy()
{
    return m_line_busy;
}

void CRS485::set_line_busy(bool busy)
{
    m_line_busy = busy;
}

void CRS485::register_callback_tx(fn_rs485_callback callback)
{
    m_callback_ptr_tx = std::move(callback);
}

void CRS485::unregister_callback_tx()
{
    m_callback_ptr_tx = nullptr;
}

void CRS485::register_callback_rx(fn_rs485_callback callback)
{
    m_callback_ptr_rx = std::move(callback);
}

void CRS485::unregister_callback_rx()
{
    m_callback_ptr_rx = nullptr;
}

#if !ENABLE_TASK_RS485_INSTANCE
void CRS485::recv_callback(uint8_t *data, size_t length)
{
    if (m_callback_ptr_rx != nullptr) {
        m_callback_ptr_rx(data, length, m_index);
    }
}

QueueHandle_t CRS485::get_queue_tx()
{
    return m_queue_tx;
}
#endif

void CRS485::register_callback_rx_raw(fn_rs485_callback callback)
{
    m_callback_ptr_rx_raw = std::move(callback);
}

void CRS485::unregister_callback_rx_raw()
{
    m_callback_ptr_rx_raw = nullptr;
}

void CRS485::set_factory_config()
{
    set_config(9600, 8, 'N', 1);
}

#if ENABLE_TASK_RS485_INSTANCE
void CRS485::task_function(void *param)
{
    CRS485 *obj = static_cast<CRS485 *>(param);
    int bytes_recv;
    uint8_t *packet_recv = new uint8_t[RS485_RX_FIFO_SIZE];
    rs485_send_data_t data_send;
    int uart_port_num = obj->m_uart_port_num;
    
    GetLogger(eLogType::Info)->Log("Realtime Task for RS485 Port%d Started", uart_port_num);
    while (obj->m_keepalive) {
        // Receive 
        bytes_recv = uart_read_bytes(obj->m_uart_port_num, packet_recv, RS485_RX_FIFO_SIZE, 1);
        if (bytes_recv > 0) {
            obj->m_line_busy = true;
            if (obj->m_enable_log_rx) {
                std::string temp = prettify_packet(packet_recv, bytes_recv);
                GetLogger(eLogType::Info)->Log("RS485 Port%d Recv: %s", obj->m_uart_port_num, temp.c_str());
            }
            if (obj->m_callback_ptr_rx != nullptr) {
                obj->m_callback_ptr_rx(packet_recv, (size_t)bytes_recv, obj->m_index);
            }
            if (obj->m_callback_ptr_rx_raw != nullptr) {
                obj->m_callback_ptr_rx_raw(packet_recv, (size_t)bytes_recv, obj->m_index);
            }
            if (obj->m_enable_echo) {
                obj->send_packet(packet_recv, bytes_recv);
            }
            obj->m_line_busy = false;
        }

        // Send from Queue
        if (xQueueReceive(obj->m_queue_tx, (void *)&data_send, 1) == pdTRUE) {
            obj->m_line_busy = true;
            uart_write_bytes(obj->m_uart_port_num, (void *)&data_send.packet[0], data_send.length);
            if (obj->m_enable_log_tx && data_send.verbose) {
                std::string temp = prettify_packet(&data_send.packet[0], data_send.length);
                GetLogger(eLogType::Info)->Log("RS485 Port%d Send: %s", obj->m_uart_port_num, temp.c_str());
            }
            if (obj->m_callback_ptr_tx != nullptr) {
                obj->m_callback_ptr_tx(&data_send.packet[0], data_send.length, obj->m_index);
            }
            data_send.packet.clear();
            obj->m_line_busy = false;
        }
    }
    GetLogger(eLogType::Info)->Log("Realtime Task for RS485 Port%d Terminated", uart_port_num);
    vTaskDelete(nullptr);
}
#endif