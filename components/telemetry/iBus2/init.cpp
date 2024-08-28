#include "iBus2.h"

static const char *ibus2TAG = "iBus2";

iBus2::iBus2(){
    //esp_log_level_set(ibus2TAG, ESP_LOG_WARN);
}

void iBus2::init(Sensor *sensorInst, uint8_t uartNumVal, QueueHandle_t *queue){
    sensor = sensorInst;
    uartNum = uartNumVal;
    extern_queue = queue;

    ESP_LOGI(ibus2TAG, "init iBus2 on uart%d", uartNum);

    // Configure parameters of an UART driver,
    // communication pins and install the driver
    uart_config_t uart_config;
    uart_config.baud_rate = 1500000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    //Install UART driver, and get the queue.
    uart_driver_install(uartNum, 1024 * 2, 1024 * 2, 20, &uart_queue, 0);
    uart_param_config(uartNum, &uart_config);

    switch (uartNum){
        case UART_NUM_0:
            uart_set_pin(uartNum, CONFIG_UART0_TX, CONFIG_UART0_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            break;
        case UART_NUM_1:
            uart_set_pin(uartNum, CONFIG_UART1_TX, CONFIG_UART1_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            break;
        case UART_NUM_2:
            uart_set_pin(uartNum, CONFIG_UART2_TX, CONFIG_UART2_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            break;
        default:
            ESP_LOGE(ibus2TAG, "uart num error: %d", uartNum);
            break;
    }

    memset(sendBuffer, 0, sizeof(sendBuffer));
    sendBuffer[0] = 0x05;
    sendBuffer[20] = 0x7D;

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "ibus2_uart_event_task", 3048, this, 12, NULL);
    xTaskCreate(uartSendTask, "uartSendTask", 2048, this, 12, NULL);
}