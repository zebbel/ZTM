#include "sBus.h"

static const char *sbusTAG = "sBus";


sBus::sBus(){
    //esp_log_level_set(sbusTAG, ESP_LOG_WARN);
}

void sBus::init(uint8_t uartNumVal, QueueHandle_t *channelQueueInst){
    uartNum = uartNumVal;
    channelQueue = channelQueueInst;

    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config;
    uart_config.baud_rate = 100000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_EVEN;
    uart_config.stop_bits = UART_STOP_BITS_2;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    //Install UART driver, and get the queue.
    uart_driver_install(uartNum, 1024 * 2, 1024 * 2, 20, &uart_queue, 0);
    uart_param_config(uartNum, &uart_config);

    ESP_LOGI(sbusTAG, "init sBus on uart%d", uartNum);

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
            ESP_LOGE(sbusTAG, "uart num error: %d", uartNum);
            break;
    }

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "sbus_uart_event_task", 3048, this, 12, NULL);
}

void sBus::uart_event_task(void *pvParameter){
    sBus* sbus = reinterpret_cast<sBus*>(pvParameter); //obtain the instance pointer

    uart_event_t event;
    uint8_t buffer[SBUS_UART_BUFFER_SIZE];

    while(1){
        if(xTaskGetTickCount() > (sbus->lastFrame + SBUS_TIMEOUT)) sbus->sBusData.failSave = true;
        else sbus->sBusData.failSave = false;

        if(xQueueReceive(sbus->uart_queue, &event, 0)) {
            if(event.type == UART_DATA && event.size == 25){
                memset(buffer, 0, sizeof(buffer));
                uart_read_bytes(sbus->uartNum, buffer, event.size, 0);

                if(buffer[0] == 0x0F){
                    sbus->sBusData.channels[0]  = (uint16_t)((buffer[1]        | buffer[2] << 8)                     & 0x07FF);
                    sbus->sBusData.channels[1]  = (uint16_t)((buffer[2] >> 3   | buffer[3] << 5)                     & 0x07FF);
                    sbus->sBusData.channels[2]  = (uint16_t)((buffer[3] >> 6   | buffer[4] << 2  | buffer[5] << 10)  & 0x07FF);
                    sbus->sBusData.channels[3]  = (uint16_t)((buffer[5] >> 1   | buffer[6] << 7)                     & 0x07FF);
                    sbus->sBusData.channels[4]  = (uint16_t)((buffer[6] >> 4   | buffer[7] << 4)                     & 0x07FF);
                    sbus->sBusData.channels[5]  = (uint16_t)((buffer[7] >> 7   | buffer[8] << 1  | buffer[9] << 9)   & 0x07FF);
                    sbus->sBusData.channels[6]  = (uint16_t)((buffer[9] >> 2   | buffer[10] << 6)                    & 0x07FF);
                    sbus->sBusData.channels[7]  = (uint16_t)((buffer[10] >> 5  | buffer[11] << 3)                    & 0x07FF);
                    sbus->sBusData.channels[8]  = (uint16_t)((buffer[12]       | buffer[13] << 8)                    & 0x07FF);
                    sbus->sBusData.channels[9]  = (uint16_t)((buffer[13] >> 3  | buffer[14] << 5)                    & 0x07FF);
                    sbus->sBusData.channels[10] = (uint16_t)((buffer[14] >> 6  | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
                    sbus->sBusData.channels[11] = (uint16_t)((buffer[16] >> 1  | buffer[17] << 7)                    & 0x07FF);
                    sbus->sBusData.channels[12] = (uint16_t)((buffer[17] >> 4  | buffer[18] << 4)                    & 0x07FF);
                    sbus->sBusData.channels[13] = (uint16_t)((buffer[18] >> 7  | buffer[19] << 1 | buffer[20] << 9)  & 0x07FF);
                    sbus->sBusData.channels[14] = (uint16_t)((buffer[20] >> 2  | buffer[21] << 6)                    & 0x07FF);
                    sbus->sBusData.channels[15] = (uint16_t)((buffer[21] >> 5  | buffer[22] << 3)                    & 0x07FF);

                    sbus->sBusData.failSave = buffer[23] & 0x08;

                    sbus->lastFrame = xTaskGetTickCount();

                    xQueueSend(*sbus->channelQueue, &sbus->sBusData, 0);
                    //ESP_LOGI(sbusTAG, "%d", sbus->channels[0]);
                }
               
            }else{
                uart_flush(sbus->uartNum);
            }
        }  

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}