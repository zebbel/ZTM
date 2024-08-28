#include <stdio.h>
#include "ESP_CRSF.h"
#include "byteswap.h"

#define RX_BUF_SIZE 1024    //UART buffer size

// CRC8 lookup table (poly 0xd5)
static uint8_t crc8_table[256] = {0};

CRSF::CRSF(){
}

void CRSF::CRSF_init(uint8_t uartNumVal){
    generate_CRC(0xd5);

    uartNum = uartNumVal;

    //begin uart communication with RX
    uart_config_t uart_config = {
        .baud_rate = 420000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };
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
            ESP_LOGE(TAG_CRSF, "uart num error: %d", uartNum);
            break;
    }
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(uartNum, RX_BUF_SIZE, RX_BUF_SIZE, 10, &uart_queue, 0));

    //create semaphore
    xMutex = xSemaphoreCreateMutex();
    //create task
    xTaskCreate(rx_task, "uart_rx_task", 1024*4, this, configMAX_PRIORITIES-1, NULL);
}

void CRSF::generate_CRC(uint8_t poly){
    for (int idx=0; idx<256; ++idx)
    {
        uint8_t crc = idx;
        for (int shift=0; shift<8; ++shift){
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        }
        crc8_table[idx] = crc & 0xff;
    }
}

// Function to calculate CRC8 checksum
uint8_t CRSF::crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--){
        crc = crc8_table[crc ^ *data++];
    }

    return crc;
}


void CRSF::rx_task(void *pvParameter){
    CRSF* crsf = reinterpret_cast<CRSF*>(pvParameter); //obtain the instance pointer

    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RX_BUF_SIZE);
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(crsf->uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RX_BUF_SIZE);
            if (event.type == UART_DATA ) {
                //ESP_LOGI(TAG_CRSF, "[UART DATA]: %d", event.size);
                uart_read_bytes(crsf->uartNum, dtmp, event.size, portMAX_DELAY);

                //extract length and type
                //uint8_t sync = dtmp[0];
                uint8_t length = dtmp[1];
                uint8_t type = dtmp[2];

                //read the rest of the frame
                uint8_t payload_length = length - 2;
                uint8_t payload[payload_length];

                for (int i = 0; i < payload_length; i++) {
                    payload[i] = dtmp[i+3];
                }

                if (type == CRSF_TYPE_CHANNELS) {
                    xSemaphoreTake(crsf->xMutex, portMAX_DELAY);
                    crsf->received_channels = *(crsf_channels_t*)payload;
                    xSemaphoreGive(crsf->xMutex);
                } else{
                    ESP_LOGI(TAG_CRSF, "type: 0x%X", type);
                }
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

//receive uart data frame
void CRSF::CRSF_receive_channels(crsf_channels_t *channels){
    xSemaphoreTake(xMutex, portMAX_DELAY);
    *channels = received_channels;
    xSemaphoreGive(xMutex);
}

/**
 * @brief function sends payload to a destination using uart
 * 
 * @param payload_length length of the payload type
 * @param type type of data contained in payload
 * @param payload pointer to payload of given crsf_type_t
 */
void CRSF::CRSF_send_packet(uint8_t payload_length, crsf_type_t type, const void* payload){
    uint8_t packet[payload_length+4]; //payload + dest + len + type + crc

    packet[0] = CRSF_SYNC;
    packet[1] = payload_length+2; // size of payload + type + crc
    packet[2] = type;

    memcpy(&packet[3], payload, payload_length);

    //calculate crc
    unsigned char checksum = crc8(&packet[2], payload_length+1);
    
    packet[payload_length+3] = checksum;

    //send frame
    uart_write_bytes(uartNum, &packet, payload_length+4);
}

/**
 * @brief function sends extended packet
 *
 * @param payload_length length of the payload type
 * @param type type of data contained in payload
 * @param dest destination device
 * @param src source device
 * @param payload pointer to payload of given crsf_type_t
 */
void CRSF::CRSF_send_extended_packet(uint8_t payload_length, crsf_type_t type, uint8_t dest, uint8_t src, const void* payload){
    uint8_t packet[payload_length+6]; //payload + dest + len + type + crc

    packet[0] = CRSF_SYNC;
    packet[1] = payload_length+4; // size of payload + type + crc
    packet[2] = type;
    packet[3] = dest;
    packet[4] = src;

    memcpy(&packet[5], payload, payload_length);

    //calculate crc
    unsigned char checksum = crc8(&packet[2], payload_length+1);
    
    packet[payload_length+5] = checksum;

    //send frame
    uart_write_bytes(uartNum, &packet, payload_length+6);
}

void CRSF::CRSF_send_battery_data(crsf_battery_t* payload){
    crsf_battery_t* payload_proc = 0;
    //processed payload
    payload_proc = (crsf_battery_t*)payload;
    payload_proc->voltage = __bswap16(payload_proc->voltage);
    payload_proc->current = __bswap16(payload_proc->current);
    payload_proc->capacity = __bswap16(payload_proc->capacity) << 8;

    CRSF_send_packet(sizeof(crsf_battery_t), CRSF_TYPE_BATTERY, payload_proc);
}

void CRSF::CRSF_send_gps_data(crsf_gps_t* payload){
    crsf_gps_t* payload_proc = 0;
    //processed payload
    payload_proc = (crsf_gps_t*)payload;
    payload_proc->latitude = __bswap32(payload_proc->latitude);
    payload_proc->longitude = __bswap32(payload_proc->longitude);
    payload_proc->groundspeed = __bswap16(payload_proc->groundspeed);
    payload_proc->heading = __bswap16(payload_proc->heading);
    payload_proc->altitude = __bswap16(payload_proc->altitude);

    CRSF_send_packet(sizeof(crsf_gps_t), CRSF_TYPE_GPS, payload_proc);
}

void CRSF::CRSF_send_altitute_data(crsf_altitude_t* payload){
    crsf_altitude_t* payload_proc = 0;
    //processed payload
    payload_proc = (crsf_altitude_t*)payload;
    payload_proc->altitude = __bswap16(payload_proc->altitude);
    payload_proc->verticalSpeed = __bswap16(payload_proc->verticalSpeed);

    CRSF_send_packet(sizeof(crsf_altitude_t), CRSF_TYPE_ALTITUDE, payload_proc);
}