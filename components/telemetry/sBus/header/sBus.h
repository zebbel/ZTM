#ifndef SBUS_H
#define SBUS_H

#include "driver/uart.h"
#include <stdio.h>
#include <iostream> 
#include <string.h>
#include "esp_log.h"

#include "typedef.h"


#define SBUS_UART_BUFFER_SIZE 30
#define SBUS_QUEUE_SIZE 5

#define SBUS_TIMEOUT 50

#define     SBUS_SERVO_MIN                                  172
#define     SBUS_SERVO_MAX                                  1811

typedef struct{
    uint16_t channels[16];
    bool failSave = true;
}sBus_data_t;

class sBus{
    public:
        uint64_t lastFrame = 0;
        sBus_data_t sBusData;
        QueueHandle_t *channelQueue;

        sBus();
        void init(uint8_t uartNumVal, QueueHandle_t *channelQueueInst);
    private:
        uint8_t uartNum;
        QueueHandle_t uart_queue;

        static void uart_event_task(void *pvParameter);
};

#endif