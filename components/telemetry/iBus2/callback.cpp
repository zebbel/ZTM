#include "iBus2.h"

static const char *ibus2TAG = "iBus2";

void iBus2::uart_event_task(void *pvParameter){
    iBus2* ibus2 = reinterpret_cast<iBus2*>(pvParameter); //obtain the instance pointer

    uart_event_t event;
    uint8_t buffer[IBUS2_UART_BUFFER_SIZE];

    while(1){
        if(xQueueReceive(ibus2->uart_queue, &event, 0)) {
            if(event.type == UART_DATA){
                memset(buffer, 0, sizeof(buffer));
                uart_read_bytes(ibus2->uartNum, buffer, event.size, portMAX_DELAY);

                if(buffer[0] != 0x00 && buffer[0] != 0x05 && buffer[0] !=0x09){
                    //ESP_LOGI(ibus2TAG, "%X", buffer[0]);
                    if(buffer[0] == 0x06){
                        //ESP_LOGI(ibus2TAG, "got sensor search response");
                        if(ibus2->checkCRC(buffer, 0x14)){
                            ibus2->addSensor(buffer[1]);
        
                            ibus2->sensorType = buffer[1];
                            ibus2->sendBuffer[0] = 0x09;
                            ibus2->sendBuffer[20] = 0xA9;
                        }
                    }
                    else if(buffer[0] == 0x0A){
                        if(ibus2->checkCRC(buffer, 0x14)){
                            ibus2->handelFrame(buffer);
                        }
                    }
                    
                }
            }
        }  

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void iBus2::uartSendTask(void *pvParameter){
    iBus2* ibus2 = reinterpret_cast<iBus2*>(pvParameter); //obtain the instance pointer
    while(1){
        //uart_write_bytes(ibus2->uartNum, ibus2->channelBuffer, 10); 
        //vTaskDelay(100 / portTICK_PERIOD_MS);
        uart_write_bytes(ibus2->uartNum, ibus2->sendBuffer, sizeof(ibus2->sendBuffer));        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

uint8_t iBus2::getChecksum(uint8_t *buffer, uint8_t len){
    uint16_t checksum = 0xFF;
    for(uint8_t i=0; i<len; i++){
        checksum ^= buffer[i];

        for(uint x=0; x<8; x++){
            checksum <<= 1;

            if(checksum & 0x100){
                checksum ^= 0x125;
            }
        }
    }

    return checksum;
}

bool iBus2::checkCRC(uint8_t *buffer, uint8_t len){
    return buffer[len] == getChecksum(buffer, len);
}
