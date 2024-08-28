#include "SRXL2.h"

static const char *srxl2Tag = "SRXL2";

SRXL2::SRXL2(){

}

void SRXL2::init(Sensor *sensorInst, uint8_t uartNumVal, QueueHandle_t *queue, QueueHandle_t *channelQueueInst){
    sensor = sensorInst;
    uartNum = uartNumVal;
    extern_queue = queue;
    channelQueue = channelQueueInst;

    ESP_LOGI(srxl2Tag, "init iBus2 on uart%d", uartNum);

    // Configure parameters of an UART driver,
    // communication pins and install the driver
    uart_config_t uart_config;
    uart_config.baud_rate = 115200;
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
            ESP_LOGE(srxl2Tag, "uart num error: %d", uartNum);
            break;
    }

    ESP_LOGI(srxl2Tag, "installed SRXL2 on UART %d", uartNum);

    getSrxl2Config();

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "srxl2_uart_event_task", 3048, this, 12, NULL);
    xTaskCreate(loopTask, "srxl2_loop_task", 3048, this, 12, NULL);
}


void SRXL2::uart_event_task(void *pvParameter){
    SRXL2* srxl2 = reinterpret_cast<SRXL2*>(pvParameter); //obtain the instance pointer

    uart_event_t event;
    uint8_t buffer[SRXL2_UART_BUFFER_SIZE];

    while(1){
        if(xQueueReceive(srxl2->uart_queue, &event, 0)) {
            if(event.type == UART_DATA){
                memset(buffer, 0, sizeof(buffer));
                uart_read_bytes(srxl2->uartNum, buffer, event.size, portMAX_DELAY);

                if(srxl2->checkCRC(buffer, buffer[2] + 2)){
                    switch(buffer[1]){
                        case SRXL2_PACKET_HANDSHAKE:{
                            srxl2->handleHandshake(buffer);
                        }break;
                        case SRXL2_PACKET_TELEMETRY:{
                            srxl2->handelTelemetry(buffer);
                        }break;
                    }
	            }
            }
        }  

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void SRXL2::loopTask(void *pvParameter){
    SRXL2* srxl2 = reinterpret_cast<SRXL2*>(pvParameter); //obtain the instance pointer

    uint32_t lastControllData = xTaskGetTickCount();
	uint8_t telemetryRequestCounter = 0;  
    sBus_data_t sbusData;

    while(1){
        uint32_t timeNow = xTaskGetTickCount();

        if(xQueueReceive(*srxl2->channelQueue, &sbusData, 0)){
            srxl2->setControll(sbusData.channels[2]);
            srxl2->failSave = sbusData.failSave;
        }

        if(srxl2->connected){
            if((timeNow > lastControllData + SRXL2_CONTROLL_DATA_DELAY)){
                if(srxl2->failSave){
                    srxl2->sendPacket(srxl2->controllDataUnarmed, SRXL2_CONTROLL_DATA_UNARMED_LENGTH-2);
                }
                else{
                    if(telemetryRequestCounter++ > SRXL2_TELEMETRY_REQUEST){
                        srxl2->controllData[4] = SRXL2_ESC_ID;
                        telemetryRequestCounter = 0;
                    }else{
                        srxl2->controllData[4] = 0x00;
                    }
                    srxl2->sendPacket(srxl2->controllData, SRXL2_CONTROLL_DATA_LENGTH);
                }

                lastControllData = timeNow;                                                                                                                             // save time we last send a controll data packet

                if(timeNow >= srxl2->lastTelemetryPacket + SRXL2_CONNECTION_TIMEOUT){                                                                                                // if last time a telemetry packet is greater then CONNECTION_TIMEOUT ESC is not connected anymore
                    srxl2->connected = false;
                }
            }
        }else{                                                                                                                                                          // else ESC is not connected
            srxl2->lastTelemetryPacket = timeNow;                                                                                                                              // set lastTelemetryPacket to acuall time
        }
    }
}

void SRXL2::setControll(uint16_t value){
    value = (value - SBUS_SERVO_MIN) * (SRXL2_SERVO_MAX - SRXL2_SERVO_MIN) / (SBUS_SERVO_MAX - SBUS_SERVO_MIN) + SRXL2_SERVO_MIN;                                   // map ibus style servo value to srxl2 style servo value

    controllData[12] = value & 0xFF;
    controllData[13] = value >> 8;
}

void SRXL2::handleHandshake(uint8_t *buffer){
    // if the packet is from the ESC (just in case even no other device is connected :-D)
    if(buffer[3] == SRXL2_ESC_ID){
        sendPacket(handshakePacket, SRXL2_HANDSHAKE_LENGTH);
        // ensure we don't send a controll data packet immediately after handshake packet
        vTaskDelay(SRXL2_CONTROLL_DATA_DELAY / portTICK_PERIOD_MS);
        
        connected = true;

        ESP_LOGI(srxl2Tag, "add SRXL2 esc sensors");
        sensor->addSensor(SENSOR_FUNCTION_BAT_VOLT, SENSOR_UNIT_VOLTS, 1, "batVolt");
        sensor->addSensor(SENSOR_FUNCTION_MOTOR_CURRENT, SENSOR_UNIT_AMPS, 1, "motorA");
        sensor->addSensor(SENSOR_FUNCTION_MOTOR_RPM, SENSOR_UNIT_RPM, 0, "motorRPM");
        sensor->addSensor(SENSOR_FUNCTION_ESC_TEMP, SENSOR_UNIT_CELSIUS, 0, "escT");
        sensor->addSensor(SENSOR_FUNCTION_MOTOR_POWER, SENSOR_UNIT_WATTS, 0, "motorP");
    }
}

void SRXL2::handelTelemetry(uint8_t *buffer){
    sensor_t *sensorInst;

    if(sensor->sensors.find("motorRPM") != sensor->sensors.end()){
        sensorInst = sensor->sensors["motorRPM"];
        sensorInst->value = (buffer[6] << 8) + buffer[7];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("batVolt") != sensor->sensors.end()){
        sensorInst = sensor->sensors["batVolt"];
        sensorInst->value = (buffer[8] << 8) + buffer[9];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("escT") != sensor->sensors.end()){
        sensorInst = sensor->sensors["escT"];
        sensorInst->value = (buffer[10] << 8) + buffer[11];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("motorA") != sensor->sensors.end()){
        sensorInst = sensor->sensors["motorA"];
        sensorInst->value = (buffer[12] << 8) + buffer[13];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("motorP") != sensor->sensors.end()){
        sensorInst = sensor->sensors["motorP"];
        sensorInst->value = buffer[19];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    lastTelemetryPacket = xTaskGetTickCount();
}

void SRXL2::sendPacket(uint8_t *buffer, uint8_t len){
    uint16_t checksum = getChecksum(buffer, len);
    memcpy(&buffer[len-2], &checksum, 2);
    uart_write_bytes(uartNum, buffer, len); 
}

void SRXL2::initCrc16Tab(){
	uint16_t crc;
	uint16_t c;

	for(uint16_t i=0; i<256; i++){
		crc = 0;
		c = i << 8;

		for(uint16_t j=0; j<8; j++){
			if((crc ^ c) & 0x8000 ) crc = ( crc << 1 ) ^ SRXL2_POLYNOM;
			else crc = crc << 1;
            
			c = c << 1;
		}
		crcTab16[i] = crc;
	}
}

uint8_t SRXL2::getChecksum(uint8_t *buffer, uint8_t len){
    uint16_t crc = 0x0000;

	for (uint8_t i=0; i<len; i++) {
		crc = (crc << 8) ^ crcTab16[((crc >> 8) ^ (uint16_t) buffer[i]) & 0x00FF];
	}

	return crc;
}

bool SRXL2::checkCRC(uint8_t *buffer, uint8_t len){
	return getChecksum(buffer, len) == ((buffer[buffer[2]-2] << 8) + buffer[buffer[2]-1]);
}


void SRXL2::getSrxl2Config(){
    // open nvs
    esp_err_t err = nvs_open("config", NVS_READONLY, &configNVS);

    // if nvs could not be opened print error
    if(err != ESP_OK){
        ESP_LOGE(srxl2Tag, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else{
        size_t len = sizeof(uint8_t);
        err = nvs_get_blob(configNVS, "uart", &controlChannel, &len);
        if(err != ESP_OK){
            ESP_LOGE(srxl2Tag, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        }else{
            // close auto connect nvs
            nvs_close(configNVS);

            ESP_LOGI(srxl2Tag, "config loaded");
            ESP_LOGI(srxl2Tag, "control channel: %d", controlChannel);
        }
    }
}

void SRXL2::setSrxl2Config(){
    // open nvs
    esp_err_t err = nvs_open("config", NVS_READWRITE, &configNVS);

    // if nvs could not be opened print error
    if(err != ESP_OK){
        ESP_LOGE(srxl2Tag, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    // else save to nvs
    } else{
        err = nvs_set_blob(configNVS, "uart", &controlChannel, sizeof(uint8_t));
        // write to nvs
        err = nvs_commit(configNVS);
    }
    // close nvs
    nvs_close(configNVS);

    ESP_LOGI(srxl2Tag, "config saved");
}