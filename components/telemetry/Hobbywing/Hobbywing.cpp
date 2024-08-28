#include "Hobbywing.h"

static const char *hobbywingTag = "Hobbywing";

Hobbywing::Hobbywing(){

}

void Hobbywing::init(Sensor *sensorInst, uint8_t uartNumVal, QueueHandle_t *queue){
    sensor = sensorInst;
    uartNum = uartNumVal;
    extern_queue = queue;

    ESP_LOGI(hobbywingTag, "init iBus2 on uart%d", uartNum);

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
            ESP_LOGE(hobbywingTag, "uart num error: %d", uartNum);
            break;
    }

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "hobbywing_uart_event_task", 3048, this, 12, NULL);
    xTaskCreate(loopTask, "hobbywing_loop_task", 3048, this, 12, NULL);
}

void Hobbywing::uart_event_task(void *pvParameter){
    Hobbywing* hobbywing = reinterpret_cast<Hobbywing*>(pvParameter); //obtain the instance pointer

    uart_event_t event;
    uint8_t buffer[HOBBYWING_UART_BUFFER_SIZE];

    while(1){
        if(xQueueReceive(hobbywing->uart_queue, &event, 0)) {
            if(event.type == UART_DATA){
                memset(buffer, 0, sizeof(buffer));
                uart_read_bytes(hobbywing->uartNum, buffer, event.size, portMAX_DELAY);

                if(hobbywing->checkCRC(buffer, event.size, false)){
                    if(buffer[0] == 0xFE){
                        switch(buffer[3]){
                            case 0x03:
                                switch(buffer[4]){
                                    case 0x2C:
                                        if(buffer[5] == 0x24){
                                            hobbywing->connected = true;
                                            hobbywing->addSensors();
                                        }
                                        if(buffer[5] == 0x25) hobbywing->sendPacket(hobbywing->getEscProfile, 5);
                                        break;
                                    case 0x30:
                                        if(buffer[5] == 0x5C) hobbywing->handelTelemetry(buffer);
                                        break;
                                    case 0x31:
                                        if(buffer[5] == 0x38) hobbywing->sendPacket(hobbywing->getEscParameters, 5);
                                        break;
                                }
                                break;
                        }
                    }
	            }
            }
        }  

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Hobbywing::loopTask(void *pvParameter){
    Hobbywing* hobbywing = reinterpret_cast<Hobbywing*>(pvParameter); //obtain the instance pointer

	switch(hobbywing->packetStatemachine){
		case HOBBYWING_STATEMACHINE_TELEMTRY:																															// if statemachine is telemetry
			if( hobbywing->throttle > HOBBYWING_DISCONNECTET_MIN_THROTTLE && 																									// if throttle is below DISCONNECTET_MIN_THROTTLE and
				(xTaskGetTickCount() >= hobbywing->escconnectedTimeout + HOBBYWING_CONNECTED_TIMEOUT)){																// last packet was received longer then connectedTimeout ago
				hobbywing->connected = false;																																	// set connected to false
			}
			break;
		case HOBBYWING_STATEMACHINE_START_TELEMETRY:																													// if packet state machine is start telemetry
			hobbywing->sendEscAvailable();																																		// check esc available
			break;
		case HOBBYWING_STATEMACHINE_START_PARAMETR:																													// if packet state machine is start parameter
			hobbywing->sendEscAvailable();																																		// check esc available
			break;
	}
}

void Hobbywing::handelTelemetry(uint8_t *buffer){
    sensor_t *sensorInst;

    if(sensor->sensors.find("motorRPM") != sensor->sensors.end()){
        sensorInst = sensor->sensors["motorRPM"];
        sensorInst->value = ((buffer[14] << 8) + buffer[13]);
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("batVolt") != sensor->sensors.end()){
        sensorInst = sensor->sensors["batVolt"];
        sensorInst->value = ((buffer[16] << 8) + buffer[15]);
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("escT") != sensor->sensors.end()){
        sensorInst = sensor->sensors["escT"];
        sensorInst->value = (buffer[19]);
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("motorA") != sensor->sensors.end()){
        sensorInst = sensor->sensors["motorA"];
        sensorInst->value = ((buffer[18] << 8) + buffer[17]) * 10;
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("motorT") != sensor->sensors.end()){
        sensorInst = sensor->sensors["motorT"];
        sensorInst->value = (buffer[21]);
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    escconnectedTimeout = xTaskGetTickCount();
	connected = true;
}

void Hobbywing::sendPacket(uint8_t *buffer, uint8_t len){
    uint8_t sendBuffer[HOBBYWING_UART_BUFFER_SIZE];

	uint16_t checksum = getChecksum(buffer, len, true);

    sendBuffer[0] = 0x01;
    sendBuffer[1] = 0xFE;
    sendBuffer[2] = 0x04;
    memcpy(&sendBuffer[3], buffer, len);
    memcpy(&sendBuffer[3+len], &checksum, 2);
    uart_write_bytes(uartNum, buffer, len);
}

void Hobbywing::sendEscAvailable(){
	if(!connected && availableRetryCounter > 0){																													// if not connected and retry counter > 0
		if(xTaskGetTickCount() >= (escconnectedTimeout + HOBBYWING_ESC_AVAILABLE_TIME)){																	// send escAvailableMsg every ESC_AVAILABLE_TIME
			escconnectedTimeout = xTaskGetTickCount();																							// reset resend timeout
			sendPacket(escAvailableMsg, 5);                                                                                  										// escAvailableMsg is send until ESC answers back
			availableRetryCounter--;																																// decrement availableRetryCounter
		}
	} else if(connected){																																			// else if ESC is connected
		switch(packetStatemachine){
			case HOBBYWING_STATEMACHINE_START_TELEMETRY:																												// if start teletry was requested
				sendPacket(setTelemetryMode, 5);																													// send setTelemetryMode to ESC
				packetStatemachine = HOBBYWING_STATEMACHINE_TELEMTRY;																									// set packet statemachine to telemetry
				break;
			case HOBBYWING_STATEMACHINE_START_PARAMETR:																												// if start parameter was requested
				sendPacket(getEscInfo, 5);																															// request ESC infos
				packetStatemachine = HOBBYWING_STATEMACHINE_PARAMETER;																									// set packet statemachine to parameter
				break;
		}
	}
}

void Hobbywing::addSensors(){
    ESP_LOGI(hobbywingTag, "add SRXL2 esc sensors");
    sensor->addSensor(SENSOR_FUNCTION_BAT_VOLT, SENSOR_UNIT_VOLTS, 1, "batVolt");
    sensor->addSensor(SENSOR_FUNCTION_MOTOR_CURRENT, SENSOR_UNIT_AMPS, 1, "motorA");
    sensor->addSensor(SENSOR_FUNCTION_MOTOR_RPM, SENSOR_UNIT_RPM, 0, "motorRPM");
    sensor->addSensor(SENSOR_FUNCTION_ESC_TEMP, SENSOR_UNIT_CELSIUS, 0, "escT");
    sensor->addSensor(SENSOR_FUNCTION_MOTOR_TEMP, SENSOR_UNIT_CELSIUS, 0, "motorT");
}


void Hobbywing::initCrc16Tab(){
	uint16_t crc;
	uint16_t c;

	for(uint16_t i=0; i<256; i++){
		crc = 0;
		c = i;

		for(uint16_t j=0; j<8; j++){
			if((crc ^ c) & 0x0001) crc = (crc >> 1) ^ HOBBYWING_POLYNOM;
			else crc = crc >> 1;

			c = c >> 1;
		}
		crcTab16[i] = crc;
	}
}

uint8_t Hobbywing::getChecksum(uint8_t *buffer, uint8_t len, bool send){
    uint16_t crc = 0xFFFF;																																			// 0xFFFF = start value for crc calcuation
	
	if(send){
		crc = (crc >> 8) ^ crcTab16[ (crc ^ (uint16_t) 0x01) & 0x00FF ];
		crc = (crc >> 8) ^ crcTab16[ (crc ^ (uint16_t) 0xFE) & 0x00FF ];
		crc = (crc >> 8) ^ crcTab16[ (crc ^ (uint16_t) 0x04) & 0x00FF ];
	}

	for (uint8_t i=0; i<len; i++){
		crc = (crc >> 8) ^ crcTab16[ (crc ^ (uint16_t) buffer[i]) & 0x00FF ];
	}

	return crc;
}

bool Hobbywing::checkCRC(uint8_t *buffer, uint8_t len, bool send){
	return getChecksum(buffer, len, send) == ((buffer[len-1] << 8) + buffer[len-2]);
}