#ifndef HOBBYWING_H
#define HOBBYWING_H

#include "driver/uart.h"
#include <string.h>

#include "HobbywingDefine.h"
#include "sensor.h"

#define HOBBYWING_UART_BUFFER_SIZE      40

class Hobbywing{
    private:
        Sensor *sensor;
        uint8_t uartNum;
        QueueHandle_t *extern_queue;
        QueueHandle_t uart_queue;

        static void uart_event_task(void *pvParameter);

        uint8_t packetStatemachine = HOBBYWING_STATEMACHINE_TELEMTRY;
        static void loopTask(void *pvParameter);

        uint32_t escconnectedTimeout = 0; 
        void handelTelemetry(uint8_t *buffer);

        void sendPacket(uint8_t *buffer, uint8_t len);
        uint8_t availableRetryCounter = 10;    
        void sendEscAvailable();

        void addSensors();

        uint16_t crcTab16[256];
        void initCrc16Tab();
        uint8_t getChecksum(uint8_t *buffer, uint8_t len, bool send);
        bool checkCRC(uint8_t *buffer, uint8_t len, bool send);

    public:
        Hobbywing();
        void init(Sensor *sensorInst, uint8_t uartNumVal, QueueHandle_t *queue);

        bool connected = false;
        uint16_t throttle;

    private:
        uint8_t escAvailableMsg[5] =			{0x03, 0x2C, 0x24, 0x00, 0x01};            																			// message has to be send on startup
        uint8_t setTelemetryMode[5] = 			{0x06, 0x2B, 0x02, 0x55, 0x00};                    																	// message to get ESC in telemetry mode

        uint8_t getEscInfo[5] = 				{0x03, 0x2C, 0x25, 0x00, 0x20};                    																	// ask ESC for device infos
        uint8_t getEscProfile[5] = 				{0x03, 0x31, 0x38, 0x00, 0x40};																						// ask ESC for profile
        uint8_t getEscParameters[5] = 			{0x03, 0x35, 0x38, 0x00, 0x40};																						// ask ESC for paramters
        uint8_t getEscParameterProfileName[5] = {0x03, 0x30, 0x0C, 0x00, 0x40};																						// ask ESC for parameter profile name
        
        uint8_t sendParameterStart[14] = 		{0x17, 0x35, 0x38, 0x00, 0x20, 0x35, 0x38, 0x00, 0x20, 0x40, 0x00};													// fixed start bytes for parameter sending
        uint8_t sendParameterEnd[36] = 			{0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x14, 0x00, 0x01, 0x03, 0x03, 				
                                                0x01, 0x00, 0x00, 0x0B, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
};


#endif