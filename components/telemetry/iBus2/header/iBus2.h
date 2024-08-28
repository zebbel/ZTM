#ifndef IBUS2_H
#define IBUS2_H

#include <stdio.h>
#include <iostream> 
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include <map>
#include "nvs_flash.h"

#include "iBus2SensorTypes.h"
#include "typedef.h"
#include "sensor.h"

#define IBUS2_UART_BUFFER_SIZE 21
#define IBUS2_QUEUE_SIZE 5

class iBus2{
    public:
        iBus2();
        void init(Sensor *sensorInst, uint8_t uartNumVal, QueueHandle_t *queue);

    private:
        Sensor *sensor;
        uint8_t uartNum;
        QueueHandle_t *extern_queue;
        QueueHandle_t uart_queue;

        uint8_t sendBuffer[IBUS2_UART_BUFFER_SIZE];
        uint8_t channelBuffer[IBUS2_UART_BUFFER_SIZE] = {0x00, 0x0A, 0x07, 0x00, 0xC0, 0xFF, 0x4F, 0x83, 0xC1, 0xC3};
        uint8_t sensorType;
        //std::map<std::string, sensor_t*> sensors;

        static void uart_event_task(void *pvParameter);
        static void uartSendTask(void *pvParameter);
    public:
        uint8_t getChecksum(uint8_t *buffer, uint8_t len);
    private:
        bool checkCRC(uint8_t *buffer, uint8_t len);

        void handelFrame(uint8_t *buffer);
        void handelTemperaturFrame(int16_t temp);
        void handelXerunAxeFrame(uint8_t *buffer);

        void addSensor(uint8_t sensorType);
        //void convertSensorType(sensor_t *sensor, uint8_t ibusSensorType);

        nvs_handle_t configNVS;
        //void getIbus2Config(const char *device, sensor_t *sensor);

    public:
        //void setIbus2Config();
        uint8_t getIbus2SensorFunction();
        void setIbus2SensorFunction(uint8_t function);

};

#endif