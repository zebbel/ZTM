#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "nvs_flash.h"
#include "typedef.h"

#include "sensor.h"
#include "sBus.h"
#include "iBus2.h"
#include "SRXL2.h"
#include "Hobbywing.h"


class Telemetry: public iBus2, public sBus, public SRXL2, public Hobbywing{
    private:
        nvs_handle_t configNVS;

        void initUarts();
        void getTelemetryConfig();

    public:
        Sensor sensor;
        uint8_t uartConfig[3];
        QueueHandle_t sensorQueue = xQueueCreate(TELEMETRY_QUEUE_SIZE, sizeof(sensor_t));
        QueueHandle_t channelQueue = xQueueCreate(SBUS_QUEUE_SIZE, sizeof(sBus_data_t));
        Telemetry();
        void setTelemetryConfig();
};

#endif