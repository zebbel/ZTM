#ifndef SENSOR_H
#define SENSOR_H

#include <map>
#include <iostream>
#include "nvs_flash.h"
#include "esp_log.h"

#include "typedef.h"

class Sensor{
    private:
        nvs_handle_t configNVS;
        void getSensorConfig(const char *device, sensor_t *sensor);
    public:
        void setSensorConfig(const char *device, sensor_t *sensor);
        void resetConfig();

    private:

    public:
        std::map<std::string, sensor_t*> sensors;
        Sensor();

        void addSensor(uint8_t function, uint8_t unit, uint8_t precision, const char *device);
};



#endif