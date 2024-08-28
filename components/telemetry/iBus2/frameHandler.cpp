#include "iBus2.h"

void iBus2::handelFrame(uint8_t *buffer){
    switch (sensorType){
        case IBUS2_SENSOR_TEMPERATUR:
            handelTemperaturFrame((buffer[2] << 8) + buffer[1]);
            break;
        case IBUS2_SENSOR_XERUN_AXE:
            handelXerunAxeFrame(buffer);
            break;
        default:
            break;
    }
}

void iBus2::handelTemperaturFrame(int16_t temp){
    if(sensor->sensors.find("temp") != sensor->sensors.end()){
        sensor_t *sensorInst = sensor->sensors["temp"];
        temp = ((temp - 320) / 1.8) + sensorInst->offset;                                      // convert to °C
        sensorInst->value = temp;
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }
}

void iBus2::handelXerunAxeFrame(uint8_t *buffer){
    sensor_t *sensorInst;

    if(sensor->sensors.find("batVolt") != sensor->sensors.end()){
        sensorInst = sensor->sensors["batVolt"];
        sensorInst->value = (buffer[2] << 8) + buffer[1];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("motorA") != sensor->sensors.end()){
        sensorInst = sensor->sensors["motorA"];
        sensorInst->value = (buffer[4] << 8) + buffer[3];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("motorRPM") != sensor->sensors.end()){
        sensorInst = sensor->sensors["motorRPM"];
        sensorInst->value = (buffer[6] << 8) + buffer[5];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("batMHA") != sensor->sensors.end()){
        sensorInst = sensor->sensors["batMHA"];
        sensorInst->value = (buffer[9] << 8) + buffer[8];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("escT") != sensor->sensors.end()){
        sensorInst = sensor->sensors["escT"];
        sensorInst->value = buffer[10];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }

    if(sensor->sensors.find("motorT") != sensor->sensors.end()){
        sensorInst = sensor->sensors["motorT"];
        sensorInst->value = buffer[11];
        xQueueSend(*extern_queue, sensorInst, portMAX_DELAY);
    }
}