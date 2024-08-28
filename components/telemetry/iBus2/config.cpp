#include "iBus2.h"

/*
void iBus2::getIbus2Config(const char *device, sensor_t *sensor){
    // open nvs
    esp_err_t err = nvs_open("ibus2Config", NVS_READONLY, &configNVS);

    // if nvs could not be opened print error
    if(err != ESP_OK){
        ESP_LOGE(ibus2TAG, "Error (%s) opening NVS handle! ibus2", esp_err_to_name(err));
    // else read mac from nvs
    } else{
        size_t len = 1;
        err = nvs_get_blob(configNVS, device, &sensor->id, &len);
        if(err != ESP_OK){
            ESP_LOGE(ibus2TAG, "Error (%s) ibus2 NVS for device: %s", esp_err_to_name(err), device);
        }
    }
    // close auto connect nvs
    nvs_close(configNVS);
}
*/

/*
void iBus2::setIbus2Config(){
    const char *device;
    sensor_t *sensor;
    switch (sensorType){
        case IBUS2_SENSOR_TEMPERATUR:
            device = "temp";
            sensor = sensors["temp"];
            break;
        
        default:
            device = NULL;
            break;
    }

    if(device != NULL){
        // open nvs
        esp_err_t err = nvs_open("ibus2Config", NVS_READWRITE, &configNVS);

        // if nvs could not be opened print error
        if(err != ESP_OK){
            ESP_LOGE(ibus2TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        // else save to nvs
        } else{
            err = nvs_set_blob(configNVS, device, &sensor->id, 1);
            // write to nvs
            err = nvs_commit(configNVS);
        }
        // close nvs
        nvs_close(configNVS);
    }
}
*/

uint8_t iBus2::getIbus2SensorFunction(){
    switch(sensorType){
        case IBUS2_SENSOR_TEMPERATUR:{
            if(sensor->sensors.find("temp") != sensor->sensors.end()){
                sensor_t *sensorInst = sensor->sensors["temp"];
                return sensorInst->id;
            }else{
                return NULL;
            }
            break;
        }
        case IBUS2_SENSOR_XERUN_AXE:{
            return SENSOR_FUNCTION_XERUN_AXE;
        }
        default:
            return NULL;
            break;
    }
}

void iBus2::setIbus2SensorFunction(uint8_t function){
    switch(sensorType){
        case IBUS2_SENSOR_TEMPERATUR:{
            if(sensor->sensors.find("temp") != sensor->sensors.end()){
                sensor_t *sensorInst = sensor->sensors["temp"];
                sensorInst->id = function;
                sensor->setSensorConfig("temp", sensorInst);
            }
            break;
        }
    }
}