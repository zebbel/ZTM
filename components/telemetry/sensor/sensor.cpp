#include "sensor.h"

static const char *sensorTAG = "sensor";

Sensor::Sensor(){
    
}

void Sensor::addSensor(uint8_t function, uint8_t unit, uint8_t precision, const char *device){
    sensor_t *sensor = (sensor_t *)malloc(sizeof(sensor_t));
    sensor->id = function;
    sensor->unit = unit;
    sensor->precision = precision;
    sensor->offset = 0;
    sensors[device] = sensor;
    getSensorConfig(device, sensor);
}

void Sensor::getSensorConfig(const char *device, sensor_t *sensor){
    ESP_LOGI(sensorTAG, "get sensor config for device: %s", device);
    // open nvs
    esp_err_t err = nvs_open("sensorConfig", NVS_READONLY, &configNVS);

    // if nvs could not be opened print error
    if(err != ESP_OK){
        ESP_LOGE(sensorTAG, "Error (%s) opening NVS handle! sensor", esp_err_to_name(err));
    // else read mac from nvs
    } else{
        size_t len = 1;
        err = nvs_get_blob(configNVS, device, &sensor->id, &len);
        if(err != ESP_OK){
            ESP_LOGE(sensorTAG, "Error (%s) ibus2 NVS for device: %s", esp_err_to_name(err), device);
        }
    }
    // close auto connect nvs
    nvs_close(configNVS);
}

void Sensor::setSensorConfig(const char *device, sensor_t *sensor){
    ESP_LOGI(sensorTAG, "set sensor config for device: %s", device);
    // open nvs
    esp_err_t err = nvs_open("sensorConfig", NVS_READWRITE, &configNVS);

    // if nvs could not be opened print error
    if(err != ESP_OK){
        ESP_LOGE(sensorTAG, "Error (%s) opening NVS handle! sensor", esp_err_to_name(err));
    // else save to nvs
    } else{
        err = nvs_set_blob(configNVS, device, &sensor->id, 1);
        // write to nvs
        err = nvs_commit(configNVS);
    }
    // close nvs
    nvs_close(configNVS);
}

void Sensor::resetConfig(){
    // open nvs
    esp_err_t err = nvs_open("sensorConfig", NVS_READWRITE, &configNVS);
    nvs_erase_all(configNVS);
    nvs_commit(configNVS);
}