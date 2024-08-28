#include "telemetry.h"

static const char *telemetryTAG = "telemetry";

Telemetry::Telemetry(){
    /*
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    getTelemetryConfig();
    */
    uartConfig[0] = TELEMETRY_BUS_NONE;
    uartConfig[1] = TELEMETRY_BUS_NONE;
    uartConfig[2] = TELEMETRY_BUS_IBUS2;
    initUarts();
}

void Telemetry::initUarts(){
    switch (uartConfig[0]){
        case TELEMETRY_BUS_SBUS:{
            sBus::init(UART_NUM_0, &channelQueue);
            break;
        }
        case TELEMETRY_BUS_IBUS2:{
            iBus2::init(&sensor, UART_NUM_0, &sensorQueue);
            break;
        }
        case TELEMETRY_BUS_SRXL2:{
            SRXL2::init(&sensor, UART_NUM_0, &sensorQueue, &channelQueue);
            break;
        }
        case TELEMETRY_BUS_HOBBYWING:{
            Hobbywing::init(&sensor, UART_NUM_0, &sensorQueue);
            break;
        }
        default:
            break;
    }

    switch (uartConfig[1]){
        case TELEMETRY_BUS_SBUS:{
            sBus::init(UART_NUM_1, &channelQueue);
            break;
        }
        case TELEMETRY_BUS_IBUS2:{
            iBus2::init(&sensor, UART_NUM_1, &sensorQueue);
            break;
        }
        case TELEMETRY_BUS_SRXL2:{
            SRXL2::init(&sensor, UART_NUM_1, &sensorQueue, &channelQueue);
            break;
        }
        case TELEMETRY_BUS_HOBBYWING:{
            Hobbywing::init(&sensor, UART_NUM_1, &sensorQueue);
            break;
        }
        default:
            break;
    }

    switch (uartConfig[2]){
        case TELEMETRY_BUS_SBUS:{
            sBus::init(UART_NUM_2, &channelQueue);
            break;
        }
        case TELEMETRY_BUS_IBUS2:{
            iBus2::init(&sensor, UART_NUM_2, &sensorQueue);
            break;
        }
        case TELEMETRY_BUS_SRXL2:{
            SRXL2::init(&sensor, UART_NUM_2, &sensorQueue, &channelQueue);
            break;
        }
        case TELEMETRY_BUS_HOBBYWING:{
            Hobbywing::init(&sensor, UART_NUM_2, &sensorQueue);
            break;
        }
        default:
            break;
    }
}

void Telemetry::getTelemetryConfig(){
    // open nvs
    esp_err_t err = nvs_open("config", NVS_READONLY, &configNVS);

    // if nvs could not be opened print error
    if(err != ESP_OK){
        ESP_LOGE(telemetryTAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    // else read mac from nvs
    } else{
        size_t len = sizeof(uartConfig);
        err = nvs_get_blob(configNVS, "uart", &uartConfig, &len);
    }
    // close auto connect nvs
    nvs_close(configNVS);

    ESP_LOGI(telemetryTAG, "config loaded");
    ESP_LOGI(telemetryTAG, "uart0: %d, uart1: %d, uart2: %d", uartConfig[0], uartConfig[1], uartConfig[2]);
}

void Telemetry::setTelemetryConfig(){
    // open nvs
    esp_err_t err = nvs_open("config", NVS_READWRITE, &configNVS);

    // if nvs could not be opened print error
    if(err != ESP_OK){
        ESP_LOGE(telemetryTAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    // else save to nvs
    } else{
        err = nvs_set_blob(configNVS, "uart", &uartConfig, sizeof(uartConfig));
        // write to nvs
        err = nvs_commit(configNVS);
    }
    // close nvs
    nvs_close(configNVS);

    ESP_LOGI(telemetryTAG, "config saved");
}