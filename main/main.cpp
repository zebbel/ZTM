#include <stdio.h>

#include "ESP_CRSF.h"
#include "telemetry.h"

//static const char *mainTag = "main";

CRSF crsf;
Telemetry telemetry;

extern "C" void app_main(void){
    
    crsf.CRSF_init(UART_NUM_0);
    crsf_altitude_t altitute = {1000, 0};
    

    while(1){
        sensor_t sensor;
        if(xQueueReceive(telemetry.sensorQueue, &sensor, 0)) {
            //ESP_LOGI(mainTag, "%d", sensor.value);
            altitute.verticalSpeed = sensor.value * 10;
            crsf.CRSF_send_altitute_data(&altitute);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}