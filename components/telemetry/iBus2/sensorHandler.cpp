#include "iBus2.h"

static const char *ibus2TAG = "iBus2";

void iBus2::addSensor(uint8_t sensorType){
    switch (sensorType){
        case IBUS2_SENSOR_TEMPERATUR:{
            ESP_LOGI(ibus2TAG, "add temperatur sensor");
            sensor->addSensor(SENSOR_FUNCTION_NONE, SENSOR_UNIT_CELSIUS, 1, "temp");
            break;
        }
        case IBUS2_SENSOR_XERUN_AXE:{
            ESP_LOGI(ibus2TAG, "add xerun axe sensors");
            sensor->addSensor(SENSOR_FUNCTION_BAT_VOLT, SENSOR_UNIT_VOLTS, 1, "batVolt");
            sensor->addSensor(SENSOR_FUNCTION_MOTOR_CURRENT, SENSOR_UNIT_AMPS, 1, "motorA");
            sensor->addSensor(SENSOR_FUNCTION_MOTOR_RPM, SENSOR_UNIT_RPM, 0, "motorRPM");
            sensor->addSensor(SENSOR_FUNCTION_BAT_MHA, SENSOR_UNIT_MAH, 0, "batMHA");
            sensor->addSensor(SENSOR_FUNCTION_ESC_TEMP, SENSOR_UNIT_CELSIUS, 0, "escT");
            sensor->addSensor(SENSOR_FUNCTION_MOTOR_TEMP, SENSOR_UNIT_CELSIUS, 0, "motorT");

            //sensor = (sensor_t *)malloc(sizeof(sensor_t));
            //convertSensorType(sensor, IBUS2_SENSOR_XERUN_AXE);
            break;
        }
        default:
            break;
    }
}

/*
void iBus2::convertSensorType(sensor_t *sensor, uint8_t ibusSensorType){
    switch (ibusSensorType){
        case IBUS2_SENSOR_TEMPERATUR:{
            sensor->id = SENSOR_FUNCTION_NONE;
            sensor->unit = SENSOR_UNIT_CELSIUS;
            sensor->precision = 1;
            sensors["temp"] = sensor;
            getIbus2Config("temp", sensor);
            break;
        }
        case IBUS2_SENSOR_EXT_VOLT:{
            sensor->id = SENSOR_FUNCTION_BAT_VOLT;
            sensor->unit = SENSOR_UNIT_VOLTS;
            sensor->precision = 1;
            sensors["batVolt"] = sensor;
            getIbus2Config("batVolt", sensor);
            break;
        }
        case IBUS2_SENSOR_CURRENT:{
            sensor->id = SENSOR_FUNCTION_MOTOR_CURRENT;
            sensor->unit = SENSOR_UNIT_AMPS;
            sensor->precision = 1;
            sensors["motorA"] = sensor;
            getIbus2Config("motorA", sensor);
            break;
        }
        case IBUS2_SENSOR_MOTOR_SPEED:{
            sensor->id = SENSOR_FUNCTION_MOTOR_RPM;
            sensor->unit = SENSOR_UNIT_RPM;
            sensor->precision = 0;
            sensors["motorRPM"] = sensor;
            getIbus2Config("motorRPM", sensor);
            break;
        }
        case IBUS2_SENSOR_MHA:{
            sensor->id = SENSOR_FUNCTION_BAT_MHA;
            sensor->unit = SENSOR_UNIT_MAH;
            sensor->precision = 0;
            sensors["batMHA"] = sensor;
            getIbus2Config("batMHA", sensor);
            break;
        }
        case IBUS2_SENSOR_ESC_TEMPERATUR:{
            sensor->id = SENSOR_FUNCTION_ESC_TEMP;
            sensor->unit = SENSOR_UNIT_CELSIUS;
            sensor->precision = 0;
            sensors["escT"] = sensor;
            getIbus2Config("escT", sensor);
            break;
        }
        case IBUS2_SENSOR_MOTOR_TEMPERATUR:{
            sensor->id = SENSOR_FUNCTION_MOTOR_TEMP;
            sensor->unit = SENSOR_UNIT_CELSIUS;
            sensor->precision = 0;
            sensors["motorT"] = sensor;
            getIbus2Config("motorT", sensor);
            break;
        }
        case IBUS2_SENSOR_XERUN_AXE:{
            sensor->id = SENSOR_FUNCTION_XERUN_AXE;
            sensor->unit = SENSOR_UNIT_NONE;
            sensor->precision = 0;
            sensors["xerun"] = sensor;
            break;
        }
        default:
            ESP_LOGE(ibus2TAG, "unsupportet sensor type: %d", ibusSensorType);
            break;
    }
}
*/