#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <stdint.h>

#include "units.h"
#include "sensorTypes.h"

#define TELEMETRY_QUEUE_SIZE            5
#define TELEMETRY_PAYLOAD_SIZE          30

typedef enum{
    TELEMETRY_BUS_NONE,
    TELEMETRY_BUS_SBUS,
    TELEMETRY_BUS_IBUS,
    TELEMETRY_BUS_IBUS2,
    TELEMETRY_BUS_HOBBYWING,
    TELEMETRY_BUS_SRXL2,
    TELEMETRY_BUS_GPS
} busType_t;

typedef struct{
    uint8_t uart0 = TELEMETRY_BUS_NONE;
    uint8_t uart1 = TELEMETRY_BUS_NONE;
    uint8_t uart2 = TELEMETRY_BUS_NONE;
} uartConfig_t;

typedef struct{
    uint16_t id = SENSOR_FUNCTION_NONE;
    uint8_t unit = SENSOR_UNIT_NONE;
    uint8_t precision = 0;
    int8_t offset = 0;
    int16_t value;
} sensor_t;

#endif