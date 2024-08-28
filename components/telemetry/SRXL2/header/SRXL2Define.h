#ifndef SRXL2DEFINE_H
#define SRXL2DEFINE_H

#define     SRXL2_POLYNOM                                   0x1021

#define     SRXL2_RCV_ID                                    0x21
#define     SRXL2_ESC_ID                                    0x40

#define     SRXL2_PACKET_HANDSHAKE                          0x21
#define     SRXL2_PACKET_TELEMETRY                          0x80
#define     SRXL2_PACKET_CONTROLL_DATA                      0xCD

#define     SRXL2_CONTROLL_DATA_DELAY                       11
#define     SRXL2_HANDSHAKE_LENGTH                          0x0E
#define     SRXL2_CONTROLL_DATA_UNARMED_LENGTH              0x0E
#define     SRXL2_CONTROLL_DATA_LENGTH                      0x10

#define     SRXL2_TELEMETRY_REQUEST                         5
#define     SRXL2_CONNECTION_TIMEOUT                        1000

#define     SRXL2_SERVO_MIN                                 10912
#define     SRXL2_SERVO_MAX                                 54612 

#endif