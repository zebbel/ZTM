#ifndef HOBBYWINGDEFINE_H
#define HOBBYWINGDEFINE_H

#define     HOBBYWING_POLYNOM                                   0xA001

#define		HOBBYWING_STATEMACHINE_START_TELEMETRY			    0x00
#define		HOBBYWING_STATEMACHINE_TELEMTRY				        0x01
#define     HOBBYWING_STATEMACHINE_START_PARAMETR               0x02
#define     HOBBYWING_STATEMACHINE_PARAMETER                    0x03

#define		HOBBYWING_DISCONNECTET_MIN_THROTTLE					5																									        // if ESC is in telemetry mode and throttel is below that value it prevents connectet to go to false
#define		HOBBYWING_CONNECTED_TIMEOUT							500	
#define		HOBBYWING_ESC_AVAILABLE_TIME						500


#endif