#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

/* Unterstützte Sensor-ID für den Plotter (double precision) */
#define SENSOR_ID_IMURATES_DOUBLE           0x00000008u

/* Beispielhafte IMU-SubID */
#define SENSOR_SUBID_MEMS_MINI_V12_0        0x00000107u

typedef struct
{
    uint32_t ID;
    uint32_t SubID;
    uint32_t Index;
} tSensorInfo;

typedef struct
{
    double Time;
    double AngularRateIBB[3];
    double AccelerationIBB[3];
    double TimePeriod;

    struct
    {
        uint32_t vTime              : 1;
        uint32_t vAngularRateIBB_X  : 1;
        uint32_t vAngularRateIBB_Y  : 1;
        uint32_t vAngularRateIBB_Z  : 1;
        uint32_t vAccelerationIBB_X : 1;
        uint32_t vAccelerationIBB_Y : 1;
        uint32_t vAccelerationIBB_Z : 1;
        uint32_t vTimePeriod        : 1;
        uint32_t vCheckSum          : 1;
        uint32_t reserved           : 23;
    } Validity;

    uint32_t errorCode;
} tImuRates;   /* IMURates (double precision) - ID: 0x00000008 */

typedef union
{
    tImuRates ImuRates;
} tSensorMeasurement;

typedef struct
{
    tSensorInfo Ident;
    tSensorMeasurement Values;
} tSensorData;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* SENSORDATA_H */
