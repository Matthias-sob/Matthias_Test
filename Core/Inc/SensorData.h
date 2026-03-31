#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

/* Unterstützte Sensor-ID für den Plotter */
#define SENSOR_ID_IMURATES_SINGLE_PRE       0x0000000Au

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
    float Time;
    float AngularRateIBB[3];
    float AccelerationIBB[3];
    float DeltaTime;

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
        uint32_t reserved           : 24;
    } Validity;

    uint32_t errorCode;
} tMiniImuOpRates;

typedef union
{
    tMiniImuOpRates MiniImuOpRates;
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
