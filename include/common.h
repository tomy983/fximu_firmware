#ifndef COMMON_H_
#define COMMON_H_

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} tRawData;

typedef enum {
    ODR_800HZ,
    ODR_400HZ,
    ODR_200HZ,
    ODR_100HZ,
    ODR_50HZ,
    ODR_25HZ,
    ODR_12_5HZ
} tOutputDataRate;

typedef enum {
    GFSR_2000PS,
    GFSR_1000PS,
    GFSR_500PS,
    GFSR_250PS
} tGyroRange;

typedef enum {
    AFSR_2G = 0,
    AFSR_4G = 1,
    AFSR_8G = 2
} tAccelRange;

typedef enum {
    ACCEL_ONLY,
    MAG_ONLY,
    ACCEL_AND_MAG = 3
} tHybridMode;

#define GYRO_SENSITIVITY_250DPS  (0.0078125F)
#define GYRO_SENSITIVITY_500DPS  (0.015625F)
#define GYRO_SENSITIVITY_1000DPS (0.03125F)
#define GYRO_SENSITIVITY_2000DPS (0.0625F)

//#define ACCEL_MG_LSB_2G (0.000244F)
//#define ACCEL_MG_LSB_4G (0.000488F)
//#define ACCEL_MG_LSB_8G (0.000976F)

// TODO: get around to remeasuring all these.

#define ACCEL_MG_LSB_2G (0.000244F)
#define ACCEL_MG_LSB_4G (0.000483168F)
#define ACCEL_MG_LSB_8G (0.000976F)

#define MAG_UT_LSB (0.1F)

#define SENSORS_DPS_TO_RADS (0.017453293F)
#define SENSORS_GRAVITY_EARTH (9.80665F)

#endif