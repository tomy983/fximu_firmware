#ifndef FXAS21002C_H_
#define FXAS21002C_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "common.h"

#define GYRO_STATUS         0x00
#define GYRO_OUT_X_MSB      0x01
#define GYRO_OUT_X_LSB      0x02
#define GYRO_OUT_Y_MSB      0x03
#define GYRO_OUT_Y_LSB      0x04
#define GYRO_OUT_Z_MSB      0x05
#define GYRO_OUT_Z_LSB      0x06
#define GYRO_DR_STATUS      0x07
#define GYRO_F_STATUS       0x08
#define GYRO_F_SETUP        0x09
#define GYRO_F_EVENT        0x0A
#define GYRO_INT_SRC_FLAG   0x0B
#define GYRO_WHO_AM_I       0x0C
#define GYRO_CTRL_REG0      0x0D
#define GYRO_RT_CFG         0x0E
#define GYRO_RT_SRC         0x0F
#define GYRO_RT_THS         0x10
#define GYRO_RT_COUNT       0x11
#define GYRO_TEMP           0x12
#define GYRO_CTRL_REG1      0x13
#define GYRO_CTRL_REG2      0x14
#define GYRO_CTRL_REG3      0x15

#define GYRO_RESET          0x40

uint8_t fxas_register[1];
uint8_t fxas_data[7];

extern void I2CGyroReceive(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pReceiveData, uint8_t ui8NumBytes);
extern void I2CGyroSend(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pTransmitData, uint8_t ui8NumBytes);
extern void I2CGyroSingleByteSend(uint8_t ui32WorkerAddress, uint8_t byte);

void GyroInit(uint32_t ui32WorkerAddress, tGyroRange tGFSR, tOutputDataRate tODR);
void GyroGetData(uint32_t ui32WorkerAddress, tRawData *tRD);
uint8_t GyroWhoAmI(uint32_t ui32WorkerAddress);

#ifdef __cplusplus
}
#endif

#endif