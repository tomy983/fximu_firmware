#ifndef FXOS8700CQ_H_
#define FXOS8700CQ_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "common.h"

#define AG_STATUS           0x00
#define AG_DR_STATUS        0x00
#define AG_F_STATUS         0x00
#define AG_OUT_X_MSB        0x01
#define AG_OUT_X_LSB        0x02
#define AG_OUT_Y_MSB        0x03
#define AG_OUT_Y_LSB        0x04
#define AG_OUT_Z_MSB        0x05
#define AG_OUT_Z_LSB        0x06
#define AG_F_SETUP          0x09
#define AG_TRIG_CFG         0x0A
#define AG_SYSMOD           0x0B
#define AG_INT_SOURCE       0x0C
#define AG_WHO_AM_I         0x0D
#define AG_XYZ_DATA_CFG     0x0E
#define AG_HP_FILTER_CUTOFF 0x0F
#define AG_PL_STATUS        0x10
#define AG_PL_CFG           0x11
#define AG_PL_COUNT         0x12
#define AG_PL_BF_ZCOMP      0x13
#define AG_P_L_THS_REG      0x14
#define AG_A_FFMT_CFG       0x15
#define AG_A_FFMT_SRC       0x16
#define AG_A_FFMT_THS       0x17
#define AG_A_FFMT_COUNT     0x18
#define AG_TRANSIENT_CFG    0x1D
#define AG_TRANSIENT_SRC    0x1E
#define AG_TRANSIENT_THS    0x1F
#define AG_TRANSIENT_COUNT  0x20
#define AG_PULSE_CFG        0x21
#define AG_PULSE_SRC        0x22
#define AG_PULSE_THSX       0x23
#define AG_PULSE_THSY       0x24
#define AG_PULSE_THSZ       0x25
#define AG_PULSE_TMLT       0x26
#define AG_PULSE_LTCY       0x27
#define AG_PULSE_WIND       0x28
#define AG_ASLP_COUNT       0x29
#define AG_CTRL_REG1        0x2A
#define AG_CTRL_REG2        0x2B
#define AG_CTRL_REG3        0x2C
#define AG_CTRL_REG4        0x2D
#define AG_CTRL_REG5        0x2E
#define AG_OFF_X            0x2F
#define AG_OFF_Y            0x30
#define AG_OFF_Z            0x31
#define AG_M_DR_STATUS      0x32
#define AG_M_OUT_X_MSB      0x33
#define AG_M_OUT_X_LSB      0x34
#define AG_M_OUT_Y_MSB      0x35
#define AG_M_OUT_Y_LSB      0x36
#define AG_M_OUT_Z_MSB      0x37
#define AG_M_OUT_Z_LSB      0x38
#define AG_CMP_OUT_X_MSB    0x39
#define AG_CMP_OUT_X_LSB    0x3A
#define AG_CMP_OUT_Y_MSB    0x3B
#define AG_CMP_OUT_Y_LSB    0x3C
#define AG_CMP_OUT_Z_MSB    0x3D
#define AG_CMP_OUT_Z_LSB    0x3E
#define AG_M_OFF_X_MSB      0x3F
#define AG_M_OFF_X_LSB      0x40
#define AG_M_OFF_Y_MSB      0x41
#define AG_M_OFF_Y_LSB      0x42
#define AG_M_OFF_Z_MSB      0x43
#define AG_M_OFF_Z_LSB      0x44
#define AG_MAX_X_MSB        0x45
#define AG_MAX_X_LSB        0x46
#define AG_MAX_Y_MSB        0x47
#define AG_MAX_Y_LSB        0x48
#define AG_MAX_Z_MSB        0x49
#define AG_MAX_Z_LSB        0x4A
#define AG_MIN_X_MSB        0x4B
#define AG_MIN_X_LSB        0x4C
#define AG_MIN_Y_MSB        0x4D
#define AG_MIN_Y_LSB        0x4E
#define AG_MIN_Z_MSB        0x4F
#define AG_MIN_Z_LSB        0x50
#define AG_TEMP             0x51
#define AG_M_THS_CFG        0x52
#define AG_M_THS_SRC        0x53
#define AG_M_THS_X_MSB      0x54
#define AG_M_THS_X_LSB      0x55
#define AG_M_THS_Y_MSB      0x56
#define AG_M_THS_Y_LSB      0x57
#define AG_M_THS_Z_MSB      0x58
#define AG_M_THS_Z_LSB      0x59
#define AG_M_THS_COUNT      0x5A
#define AG_M_CTRL_REG1      0x5B
#define AG_M_CTRL_REG2      0x5C
#define AG_M_CTRL_REG3      0x5D
#define AG_M_INT_SRC        0x5E
#define AG_A_VECM_CFG       0x5F
#define AG_A_VECM_THS_MSB   0x60
#define AG_A_VECM_THS_LSB   0x61
#define AG_A_VECM_CNT       0x62
#define AG_A_VECM_INITX_MSB 0x63
#define AG_A_VECM_INITX_LSB 0x64
#define AG_A_VECM_INITY_MSB 0x65
#define AG_A_VECM_INITY_LSB 0x66
#define AG_A_VECM_INITZ_MSB 0x67
#define AG_A_VECM_INITZ_LSB 0x68
#define AG_M_VECM_CFG       0x69
#define AG_M_VECM_THS_MSB   0x6A
#define AG_M_VECM_THS_LSB   0x6B
#define AG_M_VECM_CNT       0x6C
#define AG_M_VECM_INITX_MSB 0x6D
#define AG_M_VECM_INITX_LSB 0x6E
#define AG_M_VECM_INITY_MSB 0x6F
#define AG_M_VECM_INITY_LSB 0x70
#define AG_M_VECM_INITZ_MSB 0x71
#define AG_M_VECM_INITZ_LSB 0x72
#define AG_A_FFMT_THS_X_MSB 0x73
#define AG_A_FFMT_THS_X_LSB 0x74
#define AG_A_FFMT_THS_Y_MSB 0x75
#define AG_A_FFMT_THS_Y_LSB 0x76
#define AG_A_FFMT_THS_Z_MSB 0x77
#define AG_A_FFMT_THS_Z_LSB 0x78

uint8_t fxos_register[1];
uint8_t fxos_data[13];

// I2C General Send and receive functions
extern void I2CAccelMagReceive(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pReceiveData, uint8_t ui8NumBytes);
extern void I2CAccelMagSend(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pTransmitData, uint8_t ui8NumBytes);
extern void I2CAccelMagSingleByteSend(uint8_t ui32WorkerAddress, uint8_t byte);

// FXOS8700CQ functions
void AccelMagInit(uint32_t ui32WorkerAddress, tAccelRange tAFSR, tOutputDataRate tODR);
void AccelMagGetData(uint32_t ui32WorkerAddress, tRawData *accelRD , tRawData *magRD);
uint8_t AccelMagWhoAmI(uint32_t ui32WorkerAddress);

#ifdef __cplusplus
}
#endif

#endif