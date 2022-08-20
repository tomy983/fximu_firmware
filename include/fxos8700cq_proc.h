#ifndef FXOS8700CQ_PROC_H_
#define FXOS8700CQ_PROC_H_

#ifdef __cplusplus
extern "C"
{
#endif

void I2CAccelMagReceive(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pReceiveData, uint8_t ui8NumBytes);
void I2CAccelMagSend(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pTransmitData, uint8_t ui8NumBytes);
void I2CAccelMagSingleByteSend(uint8_t ui32WorkerAddress, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif