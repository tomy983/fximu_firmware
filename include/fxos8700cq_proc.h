#ifndef FXOS8700CQ_PROC_H_
#define FXOS8700CQ_PROC_H_

#ifdef __cplusplus
extern "C"
{
#endif

void I2CAGReceive(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pReceiveData, uint8_t ui8NumBytes);
void I2CAGSend(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pTransmitData, uint8_t ui8NumBytes);
void I2CAGSingleByteSend(uint8_t ui32WorkerAddress, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif
