#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "fxos8700cq.h"

void AGInit(uint32_t ui32WorkerAddress, tAccelRange tAFSR, tOutputDataRate tODR) {

    uint8_t ui8Register[1];

    // standby
    I2CAGReceive(ui32WorkerAddress, AG_CTRL_REG1, ui8Register, sizeof(ui8Register));
    ui8Register[0] &= ~(0B00000001);
    I2CAGSend(ui32WorkerAddress, AG_CTRL_REG1, ui8Register, sizeof(ui8Register));

    // reset
    I2CAGReceive(ui32WorkerAddress, AG_CTRL_REG2, ui8Register, sizeof(ui8Register));
    ui8Register[0] = 0x40;
    I2CAGSend(ui32WorkerAddress, AG_CTRL_REG1, ui8Register, sizeof(ui8Register));

    // set output data rate
    I2CAGReceive(ui32WorkerAddress, AG_CTRL_REG1, ui8Register, sizeof(ui8Register));
    ui8Register[0] &= ~(0B00111000);
    ui8Register[0] |= (tODR << 3 );
    I2CAGSend(ui32WorkerAddress, AG_CTRL_REG1, ui8Register, sizeof(ui8Register));

    // high resolution
    ui8Register[0] = 0x02;
    I2CAGSend(ui32WorkerAddress, AG_CTRL_REG2, ui8Register, sizeof(ui8Register));

    // interrupt configuration, active low, //push-pull
    // active low
    ui8Register[0] = 0x00; 
    I2CAGSend(ui32WorkerAddress, AG_CTRL_REG3, ui8Register, sizeof(ui8Register));

    // data ready interrupt is routed to pin 2
    ui8Register[0] = 0x00;
    I2CAGSend(ui32WorkerAddress, AG_CTRL_REG5, ui8Register, sizeof(ui8Register));

    // enable data ready interrupt
    ui8Register[0] = 0x01;
    I2CAGSend(ui32WorkerAddress, AG_CTRL_REG4, ui8Register, sizeof(ui8Register));

    // set accel range
    I2CAGReceive(ui32WorkerAddress, AG_XYZ_DATA_CFG, ui8Register, sizeof(ui8Register));
    ui8Register[0] &= ~(0B00000011);
    ui8Register[0] |= tAFSR;
    I2CAGSend(ui32WorkerAddress, AG_XYZ_DATA_CFG, ui8Register, sizeof(ui8Register));

    // hybrid mode, osr = 7
    I2CAGReceive(ui32WorkerAddress, AG_M_CTRL_REG1, ui8Register, sizeof(ui8Register));
    ui8Register[0] &= ~(0B00011111);
    ui8Register[0] |= ACCEL_AND_MAG;
    I2CAGSend(ui32WorkerAddress, AG_M_CTRL_REG1, ui8Register, sizeof(ui8Register));

    // Jump to reg 0x33 after reading 0x06
    ui8Register[0] = 0x20;
    I2CAGSend(ui32WorkerAddress, AG_M_CTRL_REG2, ui8Register, sizeof(ui8Register));

    // active
    I2CAGReceive(ui32WorkerAddress, AG_CTRL_REG1, ui8Register, sizeof(ui8Register));
    ui8Register[0] |= 0B00000001;
    I2CAGSend(ui32WorkerAddress, AG_CTRL_REG1, ui8Register, sizeof(ui8Register));

}

void AGGetData(uint32_t ui32WorkerAddress, tRawData *accelRD , tRawData *magRD) {

    uint8_t ui8Data[13];
    I2CAGReceive(ui32WorkerAddress, AG_STATUS, ui8Data, sizeof(ui8Data));

    // copy the 14 bit accelerometer byte data into 16 bit words
    // notice accel data is 14-bit and left-aligned, so we shift two bit right
    accelRD->x = (int16_t) ((ui8Data[1] << 8) | ui8Data[2]) >> 2;
    accelRD->y = (int16_t) ((ui8Data[3] << 8) | ui8Data[4]) >> 2;
    accelRD->z = (int16_t) ((ui8Data[5] << 8) | ui8Data[6]) >> 2;

    // copy the magnetometer byte data into 16 bit words
    magRD->x = (int16_t)((ui8Data[7]  << 8) | ui8Data[8]);
    magRD->y = (int16_t)((ui8Data[9]  << 8) | ui8Data[10]);
    magRD->z = (int16_t)((ui8Data[11] << 8) | ui8Data[12]);

}
