#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "fxas21002c.h"

void GyroInit(uint32_t ui32WorkerAddress, tGyroRange tGFSR, tOutputDataRate tODR) {

    uint8_t ui8Register[1];

    // standby
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG1, ui8Register, sizeof(ui8Register));
    ui8Register[0] &= ~(0B00000011);
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG1, ui8Register, sizeof(ui8Register));

    // reset
    ui8Register[0] = GYRO_RESET;
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG1, ui8Register, sizeof(ui8Register));

    // set range
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG0, ui8Register, sizeof(ui8Register));
    ui8Register[0] &= ~(0B00000011);
    ui8Register[0] |= tGFSR;
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG0, ui8Register, sizeof(ui8Register));

    // set output data rate
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG1, ui8Register, sizeof(ui8Register));
    ui8Register[0] &= ~(0B00011100);
    ui8Register[0] |= (tODR << 2 );
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG1, ui8Register, sizeof(ui8Register));

    // enable int, active low, open drain
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG2, ui8Register, sizeof(ui8Register));
    ui8Register[0] |= 0B00000101;
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG2, ui8Register, sizeof(ui8Register));

    // set active
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG1, ui8Register, sizeof(ui8Register));
    ui8Register[0] |= 0B00000010;
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG1, ui8Register, sizeof(ui8Register));

}

void GyroGetData(uint32_t ui32WorkerAddress, tRawData *tRD) {

    uint8_t ui8Register[7];
    I2CGyroReceive(ui32WorkerAddress, GYRO_STATUS, ui8Register, sizeof(ui8Register));

    // copy the 16 bit gyroscope byte data into 16 bit words
    tRD->x = (int16_t)((ui8Register[1] << 8) | (ui8Register[2] >> 0));
    tRD->y = (int16_t)((ui8Register[3] << 8) | (ui8Register[4] >> 0));
    tRD->z = (int16_t)((ui8Register[5] << 8) | (ui8Register[6] >> 0));

}
