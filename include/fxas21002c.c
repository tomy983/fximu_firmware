#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "fxas21002c.h"

void GyroInit(uint32_t ui32WorkerAddress, tGyroRange tGFSR, tOutputDataRate tODR) {

    fxas_register[0] = 0;

    // standby
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG1, fxas_register, sizeof(fxas_register));
    fxas_register[0] &= ~(0B00000011);
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG1, fxas_register, sizeof(fxas_register));

    // reset
    fxas_register[0] = GYRO_RESET;
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG1, fxas_register, sizeof(fxas_register));

    // set range
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG0, fxas_register, sizeof(fxas_register));
    fxas_register[0] &= ~(0B00000011);
    fxas_register[0] |= tGFSR;

    // 7:6  | LPF BW
    // 5    | SPI
    // 4:3  | HPF
    // 2    | HPF_EN
    // 1:0  | RANGE

    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG0, fxas_register, sizeof(fxas_register));

    // set output data rate
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG1, fxas_register, sizeof(fxas_register));
    fxas_register[0] &= ~(0B00011100);
    fxas_register[0] |= (tODR << 2 );
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG1, fxas_register, sizeof(fxas_register));

    // enable int, active low, open drain
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG2, fxas_register, sizeof(fxas_register));
    fxas_register[0] |= 0B00000101;
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG2, fxas_register, sizeof(fxas_register));

    // set active
    I2CGyroReceive(ui32WorkerAddress, GYRO_CTRL_REG1, fxas_register, sizeof(fxas_register));
    fxas_register[0] |= 0B00000010;
    I2CGyroSend(ui32WorkerAddress, GYRO_CTRL_REG1, fxas_register, sizeof(fxas_register));

}

void GyroGetData(uint32_t ui32WorkerAddress, tRawData *tRD) {

    I2CGyroReceive(ui32WorkerAddress, GYRO_STATUS, fxas_data, sizeof(fxas_data));

    // copy the 16 bit gyroscope byte data into 16 bit words
    tRD->x = (int16_t)((fxas_data[1] << 8) | (fxas_data[2] >> 0));
    tRD->y = (int16_t)((fxas_data[3] << 8) | (fxas_data[4] >> 0));
    tRD->z = (int16_t)((fxas_data[5] << 8) | (fxas_data[6] >> 0));

}

uint8_t GyroWhoAmI(uint32_t ui32WorkerAddress) {
    // WHO_AM_I should return 0xD7
    fxas_register[0] = 0;
    I2CGyroReceive(ui32WorkerAddress, GYRO_WHO_AM_I, fxas_register, sizeof(fxas_register));
    return fxas_register[0];
}