#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "fxos8700cq.h"

void AccelMagInit(uint32_t ui32WorkerAddress, tAccelRange tAFSR, tOutputDataRate tODR) {

    fxos_register[0] = 0;

    // standby
    I2CAccelMagReceive(ui32WorkerAddress, AG_CTRL_REG1, fxos_register, sizeof(fxos_register));
    fxos_register[0] &= ~(0B00000001);
    I2CAccelMagSend(ui32WorkerAddress, AG_CTRL_REG1, fxos_register, sizeof(fxos_register));

    // reset
    I2CAccelMagReceive(ui32WorkerAddress, AG_CTRL_REG2, fxos_register, sizeof(fxos_register));
    fxos_register[0] = 0x40;
    I2CAccelMagSend(ui32WorkerAddress, AG_CTRL_REG1, fxos_register, sizeof(fxos_register));

    // set output data rate
    I2CAccelMagReceive(ui32WorkerAddress, AG_CTRL_REG1, fxos_register, sizeof(fxos_register));
    fxos_register[0] &= ~(0B00111000);
    fxos_register[0] |= (tODR << 3 );
    I2CAccelMagSend(ui32WorkerAddress, AG_CTRL_REG1, fxos_register, sizeof(fxos_register));

    // high resolution
    fxos_register[0] = 0x02;
    I2CAccelMagSend(ui32WorkerAddress, AG_CTRL_REG2, fxos_register, sizeof(fxos_register));

    // interrupt configuration, active low, open drain
    // active low
    fxos_register[0] = 0x01;
    I2CAccelMagSend(ui32WorkerAddress, AG_CTRL_REG3, fxos_register, sizeof(fxos_register));

    // data ready interrupt is routed to pin 2
    fxos_register[0] = 0x00;
    I2CAccelMagSend(ui32WorkerAddress, AG_CTRL_REG5, fxos_register, sizeof(fxos_register));

    // enable data ready interrupt
    fxos_register[0] = 0x01;
    I2CAccelMagSend(ui32WorkerAddress, AG_CTRL_REG4, fxos_register, sizeof(fxos_register));

    // set accel range
    I2CAccelMagReceive(ui32WorkerAddress, AG_XYZ_DATA_CFG, fxos_register, sizeof(fxos_register));
    fxos_register[0] &= ~(0B00000011);
    fxos_register[0] |= tAFSR;
    I2CAccelMagSend(ui32WorkerAddress, AG_XYZ_DATA_CFG, fxos_register, sizeof(fxos_register));

    // XYZ_DATA_CFG
    // 4: hpf_out

    // HP_FILTER_CUTOFF
    // 5 : pulse_hpf_byp
    // 4 : pulse_lpf_en
    // 1:0 : hpf_cutoff sel

    // hybrid mode, osr = 7
    I2CAccelMagReceive(ui32WorkerAddress, AG_M_CTRL_REG1, fxos_register, sizeof(fxos_register));
    fxos_register[0] &= ~(0B00011111);
    fxos_register[0] |= ACCEL_AND_MAG;
    I2CAccelMagSend(ui32WorkerAddress, AG_M_CTRL_REG1, fxos_register, sizeof(fxos_register));

    // Jump to reg 0x33 after reading 0x06
    fxos_register[0] = 0x20;
    I2CAccelMagSend(ui32WorkerAddress, AG_M_CTRL_REG2, fxos_register, sizeof(fxos_register));

    // active
    I2CAccelMagReceive(ui32WorkerAddress, AG_CTRL_REG1, fxos_register, sizeof(fxos_register));
    fxos_register[0] |= 0B00000001;
    I2CAccelMagSend(ui32WorkerAddress, AG_CTRL_REG1, fxos_register, sizeof(fxos_register));

}

void AccelMagGetData(uint32_t ui32WorkerAddress, tRawData *accelRD , tRawData *magRD) {

    I2CAccelMagReceive(ui32WorkerAddress, AG_STATUS, fxos_data, sizeof(fxos_data));

    // copy the 14 bit accelerometer byte data into 16 bit words
    // notice accel data is 14-bit and left-aligned, so we shift two bit right
    accelRD->x = (int16_t) ((fxos_data[1] << 8) | fxos_data[2]) >> 2;
    accelRD->y = (int16_t) ((fxos_data[3] << 8) | fxos_data[4]) >> 2;
    accelRD->z = (int16_t) ((fxos_data[5] << 8) | fxos_data[6]) >> 2;

    // copy the magnetometer byte data into 16 bit words
    magRD->x = (int16_t)((fxos_data[7]  << 8) | fxos_data[8]);
    magRD->y = (int16_t)((fxos_data[9]  << 8) | fxos_data[10]);
    magRD->z = (int16_t)((fxos_data[11] << 8) | fxos_data[12]);

}

uint8_t AccelMagWhoAmI(uint32_t ui32WorkerAddress) {
    // WHO_AM_I should return 0xC7
    fxos_register[0] = 0;
    I2CGyroReceive(ui32WorkerAddress, AG_WHO_AM_I, fxos_register, sizeof(fxos_register));
    return fxos_register[0];
}