#ifndef PARAMETERS_H
#define PARAMETERS_H

#define HW_VERSION_CODE FXIMU2C

#include <stdio.h>
char loginfo_buffer[100];

#define PARAM_SIZE 30
bool parameters[PARAM_SIZE];

int p_calibration_mode;
bool imu_disabled = false;
bool eeprom_init = false;

float p_gain_acc = 0.02;
float p_gain_mag = 0.01;
float p_bias_alpha = 0.25;

float p_kAngularVelocityThreshold = 0.06;
float p_kAccelerationThreshold = 0.25;
float p_kDeltaAngularVelocityThreshold = 0.05;

float mag_offsets[3]            = {0.0, 0.0, 0.0};
float mag_softiron_matrix[3][3] = {{0.0, 0.0, 0.0},
                                   {0.0, 0.0, 0.0},
                                   {0.0, 0.0, 0.0}};

int p_sensor_read_rate = 400;   // valid values are 50, 100, 200, 400
int p_output_rate_divider = 8;  // value values are 2, 4, 8, 16

bool p_adaptive_gain = true;
bool p_bias_estimation = true;

int p_gfsr = 2; // GFSR_500PS
int p_afsr = 1; // AFSR_4G
int p_steady_limit = 32;

int p_world_frame = 0;
int p_use_mag = 1;

char imu_link[16] = "base_imu_link";
char mag_link[16] = "base_mag_link";

char *imu_link_ptr[1] = {imu_link};
char *mag_link_ptr[1] = {mag_link};

uint32_t pui32Data[1];
uint32_t pui32Read[1];

union conv32 {
    uint32_t u32;
    float f32;
};

void write_eeprom_float(uint32_t addr, float data) {
    pui32Data[0] = ((union conv32){.f32 = data}).u32;
    EEPROMProgram(pui32Data, addr, sizeof(pui32Data));
}

float read_eeprom_float(uint32_t addr) {
    EEPROMRead(pui32Read, addr, sizeof(pui32Read));
    return ((union conv32){.u32 = pui32Read[0]}).f32;
}

void write_eeprom_int(uint32_t addr, int data) {
    EEPROMProgram((uint32_t *)&data, addr, sizeof(data));
}

void read_eeprom_int(uint32_t addr, int data) {
    EEPROMRead((uint32_t *)&data, addr, sizeof(data));
}

void write_eeprom_string(uint32_t addr, char **link_ptr) {
    EEPROMProgram((uint32_t *)&link_ptr, addr, sizeof(link_ptr));
}

void read_eeprom_string(uint32_t addr, char **link_ptr) {
    EEPROMRead((uint32_t *)&link_ptr, addr, sizeof(link_ptr));
}

void read_defaults() {

    if(!eeprom_init) { return; }

    p_gain_acc = read_eeprom_float(0x00);
    p_gain_mag = read_eeprom_float(0x04);
    p_bias_alpha = read_eeprom_float(0x08);

    p_kAngularVelocityThreshold = read_eeprom_float(0x0C);
    p_kAccelerationThreshold = read_eeprom_float(0x10);
    p_kDeltaAngularVelocityThreshold = read_eeprom_float(0x14);

    mag_offsets[0] = read_eeprom_float(0x18);
    mag_offsets[1] = read_eeprom_float(0x1C);
    mag_offsets[2] = read_eeprom_float(0x20);

    mag_softiron_matrix[0][0] = read_eeprom_float(0x24);
    mag_softiron_matrix[0][1] = read_eeprom_float(0x28);
    mag_softiron_matrix[0][2] = read_eeprom_float(0x2C);
    mag_softiron_matrix[1][0] = read_eeprom_float(0x30);
    mag_softiron_matrix[1][1] = read_eeprom_float(0x34);
    mag_softiron_matrix[1][2] = read_eeprom_float(0x38);
    mag_softiron_matrix[2][0] = read_eeprom_float(0x3C);
    mag_softiron_matrix[2][1] = read_eeprom_float(0x40);
    mag_softiron_matrix[2][2] = read_eeprom_float(0x44);

    read_eeprom_int(0x48, p_sensor_read_rate);
    read_eeprom_int(0x4C, p_output_rate_divider);
    read_eeprom_int(0x50, p_adaptive_gain);
    read_eeprom_int(0x54, p_bias_estimation);

    read_eeprom_int(0x58, p_gfsr);
    read_eeprom_int(0x5C, p_afsr);
    read_eeprom_int(0x60, p_steady_limit);
    read_eeprom_int(0x64, p_world_frame);
    read_eeprom_int(0x68, p_use_mag);

    read_eeprom_string(0x70, imu_link_ptr);
    read_eeprom_string(0x80, mag_link_ptr);

}

void write_defaults() {

    if(!eeprom_init) { return; }

    write_eeprom_float(0x00, p_gain_acc);
    write_eeprom_float(0x04, p_gain_mag);
    write_eeprom_float(0x08, p_bias_alpha);

    write_eeprom_float(0x0C, p_kAngularVelocityThreshold);
    write_eeprom_float(0x10, p_kAccelerationThreshold);
    write_eeprom_float(0x14, p_kDeltaAngularVelocityThreshold);

    write_eeprom_float(0x18, mag_offsets[0]);
    write_eeprom_float(0x1C, mag_offsets[1]);
    write_eeprom_float(0x20, mag_offsets[2]);

    write_eeprom_float(0x24, mag_softiron_matrix[0][0]);
    write_eeprom_float(0x28, mag_softiron_matrix[0][1]);
    write_eeprom_float(0x2C, mag_softiron_matrix[0][2]);
    write_eeprom_float(0x30, mag_softiron_matrix[1][0]);
    write_eeprom_float(0x34, mag_softiron_matrix[1][1]);
    write_eeprom_float(0x38, mag_softiron_matrix[1][2]);
    write_eeprom_float(0x3C, mag_softiron_matrix[2][0]);
    write_eeprom_float(0x40, mag_softiron_matrix[2][1]);
    write_eeprom_float(0x44, mag_softiron_matrix[2][2]);

    write_eeprom_int(0x48, p_sensor_read_rate);
    write_eeprom_int(0x4C, p_output_rate_divider);
    write_eeprom_int(0x50, p_adaptive_gain ? 0 : 1);
    write_eeprom_int(0x54, p_bias_estimation ? 0 : 1);

    write_eeprom_int(0x58, p_gfsr);
    write_eeprom_int(0x5C, p_afsr);
    write_eeprom_int(0x60, p_steady_limit);
    write_eeprom_int(0x64, p_world_frame);
    write_eeprom_int(0x68, p_use_mag);

    write_eeprom_string(0x70, imu_link_ptr);
    write_eeprom_string(0x80, mag_link_ptr);

}

void spin_once(ros::NodeHandle &nh) {
    nh.spinOnce();
    nh.getHardware()->delay(10);
}

void reset_filter_parameters() {

    p_gain_acc = 0.02;
    p_gain_mag = 0.01;
    p_bias_alpha = 0.25;

    p_kAngularVelocityThreshold = 0.06;
    p_kAccelerationThreshold = 0.25;
    p_kDeltaAngularVelocityThreshold = 0.05;

    mag_offsets[0] = 0.0;
    mag_offsets[1] = 0.0;
    mag_offsets[2] = 0.0;

    mag_softiron_matrix[0][0] = 0.0;
    mag_softiron_matrix[0][1] = 0.0;
    mag_softiron_matrix[0][2] = 0.0;
    mag_softiron_matrix[1][0] = 0.0;
    mag_softiron_matrix[1][1] = 0.0;
    mag_softiron_matrix[1][2] = 0.0;
    mag_softiron_matrix[2][0] = 0.0;
    mag_softiron_matrix[2][1] = 0.0;
    mag_softiron_matrix[2][2] = 0.0;

    imu_disabled = false;

    /*
    p_sensor_read_rate = 400;
    p_output_rate_divider = 8;
    p_adaptive_gain = true;
    p_bias_estimation = true;

    p_gfsr = 2;
    p_afsr = 1;
    p_steady_limit = 32;
    p_world_frame = 0;
    p_use_mag = 1;
    */

}

void print_defaults(ros::NodeHandle &nh) {

    sprintf(loginfo_buffer, "Firmware Revision: 01MARCH2021");
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "sensor_read_rate: %d", p_sensor_read_rate);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "output_rate_divider: %d", p_output_rate_divider);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

    sprintf(loginfo_buffer, "adaptive_gain: %d", p_adaptive_gain);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "bias_estimation: %d", p_bias_estimation);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

    sprintf(loginfo_buffer, "gain_acc: %.3f", p_gain_acc);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "gain_mag: %.3f", p_gain_mag);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "bias_alpha: %.3f", p_bias_alpha);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

    sprintf(loginfo_buffer, "imu_frame_id: %s", *imu_link_ptr);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "mag_frame_id: %s", *mag_link_ptr);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

    sprintf(loginfo_buffer, "GFSR: %d", p_gfsr);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "AFSR: %d", p_afsr);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "steady_limit: %d", p_steady_limit);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

    sprintf(loginfo_buffer, "world_frame: %d", p_world_frame);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "use_mag: %d", p_use_mag);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

    sprintf(loginfo_buffer, "kAngularVelocityThreshold: %.3f", p_kAngularVelocityThreshold);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "kAccelerationThreshold: %.3f", p_kAccelerationThreshold);
    nh.loginfo(loginfo_buffer);

    sprintf(loginfo_buffer, "kDeltaAngularVelocityThreshold: %.3f", p_kDeltaAngularVelocityThreshold);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

    sprintf(loginfo_buffer, "mag_offsets: %.3f, %.3f, %.3f", mag_offsets[0], mag_offsets[1], mag_offsets[2]);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

    sprintf(loginfo_buffer, "soft_iron_matrix[0]: %.3f, %.3f, %.3f", mag_softiron_matrix[0][0], mag_softiron_matrix[0][1], mag_softiron_matrix[0][2]);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

    sprintf(loginfo_buffer, "soft_iron_matrix[1]: %.3f, %.3f, %.3f", mag_softiron_matrix[1][0], mag_softiron_matrix[1][1], mag_softiron_matrix[1][2]);
    nh.loginfo(loginfo_buffer);

    spin_once(nh);

}

void print_status(ros::NodeHandle &nh) {

    if(!eeprom_init) {
        sprintf(loginfo_buffer, "EEPROM Init Failure");
        nh.loginfo(loginfo_buffer);
        spin_once(nh);
    }

    if(imu_disabled) {
        printf(loginfo_buffer, "Parameter Load Failure. IMU DISABLED");
        nh.loginfo(loginfo_buffer);
        spin_once(nh);
    }

}

void handle_parameters(ros::NodeHandle &nh) {

    // set all to false
    for(int i=0; i<PARAM_SIZE; i++) {
        parameters[i] = false;
    }

    p_calibration_mode = 0;

    parameters[0] = nh.getParam("/params/imu/calibration_mode", (int*) &p_calibration_mode, 1, 1000);
    spin_once(nh);

    bool success = true;

    sprintf(loginfo_buffer, "calibration_mode: %d", p_calibration_mode);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    if(p_calibration_mode == 0) {

        // use eeprom defaults
        read_defaults();

        sprintf(loginfo_buffer, "FXIMU Parameters read from EEPROM");
        nh.loginfo(loginfo_buffer);
        spin_once(nh);

    } else if(p_calibration_mode == 1) {

        // calibration mode
        reset_filter_parameters();

    } else if(p_calibration_mode == 2 || p_calibration_mode == 3) {

        parameters[1] = nh.getParam("/params/imu/sensor_read_rate", (int*) &p_sensor_read_rate, 1, 1000);
        parameters[2] = nh.getParam("/params/imu/output_rate_divider", (int*) &p_output_rate_divider, 1, 1000);
        spin_once(nh);

        parameters[3] = nh.getParam("/params/imu/adaptive_gain", (bool*) &p_adaptive_gain, 1, 1000);
        parameters[4] = nh.getParam("/params/imu/bias_estimation", (bool*) &p_bias_estimation, 1, 1000);
        spin_once(nh);

        parameters[5] = nh.getParam("/params/imu/gain_mag", (float*) &p_gain_mag, 1, 1000);
        parameters[6] = nh.getParam("/params/imu/gain_acc", (float*) &p_gain_acc, 1, 1000);
        parameters[7] = nh.getParam("/params/imu/bias_alpha", (float*) &p_bias_alpha, 1, 1000);
        spin_once(nh);

        parameters[8] = nh.getParam("/params/imu/imu_frame_id", imu_link_ptr, 1, 1000);
        parameters[9] = nh.getParam("/params/imu/mag_frame_id", mag_link_ptr, 1, 1000);
        spin_once(nh);

        parameters[10] = nh.getParam("/params/imu/gfsr", (int*) &p_gfsr, 1, 1000);
        parameters[11] = nh.getParam("/params/imu/afsr", (int*) &p_afsr, 1, 1000);
        spin_once(nh);

        parameters[12] = nh.getParam("/params/imu/steady_limit", (int*) &p_steady_limit, 1, 1000);
        spin_once(nh);

        parameters[13] = nh.getParam("/params/imu/world_frame", (int*) &p_world_frame, 1, 1000);
        spin_once(nh);

        parameters[14] = nh.getParam("/params/imu/use_mag", (int*) &p_use_mag, 1, 1000);
        spin_once(nh);

        parameters[15] = nh.getParam("/params/imu/kAngularVelocityThreshold", (float*) &p_kAngularVelocityThreshold, 1, 1000);
        parameters[16] = nh.getParam("/params/imu/kAccelerationThreshold", (float*) &p_kAccelerationThreshold, 1, 1000);
        parameters[17] = nh.getParam("/params/imu/kDeltaAngularVelocityThreshold", (float*) &p_kDeltaAngularVelocityThreshold, 1, 1000);
        spin_once(nh);

        parameters[18] = nh.getParam("/params/imu/mag_offset_x", (float*) &mag_offsets[0], 1, 1000);
        parameters[19] = nh.getParam("/params/imu/mag_offset_y", (float*) &mag_offsets[1], 1, 1000);
        parameters[20] = nh.getParam("/params/imu/mag_offset_z", (float*) &mag_offsets[2], 1, 1000);
        spin_once(nh);

        parameters[21] = nh.getParam("/params/imu/mag_soft_iron_ix", (float*) &mag_softiron_matrix[0][0], 1, 1000);
        parameters[22] = nh.getParam("/params/imu/mag_soft_iron_iy", (float*) &mag_softiron_matrix[0][1], 1, 1000);
        parameters[23] = nh.getParam("/params/imu/mag_soft_iron_iz", (float*) &mag_softiron_matrix[0][2], 1, 1000);
        parameters[24] = nh.getParam("/params/imu/mag_soft_iron_jx", (float*) &mag_softiron_matrix[1][0], 1, 1000);
        parameters[25] = nh.getParam("/params/imu/mag_soft_iron_jy", (float*) &mag_softiron_matrix[1][1], 1, 1000);
        parameters[26] = nh.getParam("/params/imu/mag_soft_iron_jz", (float*) &mag_softiron_matrix[1][2], 1, 1000);
        parameters[27] = nh.getParam("/params/imu/mag_soft_iron_kx", (float*) &mag_softiron_matrix[2][0], 1, 1000);
        parameters[28] = nh.getParam("/params/imu/mag_soft_iron_ky", (float*) &mag_softiron_matrix[2][1], 1, 1000);
        parameters[29] = nh.getParam("/params/imu/mag_soft_iron_kz", (float*) &mag_softiron_matrix[2][2], 1, 1000);
        spin_once(nh);

        for(int i=0; i<PARAM_SIZE; i++) {
            if(!parameters[i]) {
                sprintf(loginfo_buffer, "FXIMU Parameter for Index=%d failed", i);
                nh.loginfo(loginfo_buffer);
                success = false;
            }
        }

        if(success && p_calibration_mode == 2) {
            sprintf(loginfo_buffer, "FXIMU Parameters read from ROS");
            nh.loginfo(loginfo_buffer);
        }

        if(success && p_calibration_mode == 3) {
            write_defaults();
            sprintf(loginfo_buffer, "FXIMU Parameters written to EEPROM");
            nh.loginfo(loginfo_buffer);
        }

        if(!success) { imu_disabled = true; }

        spin_once(nh);

    }

}

#endif