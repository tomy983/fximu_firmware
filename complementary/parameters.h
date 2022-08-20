#ifndef PARAMETERS_H
#define PARAMETERS_H

#define HW_VERSION_CODE FXIMU2C
#define PARAM_SIZE 33

#include <stdio.h>

char loginfo_buffer[100];
bool parameters[PARAM_SIZE];

int p_calibration_mode = 0;
bool eeprom_init = false;

typedef enum {
    FATAL,
    SUCCESS,
    RETRY,
    WRITE_ERROR
} tParameterResult;

// PARAMETERS BEGIN

float p_gain_acc = DEFAULT_GAIN_ACC;
float p_gain_mag = DEFAULT_GAIN_MAG;
float p_bias_alpha = DEFAULT_BIAS_ALPHA;

float p_kAngularVelocityThreshold = DEFAULT_ANGULAR_VELOCITY_THRESHOLD;
float p_kAccelerationThreshold = DEFAULT_ACCELERATION_THRESHOLD;
float p_kSteadyGravity = SENSORS_GRAVITY_EARTH;
float p_kDeltaAngularVelocityThreshold = DEFAULT_DELTA_ANGULAR_VELOCITY_THRESHOLD;

float mag_offsets[3]            =  { 0.0, 0.0, 0.0 };
float mag_softiron_matrix[3][3] = {{ 0.0, 0.0, 0.0 },
                                   { 0.0, 0.0, 0.0 },
                                   { 0.0, 0.0, 0.0 }};

int p_sensor_read_rate = 100;   // valid values are 50, 100, 200, 400
int p_output_rate_divider = 2;  // value values are 2, 4, 8, 16

bool p_adaptive_gain = true;
bool p_bias_estimation = true;

int p_gfsr = 0; // GFSR_2000PS
int p_afsr = 2; // AFSR_8G

int p_world_frame = 0;

bool p_USE_MAG = true;

char imu_link[16] = "imu_link";
char mag_link[16] = "imu_link";

char *imu_link_ptr[1] = {imu_link};
char *mag_link_ptr[1] = {mag_link};

float p_GYRO_BIAS[3] = { 0.0, 0.0, 0.0 };

float p_angular_velocity_covariance[9] = { 0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02 };        // not implemented yed
float p_linear_acceleration_covariance[9] = { 0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04 };     // not implemented yed
float p_orientation_covariance[9] = { 0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025 };       // not implemented yed

// PARAMETERS END

uint32_t pui32Data[1];
uint32_t pui32Read[1];

union conv32 {
    uint32_t u32;
    float f32;
};

void write_eeprom_int(uint32_t addr, int data) {
    EEPROMProgram((uint32_t *)&data, addr, sizeof(data));
}

void write_eeprom_float(uint32_t addr, float data) {
    pui32Data[0] = ((union conv32){.f32 = data}).u32;
    EEPROMProgram(pui32Data, addr, sizeof(pui32Data));
}

void write_eeprom_string(uint32_t addr, char **link_ptr) {
    EEPROMProgram((uint32_t *)&link_ptr, addr, sizeof(link_ptr));
}

void read_eeprom_int(uint32_t addr, int data) {
    EEPROMRead((uint32_t *)&data, addr, sizeof(data));
}

float read_eeprom_float(uint32_t addr) {
    EEPROMRead(pui32Read, addr, sizeof(pui32Read));
    return ((union conv32){.u32 = pui32Read[0]}).f32;
}

void read_eeprom_string(uint32_t addr, char **link_ptr) {
    EEPROMRead((uint32_t *)&link_ptr, addr, sizeof(link_ptr));
}

bool read_defaults() {

    if(!eeprom_init) { return false; }

    p_gain_acc = read_eeprom_float(0x00);
    p_gain_mag = read_eeprom_float(0x04);
    p_bias_alpha = read_eeprom_float(0x08);

    p_kAngularVelocityThreshold = read_eeprom_float(0x0C);
    p_kAccelerationThreshold = read_eeprom_float(0x10);
    p_kSteadyGravity = read_eeprom_float(0x14);
    p_kDeltaAngularVelocityThreshold = read_eeprom_float(0x18);

    mag_offsets[0] = read_eeprom_float(0x1C);
    mag_offsets[1] = read_eeprom_float(0x20);
    mag_offsets[2] = read_eeprom_float(0x24);

    mag_softiron_matrix[0][0] = read_eeprom_float(0x28);
    mag_softiron_matrix[0][1] = read_eeprom_float(0x2C);
    mag_softiron_matrix[0][2] = read_eeprom_float(0x30);
    mag_softiron_matrix[1][0] = read_eeprom_float(0x34);
    mag_softiron_matrix[1][1] = read_eeprom_float(0x38);
    mag_softiron_matrix[1][2] = read_eeprom_float(0x3C);
    mag_softiron_matrix[2][0] = read_eeprom_float(0x40);
    mag_softiron_matrix[2][1] = read_eeprom_float(0x44);
    mag_softiron_matrix[2][2] = read_eeprom_float(0x48);

    read_eeprom_int(0x4C, p_sensor_read_rate);
    read_eeprom_int(0x50, p_output_rate_divider);

    read_eeprom_int(0x54, p_adaptive_gain); // bool
    read_eeprom_int(0x58, p_bias_estimation); // bool

    read_eeprom_int(0x5C, p_gfsr);
    read_eeprom_int(0x60, p_afsr);

    read_eeprom_int(0x64, p_world_frame);

    // 0x6C empty

    read_eeprom_string(0x70, imu_link_ptr);
    read_eeprom_string(0x80, mag_link_ptr);

    p_GYRO_BIAS[0] = read_eeprom_float(0x90);
    p_GYRO_BIAS[1] = read_eeprom_float(0x94);
    p_GYRO_BIAS[2] = read_eeprom_float(0x98);

    return true;

}

bool write_defaults() {

    if(!eeprom_init) { return false; }

    write_eeprom_float(0x00, p_gain_acc);
    write_eeprom_float(0x04, p_gain_mag);
    write_eeprom_float(0x08, p_bias_alpha);

    write_eeprom_float(0x0C, p_kAngularVelocityThreshold);
    write_eeprom_float(0x10, p_kAccelerationThreshold);
    write_eeprom_float(0x14, p_kSteadyGravity);
    write_eeprom_float(0x18, p_kDeltaAngularVelocityThreshold);

    write_eeprom_float(0x1C, mag_offsets[0]);
    write_eeprom_float(0x20, mag_offsets[1]);
    write_eeprom_float(0x24, mag_offsets[2]);

    write_eeprom_float(0x28, mag_softiron_matrix[0][0]);
    write_eeprom_float(0x2C, mag_softiron_matrix[0][1]);
    write_eeprom_float(0x30, mag_softiron_matrix[0][2]);
    write_eeprom_float(0x34, mag_softiron_matrix[1][0]);
    write_eeprom_float(0x38, mag_softiron_matrix[1][1]);
    write_eeprom_float(0x3C, mag_softiron_matrix[1][2]);
    write_eeprom_float(0x40, mag_softiron_matrix[2][0]);
    write_eeprom_float(0x44, mag_softiron_matrix[2][1]);
    write_eeprom_float(0x48, mag_softiron_matrix[2][2]);

    write_eeprom_int(0x4C, p_sensor_read_rate);
    write_eeprom_int(0x50, p_output_rate_divider);

    write_eeprom_int(0x54, p_adaptive_gain ? 0 : 1); // bool
    write_eeprom_int(0x58, p_bias_estimation ? 0 : 1); // bool

    write_eeprom_int(0x5C, p_gfsr);
    write_eeprom_int(0x60, p_afsr);

    write_eeprom_int(0x64, p_world_frame);

    write_eeprom_int(0x68, p_USE_MAG ? 0 : 1); // bool

    // 0x6C empty

    write_eeprom_string(0x70, imu_link_ptr);
    write_eeprom_string(0x80, mag_link_ptr);

    write_eeprom_float(0x90, p_GYRO_BIAS[0]);
    write_eeprom_float(0x94, p_GYRO_BIAS[1]);
    write_eeprom_float(0x98, p_GYRO_BIAS[2]);

    return true;

}

void reset_filter_parameters() {

    // this method is called before calibration mode

    p_gain_acc = DEFAULT_GAIN_ACC;
    p_gain_mag = DEFAULT_GAIN_MAG;
    p_bias_alpha = DEFAULT_BIAS_ALPHA;

    p_kAngularVelocityThreshold = DEFAULT_ANGULAR_VELOCITY_THRESHOLD;
    p_kAccelerationThreshold = DEFAULT_ACCELERATION_THRESHOLD;
    p_kSteadyGravity = SENSORS_GRAVITY_EARTH;
    p_kDeltaAngularVelocityThreshold = DEFAULT_DELTA_ANGULAR_VELOCITY_THRESHOLD;

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

    p_GYRO_BIAS[0] = 0.0;
    p_GYRO_BIAS[1] = 0.0;
    p_GYRO_BIAS[2] = 0.0;

}

void spin_once(ros::NodeHandle &nh) {
    nh.spinOnce();
    nh.getHardware()->delay(10);
}

const char *getAFSR(int afsr) {
    switch(afsr) {
        case 0:
            return "AFSR_2G";
            break;
        case 1:
            return "AFSR_4G";
            break;
        case 2:
            return "AFSR_8G";
            break;
    }
}

const char *getGFSR(int gfsr) {
    switch(gfsr) {
        case 0:
            return "GFSR_2000PS";
        case 1:
            return "GFSR_1000PS";
        case 2:
            return "GFSR_500PS";
        case 3:
            return "GFSR_250PS";
    }
}

const char *getWorldFrame(int world_frame) {
    switch(world_frame) {
        case 0:
            return "NWU";
        case 1:
            return "ENU";
    }
}

void print_parameters(ros::NodeHandle &nh) {

    sprintf(loginfo_buffer, "firmware: 2022A");
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "%s", getGFSR(p_gfsr));
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "%s", getAFSR(p_afsr));
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "gain_acc: %.3f", p_gain_acc);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "gain_mag: %.3f", p_gain_mag);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "bias_alpha: %.3f", p_bias_alpha);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "sensor_read_rate: %d", p_sensor_read_rate);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "output_rate_divider: %d", p_output_rate_divider);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "adaptive_gain: %s", p_adaptive_gain ? "true" : "false");
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "bias_estimation: %s", p_bias_estimation ? "true" : "false");
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "use_mag: %s", p_USE_MAG  ? "true" : "false");
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "world_frame: %s", getWorldFrame(p_world_frame));
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "kAngularVelocityThreshold: %.3f", p_kAngularVelocityThreshold);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "kAccelerationThreshold: %.3f", p_kAccelerationThreshold);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "kSteadyGravity: %.6f", p_kSteadyGravity);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "kDeltaAngularVelocityThreshold: %.3f", p_kDeltaAngularVelocityThreshold);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "mag_offsets: %.2f, %.2f, %.2f", mag_offsets[0], mag_offsets[1], mag_offsets[2]);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "soft_iron_matrix[0]: %.3f, %.3f, %.3f", mag_softiron_matrix[0][0], mag_softiron_matrix[0][1], mag_softiron_matrix[0][2]);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "soft_iron_matrix[1]: %.3f, %.3f, %.3f", mag_softiron_matrix[1][0], mag_softiron_matrix[1][1], mag_softiron_matrix[1][2]);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "soft_iron_matrix[2]: %.3f, %.3f, %.3f", mag_softiron_matrix[2][0], mag_softiron_matrix[2][1], mag_softiron_matrix[2][2]);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    // TODO: DOC: mention the 16 char limit on frame_id's in the documentation
    sprintf(loginfo_buffer, "imu_frame_id: %s", *imu_link_ptr);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);
    sprintf(loginfo_buffer, "mag_frame_id: %s", *mag_link_ptr);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    sprintf(loginfo_buffer, "default gyro_bias: %.3f, %.3f, %.3f", p_GYRO_BIAS[0], p_GYRO_BIAS[1], p_GYRO_BIAS[2]);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

}

tParameterResult handle_parameters(ros::NodeHandle &nh) {

    tParameterResult handle_parameters_result = SUCCESS;

    // set all to false
    for(int i=0; i<PARAM_SIZE; i++) {
        parameters[i] = false;
    }

    // do not remove, required for soft-reset
    p_calibration_mode = 0;

    // get calibration mode
    parameters[0] = nh.getParam("/params/imu/calibration_mode", (int*) &p_calibration_mode, 1, 1000);
    spin_once(nh);

    // if calibration mode is not found, set it to 0
    if(!parameters[0]) { p_calibration_mode = 0; }

    sprintf(loginfo_buffer, "calibration_mode: %d", p_calibration_mode);
    nh.loginfo(loginfo_buffer);
    spin_once(nh);

    bool eeprom_read = false;
    bool params_read = true;

    switch(p_calibration_mode) {

        case 0:
            // read defaults from eeprom
            eeprom_read = read_defaults();
            spin_once(nh);
            break;

        case 1:

            // calibration mode
            reset_filter_parameters();

            // get afsr
            nh.getParam("/params/imu/afsr", (int*) &p_afsr, 1, 1000);
            spin_once(nh);

            // get gfsr
            nh.getParam("/params/imu/gfsr", (int*) &p_gfsr, 1, 1000);
            spin_once(nh);

            // get sensor read rate
            nh.getParam("/params/imu/sensor_read_rate", (int*) &p_sensor_read_rate, 1, 1000);
            spin_once(nh);

            // get output rate divider
            nh.getParam("/params/imu/output_rate_divider", (int*) &p_output_rate_divider, 1, 1000);
            spin_once(nh);

            break;

        default:

            // notice we dont have a case 2 or 3 here since both reads parameters

            parameters[1] = nh.getParam("/params/imu/gain_acc", (float*) &p_gain_acc, 1, 1000);
            spin_once(nh);
            parameters[2] = nh.getParam("/params/imu/gain_mag", (float*) &p_gain_mag, 1, 1000);
            spin_once(nh);
            parameters[3] = nh.getParam("/params/imu/bias_alpha", (float*) &p_bias_alpha, 1, 1000);
            spin_once(nh);

            parameters[4] = nh.getParam("/params/imu/kAngularVelocityThreshold", (float*) &p_kAngularVelocityThreshold, 1, 1000);
            spin_once(nh);
            parameters[5] = nh.getParam("/params/imu/kAccelerationThreshold", (float*) &p_kAccelerationThreshold, 1, 1000);
            spin_once(nh);
            parameters[6] = nh.getParam("/params/imu/kSteadyGravity", (float*) &p_kSteadyGravity, 1, 1000);
            spin_once(nh);
            parameters[7] = nh.getParam("/params/imu/kDeltaAngularVelocityThreshold", (float*) &p_kDeltaAngularVelocityThreshold, 1, 1000);
            spin_once(nh);

            parameters[8] = nh.getParam("/params/imu/mag_offset_x", (float*) &mag_offsets[0], 1, 1000);
            spin_once(nh);
            parameters[9] = nh.getParam("/params/imu/mag_offset_y", (float*) &mag_offsets[1], 1, 1000);
            spin_once(nh);
            parameters[10] = nh.getParam("/params/imu/mag_offset_z", (float*) &mag_offsets[2], 1, 1000);
            spin_once(nh);

            parameters[11] = nh.getParam("/params/imu/mag_soft_iron_ix", (float*) &mag_softiron_matrix[0][0], 1, 1000);
            spin_once(nh);
            parameters[12] = nh.getParam("/params/imu/mag_soft_iron_iy", (float*) &mag_softiron_matrix[0][1], 1, 1000);
            spin_once(nh);
            parameters[13] = nh.getParam("/params/imu/mag_soft_iron_iz", (float*) &mag_softiron_matrix[0][2], 1, 1000);
            spin_once(nh);
            parameters[14] = nh.getParam("/params/imu/mag_soft_iron_jx", (float*) &mag_softiron_matrix[1][0], 1, 1000);
            spin_once(nh);
            parameters[15] = nh.getParam("/params/imu/mag_soft_iron_jy", (float*) &mag_softiron_matrix[1][1], 1, 1000);
            spin_once(nh);
            parameters[16] = nh.getParam("/params/imu/mag_soft_iron_jz", (float*) &mag_softiron_matrix[1][2], 1, 1000);
            spin_once(nh);
            parameters[17] = nh.getParam("/params/imu/mag_soft_iron_kx", (float*) &mag_softiron_matrix[2][0], 1, 1000);
            spin_once(nh);
            parameters[18] = nh.getParam("/params/imu/mag_soft_iron_ky", (float*) &mag_softiron_matrix[2][1], 1, 1000);
            spin_once(nh);
            parameters[19] = nh.getParam("/params/imu/mag_soft_iron_kz", (float*) &mag_softiron_matrix[2][2], 1, 1000);
            spin_once(nh);

            parameters[20] = nh.getParam("/params/imu/sensor_read_rate", (int*) &p_sensor_read_rate, 1, 1000);
            spin_once(nh);
            parameters[21] = nh.getParam("/params/imu/output_rate_divider", (int*) &p_output_rate_divider, 1, 1000);
            spin_once(nh);

            parameters[22] = nh.getParam("/params/imu/adaptive_gain", (bool*) &p_adaptive_gain, 1, 1000);
            spin_once(nh);
            parameters[23] = nh.getParam("/params/imu/bias_estimation", (bool*) &p_bias_estimation, 1, 1000);
            spin_once(nh);

            parameters[24] = nh.getParam("/params/imu/gfsr", (int*) &p_gfsr, 1, 1000);
            spin_once(nh);
            parameters[25] = nh.getParam("/params/imu/afsr", (int*) &p_afsr, 1, 1000);
            spin_once(nh);

            parameters[26] = nh.getParam("/params/imu/world_frame", (int*) &p_world_frame, 1, 1000);
            spin_once(nh);

            parameters[27] = nh.getParam("/params/imu/use_mag", (bool*) &p_USE_MAG, 1, 1000);
            spin_once(nh);

            parameters[28] = nh.getParam("/params/imu/imu_frame_id", imu_link_ptr, 1, 1000);
            spin_once(nh);
            parameters[29] = nh.getParam("/params/imu/mag_frame_id", mag_link_ptr, 1, 1000);
            spin_once(nh);

            parameters[30] = nh.getParam("/params/imu/gyro_bias_x", (float*) &p_GYRO_BIAS[0]);
            spin_once(nh);
            parameters[31] = nh.getParam("/params/imu/gyro_bias_y", (float*) &p_GYRO_BIAS[1]);
            spin_once(nh);
            parameters[32] = nh.getParam("/params/imu/gyro_bias_z", (float*) &p_GYRO_BIAS[2]);
            spin_once(nh);

            for(int i=0; i<PARAM_SIZE; i++) {
                if(!parameters[i]) {
                    sprintf(loginfo_buffer, "FXIMU Parameter for index=%d failed", i);
                    nh.loginfo(loginfo_buffer);
                    params_read = false;
                    spin_once(nh);
                }
            }
            break;
    }

    switch(p_calibration_mode) {
        case 0:
            // read paramse from eeprom
            if(eeprom_read) {
                sprintf(loginfo_buffer, "parameters read from eeprom");
                handle_parameters_result = SUCCESS;
            } else {
                sprintf(loginfo_buffer, "eeprom read error. imu disabled");
                handle_parameters_result = FATAL;
            }
            nh.loginfo(loginfo_buffer);
            spin_once(nh);
            break;
        case 1:
            // calibration mode
            handle_parameters_result = SUCCESS;
            break;
        case 2:
            // read params from rosparam server
            if(params_read) {
                sprintf(loginfo_buffer, "parameters read from rosparam");
                handle_parameters_result = SUCCESS;
            } else {
                sprintf(loginfo_buffer, "parameter read error");
                handle_parameters_result = RETRY;
            }
            nh.loginfo(loginfo_buffer);
            spin_once(nh);
            break;
        case 3:
            // read params from rosparam server and write to eeprom
            if(params_read) {
                sprintf(loginfo_buffer, "parameters read from rosparam");
                 if(write_defaults()) {
                    sprintf(loginfo_buffer, "parameters written to eeprom");
                    handle_parameters_result = SUCCESS;
                 } else {
                    sprintf(loginfo_buffer, "write to eeprom failed");
                    handle_parameters_result = WRITE_ERROR;
                }
            } else {
                sprintf(loginfo_buffer, "parameter read error");
                handle_parameters_result = RETRY;
            }
            nh.loginfo(loginfo_buffer);
            spin_once(nh);
            break;
        default:
            handle_parameters_result = FATAL;
            break;
    }

    return handle_parameters_result;

}

#endif