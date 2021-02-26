#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Int16MultiArray.h>

#include "fximu_complementary.h"
#include "parameters.h"

// TODO: research LPF and HPF on both gyro and accel

#ifndef HW_VERSION_CODE
  #error PLEASE SELECT HW_VERSION_CODE in parameters.h
#endif

int main(void) {

    // initialize tiva-c @ 80mhz
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // init ports, hardware reset of sensor.
    init_system();

    // init i2c
    init_I2C2();

    // accelmag interrupt init
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2);

    #if HW_VERSION_CODE == FXIMU2C
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    #endif
    #if HW_VERSION_CODE == FXIMU2B
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    #endif

    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOIntRegister(GPIO_PORTE_BASE, accelmag_isr);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_2);

    // gyro interrupt pin setup
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);

    #if HW_VERSION_CODE == FXIMU2C
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    #endif
    #if HW_VERSION_CODE == FXIMU2B
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    #endif

    GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOIntRegister(GPIO_PORTC_BASE, gyro_isr);
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);

    // set interrupt priorities
    IntPrioritySet(INT_GPIOA, 0x00); // accelmag
    IntPrioritySet(INT_GPIOC, 0x01); // gyro

    // timer interrupt initialization
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    IntMasterEnable();
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq / p_sensor_read_rate);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // init sensors
    init_sensors();

    // init node
    nh.initNode();

    // advertise publishers
    nh.advertise(pub_imu_msg);
    nh.advertise(pub_mag_msg);
    nh.advertise(pub_array);

    // Wait for connection to establish
    while(!nh.connected()) {
      nh.spinOnce();
      nh.getHardware()->delay(10);
    }

    bool nh_prev_state = false;
    bool nh_connected = false;

    float gx = 0.0F, gy = 0.0F, gz = 0.0F;
    float ax = 0.0F, ay = 0.0F, az = 0.0F;
    float mx = 0.0F, my = 0.0F, mz = 0.0F;
    float q0, q1, q2, q3;

    while(1) {

        nh_connected = nh.connected();

        if(nh_connected && !nh_prev_state) {

            nh_prev_state = true;

            // handle parameters
            handle_parameters(nh);
            print_defaults(nh);

            // reset sequences
            read_sequence = 1;
            pub_sequence = 1;

            // re-set timer, with p_sensor_read_rate from handle_parameters
            TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq / p_sensor_read_rate);

            if(p_calibration_mode != 1) {

                filter_.setDoBiasEstimation(p_bias_estimation);
                filter_.setDoAdaptiveGain(p_adaptive_gain);

                if(!filter_.setGainAcc(p_gain_acc)) {
                    sprintf(loginfo_buffer, "set gain acc failed");
                    nh.loginfo(loginfo_buffer);
                }

                if(!filter_.setGainMag(p_gain_mag)) {
                    sprintf(loginfo_buffer, "set gain mag failed");
                    nh.loginfo(loginfo_buffer);
                }

                if(!filter_.setBiasAlpha(p_bias_alpha)) {
                    sprintf(loginfo_buffer, "set gain bias alpha failed");
                    nh.loginfo(loginfo_buffer);
                }

                if(!filter_.setKAngularVelocityThreshold(p_kAngularVelocityThreshold)) {
                    sprintf(loginfo_buffer, "set kAngularVelocityThresholdfailed");
                    nh.loginfo(loginfo_buffer);
                }

                if(!filter_.setKAccelerationThreshold(p_kAccelerationThreshold)) {
                    sprintf(loginfo_buffer, "set kAccelerationThreshold failed");
                    nh.loginfo(loginfo_buffer);
                }

                if(!filter_.setKDeltaAngularVelocityThreshold(p_kDeltaAngularVelocityThreshold)) {
                    sprintf(loginfo_buffer, "set kDeltaAngularVelocityThreshold failed");
                    nh.loginfo(loginfo_buffer);
                }

                if(!filter_.setSteadyLimit(p_steady_limit)) {
                    sprintf(loginfo_buffer, "set steady limit failed");
                    nh.loginfo(loginfo_buffer);
                }

            } else {
                array.data_length = 9;
            }

            // re-init sensors
            init_sensors();

            nh.spinOnce();
            nh.getHardware()->delay(10);
            continue;
        }

        if(!nh_connected && nh_prev_state) {
            nh_prev_state = false;
            MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);
            nh.spinOnce();
            nh.getHardware()->delay(10);
            continue;
        }

        if(data_ready) {

            currentTime = nh.now();

            if(!initialized_filter_ & p_calibration_mode != 1) {
                markedTime = currentTime;
                initialized_filter_ = true;
                continue;
            }

            double dt = currentTime.toSec() - markedTime.toSec();
            markedTime = currentTime;

            // TODO: magnetic field strength. research this.
            // TODO: string check base_imu_link, etc.

            if(p_calibration_mode != 1) {

                // store gyro values
                gx = (float) (gyroRD.x * _GYRO_SENSITIVITY * SENSORS_DPS_TO_RADS);
                gy = (float) (gyroRD.y * _GYRO_SENSITIVITY * SENSORS_DPS_TO_RADS);
                gz = (float) (gyroRD.z * _GYRO_SENSITIVITY * SENSORS_DPS_TO_RADS);

                // store accel values
                ax = (float) (accelRD.x * _ACCEL_SENSITIVITY * SENSORS_GRAVITY_EARTH);
                ay = (float) (accelRD.y * _ACCEL_SENSITIVITY * SENSORS_GRAVITY_EARTH);
                az = (float) (accelRD.z * _ACCEL_SENSITIVITY * SENSORS_GRAVITY_EARTH);

                // process mag values
                float x = (float) (magRD.x * MAG_UT_LSB);
                float y = (float) (magRD.y * MAG_UT_LSB);
                float z = (float) (magRD.z * MAG_UT_LSB);

                // apply mag offset compensation (base values in uTesla)
                x = x - mag_offsets[0];
                y = y - mag_offsets[1];
                z = z - mag_offsets[2];

                // apply mag soft iron error compensation
                mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
                my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
                mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

                // update the filter
                if(p_use_mag==0) {
                    filter_.update(ax, ay, az, gx, gy, gz, dt);
                } else {
                    filter_.update(ax, ay, az, gx, gy, gz, mx, my, mz, dt);
                }

            }

            if(read_sequence % p_output_rate_divider == 0) {

                if(p_calibration_mode != 1) {

                    imu_msg.header.stamp = currentTime;
                    imu_msg.header.frame_id = imu_link;
                    imu_msg.header.seq = pub_sequence;

                    imu_msg.linear_acceleration.x = ax;
                    imu_msg.linear_acceleration.y = ay;
                    imu_msg.linear_acceleration.z = az;

                    imu_msg.angular_velocity.x = gx;
                    imu_msg.angular_velocity.y = gy;
                    imu_msg.angular_velocity.z = gz;

                    if(filter_.getDoBiasEstimation()) {
                        imu_msg.angular_velocity.x -= filter_.getAngularVelocityBiasX();
                        imu_msg.angular_velocity.y -= filter_.getAngularVelocityBiasY();
                        imu_msg.angular_velocity.z -= filter_.getAngularVelocityBiasZ();
                    }

                    mag_msg.header.stamp = currentTime;
                    mag_msg.header.frame_id = mag_link;
                    mag_msg.header.seq = pub_sequence;

                    mag_msg.magnetic_field.x = mx;
                    mag_msg.magnetic_field.y = my;
                    mag_msg.magnetic_field.z = mz;

                    // get orientiation from filter
                    filter_.getOrientation(q0, q1, q2, q3);

                    switch(p_world_frame) {
                        case 0:
                            imu_msg.orientation.x = q1;
                            imu_msg.orientation.y = q2;
                            imu_msg.orientation.z = q3;
                            imu_msg.orientation.w = q0;
                            break;
                        case 1:
                            float q0_temp, q1_temp, q2_temp, q3_temp;
                            quaternionMultiplication(0.707106f, 0.0f, 0.0f, 0.707106f, q0, q1, q2, q3, q0_temp, q1_temp, q2_temp, q3_temp);
                            imu_msg.orientation.x = q1_temp;
                            imu_msg.orientation.y = q2_temp;
                            imu_msg.orientation.z = q3_temp;
                            imu_msg.orientation.w = q0_temp;
                            break;
                        default:
                            imu_msg.orientation.x = q1;
                            imu_msg.orientation.y = q2;
                            imu_msg.orientation.z = q3;
                            imu_msg.orientation.w = q0;
                            break;
                    }

                    imu_msg.angular_velocity_covariance[0] = 0.02;
                    imu_msg.angular_velocity_covariance[1] = 0;
                    imu_msg.angular_velocity_covariance[2] = 0;
                    imu_msg.angular_velocity_covariance[3] = 0;
                    imu_msg.angular_velocity_covariance[4] = 0.02;
                    imu_msg.angular_velocity_covariance[5] = 0;
                    imu_msg.angular_velocity_covariance[6] = 0;
                    imu_msg.angular_velocity_covariance[7] = 0;
                    imu_msg.angular_velocity_covariance[8] = 0.02;

                    imu_msg.linear_acceleration_covariance[0] = 0.04;
                    imu_msg.linear_acceleration_covariance[1] = 0;
                    imu_msg.linear_acceleration_covariance[2] = 0;
                    imu_msg.linear_acceleration_covariance[3] = 0;
                    imu_msg.linear_acceleration_covariance[4] = 0.04;
                    imu_msg.linear_acceleration_covariance[5] = 0;
                    imu_msg.linear_acceleration_covariance[6] = 0;
                    imu_msg.linear_acceleration_covariance[7] = 0;
                    imu_msg.linear_acceleration_covariance[8] = 0.04;

                    imu_msg.orientation_covariance[0] = 0.0025;
                    imu_msg.orientation_covariance[1] = 0;
                    imu_msg.orientation_covariance[2] = 0;
                    imu_msg.orientation_covariance[3] = 0;
                    imu_msg.orientation_covariance[4] = 0.0025;
                    imu_msg.orientation_covariance[5] = 0;
                    imu_msg.orientation_covariance[6] = 0;
                    imu_msg.orientation_covariance[7] = 0;
                    imu_msg.orientation_covariance[8] = 0.0025;

                    // publish objects
                    pub_imu_msg.publish(&imu_msg);
                    pub_mag_msg.publish(&mag_msg);

                    if(pub_sequence % 1024 == 0) {

                        sprintf(loginfo_buffer, "gyro biases: %.3f, %.3f, %.3f", filter_.getAngularVelocityBiasX(), filter_.getAngularVelocityBiasY(), filter_.getAngularVelocityBiasZ());
                        nh.loginfo(loginfo_buffer);

                        sprintf(loginfo_buffer, "accel gain: %.3f", filter_.getGain());
                        nh.loginfo(loginfo_buffer);

                    }

                    pub_sequence++;

                } else {
                    // publish array containing raw sensor values
                    int16_t sensor_data[9] = {accelRD.x, accelRD.y, accelRD.z, gyroRD.x, gyroRD.y, gyroRD.z, magRD.x, magRD.y, magRD.z};
                    array.data = sensor_data;
                    pub_array.publish(&array);

                }

                nh.spinOnce();

            }

            if(nh_connected && filter_.getSteadyState() && p_calibration_mode != 1) {
                MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
            } else {
                MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);
            }

            read_sequence++;
            data_ready = false;

        }

    }

}