#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Int16MultiArray.h>

#include "fximu_complementary.h"
#include "parameters.h"

// TODO: RESEARCH: more adaptive gain
// TODO: AUDIT: different bias_alpha and gain and gain_mag, then put in defaults
// TODO: IDEA: disable red_on, and use it for debug, like in auto gain debug

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

    // accelmag and gyro interrupt init
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2);

    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);    
    GPIOIntRegister(GPIO_PORTE_BASE, accelmaggyro_isr);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_LOW_LEVEL);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_LOW_LEVEL);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_3);       
    IntMasterEnable();
    IntEnable(INT_GPIOE);

    // timer interrupt initialization
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    IntMasterEnable();
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq / p_sensor_read_rate);
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // reset sensors
    sensor_hard_reset();

    // request WHOAMI from sensors
    fxas21002_WHOAMI = GyroWhoAmI(FXAS21002C_ADDRESS);
    MAP_SysCtlDelay(DELAY_5MS);

    fxos8700_WHOAMI = AccelMagWhoAmI(FXOS8700_ADDRESS);
    MAP_SysCtlDelay(DELAY_5MS);

    // init sensors, if both are online
    if(fxas21002_WHOAMI == 0xD7 && fxos8700_WHOAMI == 0xC7) {
        init_sensors();
    }

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

        // connect handler, will trigger only if nh_connected, and nh_prev_state is false
        // continues the loop when done
        if(nh_connected && !nh_prev_state) {

            // check eeprom_init, gyro and accel_mag status, block program if error
            print_diagnosis();

            // handle parameters
            switch(handle_parameters(nh)) {
                case SUCCESS:
                    // setting nh_prev_state = true
                    nh_prev_state = true;
                    break;
                case RETRY:
                    // TODO: DOC
                    // blink 3 red, 50ms
                    for(uint8_t i=0; i<3; i++) { blink_blocking_red(DELAY_50MS); }
                    continue;
                case FATAL:
                    // TODO: DOC
                    // blink red, 500ms
                    // block program
                    while(1) { blink_blocking_red(DELAY_500MS); }
                    break;
                case WRITE_ERROR:
                    // TODO: DOC
                    // blink 2 red, 50ms
                    // not fatal error: will continue to work, reading parameters from rosparam server
                    for(uint8_t i=0; i<2; i++) { blink_blocking_red(DELAY_50MS); }
                    nh_prev_state = true;
                    break;
            }

            // print parameters
            print_parameters(nh);

            // reset sequences
            read_sequence = 1;
            pub_sequence = 1;

            // re-set timer, with p_sensor_read_rate from handle_parameters
            TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq / p_sensor_read_rate);

            if(p_calibration_mode != 1) {

                filter_.setDoBiasEstimation(p_bias_estimation);
                filter_.setDoAdaptiveGain(p_adaptive_gain);

                // notice, each statement below returns success state
                filter_.setGainAcc(p_gain_acc);
                filter_.setGainMag(p_gain_mag);
                filter_.setBiasAlpha(p_bias_alpha);
                filter_.setKAngularVelocityThreshold(p_kAngularVelocityThreshold);
                filter_.setKAccelerationThreshold(p_kAccelerationThreshold);
                filter_.setKSteadyGravity(p_kSteadyGravity);
                filter_.setKDeltaAngularVelocityThreshold(p_kDeltaAngularVelocityThreshold);

                // insert initial gyro biases into filter
                filter_.setAngularVelocityBiasX(p_GYRO_BIAS[0]);
                filter_.setAngularVelocityBiasY(p_GYRO_BIAS[1]);
                filter_.setAngularVelocityBiasZ(p_GYRO_BIAS[2]);

            } else {
                array.data_length = 9;
            }

            // re-init sensors
            init_sensors();

            nh.spinOnce();
            nh.getHardware()->delay(10);
            continue;
        }

        // disconnect handler
        if(!nh_connected && nh_prev_state) {
            nh_prev_state = false;
            red_off();
            green_off();
            nh.spinOnce();
            nh.getHardware()->delay(10);
            continue;
        }

        if(data_ready && nh_connected) {

            currentTime = nh.now();

            // TODO: DOC: ADD BLINK colors when connected
            // if connected blink red, if in steady state blink green
            // if in calibration mode blink red
            if(p_calibration_mode != 1) {
                if(filter_.getSteadyState()) { green_on(); } else { red_on(); }
            } else {
                red_on();
            }

            // so we have dt at next iteration
            if(!initialized_filter_ & p_calibration_mode != 1) {
                markedTime = currentTime;
                initialized_filter_ = true;
                continue;
            }

            double dt = currentTime.toSec() - markedTime.toSec();
            markedTime = currentTime;

            // updates filter with sensor values if not in calibration
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
                if(p_USE_MAG) {
                    filter_.update(ax, ay, az, gx, gy, gz, mx, my, mz, dt);
                } else {
                    filter_.update(ax, ay, az, gx, gy, gz, dt);
                }

            }

            // output values, only if connected
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
                            // NWU
                            imu_msg.orientation.x = q1;
                            imu_msg.orientation.y = q2;
                            imu_msg.orientation.z = q3;
                            imu_msg.orientation.w = q0;
                            break;
                        case 1:
                            // ENU
                            float q0_temp, q1_temp, q2_temp, q3_temp;
                            quaternionMultiplication(0.707106f, 0.0f, 0.0f, 0.707106f, q0, q1, q2, q3, q0_temp, q1_temp, q2_temp, q3_temp);
                            imu_msg.orientation.x = q1_temp;
                            imu_msg.orientation.y = q2_temp;
                            imu_msg.orientation.z = q3_temp;
                            imu_msg.orientation.w = q0_temp;
                            break;
                        default:
                            // NWU
                            imu_msg.orientation.x = q1;
                            imu_msg.orientation.y = q2;
                            imu_msg.orientation.z = q3;
                            imu_msg.orientation.w = q0;
                            break;
                    }

                    imu_msg.angular_velocity_covariance[0] = ANGULAR_VELOCITY_COVARIANCE_0;
                    imu_msg.angular_velocity_covariance[1] = ANGULAR_VELOCITY_COVARIANCE_1;
                    imu_msg.angular_velocity_covariance[2] = ANGULAR_VELOCITY_COVARIANCE_2;
                    imu_msg.angular_velocity_covariance[3] = ANGULAR_VELOCITY_COVARIANCE_3;
                    imu_msg.angular_velocity_covariance[4] = ANGULAR_VELOCITY_COVARIANCE_4;
                    imu_msg.angular_velocity_covariance[5] = ANGULAR_VELOCITY_COVARIANCE_5;
                    imu_msg.angular_velocity_covariance[6] = ANGULAR_VELOCITY_COVARIANCE_6;
                    imu_msg.angular_velocity_covariance[7] = ANGULAR_VELOCITY_COVARIANCE_7;
                    imu_msg.angular_velocity_covariance[8] = ANGULAR_VELOCITY_COVARIANCE_8;

                    imu_msg.linear_acceleration_covariance[0] = LINEAR_ACCELERATION_COVARIANCE_0;
                    imu_msg.linear_acceleration_covariance[1] = LINEAR_ACCELERATION_COVARIANCE_1;
                    imu_msg.linear_acceleration_covariance[2] = LINEAR_ACCELERATION_COVARIANCE_2;
                    imu_msg.linear_acceleration_covariance[3] = LINEAR_ACCELERATION_COVARIANCE_3;
                    imu_msg.linear_acceleration_covariance[4] = LINEAR_ACCELERATION_COVARIANCE_4;
                    imu_msg.linear_acceleration_covariance[5] = LINEAR_ACCELERATION_COVARIANCE_5;
                    imu_msg.linear_acceleration_covariance[6] = LINEAR_ACCELERATION_COVARIANCE_6;
                    imu_msg.linear_acceleration_covariance[7] = LINEAR_ACCELERATION_COVARIANCE_7;
                    imu_msg.linear_acceleration_covariance[8] = LINEAR_ACCELERATION_COVARIANCE_8;

                    imu_msg.orientation_covariance[0] = ORIENTATION_COVARIANCE_0;
                    imu_msg.orientation_covariance[1] = ORIENTATION_COVARIANCE_1;
                    imu_msg.orientation_covariance[2] = ORIENTATION_COVARIANCE_2;
                    imu_msg.orientation_covariance[3] = ORIENTATION_COVARIANCE_3;
                    imu_msg.orientation_covariance[4] = ORIENTATION_COVARIANCE_4;
                    imu_msg.orientation_covariance[5] = ORIENTATION_COVARIANCE_5;
                    imu_msg.orientation_covariance[6] = ORIENTATION_COVARIANCE_6;
                    imu_msg.orientation_covariance[7] = ORIENTATION_COVARIANCE_7;
                    imu_msg.orientation_covariance[8] = ORIENTATION_COVARIANCE_8;

                    // publish imu and mag messages, if connected
                    pub_imu_msg.publish(&imu_msg);
                    pub_mag_msg.publish(&mag_msg);

                    // report gyro biases and accel gain
                    if((p_bias_estimation || p_adaptive_gain) && pub_sequence % 2048 == 0) {
                        // TODO: DOC: this only works if bias_estimation and adaptive_gain is set
                        // TODO: DOC: also steady state measurement only works if bias_estimation is true
                        // TODO: DOC: adaptive_gain does not check for steady state.
                        sprintf(loginfo_buffer, "gyro_bias: %.3f, %.3f, %.3f, gain: %.3f ", filter_.getAngularVelocityBiasX(), filter_.getAngularVelocityBiasY(), filter_.getAngularVelocityBiasZ(), filter_.getGain());
                        nh.loginfo(loginfo_buffer);
                    }

                    pub_sequence++;

                } else {

                    // publish array containing raw sensor values, only if connected
                    raw_sensor_data[0] = accelRD.x;
                    raw_sensor_data[1] = accelRD.y;
                    raw_sensor_data[2] = accelRD.z;
                    raw_sensor_data[3] = gyroRD.x;
                    raw_sensor_data[4] = gyroRD.y;
                    raw_sensor_data[5] = gyroRD.z;
                    raw_sensor_data[6] = magRD.x;
                    raw_sensor_data[7] = magRD.y;
                    raw_sensor_data[8] = magRD.z;
                    array.data = raw_sensor_data;
                    pub_array.publish(&array);

                }

                // the is the main spin after publishing
                nh.spinOnce();

                // PUBLISH END

            }

            red_off();
            green_off();
            read_sequence++;
            data_ready = false;

            // DATA READY END

        } else if(data_ready && !nh_connected) {

            // so it can reconnect
            if(p_calibration_mode != 1) {  initialized_filter_ = false; }
            nh.spinOnce();
            red_on(); // TODO: DOC: add disconnect

         }

        // WHILE END
    }

}