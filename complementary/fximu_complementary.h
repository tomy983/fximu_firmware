#ifndef ros_complementary_h
#define ros_complementary_h

#define FXAS21002C_ADDRESS (0x21)
#define FXOS8700_ADDRESS (0x1F)

#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"

#include "driverlib/flash.h"
#include "driverlib/eeprom.h"

#include "fxas21002c.h"
#include "fxos8700cq.h"

#include "complementary_filter.h"
#include "parameters.h"

tRawData gyroRD;
tRawData accelRD;
tRawData magRD;

tAccelRange _AFSR = AFSR_4G;
tGyroRange _GFSR = GFSR_500PS;

float _GYRO_SENSITIVITY = GYRO_SENSITIVITY_500DPS;
float _ACCEL_SENSITIVITY = ACCEL_MG_LSB_4G;

volatile bool data_ready = false;
uint32_t ui32SysClkFreq;
uint32_t read_sequence = 0;
uint32_t pub_sequence = 0;

ros::NodeHandle nh;
ros::Time markedTime;
ros::Time currentTime;

int16_t raw_sensor_data[9];

sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
std_msgs::Int16MultiArray array;

ros::Publisher pub_imu_msg("imu/data", &imu_msg);
ros::Publisher pub_mag_msg("imu/mag", &mag_msg);
ros::Publisher pub_array("imu/array", &array);

ComplementaryFilter filter_;
bool initialized_filter_;

// gyro interrupt service routine
void gyro_isr(void) {
    if(GPIOIntStatus(GPIO_PORTC_BASE, true) & GPIO_PIN_4) {
        GyroGetData(FXAS21002C_ADDRESS, &gyroRD);
        GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);
    }
}

// accelmag interrupt service routine
void accelmag_isr(void) {
    if(GPIOIntStatus(GPIO_PORTE_BASE, true) & GPIO_PIN_2) {
        AGGetData(FXOS8700_ADDRESS, &accelRD, &magRD);
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
    }
}

// timer interrupt handler
void Timer0IntHandler(void) {
    data_ready = true;
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void fx_delay() {
    // 50ms
    MAP_SysCtlDelay(1333333UL);
}

void hard_reset() {

    // gyro hard reset
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);

    // accelmag hard reset
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

    fx_delay();

    // gyro operational
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    // accelmag operational
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);

    fx_delay();
}

void init_system() {

    // record system clock value after clock initialization
    ui32SysClkFreq = MAP_SysCtlClockGet();

    // Peripheral F enable, PF0:LED, PF2:GYRO_RST, PF3:ACC_RST, configured as output
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable eeprom
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

    // Try 3 times to enable eeprom, if not fail safe
    for(int i=0; i<3; i++) {
        if(EEPROMInit()==EEPROM_INIT_OK) { eeprom_init = true; break; }
        MAP_SysCtlDelay(13333UL);
    }

    // unlock PF0, after enabling GPIOF
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;

    // configure port f
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // blink green led
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
    fx_delay();
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);

    // initialize raw sensor data
    for(int i=0; i<9; i++) { raw_sensor_data[i] = 0; }

    // blink red led
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    fx_delay();
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);

}

void init_I2C2(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), true);
    HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
}

void init_I2C0(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void red_led(bool on) {
    if(on) {
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);
    }
}

void green_led(bool on) {
    if(on) {
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
    } else {
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);
    }
}

void init_sensors() {

    hard_reset();

    switch(p_gfsr) {
        case 0:
            _GFSR = GFSR_2000PS;
            _GYRO_SENSITIVITY = GYRO_SENSITIVITY_2000DPS;
            break;
        case 1:
            _GFSR = GFSR_1000PS;
            _GYRO_SENSITIVITY = GYRO_SENSITIVITY_1000DPS;
            break;
        case 2:
            _GFSR = GFSR_500PS;
            _GYRO_SENSITIVITY = GYRO_SENSITIVITY_500DPS;
            break;
        case 3:
            _GFSR = GFSR_250PS;
            _GYRO_SENSITIVITY = GYRO_SENSITIVITY_250DPS;
            break;
        default:
            _GFSR = GFSR_500PS;
            _GYRO_SENSITIVITY = GYRO_SENSITIVITY_500DPS;
            break;
    }

    switch(p_afsr) {
        case 0:
            _AFSR = AFSR_2G;
            _ACCEL_SENSITIVITY = ACCEL_MG_LSB_2G;
            break;
        case 1:
            _AFSR = AFSR_4G;
            _ACCEL_SENSITIVITY = ACCEL_MG_LSB_4G;
            break;
        case 2:
            _AFSR = AFSR_8G;
            _ACCEL_SENSITIVITY = ACCEL_MG_LSB_8G;
            break;
        default:
            _AFSR = AFSR_4G;
            _ACCEL_SENSITIVITY = ACCEL_MG_LSB_4G;
            break;
    }

    switch(p_sensor_read_rate) {
        case 50:
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_50HZ);
            AGInit(FXOS8700_ADDRESS, _AFSR, ODR_100HZ);
            break;
        case 100:
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_100HZ);
            AGInit(FXOS8700_ADDRESS, _AFSR, ODR_200HZ);
            break;
        case 200:
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_200HZ);
            AGInit(FXOS8700_ADDRESS, _AFSR, ODR_400HZ);
            break;
        case 400:
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_400HZ);
            AGInit(FXOS8700_ADDRESS, _AFSR, ODR_800HZ);
            break;
        default:
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_100HZ);
            AGInit(FXOS8700_ADDRESS, _AFSR, ODR_200HZ);
            break;
    }

    switch(p_output_rate_divider) {
        case 1:
            break;
        case 2:
            break;
        case 4:
            break;
        case 8:
            break;
        case 16:
            break;
        default:
            p_output_rate_divider = 8;
            break;
    }

}

#endif