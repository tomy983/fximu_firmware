#ifndef ros_complementary_h
#define ros_complementary_h

#define FXAS21002C_ADDRESS (0x21)
#define FXOS8700_ADDRESS (0x1F)

#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"

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

tAccelRange _AFSR = AFSR_8G;
tGyroRange _GFSR = GFSR_2000PS;

float _GYRO_SENSITIVITY;
float _ACCEL_SENSITIVITY;

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
ros::Publisher pub_array("imu/raw", &array);

ComplementaryFilter filter_;
bool initialized_filter_;

uint8_t fxas21002_WHOAMI;
uint8_t fxos8700_WHOAMI;

// accelmag + gyro interrupt service routine
void accelmaggyro_isr(void) {
int32_t status=0;
status = GPIOIntStatus(GPIO_PORTE_BASE,true);
GPIOIntClear(GPIO_PORTE_BASE,status);
    if((status & GPIO_INT_PIN_3) == GPIO_INT_PIN_3) {
        GyroGetData(FXAS21002C_ADDRESS, &gyroRD);
    }
    if((status & GPIO_INT_PIN_1) == GPIO_INT_PIN_1) {
    
        AccelMagGetData(FXOS8700_ADDRESS, &accelRD, &magRD);
    }
   
}

// timer interrupt handler
void Timer0IntHandler(void) {
    data_ready = true;
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void red_on() { MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); }
void red_off() { MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0); }
void green_on() { MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); }
void green_off() { MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0); }

void blink_blocking_red(uint32_t delay) {
    red_on(); MAP_SysCtlDelay(delay);
    red_off(); MAP_SysCtlDelay(delay);
}

void blink_blocking_green(uint32_t delay) {
    green_on(); MAP_SysCtlDelay(delay);
    green_off(); MAP_SysCtlDelay(delay);
}

void blink_blocking_both(uint32_t delay) {
    green_on(); MAP_SysCtlDelay(delay); green_off();
    red_on(); MAP_SysCtlDelay(delay); red_off();
}

void sensor_hard_reset() {

    // gyro + accelmag hard reset
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);

    MAP_SysCtlDelay(DELAY_50MS);

    // gyro + accelmag operational
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);

    MAP_SysCtlDelay(DELAY_50MS);

}

void print_diagnosis() {

    if(!eeprom_init) {
        sprintf(loginfo_buffer, "eeprom init error");
        nh.loginfo(loginfo_buffer);
        spin_once(nh);
    }

    if(fxas21002_WHOAMI != 0xD7) {
        sprintf(loginfo_buffer, "fxas21002 fail: %d", fxas21002_WHOAMI);
        nh.loginfo(loginfo_buffer);
        spin_once(nh);
    }

    if(fxos8700_WHOAMI != 0xC7) {
        sprintf(loginfo_buffer, "fxos8700 fail: %d", fxos8700_WHOAMI);
        nh.loginfo(loginfo_buffer);
        spin_once(nh);
    }

    if(fxas21002_WHOAMI != 0xD7 || fxos8700_WHOAMI != 0xC7) {
        sprintf(loginfo_buffer, "sensor error: imu disabled");
        nh.loginfo(loginfo_buffer);
        spin_once(nh);
        // TODO: DOC: add fatal status
        // blink red, 500ms
        // block program
        while(1) { blink_blocking_red(DELAY_500MS); }
    }

}

void init_system() {

    // record system clock value after clock initialization
    ui32SysClkFreq = MAP_SysCtlClockGet();

    // Peripheral F enable, PF0:USR_SW2, PF1:red led, PF2:blue led, PF3:green led, configured as output
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Peripheral D enable, PD2:GYRO_RST + ACC_RST, configured as output
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Peripheral A enable, PA2 is bootloader trigger
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // set PA2 as input
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Enable eeprom
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

    // Try 3 times to enable eeprom, if not fail safe
    for(int i=0; i<3; i++) {
        if(EEPROMInit()==EEPROM_INIT_OK) { eeprom_init = true; break; }
        MAP_SysCtlDelay(DELAY_5MS);
    }

    // configure port f
     MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    // configure port d
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);

    // initialize raw sensor data
    for(int i=0; i<9; i++) { raw_sensor_data[i] = 0; }

    // 50mS delay
    MAP_SysCtlDelay(1333333UL); // 50ms

    // check if bootloader mode
    if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2)==0) {

        // we must make sure we turn off SysTick and its interrupt before entering  the boot loader
        MAP_SysTickIntDisable();
        MAP_SysTickDisable();

        // disable all processor interrupts
        // instead of disabling them one at a time, a direct write to NVIC is done to disable all peripheral interrupts
        HWREG(NVIC_DIS0) = 0xffffffff;
        HWREG(NVIC_DIS1) = 0xffffffff;

        // return control to the boot loader
        // this is a call to the SVC handler in the boot loader
        (*((void (*)(void))(*(uint32_t *)0x2c)))();

    }

    // TODO: DOC: BLINK INIT
    // flash green and then red led when system initialized
    if(eeprom_init) { blink_blocking_both(DELAY_50MS); }

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

void init_sensors() {

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
            _GFSR = GFSR_2000PS;
            _GYRO_SENSITIVITY = GYRO_SENSITIVITY_2000DPS;
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
            _AFSR = AFSR_8G;
            _ACCEL_SENSITIVITY = ACCEL_MG_LSB_8G;
            break;
    }

    switch(p_sensor_read_rate) {
        case 50:
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_50HZ);
            AccelMagInit(FXOS8700_ADDRESS, _AFSR, ODR_100HZ);
            break;
        case 100:
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_100HZ);
            AccelMagInit(FXOS8700_ADDRESS, _AFSR, ODR_200HZ);
            break;
        case 200:
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_200HZ);
            AccelMagInit(FXOS8700_ADDRESS, _AFSR, ODR_400HZ);
            break;
        case 400:
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_400HZ);
            AccelMagInit(FXOS8700_ADDRESS, _AFSR, ODR_800HZ);
            break;
        default:
            // default 100hz
            GyroInit(FXAS21002C_ADDRESS, _GFSR, ODR_100HZ);
            AccelMagInit(FXOS8700_ADDRESS, _AFSR, ODR_200HZ);
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
            // default 2
            p_output_rate_divider = 2;
            break;
    }

}

#endif