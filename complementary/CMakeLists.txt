cmake_minimum_required(VERSION 2.8.3)
project(complementary)

include_directories(${ROS_LIB_DIR})

add_definitions(-DTX_BUFFER_SIZE=512)
add_definitions(-DRX_BUFFER_SIZE=512)
add_definitions(-DUART_BUFFER_SIZE=512)
add_definitions(-DROSSERIAL_BAUDRATE=230400)

generate_tivac_firmware(
  USB
  STARTUP startup_gcc.c
  SRCS fximu_complementary.cpp complementary_filter.cpp ../include/fxas21002c.c ../include/fxas21002c_tiva.c ../include/fxos8700cq.c ../include/fxos8700cq_tiva.c
  INCS . ../include
  LIBS ${TIVA_WARE_PATH}/driverlib/gcc/libdriver.a ${TIVA_WARE_PATH}/sensorlib/gcc/libsensor.a
  BOARD tm4c123gxl
)