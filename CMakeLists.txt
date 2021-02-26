cmake_minimum_required(VERSION 2.8.3)
project(fximu_firmware)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  rosserial_client
  rosserial_tivac
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_tivac
  SCRIPT make_libraries_tiva
)

rosserial_configure_client(
  DIRECTORY complementary
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(complementary complementary.axf ALL)
rosserial_add_client_target(complementary flash)
rosserial_add_client_target(complementary size)
