cmake_minimum_required(VERSION 3.5)
project(unitree_go)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AudioData.msg"
    "msg/BmsCmd.msg"
    "msg/BmsState.msg"
    "msg/Error.msg"
    "msg/Go2FrontVideoData.msg"
    "msg/HeightMap.msg"
    "msg/IMUState.msg"
    "msg/InterfaceConfig.msg"
    "msg/LidarState.msg"
    "msg/LowCmd.msg"
    "msg/LowState.msg"
    "msg/MotorCmd.msg"
    "msg/MotorState.msg"
    "msg/PathPoint.msg"
    "msg/Req.msg"
    "msg/Res.msg"
    "msg/SportModeCmd.msg"
    "msg/SportModeState.msg"
    "msg/TimeSpec.msg"
    "msg/UwbState.msg"
    "msg/UwbSwitch.msg"
    "msg/WirelessController.msg"
    DEPENDENCIES geometry_msgs
)

ament_package()
