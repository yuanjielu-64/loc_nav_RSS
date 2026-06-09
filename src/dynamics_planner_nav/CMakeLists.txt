cmake_minimum_required(VERSION 3.8)
project(dynamics_planner_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)

# Optional TEB planner support
# To enable: git clone https://github.com/rst-tu-dortmund/teb_local_planner.git -b ros2
find_package(teb_local_planner QUIET)
if(teb_local_planner_FOUND)
  message(STATUS "TEB local planner found - building with TEB support")
  set(USE_TEB_PLANNER ON)
else()
  message(STATUS "TEB local planner NOT found - building without TEB support")
  set(USE_TEB_PLANNER OFF)
endif()

# Find external libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Collect source files
file(GLOB ROBOT_SOURCES CONFIGURE_DEPENDS src/robot/*.cpp)
file(GLOB LOCAL_PLANNER_SOURCES CONFIGURE_DEPENDS src/localPlanners/*.cpp)
file(GLOB GLOBAL_PLANNER_SOURCES CONFIGURE_DEPENDS src/globalPlanners/*.cpp)
file(GLOB UTILS_SOURCES CONFIGURE_DEPENDS src/utils/*.cpp)

# Exclude collision_publisher.cpp from UTILS_SOURCES (it's a separate executable)
list(FILTER UTILS_SOURCES EXCLUDE REGEX ".*collision_publisher\\.cpp$")

# Exclude TEB planner files if TEB is not found
if(NOT USE_TEB_PLANNER)
  list(FILTER LOCAL_PLANNER_SOURCES EXCLUDE REGEX ".*TEBPlanner\\.cpp$")
  list(FILTER LOCAL_PLANNER_SOURCES EXCLUDE REGEX ".*TEB_DDPPlanner\\.cpp$")
else()
  add_compile_definitions(USE_TEB_PLANNER)
endif()

# Define dependencies list
set(DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_costmap_2d
  nav2_util
)

# Build main executable
add_executable(${PROJECT_NAME}_node
  src/programs/RunMP.cpp
  ${ROBOT_SOURCES}
  ${LOCAL_PLANNER_SOURCES}
  ${GLOBAL_PLANNER_SOURCES}
  ${UTILS_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
  pthread
)

ament_target_dependencies(${PROJECT_NAME}_node ${DEPENDENCIES})

# Build SwitchMP executable (dynamic planner switching)
add_executable(${PROJECT_NAME}_switch_node
  src/programs/SwitchMP.cpp
  ${ROBOT_SOURCES}
  ${LOCAL_PLANNER_SOURCES}
  ${GLOBAL_PLANNER_SOURCES}
  ${UTILS_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_switch_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
  pthread
)

ament_target_dependencies(${PROJECT_NAME}_switch_node ${DEPENDENCIES})

# Install executables
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_switch_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  params
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
