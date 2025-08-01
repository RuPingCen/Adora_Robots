cmake_minimum_required(VERSION 3.5)
project(yewai_slam)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2") 
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

#========================
#  common dependencies
#========================
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(serial REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


#========================
#  dora
#========================
set(DORA_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/dora/include)

set(DORA_NODE_API_LIB 
${CMAKE_SOURCE_DIR}/dora/lib/libdora_node_api_c.a
    CACHE FILEPATH "Path to libdora_node_api_c.a"
)

#===============================
#  include directories
#===============================

set(COMMON_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(LIDAR_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/rslidar_driver/include)
set(IMU_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/HWT9053modbus/include)
set(LOCALIZATION_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/dora-hdl_localization/include)
set(ROUTING_PLANNING_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/planning/routing_planning/include)
set(MISSION_EXC_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/planning/mission_planning/task_exc/include)
set(MISSION_PUB_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/planning/mission_planning/task_pub/include)
# set(PUB_ROAD_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/map/pub_road/include)
set(ROAD_PUB_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/map/road_line_publisher/include)
set(CONTROL_COMMON_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/vehicle_control/include)
set(LAT_CONTROL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/vehicle_control/lat_control/include)
set(LON_CONTROL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/vehicle_control/lon_control/include)
set(CHAISSISE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/vehicle_control/vehicle_chassis_n3/include)


# add_subdirectory(rslidar_driver)
add_subdirectory(livox/livox_dora_driver2)
# add_subdirectory(HWT9053modbus)
add_subdirectory(dora-hdl_localization)
add_subdirectory(planning/routing_planning)
add_subdirectory(planning/mission_planning/task_exc)
add_subdirectory(planning/mission_planning/task_pub)
add_subdirectory(map/road_line_publisher)
add_subdirectory(map/pub_road)
add_subdirectory(control/vehicle_control/lat_controller)
add_subdirectory(control/vehicle_control/lon_controller)
add_subdirectory(control/vehicle_control/dora_mickrobot)
add_subdirectory(rerun)
