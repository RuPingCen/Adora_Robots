
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${CONTROL_COMMON_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(lat_controller_node interface_lat.cpp pure_pursuit.cpp)
target_link_libraries(lat_controller_node
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
  pthread
)




