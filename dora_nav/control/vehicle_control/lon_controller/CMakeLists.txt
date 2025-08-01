

include_directories(
  ${DORA_INCLUDE_DIR}
  ${CONTROL_COMMON_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(lon_controller_node interface_lon.cpp lon_controller.cpp)
target_link_libraries(lon_controller_node
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
  pthread
)




