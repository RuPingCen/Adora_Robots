
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


add_executable(mickrobotx4 src/mickx4_bringup_v3.cc 
)

add_executable(adora_chassis src/adora_chassis_dora_node_V1.cc)

target_link_libraries(mickrobotx4
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  #$ENV{HOME}/dora/target/debug/libdora_node_api_cxx.a
  ${DORA_NODE_API_LIB}
  serial
  # ${CMAKE_CURRENT_SOURCE_DIR}/serial/build/libserial.so   
  m
  rt
  dl 
  pthread
)

target_link_libraries(adora_chassis
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  #$ENV{HOME}/dora/target/debug/libdora_node_api_cxx.a
  ${DORA_NODE_API_LIB}
  serial
  # ${CMAKE_CURRENT_SOURCE_DIR}/serial/build/libserial.so   
  m
  rt
  dl 
  pthread
)
 
