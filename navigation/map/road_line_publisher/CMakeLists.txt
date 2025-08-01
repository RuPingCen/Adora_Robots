
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${DORA_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(road_lane_publisher_node road_lane.cpp frenet.cpp)
target_link_libraries(road_lane_publisher_node
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
  pthread
)




