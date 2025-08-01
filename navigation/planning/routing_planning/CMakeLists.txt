include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(routing_planning_node node_routing_core.cpp frenet.cpp pathplanning.cpp)
target_link_libraries(routing_planning_node
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
  pthread
)




