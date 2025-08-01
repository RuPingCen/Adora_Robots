include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(task_exc_node main.cpp)
target_link_libraries(task_exc_node
  ${EIGEN_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
  pthread
)




