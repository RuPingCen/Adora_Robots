include_directories(
  ${DORA_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(task_pub_node main.cpp)
target_link_libraries(task_pub_node
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
  pthread 
)




