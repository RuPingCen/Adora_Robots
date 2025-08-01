include_directories(
  ${DORA_INCLUDE_DIR}
)

add_executable(pubroad src/pubroad.cpp)
target_link_libraries(pubroad
  ${DORA_NODE_API_LIB}
  m
  rt
  dl 
)




