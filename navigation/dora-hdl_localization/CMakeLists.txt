
set(NDT_OMP_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/hdl_ndt_omp/build/devel/lib)
link_directories(${NDT_OMP_LIB_PATH})


include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/hdl_ndt_omp/include
)


add_executable(hdl_localization src/dora_hdl_node.cpp src/pose_estimator.cpp)

target_link_libraries(hdl_localization
  ${PCL_LIBRARIES}
  ${DORA_NODE_API_LIB}
  ndt_omp
  rt
)
