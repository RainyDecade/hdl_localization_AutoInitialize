cmake_minimum_required(VERSION 3.0.2)
project(btc_init_localizer)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  message_generation
  ndt_omp
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(TBB REQUIRED COMPONENTS tbb)

add_service_files(
  FILES
  EstimateInitialPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES btc_core btc_database
  CATKIN_DEPENDS message_filters roscpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros pcl_ros pcl_conversions visualization_msgs message_runtime ndt_omp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(btc_core
  src/BTC.cpp
)
add_dependencies(btc_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(btc_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  TBB::tbb
)

add_library(btc_database
  src/btc_database.cpp
)
add_dependencies(btc_database
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(btc_database
  btc_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  TBB::tbb
)

add_executable(btc_map_builder_node
  src/btc_map_builder_node.cpp
)
add_dependencies(btc_map_builder_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(btc_map_builder_node
  btc_core
  btc_database
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  TBB::tbb
)

add_executable(btc_ndt_initializer_node
  src/btc_ndt_initializer_node.cpp
)
add_dependencies(btc_ndt_initializer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(btc_ndt_initializer_node
  btc_core
  btc_database
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ${Boost_LIBRARIES}
  TBB::tbb
)

install(TARGETS
  btc_core
  btc_database
  btc_map_builder_node
  btc_ndt_initializer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
