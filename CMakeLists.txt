cmake_minimum_required(VERSION 2.8.3)
project(slam_karto)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    sparse_bundle_adjustment
    tf
    visualization_msgs
    pluginlib
)
find_package(Eigen REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package()

add_library(karto_plugins src/spa_solver.cpp)
add_executable(slam_karto src/slam_karto.cpp)

target_link_libraries(karto_plugins ${catkin_LIBRARIES})
target_link_libraries(slam_karto ${catkin_LIBRARIES})
install(TARGETS slam_karto
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
