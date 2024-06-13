cmake_minimum_required(VERSION 3.0.2)
project(tf_to_fake_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  sensor_msgs
  eigen_conversions
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

catkin_package(
  DEPENDS EIGEN3
  CATKIN_DEPENDS geometry_msgs sensor_msgs tf tf_conversions eigen_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/fake_imu_publisher.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


install(TARGETS gravity_compensation gravity_compensation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)