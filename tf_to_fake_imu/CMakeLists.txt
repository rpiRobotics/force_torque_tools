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
  # COMMENT OUT OR REMOVE THE FOLLOWING LINE IF NO INCLUDE DIRECTORY IS USED
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

# COMMENT OUT OR REMOVE THE FOLLOWING LINE IF NO INCLUDE DIRECTORY IS USED
# include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/fake_imu_publisher.cpp)

target_link_libraries(${PROJECT_NAME}__node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
