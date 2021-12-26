cmake_minimum_required(VERSION 3.0.2)
project(yamaopt_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  geometry_msgs
  jsk_recognition_msgs
  )

add_service_files(
  FILES
  SensorPlacement.srv
  AccumulatePolygons.srv
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  jsk_recognition_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
)
