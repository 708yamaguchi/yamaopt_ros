cmake_minimum_required(VERSION 3.0.2)
project(yamaopt_ros)

# Install yamaopt with pip
find_package(PythonInterp REQUIRED)
if(NOT ${PYTHONINTERP_FOUND})
  message(FATAL_ERROR "Python interpreter is not found.")
endif()
set(YAMAOPT_INSTALL_COMMAND
  ${PYTHON_EXECUTABLE} -m pip install -e ${CMAKE_SOURCE_DIR}/yamaopt --user)
execute_process(COMMAND ${YAMAOPT_INSTALL_COMMAND} RESULT_VARIABLE RET)
if("${RET}" STREQUAL "0")
  message("Execute ${YAMAOPT_INSTALL_COMMAND}")
else()
  message(FATAL_ERROR "Fail ${YAMAOPT_INSTALL_COMMAND}")
endif()

# Install yamaopt_ros with catkin
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

# Test yamaopt_ros
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_rostest(test/test_pr2_sensor_placement.test ARGS gui:=false)
endif()
