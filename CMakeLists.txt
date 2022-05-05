cmake_minimum_required(VERSION 3.0.2)
project(yamaopt_ros)

# Install yamaopt with pip
find_package(PythonInterp REQUIRED)
if(NOT ${PYTHONINTERP_FOUND})
  message(FATAL_ERROR "Python interpreter is not found.")
endif()
set(YAMAOPT_INSTALL_COMMAND
  ${PYTHON_EXECUTABLE} -m pip install -e ${CMAKE_SOURCE_DIR}/yamaopt)
execute_process(COMMAND ${YAMAOPT_INSTALL_COMMAND}
  RESULT_VARIABLE RET
  OUTPUT_VARIABLE STD_OUTERR ERROR_VARIABLE STD_OUTERR)
if("${RET}" STREQUAL "0")
  message("Execute ${YAMAOPT_INSTALL_COMMAND}")
else()
  message(WARNING ${STD_OUTERR})
  message(FATAL_ERROR "Fail ${YAMAOPT_INSTALL_COMMAND}")
endif()

# Download sample rosbag data
add_custom_target(${PROJECT_NAME}_download_sample_data ALL
  COMMAND ${PROJECT_SOURCE_DIR}/scripts/download_sample_data.py)

# Install yamaopt_ros with catkin
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  geometry_msgs
  jsk_data
  jsk_recognition_msgs
  )

add_service_files(
  FILES
  SensorPlacement.srv
  AccumulatePolygons.srv
  )

add_message_files(
  FILES
  SensorPlacementRequest.msg
  SensorPlacementResponse.msg
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
  # Currently, we do not test fetch in GitHub Action
  # add_rostest(test/test_fetch_sensor_placement.test ARGS gui:=false)
  # add_rostest(test/test_fetch_body_sensor_placement.test ARGS gui:=false)
endif()
