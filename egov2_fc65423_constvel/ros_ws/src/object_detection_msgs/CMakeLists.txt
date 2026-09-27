cmake_minimum_required(VERSION 3.0.2)
project(object_detection_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
  CarPosition.msg
  ObjectCount.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

