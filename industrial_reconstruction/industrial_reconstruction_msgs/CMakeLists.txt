cmake_minimum_required(VERSION 3.0.2)
project(industrial_reconstruction_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  FILES
    RGBDImageParams.msg
    TSDFVolumeParams.msg
    NormalFilterParams.msg
)

add_service_files(
  FILES
    StartReconstruction.srv
    StopReconstruction.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
