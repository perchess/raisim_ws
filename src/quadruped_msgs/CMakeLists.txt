cmake_minimum_required(VERSION 3.0.2)
project(quadruped_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/general.cfg
)

add_message_files(
  FILES
    commandDes.msg
    LegControllerCommand.msg
    CMPC_Result.msg
)

add_service_files(
  FILES
    QuadrupedCmd.srv
    QuadrupedCmdBool.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    message_runtime
    dynamic_reconfigure
)

include_directories(${catkin_INCLUDE_DIRS})

#add_dependencies(quadruped_msgs ${PROJECT_NAME}_gencfg)
