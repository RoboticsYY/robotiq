cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_gripper_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(robotiq_ethercat REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Robotiq2FGripperInput.msg"
  "msg/Robotiq2FGripperOutput.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

# TODO: when ethercat is enabled
# include_directories(include)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${robotiq_ethercat_INCLUDE_DIRS}
# )

# add_executable(robotiq_2f_gripper_ethercat_node
#   src/robotiq_2f_gripper_control/robotiq_2f_gripper_ethercat_node.cpp
#   src/robotiq_2f_gripper_control/robotiq_2f_gripper_ethercat_client.cpp
# )

# target_link_libraries(robotiq_2f_gripper_ethercat_node
#    ${robotiq_ethercat_LIBRARIES}
#    ${catkin_LIBRARIES}
# )

# add_dependencies(robotiq_2f_gripper_ethercat_node robotiq_2f_gripper_control_generate_messages_cpp)

# install(PROGRAMS nodes/Robotiq2FGripperSimpleController.py nodes/Robotiq2FGripperStatusListener.py
#     nodes/Robotiq2FGripperTcpNode.py nodes/Robotiq2FGripperRtuNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})