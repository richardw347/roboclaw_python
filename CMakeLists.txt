cmake_minimum_required(VERSION 2.8.3)
project(roboclaw_python)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs nav_msgs tf)

catkin_python_setup()

add_message_files(FILES RoboClawState.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs tf)