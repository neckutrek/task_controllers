# The HiQP Control Framework, an optimal control framework targeted at robotics
# Copyright (C) 2016 Marcus A Johansson
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8.6)
project(hiqp_ros)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wl,-z,defs")

find_package(catkin REQUIRED COMPONENTS roscpp
                                        controller_interface
                                        visualization_msgs
                                        kdl_parser
                                        hiqp_core
                                        hiqp_msgs
                                        control_msgs
                                        actionlib)

find_package(orocos_kdl REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp controller_interface visualization_msgs hiqp_core hiqp_msgs kdl_parser control_msgs actionlib
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME} hiqp_client
               DEPENDS orocos_kdl)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# hiqp-ros library
add_library(${PROJECT_NAME} src/hiqp_joint_velocity_controller.cpp
                            src/hiqp_service_handler.cpp
                            src/ros_topic_subscriber.cpp
                            src/ros_visualizer.cpp
                            src/utilities.cpp)

add_dependencies(${PROJECT_NAME} hiqp_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# hiqp-client library
add_library(hiqp_client src/hiqp_client.cpp)
add_dependencies(hiqp_client hiqp_msgs_generate_messages_cpp)
target_link_libraries(hiqp_client ${catkin_LIBRARIES})

# hiqp-joint-trajectory-controller
add_executable(hiqp_joint_trajectory_controller_node src/hiqp_joint_trajectory_controller.cpp)
add_dependencies(hiqp_joint_trajectory_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(hiqp_joint_trajectory_controller_node hiqp_client ${catkin_LIBRARIES})

# hiqp_client_test
add_executable (test_hiqp_client src/test_hiqp_client.cpp)
add_dependencies(test_hiqp_client hiqp_msgs_generate_messages_cpp)
target_link_libraries(test_hiqp_client hiqp_client ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
