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

set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include") 
set(GUROBI_LIB_DIR "$ENV{GUROBI_HOME}/lib")
set(GUROBI_LIBS gurobi_c++ gurobi65)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        controller_interface
                                        visualization_msgs
                                        hiqp_core
					                    hiqp_collision_check
                                        hiqp_msgs)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp controller_interface visualization_msgs hiqp_core hiqp_msgs hiqp_collision_check
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               DEPENDS orocos_kdl)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${GUROBI_INCLUDE_DIR})

link_directories(${GUROBI_LIB_DIR})

add_library(${PROJECT_NAME} src/hiqp_joint_velocity_controller.cpp
                            src/hiqp_joint_velocity_controller_collision.cpp
			                src/ros_topic_subscriber.cpp
                            src/ros_visualizer.cpp
                            src/utilities.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${GUROBI_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
