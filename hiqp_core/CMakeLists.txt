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
project(hiqp_core)

# Available backends: qpoases, gurobi, casadi
set(HIQP_QPSOLVER_BACKEND "gurobi")

###
### --- DONT EDIT BELOW THIS LINE ---
###

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
message(${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wl,-z,defs")
set(CMAKE_VERBOSE_MAKEFILE off)

find_package(catkin REQUIRED COMPONENTS rosconsole pluginlib)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(CATKIN_DEPENDS rosconsole pluginlib
               INCLUDE_DIRS include 
               LIBRARIES ${PROJECT_NAME} 
               DEPENDS orocos_kdl)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set(SOLVER_SOURCE_FILE "")
if(${HIQP_QPSOLVER_BACKEND} STREQUAL "qpoases")
    set(SOLVER_SOURCE_FILE "src/solvers/qpoases_solver.cpp")
    add_definitions(-DHIQP_QPOASES)
elseif(${HIQP_QPSOLVER_BACKEND} STREQUAL "gurobi")
    set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include") 
    set(GUROBI_LIB_DIR "$ENV{GUROBI_HOME}/lib")
    set(GUROBI_LIBS gurobi_c++ gurobi65)
    include_directories(${GUROBI_INCLUDE_DIR})
    link_directories(${GUROBI_LIB_DIR})
    set(SOLVER_SOURCE_FILE "src/solvers/gurobi_solver.cpp")
    add_definitions(-DHIQP_GUROBI)
elseif(${HIQP_QPSOLVER_BACKEND} STREQUAL "casadi")
    set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include") 
    set(GUROBI_LIB_DIR "$ENV{GUROBI_HOME}/lib")
    set(GUROBI_LIBS gurobi_c++ gurobi65)
    find_package(CASADI REQUIRED)
    catkin_package(LIBRARIES casadi)
    include_directories(${GUROBI_INCLUDE_DIR} ${CASADI_INCLUDE_DIR})
    link_directories(${GUROBI_LIB_DIR})
    set(SOLVER_SOURCE_FILE "src/solvers/casadi_solver.cpp")
    add_definitions(-DHIQP_CASADI)
endif()

add_library(${PROJECT_NAME} src/utilities.cpp
                            src/hiqp_time_point.cpp
                            src/task_manager.cpp
                            src/task.cpp

                            src/geometric_primitives/geometric_primitive.cpp
                            src/geometric_primitives/geometric_primitive_map.cpp

                            ${SOLVER_SOURCE_FILE}

                            src/tasks/tdyn_linear.cpp
                            src/tasks/tdyn_cubic.cpp
                            src/tasks/tdyn_hyper_sin.cpp
                            src/tasks/tdyn_jnt_limits.cpp
                            src/tasks/tdyn_minimal_jerk.cpp

                            src/tasks/tdef_meta_task.cpp
                            src/tasks/tdef_full_pose.cpp
                            src/tasks/tdef_geometric_alignment.cpp
                            src/tasks/tdef_geometric_projection.cpp
                            src/tasks/tdef_geometric_projection_with_nullspace.cpp

                            src/tasks/tdef_jnt_config.cpp
                            src/tasks/tdef_jnt_limits.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} 
                                      ${orocos_kdl_LIBRARIES})
if(${HIQP_QPSOLVER_BACKEND} STREQUAL "gurobi")
    target_link_libraries(${PROJECT_NAME} ${GUROBI_LIBS})
elseif(${HIQP_QPSOLVER_BACKEND} STREQUAL "casadi")
    target_link_libraries(${PROJECT_NAME} ${CASADI_LIBRARIES} ${GUROBI_LIBS})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
