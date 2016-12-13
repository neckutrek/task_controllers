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

cmake_minimum_required(VERSION 2.8.3)
project(hiqp_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation 
                                        std_msgs
                                        visualization_msgs
                                        geometry_msgs)

add_message_files(FILES TaskMeasure.msg
                        TaskMeasures.msg
                        StringArray.msg
                        Vector3d.msg)

add_service_files(FILES AddGeometricPrimitive.srv
                        SetTask.srv
                        RemoveAllGeometricPrimitives.srv
                        RemoveAllTasks.srv
                        RemoveGeometricPrimitive.srv
                        RemoveTask.srv
                        UpdateTask.srv
                        ListAllTasks.srv)

generate_messages(DEPENDENCIES std_msgs
                               geometry_msgs
                               visualization_msgs)

catkin_package(CATKIN_DEPENDS message_runtime
                              std_msgs 
                              visualization_msgs
                              geometry_msgs)