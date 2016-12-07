// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HIQP_TASK_AVOID_COLLISIONS_SDF_H
#define HIQP_TASK_AVOID_COLLISIONS_SDF_H

#include <string>

#include <hiqp/hiqp_time_point.h>
#include <hiqp/task_definition.h>

namespace hiqp
{
namespace tasks
{

  /*! \brief A task definition that avoids collisions between given geometric primitives on the manipulator and the environment represented by a SDF map.
   *  \author Robert Krug */  
  class TaskAvoidCollisionsSDF : public TaskDefinition {
  public:
    TaskAvoidCollisionsSDF(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                  std::shared_ptr<Visualizer> visualizer)
     : TaskDefinition(geom_prim_map, visualizer) {}
    ~TaskAvoidCollisionsSDF() noexcept {}

    int init(const std::vector<std::string>& parameters,
             RobotStatePtr robot_state,
             unsigned int n_controls);

    int update(RobotStatePtr robot_state);

    int monitor();

  private:
    TaskAvoidCollisionsSDF(const TaskAvoidCollisionsSDF& other) = delete;
    TaskAvoidCollisionsSDF(TaskAvoidCollisionsSDF&& other) = delete;
    TaskAvoidCollisionsSDF& operator=(const TaskAvoidCollisionsSDF& other) = delete;
    TaskAvoidCollisionsSDF& operator=(TaskAvoidCollisionsSDF&& other) noexcept = delete;

    std::string              link_name_;
    int                      joint_q_nr_;
    double                   desired_configuration_;
  };

} // namespace tasks

} // namespace hiqp

#endif // include guard
