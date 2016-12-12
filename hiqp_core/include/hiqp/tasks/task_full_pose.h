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

#ifndef HIQP_TASK_FULL_POSE_H
#define HIQP_TASK_FULL_POSE_H

#include <string>

#include <hiqp/robot_state.h>
#include <hiqp/task_definition.h>

namespace hiqp
{
namespace tasks
{

  /*! \brief Represents a task definition that sets a specific joint configuration. This task definition does not leave any redundancy available to other tasks!
   *  \author Marcus A Johansson */ 
  class TaskFullPose : public TaskDefinition
  {
  public:

    TaskFullPose(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                 std::shared_ptr<Visualizer> visualizer)
     : TaskDefinition(geom_prim_map, visualizer) {}

    ~TaskFullPose() noexcept {}

    int init(const std::vector<std::string>& parameters,
             RobotStatePtr robot_state);

    int update(RobotStatePtr robot_state);

    int monitor();

  private:
    TaskFullPose(const TaskFullPose& other) = delete;
    TaskFullPose(TaskFullPose&& other) = delete;
    TaskFullPose& operator=(const TaskFullPose& other) = delete;
    TaskFullPose& operator=(TaskFullPose&& other) noexcept = delete;

    std::vector<double>                desired_configuration_;
  };

} // namespace tasks

} // namespace hiqp

#endif // include guard