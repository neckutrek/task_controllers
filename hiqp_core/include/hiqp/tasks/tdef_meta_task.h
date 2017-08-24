// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2017 Marcus A Johansson
// Copyright (C) 2017 Robert Krug
// Copyright (C) 2017 Chittaranjan Srinivas Swaminathan
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

#pragma once

#include <hiqp/tasks/tdef_geometric_alignment.h>
#include <hiqp/tasks/tdef_geometric_projection.h>
#include <string>
#include <vector>

#include <hiqp/robot_state.h>
#include <hiqp/task_definition.h>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>

#include <ros/ros.h>

namespace hiqp {
namespace tasks {
/*! \brief A task definition that positions geometric primitives relative to
 * each other through mutual geometric projection.
 *  \author Marcus A Johansson */
class TDefMetaTask : public TaskDefinition {
 public:
  inline TDefMetaTask() : TaskDefinition() {}

  TDefMetaTask(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
               std::shared_ptr<Visualizer> visualizer);

  ~TDefMetaTask() noexcept = default;

  int init(const std::vector<std::string>& parameters,
           RobotStatePtr robot_state);

  int update(RobotStatePtr robot_state);

  int monitor();

 private:
  TDefMetaTask(const TDefMetaTask& other) = delete;
  TDefMetaTask(TDefMetaTask&& other) = delete;
  TDefMetaTask& operator=(const TDefMetaTask& other) = delete;
  TDefMetaTask& operator=(TDefMetaTask&& other) noexcept = delete;

  std::vector<std::shared_ptr<TaskDefinition> > task_defs_;
  std::vector<int> rows;
};

}  // namespace tasks

}  // namespace hiqp
