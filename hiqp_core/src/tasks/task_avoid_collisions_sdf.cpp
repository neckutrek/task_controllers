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

#include <hiqp/tasks/task_avoid_collisions_sdf.h>
#include <hiqp/utilities.h>

#include <iostream>

namespace hiqp
{
namespace tasks
{


int TaskAvoidCollisionsSDF::init(const std::vector<std::string>& parameters,
                        RobotStatePtr robot_state,
                        unsigned int n_controls) {
  int size = parameters.size();
  if (size != 3) {
    printHiqpWarning("TaskAvoidCollisionsSDF requires 3 parameters, got " 
      + std::to_string(size) + "! Initialization failed!");
    return -1;
  }

  link_name_ = parameters.at(1);

  joint_q_nr_ = kdl_getQNrFromLinkName(robot_state->kdl_tree_, link_name_);

  if (joint_q_nr_ < 0) {
    printHiqpWarning("TaskAvoidCollisionsSDF::init, couldn't find joint '" + link_name_ + "'! Initialization failed.");
    return -2;
  }

  desired_configuration_ = std::stod( parameters.at(2) );

  e_.resize(1);
  J_.resize(1, n_controls);
  performance_measures_.resize(0);
  task_types_.insert(task_types_.begin(), 1, 0);

  for (int i=0; i<n_controls; ++i) 
    J_(0, i) = 0;

  J_(0, joint_q_nr_) = -1;

  return 0;
}

int TaskAvoidCollisionsSDF::update(RobotStatePtr robot_state) {
  const KDL::JntArray &q = robot_state->kdl_jnt_array_vel_.q;
  e_(0) = desired_configuration_ - q(joint_q_nr_);
  return 0;
}

int TaskAvoidCollisionsSDF::monitor() {
  return 0;
}


} // namespace tasks

} // namespace hiqp
