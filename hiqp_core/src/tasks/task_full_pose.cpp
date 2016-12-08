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

#include <hiqp/tasks/task_full_pose.h>

#include <hiqp/utilities.h>

#include <iostream>

namespace hiqp
{
namespace tasks
{

  int TaskFullPose::init(const std::vector<std::string>& parameters,
                         RobotStatePtr robot_state) {
    int size = parameters.size();
    unsigned int n_controls = robot_state->getNumControls();
    unsigned int n_joints = robot_state->getNumJoints();
    if (size != 1 && size != n_controls + 1) {
      printHiqpWarning("TaskFullPose requires 1 or " 
        + std::to_string(n_controls+1) + " parameters, got " 
        + std::to_string(size) + "! Initialization failed!");
      return -1;
    }

    if (size == 1) {
      desired_configuration_ = std::vector<double>(n_controls, 0);
    } else {
      desired_configuration_.resize(0);
      for (int i=1; i < n_controls+1; ++i) {
        desired_configuration_.push_back( std::stod( parameters.at(i) ) );

      }
    }

    e_ = Eigen::VectorXd::Zero(n_controls);
    J_ = Eigen::MatrixXd::Zero(n_controls, n_joints);
    performance_measures_.resize(0);
    task_types_.insert(task_types_.begin(), n_controls, 0);

    // The jacobian has zero columns for non-writable joints
    // -1  0  0  0  0
    //  0 -1  0  0  0
    //  0  0  0 -1  0
    for (int c=0, r=0; c<n_joints; ++c) {
      if (robot_state->isQNrWritable(c)) {
        J_(r, c) = -1;
        r++;
      }
    }

    return 0;
  }
/// \bug Noticed, that the measured values for the 4 gripper joints can be huge which causes the optimization to fail for this task. Maybe due to the discrepancy between specified and controlled joints ...?  
  int TaskFullPose::update(RobotStatePtr robot_state) {
    const KDL::JntArray &q = robot_state->kdl_jnt_array_vel_.q;
    int j=0;
    for (int i=0; i<q.rows(); ++i) {
      if (robot_state->isQNrWritable(i)) {
        e_(j) = desired_configuration_.at(j) - q(i);
        j++;
      }
    }
    // std::cout << "--- update ---\n";
    // std::cout << "update e_ = " << e_ << "\n";

    return 0;
  }

  int TaskFullPose::monitor() {
    return 0;
  }

} // namespace tasks

} // namespace hiqp
