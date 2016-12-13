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

#include <hiqp/utilities.h>

#include <hiqp/tasks/dynamics_jnt_limits.h>

namespace hiqp
{
namespace tasks
{

  int DynamicsJntLimits::init(const std::vector<std::string>& parameters,
                              RobotStatePtr robot_state,
                              const Eigen::VectorXd& e_initial,
                              const Eigen::VectorXd& e_final) {
    int size = parameters.size();
    if (size != 2) {
      printHiqpWarning("TDynJntLimits requires 2 parameters, got " 
        + std::to_string(size) + "! Initialization failed!");
      return -1;
    }

    e_dot_star_.resize(4);
    dq_max_ = std::stod( parameters.at(1) );
    performance_measures_.resize(0);
    return 0;
  }

  int DynamicsJntLimits::update(RobotStatePtr robot_state,
                                const Eigen::VectorXd& e,
                                const Eigen::MatrixXd& J) {
    e_dot_star_(0) = -dq_max_;
    e_dot_star_(1) = dq_max_;
    e_dot_star_(2) = -e(2); // first order dynamics with gain = -1
    e_dot_star_(3) = -e(3); // first order dynamics with gain = -1
    return 0;
  }

  int DynamicsJntLimits::monitor() {
    return 0;
  }

} // namespace tasks

} // namespace hiqp